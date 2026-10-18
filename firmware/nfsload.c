/*
 * This file is part of TriXium.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* NFS image load (boards without a host link: the VC707).
 *
 * A minimal NFSv3 client over LiteX's UDP stack (libliteeth): portmap GETPORT, MOUNT MNT of
 * the export, LOOKUP of each path component, then READs of the BONS image fed in file order
 * to the loader (imgfeed.c). libliteeth does not reassemble IP fragments, so every reply must
 * fit one frame: READs of NFS_CHUNK bytes (+ ~130 bytes of RPC/NFS header). Throughput comes
 * from NFS_WINDOW READs in flight (the MAC has 16 RX slots), reordered in a main_ram buffer
 * and retransmitted after NFS_TIMEOUT_MS. The export has no `insecure`: requests leave from a
 * privileged port. Server: the SoC's REMOTEIP (the TFTP boot server).
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <generated/mem.h>

#include "nfsload.h"

#if defined(TRIXIUM_NFSLOAD) && defined(CSR_ETHMAC_BASE)

#include <libliteeth/udp.h>

#include "imgfeed.h"
#include "perf.h"

#ifndef TRIXIUM_NFS_EXPORT
#define TRIXIUM_NFS_EXPORT "/srv/nfs/vc707"
#endif
#define NFS_CHUNK       1024u
#define NFS_WINDOW      8u
#define NFS_TIMEOUT_MS  50u
#define NFS_RETRIES     40u
#define NFS_LOCAL_PORT  700u          // Privileged: the export is not `insecure`.
#define NFS_BUF         (MAIN_RAM_BASE + 0x04000000u) // main_ram + 64 MB (see sdload.c).

#define PROG_PORTMAP 100000u
#define PROG_MOUNT   100005u
#define PROG_NFS     100003u
#define NFS3_LOOKUP  3u
#define NFS3_READ    6u
#define FH_MAX       64u

/* MAC receive counters: frames lost in the RX slots (all full), bad preamble / FCS, and (with
 * store-and-forward) frames the PHY-side packet FIFO dropped because the CDC behind it stalled. */
static void ethstat_line(void)
{
    printf("slots-full %lu", (unsigned long)ethmac_sram_writer_errors_read());
#ifdef CSR_ETHMAC_RX_DATAPATH_CRC_ERRORS_ADDR
    printf(", preamble %lu, crc %lu",
           (unsigned long)ethmac_rx_datapath_preamble_errors_read(),
           (unsigned long)ethmac_rx_datapath_crc_errors_read());
#endif
#ifdef CSR_ETHMAC_RX_DATAPATH_DROPS_ADDR
    printf(", sf-drops %lu", (unsigned long)ethmac_rx_datapath_drops_read());
#endif
}

void ethbench(void)
{
    volatile uint32_t *sram = (volatile uint32_t *)ETHMAC_BASE;
    uint32_t acc = 0, n = 256;
    uint64_t t0 = perf_now64();
    for (uint32_t i = 0; i < n; i++) acc += sram[i];
    uint64_t t1 = perf_now64();
    for (uint32_t i = 0; i < n; i++) acc += ethmac_sram_writer_ev_pending_read();
    uint64_t t2 = perf_now64();
    for (uint32_t i = 0; i < n; i++) acc += ((volatile uint8_t *)ETHMAC_BASE)[i];
    uint64_t t3 = perf_now64();
    printf("ethbench: cycles/read: mac sram word %lu, byte %lu, ev_pending csr %lu (acc %lx)\n",
           (unsigned long)((t1 - t0)/n), (unsigned long)((t3 - t2)/n),
           (unsigned long)((t2 - t1)/n), (unsigned long)acc);
}

void ethstat(void)
{
    printf("ethstat: ");
    ethstat_line();
    printf("\n");
}

/* XDR */
/*-----*/

static inline void put32(uint8_t *p, uint32_t v)
{
    p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v;
}
static inline uint32_t get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
static uint32_t put_opaque(uint8_t *p, const void *d, uint32_t n) // Length + data, 4-padded.
{
    put32(p, n);
    memcpy(p + 4, d, n);
    uint32_t pad = (4 - (n & 3)) & 3;
    memset(p + 4 + n, 0, pad);
    return 4 + n + pad;
}

/* RPC */
/*-----*/

static uint32_t server_ip;
static uint32_t xid_seq = 0x54520000u;

/* Call header (AUTH_UNIX uid/gid 0, AUTH_NONE verifier); returns its length. */
static uint32_t rpc_header(uint8_t *p, uint32_t xid, uint32_t prog, uint32_t vers, uint32_t proc)
{
    static const char host[] = "trixium";
    uint8_t *q = p;
    put32(q, xid); put32(q + 4, 0); put32(q + 8, 2);
    put32(q + 12, prog); put32(q + 16, vers); put32(q + 20, proc);
    q += 24;
    put32(q, 1);                                   // AUTH_UNIX
    uint8_t *len = q + 4; q += 8;
    uint8_t *body = q;
    put32(q, 0); q += 4;                           // stamp
    q += put_opaque(q, host, sizeof(host) - 1);    // machinename
    put32(q, 0); put32(q + 4, 0); put32(q + 8, 0); // uid, gid, no gids
    q += 12;
    put32(len, (uint32_t)(q - body));
    put32(q, 0); put32(q + 4, 0); q += 8;          // verifier AUTH_NONE
    return (uint32_t)(q - p);
}

/* The accepted reply's results, or NULL (rejected, bad xid, error status). */
static const uint8_t *rpc_results(const uint8_t *r, uint32_t len, uint32_t *rlen)
{
    if (len < 24 || get32(r + 4) != 1 || get32(r + 8) != 0) return NULL; // REPLY, MSG_ACCEPTED
    uint32_t vlen = (get32(r + 16) + 3) & ~3u;
    if (24 + vlen > len || get32(r + 20 + vlen) != 0) return NULL;       // SUCCESS
    *rlen = len - (24 + vlen);
    return r + 24 + vlen;
}

/* Synchronous calls (portmap, mount, lookup): the reply lands in sync_buf. */
static uint8_t  sync_buf[1536];
static uint32_t sync_len, sync_xid;
static volatile int sync_got;

/* Pipelined READs. */
static uint64_t file_size;
static uint32_t n_chunks, next_feed, next_send;
static uint8_t  rd_ready[NFS_WINDOW];
static uint16_t rd_len[NFS_WINDOW];
static uint64_t rd_sent[NFS_WINDOW];
static uint32_t rd_xid0;
static uint32_t rd_dup, rd_bad, rd_rx;   // rd_rx: every UDP frame for us (replies + dups).

static void rx(uint32_t src_ip, uint16_t src_port, uint16_t dst_port, void *data, uint32_t length)
{
    (void)src_port;
    rd_rx++;
    if (src_ip != server_ip || dst_port != NFS_LOCAL_PORT || length < 4) return;
    const uint8_t *r = data;
    uint32_t xid = get32(r);
    if (xid == sync_xid && !sync_got) {
        sync_len = length > sizeof(sync_buf) ? sizeof(sync_buf) : length;
        memcpy(sync_buf, r, sync_len);
        sync_got = 1;
        return;
    }
    uint32_t c = xid - rd_xid0;
    if (c < next_feed || c >= next_send) { rd_dup++; return; }
    uint32_t slot = c % NFS_WINDOW;
    if (rd_ready[slot]) { rd_dup++; return; }
    uint32_t rl;
    const uint8_t *p = rpc_results(r, length, &rl);
    if (!p || rl < 8 || get32(p) != 0) { rd_bad++; return; }            // NFS3_OK
    uint32_t o = 4;
    if (get32(p + o)) o += 4 + 84; else o += 4;                          // post_op_attr
    if (o + 12 > rl) { rd_bad++; return; }
    uint32_t n = get32(p + o + 8);                                       // count, eof, data len
    o += 12;
    if (o + n > rl || n > NFS_CHUNK) { rd_bad++; return; }
    memcpy((uint8_t *)NFS_BUF + slot*NFS_CHUNK, p + o, n);
    rd_len[slot]   = n;
    rd_ready[slot] = 1;
}

static uint64_t ms_cycles(uint32_t ms)
{
    return (uint64_t)CONFIG_CLOCK_FREQUENCY/1000*ms;
}

/* Send a call built in the tx buffer and wait for its reply; returns the results or NULL. */
static const uint8_t *rpc_sync(uint16_t port, uint32_t len, uint32_t *rlen)
{
    uint8_t *tx = udp_get_tx_buffer();
    uint8_t  copy[512];
    memcpy(copy, tx, len);                       // udp_service may reuse the tx buffer.
    sync_xid = get32(copy);
    for (uint32_t t = 0; t < 10; t++) {
        memcpy(udp_get_tx_buffer(), copy, len);
        sync_got = 0;
        udp_send(NFS_LOCAL_PORT, port, len);
        uint64_t t0 = perf_now64();
        while (!sync_got && perf_now64() - t0 < ms_cycles(500))
            udp_service();
        if (sync_got)
            return rpc_results(sync_buf, sync_len, rlen);
    }
    puts("nfsload: no reply from the server");
    return NULL;
}

static uint32_t portmap_getport(uint32_t prog, uint32_t vers)
{
    uint8_t *tx = udp_get_tx_buffer();
    uint32_t n = rpc_header(tx, ++xid_seq, PROG_PORTMAP, 2, 3);
    put32(tx + n, prog); put32(tx + n + 4, vers); put32(tx + n + 8, 17); put32(tx + n + 12, 0);
    uint32_t rl;
    const uint8_t *r = rpc_sync(111, n + 16, &rl);
    return (r && rl >= 4) ? get32(r) : 0;
}

/* fh/fh_len: in = directory, out = the entry's; size: the entry's size from its attributes. */
static int nfs_lookup(uint16_t port, uint8_t *fh, uint32_t *fh_len, const char *name,
                      uint32_t name_len, uint64_t *size)
{
    uint8_t *tx = udp_get_tx_buffer();
    uint32_t n = rpc_header(tx, ++xid_seq, PROG_NFS, 3, NFS3_LOOKUP);
    n += put_opaque(tx + n, fh, *fh_len);
    n += put_opaque(tx + n, name, name_len);
    uint32_t rl;
    const uint8_t *r = rpc_sync(port, n, &rl);
    if (!r || rl < 8) return -1;
    if (get32(r) != 0) { printf("nfsload: %.*s: NFS error %lu\n", (int)name_len, name,
                                (unsigned long)get32(r)); return -1; }
    uint32_t l = get32(r + 4);
    if (l > FH_MAX || 8 + l > rl) return -1;
    memcpy(fh, r + 8, l);
    *fh_len = l;
    uint32_t o = 8 + ((l + 3) & ~3u);
    if (o + 4 + 84 <= rl && get32(r + o))       // obj_attributes: fattr3, size at +20.
        *size = ((uint64_t)get32(r + o + 4 + 20) << 32) | get32(r + o + 4 + 24);
    return 0;
}

static void send_read(uint16_t port, const uint8_t *fh, uint32_t fh_len, uint32_t c)
{
    uint8_t *tx = udp_get_tx_buffer();
    uint32_t n = rpc_header(tx, rd_xid0 + c, PROG_NFS, 3, NFS3_READ);
    n += put_opaque(tx + n, fh, fh_len);
    uint64_t off = (uint64_t)c*NFS_CHUNK;
    put32(tx + n, (uint32_t)(off >> 32)); put32(tx + n + 4, (uint32_t)off);
    put32(tx + n + 8, NFS_CHUNK);
    udp_send(NFS_LOCAL_PORT, port, n + 12);
    rd_sent[c % NFS_WINDOW] = perf_now64();
}

/* nfsload */
/*---------*/

// The sink: the engine loader (imgfeed, nfsload) or CPU memory (nfsread).
static uint8_t *mem_dst;
static uint32_t mem_max;

static int nfs_fetch(const char *path)
{
    static const uint8_t mac[6] = {MACADDR1, MACADDR2, MACADDR3, MACADDR4, MACADDR5, MACADDR6};
    server_ip = IPTOINT(REMOTEIP1, REMOTEIP2, REMOTEIP3, REMOTEIP4);
    udp_start(mac, IPTOINT(LOCALIP1, LOCALIP2, LOCALIP3, LOCALIP4));
    udp_set_callback(rx);
    if (!udp_arp_resolve(server_ip)) { puts("nfsload: server does not answer ARP"); return -1; }

    uint32_t mount_port = portmap_getport(PROG_MOUNT, 3);
    uint32_t nfs_port   = portmap_getport(PROG_NFS, 3);
    if (!mount_port || !nfs_port) { puts("nfsload: portmap failed (mount/nfs v3 over UDP?)"); return -1; }

    // MOUNT v3 MNT: the export's root handle.
    uint8_t  fh[FH_MAX];
    uint32_t fh_len;
    {
        uint8_t *tx = udp_get_tx_buffer();
        uint32_t n = rpc_header(tx, ++xid_seq, PROG_MOUNT, 3, 1);
        n += put_opaque(tx + n, TRIXIUM_NFS_EXPORT, sizeof(TRIXIUM_NFS_EXPORT) - 1);
        uint32_t rl;
        const uint8_t *r = rpc_sync(mount_port, n, &rl);
        if (!r || rl < 8 || get32(r) != 0) {
            printf("nfsload: mount %s refused (%ld)\n", TRIXIUM_NFS_EXPORT, r ? (long)get32(r) : -1L);
            return -1;
        }
        fh_len = get32(r + 4);
        if (fh_len > FH_MAX) return -1;
        memcpy(fh, r + 8, fh_len);
    }

    // LOOKUP each component.
    file_size = 0;
    for (const char *p = path; *p; ) {
        while (*p == '/') p++;
        const char *e = p;
        while (*e && *e != '/') e++;
        if (e > p && nfs_lookup((uint16_t)nfs_port, fh, &fh_len, p, (uint32_t)(e - p), &file_size) < 0)
            return -1;
        p = e;
    }
    if (file_size == 0) { printf("nfsload: %s: empty or no size\n", path); return -1; }
    printf("nfsload: %s:%s/%s (%lu %s)\n", "server", TRIXIUM_NFS_EXPORT, path,
           (unsigned long)(file_size >= (16u << 20) ? file_size >> 20 : file_size >> 10),
           file_size >= (16u << 20) ? "MB" : "KB");
    if (mem_dst && file_size > mem_max) {
        printf("nfsload: %s is larger than its %lu KB buffer\n", path, (unsigned long)(mem_max >> 10));
        return -1;
    }

    // READs, NFS_WINDOW in flight, fed in order.
    n_chunks  = (uint32_t)((file_size + NFS_CHUNK - 1)/NFS_CHUNK);
    next_feed = next_send = 0;
    rd_xid0   = 0x4E460000u;                      // "NF": chunk c is xid rd_xid0 + c.
    rd_dup = rd_bad = rd_rx = 0;
    uint64_t cy_send = 0, cy_svc = 0, cy_feed = 0, cy_frame = 0, t_a;
    uint32_t n_frames = 0;  // Where the load's time goes.
    memset(rd_ready, 0, sizeof(rd_ready));
    uint32_t retries = 0, resent = 0;
    uint64_t t_start = perf_now64(), total = 0, t_status = t_start;
    while (next_feed < n_chunks) {
        while (next_send < n_chunks && next_send < next_feed + NFS_WINDOW) {
            rd_ready[next_send % NFS_WINDOW] = 0;
            t_a = perf_now64();
            send_read((uint16_t)nfs_port, fh, fh_len, next_send++);
            cy_send += perf_now64() - t_a;
        }
        int had = ethmac_sram_writer_ev_pending_read() & ETHMAC_EV_SRAM_WRITER;
        t_a = perf_now64();
        udp_service();
        cy_svc += perf_now64() - t_a;
        if (had) { n_frames++; cy_frame += perf_now64() - t_a; }
        while (next_feed < next_send && rd_ready[next_feed % NFS_WINDOW]) {
            uint32_t slot = next_feed % NFS_WINDOW;
            const uint8_t *d = (const uint8_t *)NFS_BUF + slot*NFS_CHUNK;
            if (next_feed == 0) {
                if (mem_dst)
                    total = file_size;
                else if (imgfeed_begin(d, rd_len[slot], file_size) < 0)
                    return -1;
                else
                    total = imgfeed_total();
            }
            t_a = perf_now64();
            if (mem_dst)
                memcpy(mem_dst + (uint64_t)next_feed*NFS_CHUNK, d, rd_len[slot]);
            else if (imgfeed_push(d, rd_len[slot]) < 0)
                return -1;
            cy_feed += perf_now64() - t_a;
            rd_ready[slot] = 0;
            next_feed++;
            retries = 0;
            if ((uint64_t)next_feed*NFS_CHUNK >= total) goto done;
        }
        // Retransmit what timed out (lost request or reply).
        uint64_t now = perf_now64();
        if (now - t_status > ms_cycles(2000)) {           // Status every 2 s, progress or not.
            t_status = now;
            uint32_t ms = (uint32_t)((now - t_start)/(CONFIG_CLOCK_FREQUENCY/1000));
            uint32_t kc = CONFIG_CLOCK_FREQUENCY/1000;
            printf("nfsload: %lu KB fed, %lu KB/s, %lu resent, %lu rx (%lu dup, %lu bad); "
                   "ms send %lu svc %lu feed %lu of %lu; %lu frames, %lu us/frame; ",
                   (unsigned long)next_feed,
                   (unsigned long)(ms ? (uint64_t)next_feed*NFS_CHUNK/ms : 0),
                   (unsigned long)resent, (unsigned long)rd_rx, (unsigned long)rd_dup,
                   (unsigned long)rd_bad, (unsigned long)(cy_send/kc), (unsigned long)(cy_svc/kc),
                   (unsigned long)(cy_feed/kc), (unsigned long)ms, (unsigned long)n_frames,
                   (unsigned long)(n_frames ? cy_frame/n_frames/(kc/1000) : 0));
            ethstat_line();
            printf("\n");
        }
        for (uint32_t c = next_feed; c < next_send; c++) {
            uint32_t slot = c % NFS_WINDOW;
            if (!rd_ready[slot] && now - rd_sent[slot] > ms_cycles(NFS_TIMEOUT_MS)) {
                if (c == next_feed && ++retries > NFS_RETRIES) {
                    printf("\nnfsload: no reply for offset %lu\n", (unsigned long)c*NFS_CHUNK);
                    return -1;
                }
                send_read((uint16_t)nfs_port, fh, fh_len, c);
                resent++;
            }
        }
    }
done:
    if (!mem_dst && imgfeed_end() < 0) return -1;
    uint64_t dt = perf_now64() - t_start;
    uint32_t ms = (uint32_t)(dt/(CONFIG_CLOCK_FREQUENCY/1000));
    printf("\rnfsload: %lu %s in %lu.%01lu s (%lu KB/s), %lu resent, %lu dup, %lu bad\n",
           (unsigned long)(total >= (16u << 20) ? total >> 20 : total >> 10),
           total >= (16u << 20) ? "MB" : "KB", (unsigned long)(ms/1000), (unsigned long)(ms%1000)/100,
           (unsigned long)(ms ? total/ms : 0), (unsigned long)resent,
           (unsigned long)rd_dup, (unsigned long)rd_bad);
    return 0;
}

int nfsload(const char *path)
{
    mem_dst = NULL;
    return nfs_fetch(path);
}

int nfsread(const char *path, void *dst, uint32_t max)
{
    mem_dst = (uint8_t *)dst;
    mem_max = max;
    int r = nfs_fetch(path);
    mem_dst = NULL;
    return r < 0 ? r : (int)file_size;
}

#else

void ethstat(void)
{
    puts("ethstat: no Ethernet in this SoC");
}

void ethbench(void)
{
    ethstat();
}

int nfsload(const char *path)
{
    (void)path;
    puts("nfsload: no Ethernet in this SoC");
    return -1;
}

int nfsread(const char *path, void *dst, uint32_t max)
{
    (void)path; (void)dst; (void)max;
    return -1;
}

#endif
