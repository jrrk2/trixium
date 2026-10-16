/*
 * This file is part of TriXium.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* SD card image load (boards without a host link: the VC707).
 *
 * The firmware side of software/user/trixium_loader.c: a BONS image on the SD card's FAT
 * partition goes through the loader's CSR poke path, the header+table to channel 0's staging
 * area and each channel's payload to its window (the loader adds the window offset), then
 * hdrload. The file is read in one sequential pass of contiguous cluster runs (FatFs clips
 * f_read at every cluster, ~0.5 MB/s with 4 KB clusters; a multi-block DMA per run approaches
 * the card's rate) into a main_ram staging buffer the SD DMA writes past any cache (the SoC has
 * no L2, memory/ddr.py), then poked word by word into the loader.
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <generated/mem.h>
#include <libbase/progress.h>
#include <system.h>

#include "sdload.h"

#if defined(TRIXIUM_SDLOAD) && defined(CSR_SDCARD_BASE)

#include <liblitesdcard/sdcard.h>
#include <libfatfs/ff.h>

#include "bons.h"

#ifndef SD_CLK_HZ
#define SD_CLK_HZ 50000000         // SDR25 (liblitesdcard's init leaves the bus at 25 MHz).
#endif
#define SD_DMA_MAX_SECTORS 256u    // 128 KB per command, inside the PHY's 1 s data timeout.
#define SD_BUF       (MAIN_RAM_BASE + 0x04000000u) // main_ram + 64 MB: between the low heap
#define SD_BUF_SIZE  0x01000000u                   // (< 64 MB) and the high heap (>= 144 MB).
#define HDR_STAGING  (TRIXIUM_CHAN_WEIGHT_BASE - 0x100000)
#define CHUNK_BEATS  (4u << 20)    // The loader's 24-bit beat count: 128 MB per armed transfer.

static int sd_ready;

static int sd_mount(FATFS *fs)
{
    if (!sd_ready) {
        if (!sdcard_init()) { puts("sdload: no card / init failed"); return 0; }
        sdcard_set_clk_freq(SD_CLK_HZ, 0);
        fatfs_set_ops_sdcard();
        sd_ready = 1;
    }
    FRESULT fr = f_mount(fs, "", 1);
    if (fr != FR_OK) { printf("sdload: mount failed (FatFs %d)\n", fr); sd_ready = 0; return 0; }
    return 1;
}

/* FAT walk */
/*----------*/

static uint8_t  fatbuf[512] __attribute__((aligned(64)));
static uint32_t fat_cached_sec;

static uint32_t fat_next(FATFS *fs, uint32_t clst)
{
    uint32_t sec, off;
    if (fs->fs_type == FS_FAT32)      { sec = fs->fatbase + clst/128; off = (clst % 128)*4; }
    else if (fs->fs_type == FS_FAT16) { sec = fs->fatbase + clst/256; off = (clst % 256)*2; }
    else return 0xFFFFFFFFu;
    if (fat_cached_sec != sec) {
        if (sdcard_read(sec, 1, fatbuf) != 0) return 0xFFFFFFFFu;
        flush_cpu_dcache();
        fat_cached_sec = sec;
    }
    if (fs->fs_type == FS_FAT32) return (*(uint32_t *)(fatbuf + off)) & 0x0FFFFFFFu;
    return *(uint16_t *)(fatbuf + off);
}

static int sd_read_sectors(uint32_t sec, uint32_t n, uint8_t *dst)
{
    while (n) {
        uint32_t k = n > SD_DMA_MAX_SECTORS ? SD_DMA_MAX_SECTORS : n;
        if (sdcard_read(sec, k, dst) != 0) {
            sdcard_stop_transmission();          // Leave the card sane, retry once.
            busy_wait(10);
            if (sdcard_read(sec, k, dst) != 0) {
                sdcard_stop_transmission();
                printf("\nsdload: read error at sector %lu\n", (unsigned long)sec);
                return -1;
            }
        }
        sec += k; n -= k; dst += k*512;
    }
    return 0;
}

/* Sequential reader: the next contiguous piece of the file, at most SD_BUF_SIZE. */
struct sd_stream {
    FATFS   *fs;
    uint32_t clst;      // Current cluster.
    uint32_t csec;      // Sectors already consumed of the current cluster.
    uint64_t left;      // File bytes not yet read.
};

static int64_t sd_stream_read(struct sd_stream *s, uint8_t *dst)
{
    uint32_t csize = s->fs->csize, got = 0;
    while (s->left > got && got < SD_BUF_SIZE) {
        // Extend the run over contiguous clusters, bounded by the buffer and the file.
        uint32_t room = (SD_BUF_SIZE - got)/512;
        uint32_t need = (uint32_t)((s->left - got + 511)/512);
        uint32_t want = room < need ? room : need;
        uint32_t nsec = csize - s->csec, last = s->clst;
        while (nsec < want) {
            uint32_t nxt = fat_next(s->fs, last);
            if (nxt != last + 1) break;
            last = nxt; nsec += csize;
        }
        if (nsec > want) nsec = want;
        uint32_t sec = (uint32_t)s->fs->database + (s->clst - 2)*csize + s->csec;
        if (sd_read_sectors(sec, nsec, dst + got) < 0) return -1;
        got += nsec*512;
        // Advance (clst, csec) by nsec sectors.
        uint32_t adv = s->csec + nsec;
        while (adv >= csize) {
            adv -= csize;
            if (s->left > got || adv) {
                uint32_t nxt = fat_next(s->fs, s->clst);
                if (nxt < 2 || nxt >= 0x0FFFFFF7u) { s->clst = 0; break; }
                s->clst = nxt;
            }
        }
        s->csec = adv;
        if (s->clst == 0 && s->left > got) { puts("\nsdload: cluster chain ends early"); return -1; }
    }
    if (got > s->left) got = (uint32_t)s->left;
    s->left -= got;
    flush_cpu_dcache();
    return got;
}

/* Loader */
/*--------*/

static int loader_wait(void)
{
    for (uint32_t t = 0; t < 2000000; t++) {
        if (trixium_loader_status_read() & 1) return 0;
        busy_wait_us(10);
    }
    uint32_t st = trixium_loader_status_read();
    printf("\nsdload: loader timeout (status %08lx, %lu beats)\n",
           (unsigned long)st, (unsigned long)((st >> 8) & 0xFFFFFF));
    return -1;
}

static void loader_arm(uint32_t chan, uint32_t base, uint32_t beats)
{
    trixium_loader_src_sel_write(1);
    trixium_loader_chan_write(chan);
    trixium_loader_base_write(base);
    trixium_loader_length_write(beats);
    trixium_loader_resync_write(0);
    trixium_loader_start_write(1);
}

/* The image as a list of transfers, fed from the file in order: segment k covers file bytes
 * [off, off + bytes) and goes to (chan, base), zero-padded to whole beats and armed in
 * CHUNK_BEATS pieces. */
struct seg { uint32_t chan, base; uint64_t off, bytes; };

struct feeder {
    const struct seg *segs;
    int      nseg, k;
    uint64_t done;      // Bytes of segs[k] poked.
    uint32_t piece;     // Beats left in the armed piece (0: none armed).
    uint32_t piece_words;
};

static int feed_arm_next(struct feeder *f)
{
    const struct seg *s = &f->segs[f->k];
    uint64_t total = (s->bytes + 31)/32;
    uint64_t start = f->done/32;
    uint32_t beats = (uint32_t)(total - start > CHUNK_BEATS ? CHUNK_BEATS : total - start);
    loader_arm(s->chan, s->base + (uint32_t)(start*32), beats);
    f->piece = beats;
    f->piece_words = beats*8;
    return 0;
}

static void poke(uint32_t w)
{
    trixium_loader_poke_write(w);
}

/* Poke the words of [pos, pos + n) of the file (pos 4-aligned). */
static int feed(struct feeder *f, uint64_t pos, const uint32_t *w, uint32_t nwords)
{
    for (uint32_t i = 0; i < nwords; i++, pos += 4) {
        while (f->k < f->nseg && pos >= f->segs[f->k].off + f->segs[f->k].bytes) {
            // Segment complete: pad its last beat, finish the piece.
            while (f->piece_words) { poke(0); f->piece_words--; }
            if (f->piece && loader_wait() < 0) return -1;
            f->piece = 0; f->k++; f->done = 0;
        }
        if (f->k >= f->nseg) return 0;
        if (pos < f->segs[f->k].off) continue;          // Gap between segments: skip.
        if (f->piece_words == 0) {
            if (f->piece && loader_wait() < 0) return -1;
            feed_arm_next(f);
        }
        if ((f->piece_words & 0xFF) == 0)
            while (trixium_loader_status_read() & 4)    // FIFO full.
                ;
        poke(w[i]);
        f->piece_words--;
        f->done += 4;
    }
    return 0;
}

static int feed_finish(struct feeder *f)
{
    while (f->k < f->nseg) {
        while (f->piece_words) { poke(0); f->piece_words--; }
        if (f->piece && loader_wait() < 0) return -1;
        f->piece = 0;
        if (f->done < f->segs[f->k].bytes) {
            puts("\nsdload: file ended inside a segment");
            return -1;
        }
        f->k++; f->done = 0;
    }
    return 0;
}

/* sdload */
/*--------*/

int sdload(const char *name)
{
    static FATFS fs;
    FIL fil;
    if (!sd_mount(&fs)) return -1;
    if (f_open(&fil, name, FA_READ) != FR_OK) { printf("sdload: %s not found\n", name); return -1; }
    uint64_t len  = f_size(&fil);
    uint32_t clst = fil.obj.sclust;
    f_close(&fil);
    if (fs.fs_type != FS_FAT32 && fs.fs_type != FS_FAT16) { puts("sdload: not FAT16/32"); return -1; }
    if (clst < 2 || len < sizeof(struct bons_header)) { printf("sdload: %s is empty\n", name); return -1; }

    fat_cached_sec = 0xFFFFFFFFu;
    struct sd_stream s = { &fs, clst, 0, len };
    uint8_t *buf = (uint8_t *)SD_BUF;
    int64_t n = sd_stream_read(&s, buf);
    if (n < (int64_t)sizeof(struct bons_header)) return -1;

    const struct bons_header *h = (const struct bons_header *)buf;
    if (h->magic != BONS_MAGIC) { printf("sdload: %s is not a BONS image\n", name); return -1; }
    if (h->n_channels != TRIXIUM_N_CHANNELS) {
        printf("sdload: image has %lu channels, the gateware %d (convert --board vc707)\n",
               (unsigned long)h->n_channels, TRIXIUM_N_CHANNELS);
        return -1;
    }
    uint64_t chan_bytes = h->payload_beats*32;
    if (h->payload_offset % 4 || len < h->payload_offset + h->n_channels*chan_bytes) {
        puts("sdload: truncated image");
        return -1;
    }
    struct seg segs[1 + 32];
    int nseg = 0;
    segs[nseg++] = (struct seg){ 0, HDR_STAGING, 0, h->payload_offset };
    for (uint32_t c = 0; c < h->n_channels; c++)
        segs[nseg++] = (struct seg){ c, TRIXIUM_CHAN_WEIGHT_BASE,
                                     h->payload_offset + c*chan_bytes, chan_bytes };
    printf("sdload: %s, %lu layers, %lu channels x %lu KB\n", name, (unsigned long)h->n_layer,
           (unsigned long)h->n_channels, (unsigned long)(chan_bytes/1024));

    uint64_t total = h->payload_offset + h->n_channels*chan_bytes;
    struct feeder f = { segs, nseg, 0, 0, 0, 0 };
    uint64_t pos = 0;
    init_progression_bar(total);
    while (1) {
        uint32_t words = (uint32_t)((n + 3)/4);
        if (n & 3) memset(buf + n, 0, 4 - (n & 3));
        if (feed(&f, pos, (const uint32_t *)buf, words) < 0) return -1;
        pos += n;
        show_progress(pos < total ? pos : total);
        if (pos >= total || s.left == 0) break;
        n = sd_stream_read(&s, buf);
        if (n <= 0) return -1;
    }
    if (feed_finish(&f) < 0) return -1;
    printf("\nsdload: %lu MB loaded\n", (unsigned long)(total >> 20));
    return 0;
}

#else

int sdload(const char *name)
{
    (void)name;
    puts("sdload: no SD card in this SoC");
    return -1;
}

#endif
