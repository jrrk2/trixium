/*
 * This file is part of TriXium.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 * SPDX-License-Identifier: BSD-2-Clause
 */

/* BONS image -> loader CSR pokes, the firmware side of software/user/trixium_loader.c.
 *
 * The image is a list of transfers fed from the file in order: segment k covers file bytes
 * [off, off + bytes) and goes to (chan, base) (the loader adds the window offset), zero-padded
 * to whole beats and armed in CHUNK_BEATS pieces (the loader's 24-bit beat count). */

#include <stdio.h>
#include <string.h>

#include <generated/csr.h>
#include <generated/soc.h>
#include <system.h>

#include "bons.h"
#include "imgfeed.h"

#define HDR_STAGING  (TRIXIUM_CHAN_WEIGHT_BASE - 0x100000)
#define CHUNK_BEATS  (4u << 20)    // 128 MB per armed transfer.

struct seg { uint32_t chan, base; uint64_t off, bytes; };

static struct seg segs[1 + 32];
static int      nseg, k;
static uint64_t done;          // Bytes of segs[k] poked.
static uint32_t piece;         // Beats of the armed piece (0: none armed).
static uint32_t piece_words;   // Words of it still to poke.
static uint64_t pos, total;    // File position of the next push; bytes the image needs.
static uint32_t beat0[8];      // The header's first beat, staged last (imgfeed_end).

static int loader_wait(void)
{
    for (uint32_t t = 0; t < 2000000; t++) {
        if (trixium_loader_status_read() & 1) return 0;
        busy_wait_us(10);
    }
    uint32_t st = trixium_loader_status_read();
    printf("\nimgfeed: loader timeout (status %08lx, %lu beats)\n",
           (unsigned long)st, (unsigned long)((st >> 8) & 0xFFFFFF));
    return -1;
}

static void arm_next(void)
{
    const struct seg *s = &segs[k];
    uint64_t beats_total = (s->bytes + 31)/32, start = done/32;
    uint32_t beats = (uint32_t)(beats_total - start > CHUNK_BEATS ? CHUNK_BEATS : beats_total - start);
    trixium_loader_src_sel_write(1);
    trixium_loader_chan_write(s->chan);
    trixium_loader_base_write(s->base + (uint32_t)(start*32));
    trixium_loader_length_write(beats);
    trixium_loader_resync_write(0);
    trixium_loader_start_write(1);
    piece = beats;
    piece_words = beats*8;
}

/* Pad the armed piece to its end and wait for it. */
static int finish_piece(void)
{
    while (piece_words) { trixium_loader_poke_write(0); piece_words--; }
    if (piece && loader_wait() < 0) return -1;
    piece = 0;
    return 0;
}

int imgfeed_begin(const uint8_t *head, uint32_t n, uint64_t file_len)
{
    const struct bons_header *h = (const struct bons_header *)head;
    if (n < sizeof(*h) || n < sizeof(beat0) || h->magic != BONS_MAGIC) { puts("imgfeed: not a BONS image"); return -1; }
    if (h->n_channels != TRIXIUM_N_CHANNELS) {
        printf("imgfeed: image has %lu channels, the gateware %d (convert --board vc707)\n",
               (unsigned long)h->n_channels, TRIXIUM_N_CHANNELS);
        return -1;
    }
    uint64_t chan_bytes = h->payload_beats*32;
    total = h->payload_offset + h->n_channels*chan_bytes;
    if (h->payload_offset % 4 || file_len < total) { puts("imgfeed: truncated image"); return -1; }
    nseg = 0;
    segs[nseg++] = (struct seg){ 0, HDR_STAGING, 0, h->payload_offset };
    for (uint32_t c = 0; c < h->n_channels; c++)
        segs[nseg++] = (struct seg){ c, TRIXIUM_CHAN_WEIGHT_BASE,
                                     h->payload_offset + c*chan_bytes, chan_bytes };
    k = 0; done = 0; piece = 0; piece_words = 0; pos = 0;
    memcpy(beat0, head, sizeof(beat0));
    printf("image: %lu layers, %lu channels x %lu KB\n", (unsigned long)h->n_layer,
           (unsigned long)h->n_channels, (unsigned long)(chan_bytes/1024));
    return 0;
}

uint64_t imgfeed_total(void)
{
    return total;
}

int imgfeed_push(const uint8_t *data, uint32_t n)
{
    uint32_t i = 0;
    while (i < n && k < nseg) {
        const struct seg *s = &segs[k];
        if (pos >= s->off + s->bytes) {          // Segment complete.
            if (finish_piece() < 0) return -1;
            k++; done = 0;
            continue;
        }
        if (pos < s->off) {                      // Gap between segments: skip.
            uint32_t skip = (uint32_t)(s->off - pos < n - i ? s->off - pos : n - i);
            i += skip; pos += skip;
            continue;
        }
        if (piece_words == 0) {
            if (finish_piece() < 0) return -1;
            arm_next();
        }
        uint32_t w;
        if (n - i >= 4) memcpy(&w, data + i, 4);
        else { w = 0; memcpy(&w, data + i, n - i); }
        // The staged magic stays "cleared" until the whole image is in: a load cut short (a
        // board reload mid-transfer leaves DRAM as it was) must not pass as a loaded image.
        if (pos == 0) w = BONS_STAGING_CLEARED;
        if ((piece_words & 0xFF) == 0)
            while (trixium_loader_status_read() & 4)    // FIFO full.
                ;
        trixium_loader_poke_write(w);
        piece_words--;
        uint32_t adv = n - i >= 4 ? 4 : n - i;
        done += adv; pos += adv; i += adv;
    }
    return 0;
}

int imgfeed_end(void)
{
    while (k < nseg) {
        if (finish_piece() < 0) return -1;
        if (pos < segs[k].off + segs[k].bytes) { puts("\nimgfeed: file ended inside a segment"); return -1; }
        k++; done = 0;
    }
    // Complete: now stage the header's real first beat (its magic).
    trixium_loader_src_sel_write(1);
    trixium_loader_chan_write(0);
    trixium_loader_base_write(HDR_STAGING);
    trixium_loader_length_write(1);
    trixium_loader_resync_write(0);
    trixium_loader_start_write(1);
    for (int j = 0; j < 8; j++) trixium_loader_poke_write(beat0[j]);
    return loader_wait();
}
