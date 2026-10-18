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

#include "imgfeed.h"

#ifndef SD_CLK_HZ
#define SD_CLK_HZ 50000000         // SDR25 (liblitesdcard's init leaves the bus at 25 MHz).
#endif
#define SD_DMA_MAX_SECTORS 256u    // 128 KB per command, inside the PHY's 1 s data timeout.
#define SD_BUF       (MAIN_RAM_BASE + 0x04000000u) // main_ram + 64 MB: between the low heap
#define SD_BUF_SIZE  0x01000000u                   // (< 64 MB) and the high heap (>= 144 MB).

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
        flush_cpu_dcache_range(fatbuf, sizeof(fatbuf)); // Not flush_cpu_dcache(): all of main_ram.
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
    flush_cpu_dcache_range(dst, got);      // The DMA'd run only (a whole flush walks main_ram).
    return got;
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
    if (clst < 2 || len == 0) { printf("sdload: %s is empty\n", name); return -1; }

    fat_cached_sec = 0xFFFFFFFFu;
    struct sd_stream s = { &fs, clst, 0, len };
    uint8_t *buf = (uint8_t *)SD_BUF;
    int64_t n = sd_stream_read(&s, buf);
    if (n <= 0) return -1;
    printf("sdload: %s\n", name);
    if (imgfeed_begin(buf, (uint32_t)n, len) < 0) return -1;

    uint64_t total = imgfeed_total(), pos = 0;
    init_progression_bar(total);
    while (1) {
        if (imgfeed_push(buf, (uint32_t)n) < 0) return -1;
        pos += n;
        show_progress(pos < total ? pos : total);
        if (pos >= total || s.left == 0) break;
        n = sd_stream_read(&s, buf);
        if (n <= 0) return -1;
    }
    if (imgfeed_end() < 0) return -1;
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
