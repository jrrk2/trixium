/*
 * This file is part of TriXium.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef TRIXIUM_NFSLOAD_H
#define TRIXIUM_NFSLOAD_H

#include <stdint.h>

/* Load a BONS image from the NFS server's export into the engine channels (0: done).
 * path is relative to the export, '/'-separated. */
int nfsload(const char *path);

/* Read a file from the NFS export into CPU memory (at most max bytes); returns its size, < 0 on
 * error. */
int nfsread(const char *path, void *dst, uint32_t max);

/* Print the MAC's receive error / drop counters. */
void ethstat(void);

/* Cycles per CPU read of the MAC's SRAM and CSRs. */
void ethbench(void);

#endif
