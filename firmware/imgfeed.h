/*
 * This file is part of TriXium.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef TRIXIUM_IMGFEED_H
#define TRIXIUM_IMGFEED_H

#include <stdint.h>

/* A BONS image fed to the loader's CSR poke path in file order (SD card, NFS): the
 * header+table to channel 0's staging area, each channel's payload to its window. */

/* Start an image from its first bytes (at least the header): 0, or -1 with a message. */
int imgfeed_begin(const uint8_t *head, uint32_t n, uint64_t file_len);
/* The next n file bytes (n a multiple of 4 except for the last piece). */
int imgfeed_push(const uint8_t *data, uint32_t n);
/* Pad and finish the last transfer: 0 when every segment is complete. */
int imgfeed_end(void);
/* Bytes of the file the image needs (header+table + payloads). */
uint64_t imgfeed_total(void);

#endif
