/* SPDX-License-Identifier: BSD-2-Clause
 *
 * TriXium on-card tokenizer (Qwen2/Qwen3 byte-level BPE)
 *
 * This file is part of TriXium.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 *
 */

#ifndef TRIXIUM_TOKENIZER_H
#define TRIXIUM_TOKENIZER_H

#include <stdint.h>

// Portable C: no LiteX includes, so the host test compiles this file as is
// (test/firmware/test_tokenizer.py) and checks it against the reference tokenizer.

// Attach the TRIXIUM.TOK table (model/tokbin.py) at `table`. 0 = ready; < 0 = bad magic,
// version, size or CRC (the card then has no tokenizer).
int  tok_attach(const uint8_t *table, uint32_t size);
int  tok_ready(void);
const uint8_t *tok_sha256(void);   // tokenizer.json's sha256 (32 bytes), NULL when not ready.

// The encoder's work arrays (tok_workspace_size() bytes, 4-byte aligned), set before encoding.
uint32_t tok_workspace_size(void);
void tok_workspace(void *work);

// Encode n bytes of UTF-8 text into at most max ids; returns the count (< 0: not ready).
// Added tokens (<|im_start|>, <think>, ...) written literally are matched as themselves.
int  tok_encode(const char *s, int n, int *out, int max);

// The bytes of token `id` (a token may hold part of a UTF-8 character); returns the length.
int  tok_piece(int id, const uint8_t **bytes);

// The id of the token whose bytes are exactly `s`, or -1 (e.g. "<|im_end|>").
int  tok_lookup(const char *s);

#endif
