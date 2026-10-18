/* SPDX-License-Identifier: BSD-2-Clause
 *
 * TriXium on-card tokenizer (Qwen2/Qwen3 byte-level BPE)
 *
 * This file is part of TriXium.
 *
 * Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
 *
 */

// The host tokenizer's pipeline, so the ids match it (test/firmware/test_tokenizer.py):
//  1. added tokens written literally split the text (longest match at each position);
//  2. each remaining span is cut by the Qwen2 pre-tokenizer regex
//       (?i:'s|'t|'re|'ve|'m|'ll|'d)|[^\r\n\p{L}\p{N}]?\p{L}+|\p{N}
//       | ?[^\s\p{L}\p{N}]+[\r\n]*|\s*[\r\n]+|\s+(?!\S)|\s+
//     hand-written below as a leftmost-first scanner over code points (Unicode classes from
//     the table's range lists);
//  3. each piece's bytes become single-byte tokens, and the adjacent pair with the lowest
//     merge rank is merged until none is left (byte-level BPE).
// The NFC normalizer is not applied (console input is NFC already).

#include <stddef.h>
#include <string.h>

#include "tokenizer.h"

/* Table */
/*-------*/

// libbase has no memcmp.
static int same(const uint8_t *a, const uint8_t *b, size_t n)
{
    while (n--)
        if (*a++ != *b++)
            return 0;
    return 1;
}

#define TOK_MAGIC   0x4B4F5454u // "TTOK"
#define TOK_VERSION 1u
#define TOK_HEADER  128u

static const uint8_t  *t_base;
static const uint32_t *t_entries, *t_byte, *t_hash, *t_added, *t_L, *t_N, *t_S;
static const uint8_t  *t_blob;
static uint32_t t_nids, t_hash_bits, t_nadded, t_nL, t_nN, t_nS;

static uint32_t rd32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint32_t crc32(const uint8_t *p, uint32_t n)
{
    static uint32_t tab[256];                   // Byte-at-a-time: 6 MB in well under a second.
    if (tab[1] == 0)
        for (uint32_t i = 0; i < 256; i++) {
            uint32_t c = i;
            for (int k = 0; k < 8; k++)
                c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
            tab[i] = c;
        }
    uint32_t c = 0xFFFFFFFFu;
    while (n--)
        c = tab[(c ^ *p++) & 0xFFu] ^ (c >> 8);
    return ~c;
}

int tok_attach(const uint8_t *table, uint32_t size)
{
    t_base = NULL;
    if (size < TOK_HEADER || rd32(table) != TOK_MAGIC || rd32(table + 4) != TOK_VERSION)
        return -1;
    uint32_t total = rd32(table + 64);
    if (total > size)
        return -2;
    if (crc32(table + TOK_HEADER, total - TOK_HEADER) != rd32(table + 68))
        return -3;
    t_nids      = rd32(table + 8);
    t_entries   = (const uint32_t *)(table + rd32(table + 12));
    t_blob      = table + rd32(table + 16);
    t_byte      = (const uint32_t *)(table + rd32(table + 20));
    t_hash      = (const uint32_t *)(table + rd32(table + 24));
    t_hash_bits = rd32(table + 28);
    t_added     = (const uint32_t *)(table + rd32(table + 32));
    t_nadded    = rd32(table + 36);
    t_L = (const uint32_t *)(table + rd32(table + 40)); t_nL = rd32(table + 44);
    t_N = (const uint32_t *)(table + rd32(table + 48)); t_nN = rd32(table + 52);
    t_S = (const uint32_t *)(table + rd32(table + 56)); t_nS = rd32(table + 60);
    t_base = table;
    return 0;
}

int tok_ready(void)
{
    return t_base != NULL;
}

const uint8_t *tok_sha256(void)
{
    return t_base ? t_base + 72 : NULL;
}

int tok_piece(int id, const uint8_t **bytes)
{
    if (!t_base || id < 0 || (uint32_t)id >= t_nids)
        return 0;
    uint32_t e = t_entries[id];
    *bytes = t_blob + (e & 0xFFFFFFu);
    return (int)(e >> 24);
}

int tok_lookup(const char *s)
{
    if (!t_base)
        return -1;
    size_t n = strlen(s);
    for (uint32_t i = 0; i < t_nadded; i++) {     // The added tokens (the specials).
        const uint8_t *p;
        int len = tok_piece((int)t_added[i], &p);
        if ((size_t)len == n && same(p, (const uint8_t *)s, n))
            return (int)t_added[i];
    }
    for (uint32_t id = 0; id < t_nids; id++) {    // Anything else: a linear scan.
        const uint8_t *p;
        int len = tok_piece((int)id, &p);
        if ((size_t)len == n && same(p, (const uint8_t *)s, n))
            return (int)id;
    }
    return -1;
}

/* Unicode Classes */
/*-----------------*/

static int in_ranges(const uint32_t *r, uint32_t n, uint32_t cp)
{
    uint32_t lo = 0, hi = n;
    while (lo < hi) {
        uint32_t mid = (lo + hi)/2;
        if (cp < r[2*mid])
            hi = mid;
        else if (cp > r[2*mid + 1])
            lo = mid + 1;
        else
            return 1;
    }
    return 0;
}

enum { C_L = 1, C_N = 2, C_S = 4 };

static int cls(uint32_t cp)
{
    if (cp < 0x80) {                    // ASCII without the tables.
        if ((cp | 0x20) >= 'a' && (cp | 0x20) <= 'z') return C_L;
        if (cp >= '0' && cp <= '9')                   return C_N;
        if (cp == ' ' || (cp >= 0x09 && cp <= 0x0D))  return C_S;
        return 0;
    }
    if (in_ranges(t_L, t_nL, cp)) return C_L;
    if (in_ranges(t_N, t_nN, cp)) return C_N;
    if (in_ranges(t_S, t_nS, cp)) return C_S;
    return 0;
}

/* Pre-tokenizer */
/*---------------*/

// The scanner's and BPE's arrays (~92 KB) live in the caller's workspace (tok_workspace):
// one decoded code point per slot - value, class, and where its bytes start in the span.
#define TOK_SPAN_MAX 4096
struct tok_work {
    uint32_t cp_v[TOK_SPAN_MAX + 1];
    uint16_t cp_off[TOK_SPAN_MAX + 1];
    uint8_t  cp_cls[TOK_SPAN_MAX + 1];
    int      sym[TOK_SPAN_MAX*4];
};
static struct tok_work *w;
#define cp_v   (w->cp_v)
#define cp_off (w->cp_off)
#define cp_cls (w->cp_cls)

uint32_t tok_workspace_size(void)
{
    return sizeof(struct tok_work);
}

void tok_workspace(void *work)
{
    w = (struct tok_work *)work;
}

static int utf8_decode(const uint8_t *s, int n, int i, uint32_t *cp)
{
    uint8_t c = s[i];
    int len = c < 0x80 ? 1 : (c >> 5) == 6 ? 2 : (c >> 4) == 14 ? 3 : (c >> 3) == 30 ? 4 : 0;
    if (len == 0 || i + len > n) { *cp = 0xFFFD; return 1; }  // Invalid: one byte, class 0.
    uint32_t v = len == 1 ? c : c & (0x7F >> len);
    for (int k = 1; k < len; k++) {
        if ((s[i + k] & 0xC0) != 0x80) { *cp = 0xFFFD; return 1; }
        v = (v << 6) | (s[i + k] & 0x3F);
    }
    *cp = v;
    return len;
}

static int is_nl(uint32_t cp) { return cp == '\r' || cp == '\n'; }
static uint32_t lower(uint32_t cp) { return (cp >= 'A' && cp <= 'Z') ? cp + 32 : cp; }

// The end (code point index) of the regex match starting at i, over cp[0..n).
static int match_at(int i, int n)
{
    const uint32_t *c = cp_v;
    const uint8_t  *k = cp_cls;
    // (?i:'s|'t|'re|'ve|'m|'ll|'d)
    if (c[i] == '\'' && i + 1 < n) {
        uint32_t a = lower(c[i + 1]), b = i + 2 < n ? lower(c[i + 2]) : 0;
        if (a == 's' || a == 't') return i + 2;
        if ((a == 'r' && b == 'e') || (a == 'v' && b == 'e')) return i + 3;
        if (a == 'm') return i + 2;
        if (a == 'l' && b == 'l') return i + 3;
        if (a == 'd') return i + 2;
    }
    // [^\r\n\p{L}\p{N}]?\p{L}+
    {
        int j = i;
        if (!is_nl(c[i]) && !(k[i] & (C_L | C_N)) && i + 1 < n && (k[i + 1] & C_L))
            j = i + 1;
        if (k[j] & C_L) {
            while (j < n && (k[j] & C_L)) j++;
            return j;
        }
    }
    // \p{N}
    if (k[i] & C_N)
        return i + 1;
    // ` ?[^\s\p{L}\p{N}]+[\r\n]*`
    {
        int j = (c[i] == ' ' && i + 1 < n && k[i + 1] == 0) ? i + 1 : i;
        if (k[j] == 0) {
            while (j < n && k[j] == 0) j++;
            while (j < n && is_nl(c[j])) j++;
            return j;
        }
    }
    // Whitespace from here: the run [i, e).
    int e = i;
    while (e < n && (k[e] & C_S)) e++;
    // \s*[\r\n]+ : through the run's last newline.
    for (int j = e - 1; j >= i; j--)
        if (is_nl(c[j])) return j + 1;
    // \s+(?!\S) : the whole run at the end of the text, else all but its last character.
    if (e == n || e - 1 > i)
        return e == n ? e : e - 1;
    // \s+
    return e;
}

/* BPE */
/*-----*/

static int merge_lookup(uint32_t a, uint32_t b, uint32_t *rank)
{
    uint32_t mask = (1u << t_hash_bits) - 1u;
    uint32_t h = (((a*0x9E3779B1u) ^ (b*0x85EBCA77u)) >> (32 - t_hash_bits));
    for (;;) {
        const uint32_t *s = t_hash + 4*h;
        if (s[0] == 0xFFFFFFFFu)
            return -1;
        if (s[0] == a && s[1] == b) {
            *rank = s[2];
            return (int)s[3];
        }
        h = (h + 1) & mask;
    }
}

static int bpe(const uint8_t *p, int len, int *out, int max)
{
    int *sym = w->sym;
    const int sym_max = TOK_SPAN_MAX*4;
    int n = 0;
    for (int i = 0; i < len && n < sym_max; i++)
        sym[n++] = (int)t_byte[p[i]];
    for (;;) {
        uint32_t best = 0xFFFFFFFFu, r;
        int at = -1, res = -1;
        for (int i = 0; i + 1 < n; i++) {
            int m = merge_lookup((uint32_t)sym[i], (uint32_t)sym[i + 1], &r);
            if (m >= 0 && r < best) { best = r; at = i; res = m; }
        }
        if (at < 0)
            break;
        sym[at] = res;
        memmove(&sym[at + 1], &sym[at + 2], (size_t)(n - at - 2)*sizeof(int));
        n--;
    }
    int k = 0;
    for (int i = 0; i < n && k < max; i++)
        out[k++] = sym[i];
    return k;
}

// One span with no added tokens in it: pre-tokenize, then BPE each piece.
static int encode_span(const uint8_t *s, int n, int *out, int max)
{
    int cnt = 0;
    while (n > 0 && cnt < max) {
        // Decode up to TOK_SPAN_MAX code points; a longer span goes in slices, and a slice's
        // last match is carried into the next one (it may continue past the boundary).
        int ncp = 0, b = 0;
        while (b < n && ncp < TOK_SPAN_MAX) {
            uint32_t cp;
            int l = utf8_decode(s, n, b, &cp);
            cp_v[ncp] = cp; cp_off[ncp] = (uint16_t)b;
            cp_cls[ncp] = (cp == 0xFFFD && l == 1 && s[b] >= 0x80) ? 0 : (uint8_t)cls(cp);
            ncp++; b += l;
        }
        cp_off[ncp] = (uint16_t)b;
        cp_cls[ncp] = 0;
        int i = 0;
        while (i < ncp && cnt < max) {
            int e = match_at(i, ncp);
            if (e == ncp && b < n && i > 0)    // Reaches the slice end with text after it.
                break;
            cnt += bpe(s + cp_off[i], cp_off[e] - cp_off[i], out + cnt, max - cnt);
            i = e;
        }
        s += cp_off[i]; n -= cp_off[i];
    }
    return cnt;
}

int tok_encode(const char *text, int n, int *out, int max)
{
    if (!t_base || !w)
        return -1;
    const uint8_t *s = (const uint8_t *)text;
    int cnt = 0, start = 0;
    for (int i = 0; i < n && cnt < max; ) {
        int hit = -1, hit_len = 0;
        for (uint32_t a = 0; a < t_nadded; a++) {
            const uint8_t *p;
            int len = tok_piece((int)t_added[a], &p);
            if (len > hit_len && len <= n - i && p[0] == s[i] && same(s + i, p, (size_t)len)) {
                hit = (int)t_added[a];
                hit_len = len;
            }
        }
        if (hit < 0) { i++; continue; }
        cnt += encode_span(s + start, i - start, out + cnt, max - cnt);
        if (cnt < max)
            out[cnt++] = hit;
        i += hit_len;
        start = i;
    }
    if (cnt < max && start < n)
        cnt += encode_span(s + start, n - start, out + cnt, max - cnt);
    return cnt;
}
