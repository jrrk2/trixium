#
# This file is part of TriXium.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""TRIXIUM.TOK: the card's tokenizer table (firmware/tokenizer.c), built from a Qwen2/Qwen3
byte-level BPE tokenizer.json.

The firmware runs the same pipeline as the host tokenizer - added tokens split out first,
the Qwen2 pre-tokenizer regex (hand-written in C; its Unicode classes come from the range
tables here), then byte-level BPE by merge rank - so the card's ids equal the host's. The NFC
normalizer is not applied on the card (console input is NFC already).

Layout (little endian, all offsets from the file start, every section 16-byte aligned):

    header (128 bytes):
      0  magic 'TTOK'          4  version (1)          8  n_ids (max id + 1)
      12 entries_off           16 blob_off             20 byte_off
      24 hash_off              28 hash_bits            32 added_off     36 n_added
      40 L_off   44 n_L        48 N_off   52 n_N       56 S_off   60 n_S
      64 total size            68 crc32 of [128, size)
      72 tokenizer.json sha256 (32 bytes)
    entries: u32 per id = len << 24 | blob offset (len 0: no such id)
    blob:    the token bytes (byte-level decoded; added tokens: their content)
    byte:    256 x u32, the id of each single byte
    hash:    2^hash_bits slots of {u32 a, u32 b, u32 rank, u32 merged}, a = 0xFFFFFFFF empty;
             slot = ((a*0x9E3779B1) ^ (b*0x85EBCA77)) >> (32 - hash_bits), linear probing
    added:   u32 ids, longest content first (the card matches the longest at each position)
    L/N/S:   sorted inclusive code point ranges {u32 lo, u32 hi}: \\p{L}, \\p{N}, \\s

    python3 -m model.tokbin models/1.7b/tokenizer.json -o build/TRIXIUM.TOK
"""

import sys
import json
import struct
import hashlib
import argparse
import unicodedata
import zlib

MAGIC   = 0x4B4F5454 # "TTOK"
VERSION = 1
HEADER  = 128

# Rust regex's \s: the Unicode White_Space property (unicodedata does not expose it).
WHITE_SPACE = [(0x09, 0x0D), (0x20, 0x20), (0x85, 0x85), (0xA0, 0xA0), (0x1680, 0x1680),
               (0x2000, 0x200A), (0x2028, 0x2029), (0x202F, 0x202F), (0x205F, 0x205F),
               (0x3000, 0x3000)]

# Helpers ------------------------------------------------------------------------------------------

def bytes_to_unicode():
    """GPT-2's byte <-> printable-character map (the ByteLevel pre-tokenizer's)."""
    bs = (list(range(ord("!"), ord("~") + 1)) + list(range(ord("¡"), ord("¬") + 1))
          + list(range(ord("®"), ord("ÿ") + 1)))
    cs = bs[:]
    n = 0
    for b in range(256):
        if b not in bs:
            bs.append(b)
            cs.append(256 + n)
            n += 1
    return dict(zip(bs, (chr(c) for c in cs)))

def ranges(pred):
    out, lo = [], None
    for cp in range(sys.maxunicode + 1):
        if pred(cp):
            if lo is None:
                lo = cp
        elif lo is not None:
            out.append((lo, cp - 1))
            lo = None
    if lo is not None:
        out.append((lo, sys.maxunicode))
    return out

def _cat(cp):
    return unicodedata.category(chr(cp))

def _align(b):
    return b + bytes(-len(b) % 16)

def _hash(a, b, bits):
    return ((((a*0x9E3779B1) & 0xFFFFFFFF) ^ ((b*0x85EBCA77) & 0xFFFFFFFF)) >> (32 - bits))

# Build --------------------------------------------------------------------------------------------

def build(tokenizer_json):
    raw = open(tokenizer_json, "rb").read()
    j   = json.loads(raw)
    m   = j["model"]
    assert m["type"] == "BPE" and not m.get("byte_fallback") and not m.get("ignore_merges")
    pre = json.dumps(j["pre_tokenizer"])
    assert "(?i:'s|'t|'re|'ve|'m|'ll|'d)" in pre and "ByteLevel" in pre, \
        "not the Qwen2 pre-tokenizer the firmware implements"
    dec   = {c: b for b, c in bytes_to_unicode().items()}
    vocab = m["vocab"]
    added = j.get("added_tokens", [])
    n_ids = max(max(vocab.values()), max((a["id"] for a in added), default=0)) + 1

    pieces = [b""]*n_ids
    for s, i in vocab.items():
        pieces[i] = bytes(dec[c] for c in s)
    for a in added:
        pieces[a["id"]] = a["content"].encode()

    byte_ids = []
    for b in range(256):
        byte_ids.append(vocab[bytes_to_unicode()[b]])

    merges = m["merges"]
    bits = 1
    while (1 << bits) < len(merges)*5//3:   # load <= 60%: short probes
        bits += 1
    slots = [None]*(1 << bits)
    for rank, mg in enumerate(merges):
        l, r = mg if isinstance(mg, list) else mg.split(" ", 1)
        a, b, res = vocab[l], vocab[r], vocab[l + r]
        h = _hash(a, b, bits)
        while slots[h] is not None:
            h = (h + 1) & ((1 << bits) - 1)
        slots[h] = (a, b, rank, res)

    blob, entries = bytearray(), []
    for p in pieces:
        assert len(p) < 256
        entries.append((len(p) << 24) | len(blob) if p else 0)
        blob += p
    assert len(blob) < (1 << 24)

    added_ids = [a["id"] for a in sorted(added, key=lambda a: -len(a["content"].encode()))]
    L = ranges(lambda cp: _cat(cp)[0] == "L")
    N = ranges(lambda cp: _cat(cp)[0] == "N")

    secs = [
        _align(struct.pack(f"<{n_ids}I", *entries)),
        _align(bytes(blob)),
        _align(struct.pack("<256I", *byte_ids)),
        _align(b"".join(struct.pack("<4I", *(s or (0xFFFFFFFF, 0, 0, 0))) for s in slots)),
        _align(struct.pack(f"<{len(added_ids)}I", *added_ids)),
        _align(b"".join(struct.pack("<2I", *r) for r in L)),
        _align(b"".join(struct.pack("<2I", *r) for r in N)),
        _align(b"".join(struct.pack("<2I", *r) for r in WHITE_SPACE)),
    ]
    offs, o = [], HEADER
    for s in secs:
        offs.append(o)
        o += len(s)
    body = b"".join(secs)
    total = HEADER + len(body)
    hdr = struct.pack("<18I", MAGIC, VERSION, n_ids, offs[0], offs[1], offs[2], offs[3], bits,
                      offs[4], len(added_ids), offs[5], len(L), offs[6], len(N), offs[7],
                      len(WHITE_SPACE), total, zlib.crc32(body))
    hdr += hashlib.sha256(raw).digest()
    hdr += bytes(HEADER - len(hdr))
    return hdr + body

# Run ----------------------------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description="Build TRIXIUM.TOK from a tokenizer.json.")
    parser.add_argument("tokenizer", help="tokenizer.json (Qwen2/Qwen3 byte-level BPE).")
    parser.add_argument("-o", "--out", default="build/TRIXIUM.TOK", help="Output file.")
    args = parser.parse_args()
    data = build(args.tokenizer)
    with open(args.out, "wb") as f:
        f.write(data)
    print(f"{args.out}: {len(data)} bytes")

if __name__ == "__main__":
    main()
