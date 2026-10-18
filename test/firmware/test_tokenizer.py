#
# This file is part of TriXium.
#
# Copyright (c) 2026 Enjoy-Digital <enjoy-digital.fr>
# SPDX-License-Identifier: BSD-2-Clause

"""firmware/tokenizer.c (the card's tokenizer) against the reference tokenizer: the same ids for
the same text, and the token bytes back to the text."""

import shutil
import subprocess
import ctypes as ct
from pathlib import Path

import pytest

from common import ROOT

TOKENIZER = ROOT / "models/1.7b/tokenizer.json"

CORPUS = [
    "hello", "Hello, world!", "The capital of France is Paris.",
    "I'm sure they're right, but we'd've known. DON'T SHOUT; it's 'quoted'.",
    "x'S y'T z'RE w'VE v'M u'LL t'D", "'s 're",
    "1234567890 3.14159 -42 1e10 0x1F 2026-10-17 12:34:56",
    "a  b   c    d\t\te\n\nf\r\ng \n h  \n\n  i", "   leading", "trailing   ", " ", "\n", "",
    "def f(x):\n    return x**2  # square\n\n\nclass A:\n\tpass\n",
    "for (int i = 0; i < n; i++) { a[i] += b[i]; }",
    "!!! ??? ... --- *** ### $$$ %%% &&& @@@ ~~~",
    "<|im_start|>user\nWrite a haiku about FPGAs.<|im_end|>\n<|im_start|>assistant\n"
    "<think>\n\n</think>\n\n",
    "text<|im_end|>more<|endoftext|><think>x</think>",
    "<|im_start <|im_end| <think <tool_call>",
    "café naïve résumé façade Ångström Zürich", "Привет, как дела?", "Γειά σου Κόσμε",
    "日本語のテキストと漢字。", "中文：你好，世界！", "한국어 텍스트", "مرحبا بالعالم", "שלום עולם",
    "हिन्दी पाठ", "ภาษาไทย", "emoji 😀🎉👍🏽 and 🇫🇷 flags", "zero\u200bwidth\u00a0nbsp\u3000ideo",
    "Ⅻ ½ ² ٣ ൭ ⅷ", "mixed123abc ABC123 a1b2c3", "tab\tsep\tvalues\t",
    "e-mail: someone@example.com, https://example.com/path?q=1&r=2#frag",
    "Line one.\r\nLine two.\r\n\r\nLine four.", "\"Quoted\" and (parenthesised) [bracketed] {braced}",
]

def _docs():
    """Every paragraph of the repo's own documentation: a large, ordinary corpus."""
    out = []
    for p in sorted((ROOT / "doc").rglob("*.md")) + [ROOT / "README.md"]:
        out += [s for s in p.read_text(encoding="utf-8").split("\n\n") if s.strip()]
    return out


@pytest.fixture(scope="module")
def card(tmp_path_factory):
    cc = shutil.which("gcc")
    if not cc:
        pytest.skip("C compiler required")
    if not TOKENIZER.exists():
        pytest.skip(f"{TOKENIZER} not present")
    pytest.importorskip("tokenizers")
    from model.tokbin import build
    lib = tmp_path_factory.mktemp("tokenizer-host") / "tokenizer.so"
    r = subprocess.run([cc, "-std=c11", "-O2", "-Wall", "-Wextra", "-Werror", "-shared", "-fPIC",
                        str(ROOT / "firmware/tokenizer.c"), "-o", str(lib)],
                       capture_output=True, text=True)
    assert r.returncode == 0, r.stderr
    so = ct.CDLL(str(lib))
    so.tok_attach.argtypes = [ct.c_char_p, ct.c_uint32]
    so.tok_encode.argtypes = [ct.c_char_p, ct.c_int, ct.POINTER(ct.c_int), ct.c_int]
    so.tok_piece.argtypes  = [ct.c_int, ct.POINTER(ct.POINTER(ct.c_uint8))]
    so.tok_lookup.argtypes = [ct.c_char_p]
    so.tok_workspace.argtypes = [ct.c_void_p]
    table = build(str(TOKENIZER))
    buf = ct.create_string_buffer(table, len(table))
    assert so.tok_attach(buf, len(table)) == 0
    work = (ct.c_uint32*((so.tok_workspace_size() + 3)//4))()
    so.tok_workspace(work)
    so._table, so._work = buf, work # Keep both alive with the library.
    return so

@pytest.fixture(scope="module")
def ref():
    from tokenizers import Tokenizer
    return Tokenizer.from_file(str(TOKENIZER))

def _encode(so, text):
    b = text.encode()
    out = (ct.c_int*(4*len(b) + 16))()
    n = so.tok_encode(b, len(b), out, len(out))
    assert n >= 0
    return list(out[:n])

def _decode(so, ids):
    p = ct.POINTER(ct.c_uint8)()
    return b"".join(ct.string_at(p, so.tok_piece(i, ct.byref(p))) for i in ids)

def test_tokenizer_matches_reference(card, ref):
    bad = []
    for text in CORPUS:
        want = ref.encode(text, add_special_tokens=False).ids
        got  = _encode(card, text)
        if got != want:
            bad.append((text, want, got))
    assert not bad, "\n".join(f"{t!r}\n  ref  {w}\n  card {g}" for t, w, g in bad[:5])

def test_tokenizer_matches_reference_on_docs(card, ref):
    docs = _docs()
    assert len(docs) > 100
    bad = [t for t in docs if _encode(card, t) != ref.encode(t, add_special_tokens=False).ids]
    assert not bad, f"{len(bad)}/{len(docs)} paragraphs differ, first: {bad[0][:200]!r}"

def test_tokenizer_bytes_round_trip(card):
    for text in CORPUS + _docs()[:200]:
        assert _decode(card, _encode(card, text)) == text.encode(), text

def test_tokenizer_specials(card, ref):
    for s in ("<|im_start|>", "<|im_end|>", "<|endoftext|>", "<think>", "</think>"):
        assert card.tok_lookup(s.encode()) == ref.token_to_id(s), s
    assert card.tok_lookup(b"no such token here") == -1

def test_tokenizer_rejects_bad_table(card):
    table = bytearray(card._table.raw)
    table[1000] ^= 1                              # One flipped bit: the CRC refuses it.
    so = ct.CDLL(card._name)
    so.tok_attach.argtypes = [ct.c_char_p, ct.c_uint32]
    assert so.tok_attach(bytes(table), len(table)) == -3
    assert card.tok_attach(card._table, len(card._table.raw)) == 0

def test_tokenizer_long_spans(card, ref):
    # Past the scanner's 4096-code-point slice: the match at a slice end continues into the
    # next one, however long (one letter run, one whitespace run, multi-byte characters).
    texts = ["\n\n".join(_docs()[:60]), "a"*9000, " "*5000 + "x", "x" + "\n"*5000 + "y",
             "é"*4095 + " end", "日本"*3000, ("word " * 2000).strip()]
    for text in texts:
        assert _encode(card, text) == ref.encode(text, add_special_tokens=False).ids, text[:60]
