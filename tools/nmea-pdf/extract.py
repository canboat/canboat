#!/usr/bin/env python3
#
# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# This file is part of CANboat.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
"""
Repeatable extractor for the official NMEA 2000 v1.300 Appendix B.1 PGN Table PDF.

Converts the PDF into a structured JSON intermediate (sources/nmea_1300.json) that
is "close to, but not identical to" canboat's own database shape, so it can be
reconciled field-by-field and PGN-by-PGN against docs/canboat.json (see
reconcile.py and issue #654).

The source PDF is NOT redistributed in this repo (it is copyrighted NMEA
material). A member holding the PDF regenerates the JSON with:

    python3 tools/nmea-pdf/extract.py /path/to/NMEA2000_v1-300_App_B1_PGN_Table.pdf \
        -o sources/nmea_1300.json

Input may be the PDF itself (requires `pdftotext -layout` from poppler on PATH)
or a pre-extracted `pdftotext -layout` text file.

The extraction is deterministic: same PDF in -> identical JSON out.
"""

import argparse
import json
import re
import subprocess
import sys


# --------------------------------------------------------------------------- #
# Stage 0: get layout text                                                     #
# --------------------------------------------------------------------------- #
def load_text(path):
    """Return `pdftotext -layout` text for `path` (.pdf -> run poppler; else read)."""
    if path.lower().endswith(".pdf"):
        try:
            out = subprocess.run(
                ["pdftotext", "-layout", path, "-"],
                check=True,
                capture_output=True,
            )
        except FileNotFoundError:
            sys.exit("error: `pdftotext` not found; install poppler or pass a .txt")
        except subprocess.CalledProcessError as e:
            sys.exit(f"error: pdftotext failed: {e.stderr.decode(errors='replace')}")
        return out.stdout.decode("utf-8", errors="replace")
    with open(path, encoding="utf-8", errors="replace") as f:
        return f.read()


# Page furniture we strip before structural parsing.
_FURNITURE = re.compile(r"^(Appendix B\.1|Version 1\.300|\s*$)")


def clean_lines(text):
    """Drop running headers/footers; keep content lines (with original spacing)."""
    return [ln for ln in text.splitlines() if not _FURNITURE.match(ln)]


# --------------------------------------------------------------------------- #
# Stage 1: datatype dictionary (DF##)                                          #
# --------------------------------------------------------------------------- #
_DF_LINE = re.compile(
    r"\bDF(\d{2,3})\b\s+(.*?)\s+Range:\s*(.*?)\s{2,}Resolution:\s*(.*?)(?:\s{2,}|$)"
)
# Encoding token sits as the last whitespace-delimited token of the "name" group.
_ENCODING = re.compile(
    r"^(uint8|int8|uint16|int16|uint32|int32|uint64|int64|bit\(n\)|"
    r"char8\(n\)|ch8or16\(n\)|Undefined)$"
)


def _sign_bits(encoding):
    """(signed, bits) for an encoding token; bits=None when variable/unknown."""
    m = re.match(r"(u?int)(\d+)$", encoding or "")
    if m:
        return (m.group(1) == "int", int(m.group(2)))
    return (None, None)


def parse_datatypes(lines):
    """Return {df_id: {name, encoding, signed, bits, range, resolution}}."""
    types = {}
    for ln in lines:
        m = _DF_LINE.search(ln)
        if not m:
            continue
        df = "DF" + m.group(1).zfill(2) if len(m.group(1)) == 2 else "DF" + m.group(1)
        if df in types:
            continue
        body = re.sub(r"\s+", " ", m.group(2)).strip()
        toks = body.split(" ")
        encoding = toks[-1] if toks and _ENCODING.match(toks[-1]) else None
        name = " ".join(toks[:-1]) if encoding else body
        signed, bits = _sign_bits(encoding)
        types[df] = {
            "name": name,
            "encoding": encoding,
            "signed": signed,
            "bits": bits,
            "range": re.sub(r"\s+", " ", m.group(3)).strip(),
            "resolution": re.sub(r"\s+", " ", m.group(4)).strip(),
        }
    return types


# --------------------------------------------------------------------------- #
# Stage 2: PGN blocks                                                          #
# --------------------------------------------------------------------------- #
# A PGN starts on a "<Description>   PGN: NNNNN" line that is followed by "hex:".
_PGN_HEAD = re.compile(r"^(?P<desc>.+?)\s{2,}PGN:\s*(?P<pgn>\d+)\s*$")
_HEX = re.compile(r"^\s*hex:\s*([0-9A-Fa-f]+)\s*$")

_HEADER = re.compile(
    r"Single Frame:\s*(?P<single>Yes|No).*?"
    r"Priority Default:\s*(?P<prio>\d+|NA).*?"
    r"Default Update Rate:\s*(?P<rate>[\d.,]+|NA)\s*milliseconds",
    re.S,
)

# Field opener: "  N   <name...>  Byte Field Size: X  Bit Field Size: Y  Request Parameter Z"
_FIELD = re.compile(
    r"^\s{1,8}(?P<order>\d{1,2})\s+(?P<name>.+?)\s{2,}"
    r"Byte Field Size:\s*(?P<byte>\d+|\?|)\s{2,}"
    r"Bit Field Size:\s*(?P<bit>resv\s*\d+|\d+|)\s{2,}"
    r"Request Parameter",
    re.S,
)
_DD = re.compile(r"^\s+DD(\d{3})\s+(?P<name>.+?)(?:\s{2,}|$)")
_DF_REF = re.compile(r"^\s+DF(\d{2,3})\b")
_ENUM = re.compile(r"0x([0-9A-Fa-f]+)\s*=\s*(.+?)\s*;?\s*$")


def split_pgns(lines):
    """Yield (header_index, pgn_number, description) for each PGN block start."""
    for i in range(len(lines) - 1):
        m = _PGN_HEAD.match(lines[i])
        if m and _HEX.match(lines[i + 1]):
            yield i, int(m.group("pgn")), m.group("desc").strip()


def parse_bits(byte_s, bit_s):
    """Return (bits, reserved) from the Byte/Bit Field Size capture groups."""
    bit_s = (bit_s or "").strip()
    if bit_s.startswith("resv"):
        n = re.search(r"\d+", bit_s)
        return (int(n.group()) if n else None, True)
    if bit_s.isdigit():
        return (int(bit_s), False)
    byte_s = (byte_s or "").strip()
    if byte_s.isdigit():
        return (int(byte_s) * 8, False)
    return (None, False)  # "?" or blank -> variable/unknown


def classify(dd_id, dd_name, df, dtype, reserved, enums):
    """Map a field to a canboat-ish coarse type.

    The PDF models most fields as a generic "Bit field" (DF52, bit(n)) with no
    concrete datatype. Such fields carry no reliable type signal, so they become
    the vague bucket "BITFIELD" -- the reconciler treats those as wildcards
    rather than flagging canboat's refined typing as a discrepancy. A real
    enumeration needs >=2 listed values; a lone "0xNN =" in prose (e.g.
    "255 = no data") is description bleed, not a lookup table.
    """
    if reserved or dd_id == "001":
        return "RESERVED"
    enc = (dtype or {}).get("encoding")
    if enc in ("char8(n)", "ch8or16(n)"):
        return "STRING"
    if enums and len(enums) >= 2:
        return "LOOKUP"
    if enc in ("uint8", "int8", "uint16", "int16", "uint32", "int32", "uint64", "int64"):
        return "NUMBER"
    if enc == "bit(n)":
        return "BITFIELD"  # PDF gave no concrete type
    return "UNKNOWN"


def parse_pgn_block(block, datatypes):
    """Parse one PGN's lines (header line excluded) into a dict."""
    hdr = _HEADER.search("\n".join(block))
    frame_type = priority = update_rate = None
    if hdr:
        frame_type = "Single" if hdr.group("single") == "Yes" else "Fast"
        priority = None if hdr.group("prio") == "NA" else int(hdr.group("prio"))
        rate = hdr.group("rate")
        update_rate = None if rate == "NA" else float(rate.replace(",", ""))

    fields = []
    i = 0
    while i < len(block):
        fm = _FIELD.match(block[i])
        if not fm:
            i += 1
            continue
        bits, reserved = parse_bits(fm.group("byte"), fm.group("bit"))
        # Look ahead within this field for the `repeated` marker, DD and DF refs.
        repeated = False
        dd_id = dd_name = None
        df_id = None
        enums = []
        j = i + 1
        while j < len(block) and not _FIELD.match(block[j]):
            ln = block[j]
            if ln.strip() == "repeated":
                repeated = True
            ddm = _DD.match(ln)
            if ddm and dd_id is None:
                dd_id, dd_name = ddm.group(1), ddm.group("name").strip()
            dfm = _DF_REF.match(ln)
            if dfm and df_id is None:
                raw = dfm.group(1)
                df_id = "DF" + raw.zfill(2) if len(raw) == 2 else "DF" + raw
            em = _ENUM.search(ln)
            if em:
                enums.append({"value": int(em.group(1), 16), "name": em.group(2).strip()})
            j += 1

        dtype = datatypes.get(df_id)
        name = re.sub(r"\s+", " ", fm.group("name")).strip()
        repeat_of = None
        rm = re.search(r"Field number (\d+)", name)
        if repeated and rm:
            repeat_of = int(rm.group(1))
        signed = dtype.get("signed") if dtype else None
        resolution = dtype.get("resolution") if dtype else None
        fields.append(
            {
                "order": int(fm.group("order")),
                "name": name,
                "bits": bits,
                "signed": signed,
                "type": classify(dd_id, dd_name, df_id, dtype, reserved, enums),
                "resolution": resolution,
                "dataType": df_id,
                "dd": dd_id,
                "reserved": reserved,
                "repeated": repeated,
                "repeatOf": repeat_of,
                "enums": enums or None,
            }
        )
        i = j

    # Third notation: the repeat is only described in the PGN's intro prose,
    # e.g. "Fields 3 through 12 may repeat ...". Best-effort; report-only.
    prose_repeat = None
    pm = re.search(r"Fields?\s+(\d+)\s+(?:thru|through|to)\s+(\d+).{0,120}?repeat",
                   "\n".join(block), re.I | re.S)
    if pm and int(pm.group(2)) >= int(pm.group(1)):
        a, b = int(pm.group(1)), int(pm.group(2))
        prose_repeat = {"start": a, "size": b - a + 1}

    return frame_type, priority, update_rate, fields, prose_repeat


# The PDF marks repeating sets two ways: a trailing pseudo-field
# "Fields X thru Y repeat as needed", or per-field "repeated" tokens that
# reference an earlier "Field number N". Both are lifted to one {start,size}
# record so the field list holds a single copy of the repeat unit, matching
# canboat's RepeatingFieldSet model.
_REPEAT_MARKER = re.compile(r"^Fields?\s+(\d+)\s+(?:thru|through|to)\s+(\d+)\s+repeat", re.I)


def lift_repeat(fields):
    """Return (base_fields, repeat) where repeat is {start,size} or None."""
    repeat = None
    kept = []
    for f in fields:
        m = _REPEAT_MARKER.match(f["name"])
        if m:  # style 2: trailing pseudo-field
            a, b = int(m.group(1)), int(m.group(2))
            repeat = {"start": a, "size": b - a + 1}
            continue
        kept.append(f)
    if repeat is None:  # style 1: per-field "repeated" markers
        rep = [f for f in kept if f.get("repeated")]
        if rep:
            starts = [f["repeatOf"] for f in rep if f["repeatOf"]]
            repeat = {"start": min(starts) if starts else rep[0]["order"], "size": len(rep)}
            kept = [f for f in kept if not f.get("repeated")]
    return kept, repeat


def parse(text):
    lines = clean_lines(text)
    datatypes = parse_datatypes(lines)

    starts = list(split_pgns(lines))
    pgns = []
    for idx, (start, pgn, desc) in enumerate(starts):
        end = starts[idx + 1][0] if idx + 1 < len(starts) else len(lines)
        block = lines[start + 2 : end]  # skip the "PGN:" + "hex:" lines
        frame_type, priority, update_rate, fields, prose_repeat = parse_pgn_block(block, datatypes)
        # A PGN may be split across several page-blocks with the same number;
        # merge consecutive blocks of the same PGN by appending their fields.
        if pgns and pgns[-1]["pgn"] == pgn:
            existing = {f["order"] for f in pgns[-1]["fields"]}
            pgns[-1]["fields"].extend(f for f in fields if f["order"] not in existing)
            if pgns[-1]["_proseRepeat"] is None:
                pgns[-1]["_proseRepeat"] = prose_repeat
            continue
        pgns.append(
            {
                "pgn": pgn,
                "description": desc,
                "frameType": frame_type,
                "priority": priority,
                "updateRate": update_rate,
                "fieldCount": None,  # filled after merge
                "fields": fields,
                "_proseRepeat": prose_repeat,
            }
        )
    for p in pgns:
        p["fields"], rep = lift_repeat(p["fields"])
        # Structured marker wins; otherwise fall back to the intro-prose repeat.
        p["repeat"] = rep or p.pop("_proseRepeat")
        p.pop("_proseRepeat", None)
        # Total fields incl. reserved, matching canboat's FieldCount convention.
        p["fieldCount"] = len(p["fields"])
    return {"source": "NMEA 2000 Appendix B.1 PGN Table, Version 1.300 (May 2009)",
            "dataTypes": datatypes, "pgns": pgns}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("input", help="PDF file (needs pdftotext) or pre-extracted .txt")
    ap.add_argument("-o", "--output", help="output JSON (default: stdout)")
    args = ap.parse_args()

    data = parse(load_text(args.input))
    out = json.dumps(data, indent=2, ensure_ascii=False)
    if args.output:
        with open(args.output, "w", encoding="utf-8") as f:
            f.write(out + "\n")
        p = data["pgns"]
        print(
            f"wrote {args.output}: {len(p)} PGNs, "
            f"{sum(len(x['fields']) for x in p)} fields, "
            f"{len(data['dataTypes'])} datatypes",
            file=sys.stderr,
        )
    else:
        print(out)


if __name__ == "__main__":
    main()
