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
One-shot, fill-only backfill of `.priority` / `.interval` into analyzer/pgn.h
from the NMEA v1.300 PDF extraction (#654, decision: "fill pgn.h once if not set
yet").

For each Pgn entry whose PRN is in the reviewed target set AND that does not
already set the member, append the designated initializer (pulled from
sources/nmea_1300.json) before the entry's closing brace. Existing values are
NEVER overwritten. Designated initializers are order-independent, so appending
is always valid C; run `make format` afterwards to normalise layout.

    python3 tools/nmea-pdf/apply_backfill.py --dry-run   # preview
    python3 tools/nmea-pdf/apply_backfill.py             # apply
"""

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))
PGN_H = os.path.join(ROOT, "analyzer", "pgn.h")
NMEA = os.path.join(ROOT, "sources", "nmea_1300.json")

# PRNs reviewed and approved for backfill (see reconcile.py --backfill).
PRIORITY_PRNS = {59392, 60160, 60416, 65240, 126208, 126464}
INTERVAL_PRNS = {130054}

PRN_LINE = re.compile(r"^\s*(0x[0-9A-Fa-f]+|\d+)\s*,\s*$", re.M)


def iter_entries(text):
    """Yield (start, end, body) for each top-level entry in `Pgn pgnList[] = {`."""
    m = re.search(r"\bPgn\s+pgnList\b[^=]*=\s*\{", text)
    if not m:
        sys.exit("error: could not find `Pgn pgnList[] = {`")
    i = m.end()
    depth = 1  # we are inside the array
    n = len(text)
    while i < n and depth > 0:
        c = text[i]
        if c == "{":
            # start of an entry at array level
            start = i
            d = 1
            i += 1
            while i < n and d > 0:
                if text[i] == "{":
                    d += 1
                elif text[i] == "}":
                    d -= 1
                i += 1
            yield start, i, text[start:i]
        elif c == "}":
            depth -= 1
            i += 1
        else:
            i += 1


def entry_prn(body):
    """First standalone `0x..,` / `123,` line in an entry is its PRN."""
    m = PRN_LINE.search(body)
    return int(m.group(1), 0) if m else None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    nmea = json.load(open(NMEA, encoding="utf-8"))
    prio = {p["pgn"]: p["priority"] for p in nmea["pgns"] if p["priority"] is not None}
    intv = {p["pgn"]: p["updateRate"] for p in nmea["pgns"] if p["updateRate"] is not None}

    text = open(PGN_H, encoding="utf-8").read()

    # A member counts as "not set yet" when absent OR explicitly = 0 (canboat's
    # unknown sentinel for both .priority and .interval). Real, non-zero values
    # (incl. UINT16_MAX = on-request) are never overwritten.
    members = [("priority", PRIORITY_PRNS, prio, lambda v: str(int(v))),
               ("interval", INTERVAL_PRNS, intv, lambda v: f"{v:g}")]

    edits = []  # (start, end, replacement, note) -- start==end means pure insert
    for start, end, body in iter_entries(text):
        prn = entry_prn(body)
        if prn is None:
            continue
        for name, prns, valmap, fmt in members:
            if prn not in prns or prn not in valmap:
                continue
            val = fmt(valmap[prn])
            m = re.search(rf"\.{name}\s*=\s*([^,}}]+)", body)
            if m and m.group(1).strip() == "0":
                edits.append((start + m.start(1), start + m.end(1), val,
                              f"PRN {prn}: .{name} 0 -> {val}"))
            elif not m:
                # Append as a new designated initializer aligned to the entry's
                # existing `.member =` column (clang-format's vertical alignment).
                a = re.search(r"^(\s*)\.\w+\s*=", body, re.M)
                ind = a.group(1) if a else "     "
                eqcol = a.group(0).index("=") if a else len(ind) + len(name) + 2
                pad = max(1, eqcol - len(ind) - len(name) - 1)
                line = f"{ind}.{name}{' ' * pad}= {val}"
                j = end - 1  # step back over any whitespace before the closing brace
                while text[j - 1] in " \t\n":
                    j -= 1
                prefix = "" if text[j - 1] == "," else ","
                edits.append((j, j, f"{prefix}\n{line}",
                              f"PRN {prn}: append .{name} = {val}"))

    print(f"{len(edits)} backfills:")
    for _, _, _, note in sorted(edits):
        print(f"  {note}")

    if args.dry_run:
        return
    # Apply back-to-front so positions stay valid.
    for s, e, repl, _ in sorted(edits, reverse=True):
        text = text[:s] + repl + text[e:]
    open(PGN_H, "w", encoding="utf-8").write(text)
    print(f"\napplied to {PGN_H}; now run `make -C analyzer ... format` and `make generated`.")


if __name__ == "__main__":
    main()
