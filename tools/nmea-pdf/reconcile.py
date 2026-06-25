#!/usr/bin/env python3
#
# (C) 2009-2025, Kees Verruijt, Harlingen, The Netherlands.
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
Reconcile canboat's PGN database against the official NMEA 2000 v1.300 PDF (#654).

Diffs the extractor output (sources/nmea_1300.json, produced by extract.py) against
canboat's own generated database (docs/canboat.json) and emits a reviewable
worklist grouped by PGN.

This is a *maintainer report*, not a CI gate: the NMEA reference is a 2009 (v1.300)
snapshot, so many divergences are canboat improvements, not bugs. Known/expected
divergences are documented in allowlist.json and suppressed from the worklist.

Comparison dimensions
  PGN level:    frame type, default priority, update interval, field count
  Field level:  coarse type, signedness, bit length   (high signal)
                field name                              (informational; see #526)

Out of scope (v1): resolution units, lookup enum *values*, repeating-set grouping.

Usage
  python3 tools/nmea-pdf/reconcile.py                 # uses default paths
  python3 tools/nmea-pdf/reconcile.py --backfill      # also list priority/interval
                                                       #   that canboat has unset
"""

import argparse
import json
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.abspath(os.path.join(HERE, "..", ".."))

# Map canboat's rich FieldType vocabulary onto the coarse buckets the extractor
# produces, so the two sides are comparable.
CANBOAT_COARSE = {
    "NUMBER": "NUMBER", "DURATION": "NUMBER", "TIME": "NUMBER", "DATE": "NUMBER",
    "PGN": "NUMBER", "MMSI": "NUMBER", "FIELD_INDEX": "NUMBER", "FLOAT": "NUMBER",
    "DECIMAL": "NUMBER", "DYNAMIC_FIELD_LENGTH": "NUMBER",
    "LOOKUP": "LOOKUP", "BITLOOKUP": "LOOKUP", "INDIRECT_LOOKUP": "LOOKUP",
    "DYNAMIC_FIELD_KEY": "LOOKUP",
    "RESERVED": "RESERVED", "SPARE": "RESERVED",
    "BINARY": "BINARY", "VARIABLE": "BINARY", "DYNAMIC_FIELD_VALUE": "BINARY",
    "ISO_NAME": "BINARY",
    "STRING_LAU": "STRING", "STRING_FIX": "STRING", "STRING_LZ": "STRING",
}


def coarse_canboat(ft):
    return CANBOAT_COARSE.get(ft, ft)


def norm_name(s):
    """Loose name key: lowercase alphanumerics only (for informational matching)."""
    return re.sub(r"[^a-z0-9]", "", (s or "").lower())


def load(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def load_allowlist(path):
    if not os.path.exists(path):
        return {"pgn": {}, "field": {}}
    data = load(path)
    return {"pgn": data.get("pgn", {}), "field": data.get("field", {})}


def allowed(allow, pgn, kind, key):
    """True if (pgn, kind, key) is documented as an expected divergence."""
    entry = allow.get("pgn" if kind == "pgn" else "field", {}).get(str(pgn))
    if not entry:
        return False
    return key in entry or "*" in entry


def cb_index(canboat):
    idx = {}
    for p in canboat["PGNs"]:
        idx.setdefault(p["PGN"], []).append(p)
    return idx


def compare_pgn(npgn, cands, allow):
    """Compare one NMEA PGN against its canboat candidate(s). Returns issue dict."""
    issues = {"pgn": npgn["pgn"], "desc": npgn["description"],
              "level": [], "fields": [], "naming": []}

    # canboat may carry several rows per PRN (proprietary/match variants). For
    # structural reconciliation, pick the row whose field count is closest.
    cb = min(cands, key=lambda p: abs(p.get("FieldCount", 0) - (npgn["fieldCount"] or 0)))

    # ---- PGN level -------------------------------------------------------- #
    if npgn["frameType"] and cb.get("Type") not in (None, npgn["frameType"]):
        # canboat Type may be ISO-TP / Mixed; only flag the clear Single<->Fast clash
        if {npgn["frameType"], cb["Type"]} == {"Single", "Fast"} and not allowed(allow, npgn["pgn"], "pgn", "frame"):
            issues["level"].append(f"frame: NMEA={npgn['frameType']} canboat={cb['Type']}")

    cb_prio = cb.get("Priority")
    if npgn["priority"] is not None and cb_prio is not None and cb_prio != npgn["priority"]:
        if not allowed(allow, npgn["pgn"], "pgn", "priority"):
            issues["level"].append(f"priority: NMEA={npgn['priority']} canboat={cb_prio}")

    cb_int = cb.get("TransmissionInterval")
    if npgn["updateRate"] is not None and cb_int is not None and cb_int != npgn["updateRate"]:
        if not allowed(allow, npgn["pgn"], "pgn", "interval"):
            issues["level"].append(
                f"interval: NMEA={npgn['updateRate']:g}ms canboat={cb_int}ms")

    # ---- Field level ------------------------------------------------------ #
    cb_by_order = {f["Order"]: f for f in cb.get("Fields", [])}
    for nf in npgn["fields"]:
        cf = cb_by_order.get(nf["order"])
        if cf is None:
            continue  # field-count differences are reported at PGN level below
        if allowed(allow, npgn["pgn"], "field", str(nf["order"])):
            continue
        problems = []
        nt, ct = nf["type"], coarse_canboat(cf.get("FieldType"))
        # BITFIELD/UNKNOWN are the PDF being non-committal -> wildcard, not a clash.
        if nt not in ("UNKNOWN", "BITFIELD") and ct != "UNKNOWN" and nt != ct:
            problems.append(f"type NMEA={nt} canboat={ct}")
        if nf["signed"] is not None and cf.get("Signed") is not None \
                and nf["signed"] != cf.get("Signed"):
            problems.append(f"signed NMEA={nf['signed']} canboat={cf.get('Signed')}")
        if nf["bits"] is not None and cf.get("BitLength") is not None \
                and nf["bits"] != cf.get("BitLength") and not nf["reserved"]:
            problems.append(f"bits NMEA={nf['bits']} canboat={cf.get('BitLength')}")
        if problems:
            issues["fields"].append(
                {"order": nf["order"], "name": nf["name"], "problems": problems})
        elif norm_name(nf["name"]) != norm_name(cf.get("Name")):
            issues["naming"].append(
                {"order": nf["order"], "nmea": nf["name"], "canboat": cf.get("Name")})

    # Field-count differs constantly because the two databases model fields
    # differently, so only the genuinely suspicious shape is flagged: canboat
    # has FEWER fields than the PDF AND no repeating set explains the gap.
    # canboat having MORE (refinement) or fewer-with-a-repeating-set (the PDF
    # flattens every repeat) is expected and stays silent.
    n_cnt, c_cnt = npgn["fieldCount"], cb.get("FieldCount")
    if c_cnt is not None and n_cnt is not None and c_cnt < n_cnt \
            and not cb.get("RepeatingFieldSet1Size") \
            and not allowed(allow, npgn["pgn"], "pgn", "fieldcount"):
        issues["level"].append(
            f"fieldCount: canboat={c_cnt} < NMEA={n_cnt} with no repeating set "
            f"(possible missing field)")

    return issues


def backfill_candidates(nmea, cbidx):
    """PGNs where canboat has priority/interval UNSET but the PDF supplies one."""
    rows = []
    for np_ in nmea["pgns"]:
        cands = cbidx.get(np_["pgn"])
        if not cands:
            continue
        for cb in cands:
            fills = []
            if cb.get("Priority") is None and np_["priority"] is not None:
                fills.append(("priority", np_["priority"]))
            if cb.get("TransmissionInterval") is None \
                    and cb.get("TransmissionIrregular") is None \
                    and np_["updateRate"] is not None:
                fills.append(("interval", f"{np_['updateRate']:g}"))
            if fills:
                rows.append((np_["pgn"], cb.get("Id", ""), fills))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nmea", default=os.path.join(ROOT, "sources", "nmea_1300.json"))
    ap.add_argument("--canboat", default=os.path.join(ROOT, "docs", "canboat.json"))
    ap.add_argument("--allowlist", default=os.path.join(HERE, "allowlist.json"))
    ap.add_argument("--backfill", action="store_true",
                    help="list priority/interval that canboat has unset")
    ap.add_argument("--naming", action="store_true",
                    help="also show informational field-name differences (#526)")
    args = ap.parse_args()

    nmea = load(args.nmea)
    canboat = load(args.canboat)
    allow = load_allowlist(args.allowlist)
    cbidx = cb_index(canboat)

    missing, results = [], []
    for np_ in nmea["pgns"]:
        cands = cbidx.get(np_["pgn"])
        if not cands:
            missing.append((np_["pgn"], np_["description"]))
            continue
        results.append(compare_pgn(np_, cands, allow))

    n_level = sum(len(r["level"]) for r in results)
    n_field = sum(len(r["fields"]) for r in results)
    n_name = sum(len(r["naming"]) for r in results)

    print(f"# NMEA v1.300 reconciliation  ({len(nmea['pgns'])} PDF PGNs)")
    print(f"#   matched in canboat : {len(results)}")
    print(f"#   absent  in canboat : {len(missing)}")
    print(f"#   PGN-level issues   : {n_level}")
    print(f"#   field-level issues : {n_field}  (structural: type/signed/bits)")
    print(f"#   naming differences : {n_name}  (informational, #526)")
    print()

    for r in sorted(results, key=lambda x: x["pgn"]):
        if not r["level"] and not r["fields"] and not (args.naming and r["naming"]):
            continue
        print(f"PGN {r['pgn']}  {r['desc']}")
        for lv in r["level"]:
            print(f"    [PGN] {lv}")
        for fp in r["fields"]:
            print(f"    [#{fp['order']} {fp['name']}] " + "; ".join(fp["problems"]))
        if args.naming:
            for nm in r["naming"]:
                print(f"    [#{nm['order']} name] NMEA={nm['nmea']!r} canboat={nm['canboat']!r}")
        print()

    if missing:
        print("# PGNs in the PDF but not in canboat:")
        for pgn, desc in sorted(missing):
            print(f"#   {pgn}  {desc}")
        print()

    if args.backfill:
        rows = backfill_candidates(nmea, cbidx)
        print(f"# Backfill candidates (canboat unset, PDF supplies): {len(rows)}")
        for pgn, cid, fills in sorted(rows):
            for what, val in fills:
                print(f"#   PGN {pgn:<7} {cid:<28} {what} = {val}")


if __name__ == "__main__":
    main()
