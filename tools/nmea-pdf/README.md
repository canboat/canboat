# NMEA PDF reconciliation (#654)

A repeatable pipeline that reconciles canboat's PGN database against the official
**NMEA 2000 Appendix B.1 PGN Table, Version 1.300 (May 2009)** PDF.

It replaces the earlier hand-extracted `sources/NMEA_database_1_300.xml` with a
**transparent, deterministic** extraction so the comparison can be re-run and
audited by anyone holding the PDF.

## Why the PDF is not in this repo

The NMEA PDF is copyrighted NMEA material and is **not** redistributed here. Only
the *derived* field/type listing (`sources/nmea_1300.json`) is committed —
consistent with how the project has handled member disclosures before. A member
who holds the PDF can regenerate the JSON byte-for-byte.

## Pipeline

```
NMEA PDF ──extract.py──▶ sources/nmea_1300.json ┐
                                                ├─ reconcile.py ─▶ worklist
docs/canboat.json  (canboat's own generated DB) ┘
```

- **`extract.py`** — `pdftotext -layout` → structured JSON. Deterministic: same
  PDF in ⇒ identical JSON out. Extracts the datatype dictionary (DF##), and per
  PGN: frame type, default priority, update interval, and per field: order,
  name, bit length, signedness, coarse type, resolution, lookup enum values, and
  repeating-set markers.
- **`reconcile.py`** — diffs `nmea_1300.json` against `docs/canboat.json` at PGN
  level (frame / priority / interval / field count) and field level (coarse
  type / signedness / bit length; names are informational, see #526). Emits a
  worklist grouped by PGN. **This is a maintainer report, not a CI gate** — the
  reference is a 2009 snapshot, so many divergences are deliberate canboat
  improvements.
- **`allowlist.json`** — documented, expected divergences, suppressed from the
  worklist (generalises the old `validate-csv.py` ignore lists).

## Regenerating

```sh
# 1. Re-extract (needs poppler's `pdftotext` on PATH):
python3 tools/nmea-pdf/extract.py \
    ~/src/github/n2k_research/nmea/NMEA2000_v1-300_App_B1_PGN_Table.pdf \
    -o sources/nmea_1300.json

# 2. Run the reconciliation report:
python3 tools/nmea-pdf/reconcile.py              # structural worklist
python3 tools/nmea-pdf/reconcile.py --naming     # + field-name differences (#526)
python3 tools/nmea-pdf/reconcile.py --backfill   # priority/interval canboat has unset
```

`reconcile.py` reads `docs/canboat.json`, so run `make generated` first if you
have local database changes you want reflected.

## Scope

In scope: datatype dictionary; PGN frame/priority/interval/field-count; field
order/name/type/signedness/bit-length; lookup enum values and repeating-set
detection (extracted, not yet diffed).

Out of scope: resolution-unit comparison, PhysicalQuantity assignment, and
diffing of enum values / repeating-set grouping (the JSON carries them; the
reconciler does not yet compare them).
