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
  worklist grouped by PGN. `make validation` runs it with `--check`, which
  **gates** on field-level findings (`type`/`signed`/`bits`); PGN-level
  differences stay report-only, since the reference is a 2009 snapshot and many
  divergences are deliberate canboat improvements.
- **`allowlist.json`** — documented, expected divergences, suppressed from the
  worklist (this is the curated reconciliation result; supersedes the old
  `validate-csv.py` ignore lists). Anything not listed here makes `--check` fail.

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
python3 tools/nmea-pdf/reconcile.py --enums      # PDF lookup values missing in canboat
python3 tools/nmea-pdf/reconcile.py --repeating  # repeating-set disagreements
```

`reconcile.py` reads `docs/canboat.json`, so run `make generated` first if you
have local database changes you want reflected.

## Scope

In scope: datatype dictionary; PGN frame/priority/interval/field-count; field
order/name/type/signedness/bit-length; lookup enum **values** (`--enums`); and
repeating-set presence/size (`--repeating`). The extractor lifts all three of
the PDF's repeating-set notations into one `{start,size}` record: the trailing
"Fields X thru Y repeat" pseudo-field, per-field `repeated` markers, and (best
effort) the intro-prose "Fields X through Y … repeat" sentence.

Out of scope / known limits: resolution-unit comparison and PhysicalQuantity
assignment; a few PGNs describe their repeat only in prose phrasings the
extractor does not match (`--repeating` then shows `PDF=0` where canboat has a
set — canboat is correct in those). Enum and repeating reports are informational,
not gated.
