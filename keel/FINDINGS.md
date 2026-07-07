# Baseline findings

What `keel check` (the DESIGN.md §5 rule engine) reports against the
database as converted from today's `pgn.h` — i.e. **defects and oddities
inherited from the C source**, surfaced for review, not introduced by the
migration. Current baseline: **0 errors, 42 warnings**.

Two rules were deliberately softened so the inherited baseline passes;
each should be re-hardened once its findings are resolved.

## F1 — R20: unreachable PGN variants (2, softened to warning)

Runtime matching (`pgn.c getMatchingPgn`) returns the *first* variant whose
match fields all match; a later variant with an identical match set can
never be selected:

- PGN 126720: `maretronProprietaryConfiguration` duplicates the match set
  `[Manufacturer=137, Industry=4]` of `maretronSlaveResponse`.
- PGN 130820: `bepMarineProprietaryPgn130820` duplicates
  `[Manufacturer=295, Industry=4]` of `bepMarineCzoneAlarmStringResponse`.

Both later variants look like intended catch-alls ("proprietary, not
further decoded") that in fact are dead. Options: delete them, or give the
specific variants additional match fields, or accept an explicit
"catch-all after specifics" ordering rule. Needs a decision; then R20
returns to error.

## F2 — R08: lookup width disagreements (37 warnings)

The C never validated a field's bit width against its lookup's declared
`BITS(n)`; pgn.h uses both idioms freely:

- **Narrower field, values still fit** — idiomatic: `YES_NO` (declared 2
  bits) used in 1-bit flags across AIS/alert PGNs; `DIRECTION` (4) in a
  3-bit field; `TEMPERATURE_SOURCE` (8) in 6 bits; `HUMIDITY_SOURCE` (8)
  in 2 bits; `ENTERTAINMENT_PLAY_STATUS` (16) in 8; `ALERT_STATE` (8) in 3.
- **Wider field than lookup**: `YES_NO` in several 8-bit Fusion/SeaTalk
  fields (values 2..255 undefined but representable).
- **Shared bit enumerations**: `DISABLED_SATELLITES` declares 40 bit
  positions, used over 32-bit (GPS/GLONASS) and 24-bit (QZSS) fields —
  intentional sharing; out-of-width bits simply never occur.

Policy now implemented: a named value/bit that cannot fit the field is an
**error** (dead table entry) except for shared *bit* enumerations
(warning); a mere width disagreement is a **warning**. Open question for
cleanup: should `LOOKUP_TYPE` widths be corrected in lookup.h (changes
`MaxValue` in canboat.xml → contract diff), or should the width move to
being purely per-field?

## F3 — R22: unreferenced lookup enumerations (3 warnings)

`AIRMAR_FILTER`, `FUSION_SIRIUS_ALERT`, `SEATALK1_ATT` are defined in
lookup.h but referenced by no field. Caveat: the J1939 definitions
(`pgn-j1939.h`) are not yet converted — one of these may be referenced
there. Re-check after the J1939 tree lands; then delete truly dead ones.
