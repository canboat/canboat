# Baseline findings

What `keel check` (the DESIGN.md §5 rule engine) reports against the
database as converted from today's `pgn.h` — i.e. **defects and oddities
inherited from the C source**, surfaced for review, not introduced by the
migration. Current baseline: **0 errors, 5 warnings** (F1's two duplicate
variants + F3's three dead lookups).

## F1 — R20: unreachable PGN variants (2, softened to warning)

Runtime matching (`pgn.c getMatchingPgn`) returns the *first* variant whose
match fields all match; a later variant with an identical match set can
never be selected:

- PGN 126720: `maretronProprietaryConfiguration` duplicates the match set
  `[Manufacturer=137, Industry=4]` of `maretronSlaveResponse`.
- PGN 130820: `bepMarineProprietaryPgn130820` duplicates
  `[Manufacturer=295, Industry=4]` of `bepMarineCzoneAlarmStringResponse`.

Both later variants look like intended catch-alls ("proprietary, not
further decoded") that in fact are dead.

**Decision (Kees, 2026-07-07): to be fixed in canboat main before this
branch ever merges.** R20 returns to error once main's fix flows back
into this branch's database.

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

**Decision (Kees, 2026-07-07): some width disagreements are expected; the
field must state so explicitly.** Implemented as the per-field key
`allowLookupWidthMismatch: true`:

- a width disagreement without the key is an **error**;
- a named value/bit that cannot fit the field is an **error** even with
  the key, except out-of-width *bit positions* in an opted-in shared bit
  enumeration (the DISABLED_SATELLITES idiom);
- the key on a field whose widths agree is a **warning** (stale opt-in).

The bootstrap converter stamped the key onto the 40 inherited fields, so
the baseline carries the intent explicitly and any *new* mismatch fails.

## F4 — J1939: the XML document had never been emittable (fixed)

`analyzer-explain-j1939 -explain-xml` had always aborted mid-document (no
Makefile target ever exercised it, so it went unnoticed): 46 of 57 PGNs
emitted, then a fatal length error. Fixed in `pgn-j1939.h` during the
J1939 conversion, all proven by the test suite:

- **65226 DM1 "Active Trouble Codes"**: the DTC quad (SPN/FMI/CM/OC) now
  repeats until data exhausted (SAE J1939-73) — previously 6 fixed bytes,
  which both aborted the emitter and decoded only one DTC.
- **Five partial single-frame definitions** (65262 Engine Temp #1, 65266
  Fuel Economy, 65269 Ambient Conditions, 65270 Inlet/Exhaust Conditions,
  65271 Vehicle Electrical Power) padded with trailing reserved fields to
  the 8-byte frame.
- **61142 → 61442 "Electronic Transmission Controller 1"**: transposition
  typo caught by rule R02 (61142 = 0xEED6 is not a valid PGN number; SAE
  ETC1 is 61442 = 0xF002). Real ETC1 frames had been hitting the fallback.

## F3 — R22: unreferenced lookup enumerations (3 warnings)

`AIRMAR_FILTER`, `FUSION_SIRIUS_ALERT`, `SEATALK1_ATT` are defined in
lookup.h but referenced by no field. Verified 2026-07-07: they appear
**nowhere else** — not in `pgn.h`, not in `pgn-j1939.h`, not in any C
source. Genuinely dead enumerations; deleting them removes their
`<LookupEnumeration>` from canboat.xml (contract: minor). Awaiting a
delete/keep decision.

## F5 — Garmin editor test data: remove before merge

The Garmin PGN 126720 files added while validating the `keel edit` editor
(`database/pgns/126720-garminAutopilotKeyValue.yaml`,
`database/lookups/GARMIN_AP_KEY.yaml`, `database/lookups/GARMIN_SUB_PROTOCOL.yaml`,
and the `subProtocolId` edit on `126720-garminAutopilotHeadingToSteer.yaml`;
committed `0d071cd`) are **kept for now but must not merge on this branch**.
They are PGN-data work, not part of the keel tool refactor, and belong on
`master` or a separate data PR. Drop them (and re-run `make generated`) before
finalizing `refactor_pgn_database`. Outstanding on the data itself:
`GARMIN_AP_KEY` value 10 has a placeholder name and key 17 is undefined.
