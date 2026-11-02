# CZone PGN field-by-field reconciliation

Compared against `canboat/canboat:master` at `4f232faf00efc6cd07d56cb3ef7efddd216e98bb`. Candidate source: `mattsmitchell/signalk-czone-circuits`, `research/compare/stage-upstream-keel.py`, commit `a49885a`.

## Common header (all three PGNs)

| Position | Upstream | Candidate | Decision |
|---|---|---|---|
| bits 0–10 | `manufacturerCode` LOOKUP 11, match 295 | identical | Preserve |
| bits 11–12 | `reserved` RESERVED 2 | identical | Preserve |
| bits 13–15 | `industryCode` LOOKUP 3, match 4 | identical | Preserve |
| transport | Fast | Fast | Preserve |

The two leading payload bytes `27 99` encode this header, **not** a separate CZone discriminator. A manufacturer/industry match alone does not distinguish these CZone messages from other BEP payloads.

## PGN 130817 — CZone Status Extended

| Offset | Upstream | Candidate | Decision |
|---|---|---|---|
| bytes 2 | `page` UINT8 (upstream notes 0x01) | `page` UINT8 (capture 0x00) | Retain, document observed differences |
| byte 3 | `dipswitch` UINT8 | `moduleId` UINT8 | Retain upstream name until device identity reconciled |
| bytes 4–27 | `records` BINARY 192 bits | eight pairs: `outputCurrentN` UINT8 (resolution 0.2) + `outputStateCodeN` UINT16 | **Conflict; do not overwrite existing upstream interpretation yet** |

Upstream explanation cites [dirkwa/czone-spec](https://github.com/dirkwa/czone-spec/blob/main/spec/pgn-130817.md): each 3-byte record has a circuit identifier byte and a packed 10-bit magnitude/sign/flags. Candidate interprets byte 0 as current and the remaining 16 bits as state/level. The upstream schema does **not** actually decode its interpretation; it leaves records opaque. Captured SugarShack AC evidence and live calibration support our candidate, but discriminator/variant compatibility requires maintainer review. Keep existing 130817 definition untouched in the initial contribution, or propose a separately distinguishable variant after additional evidence.

## PGN 130822 — BEP proprietary / CZone DC

| Offset | Upstream | Candidate | Decision |
|---|---|---|---|
| bytes 2–end | `data` BINARY 1768 bits | byte 2 `moduleId` UINT8; byte 3 `page` UINT8; bytes 4–27 eight `outputCurrentN` UINT8 at 0.1 A/count and `outputStateCodeN` UINT16 | **Add CZone-specific variant and retain generic fallback** |

Captured 28-byte payload has 8 × 3-byte output records. CZone variant should be chosen only when a validated distinguishing signature exists. Existing 11-bit manufacturer + 3-bit industry header alone is insufficient. Preserve generic `bepMarineProprietaryPgn130822` with `data` for other payloads.

## PGN 130825 — BEP proprietary / CZone Control X Plus

| Offset | Upstream | Candidate | Decision |
|---|---|---|---|
| bytes 2–end | `data` BINARY 1768 bits | byte 2 `moduleId` UINT8; byte 3 `page` UINT8; byte 4 unidentified BINARY 8; bytes 5–26 eight 22-bit records: `outputCurrentRawN` and `outputLevelRawN`, each UNSIGNED_INTEGER 11 bits | **Add CZone-specific variant and retain generic fallback** |

The fixed-size `UINT16` with `bits: 11` fails Keel; `UNSIGNED_INTEGER` with `bits: 11` passes. Raw level code 1000 has been observed at full output in the application; do not claim it is a universally standardized percentage.

## Compatibility requirements before draft upstream PR

1. Determine whether Keel supports distinguishing CZone variants by payload length or additional field match beyond the shared BEP header. Do **not** assume that `27 99` uniquely identifies CZone.
2. Retain existing 130822/130825 generic fallback definitions; avoid overwriting their IDs, semantics or variant ordering without testing selection.
3. Hold PGN 130817 pending record-semantics reconciliation with existing public evidence and a safe discriminator.
4. Add captured samples with assertions to the actual fork, run `keel/keel check`, `make generated`, and `make pr` there; commit generated artifacts.
5. Validate which definition is selected for both CZone frames and representative non-CZone BEP frames.

The prior green CI run staged **replacement** YAML in a disposable checkout. It proves schema, generation, and PR checks passed in that configuration, **not** coexistence of multiple BEP variants.
