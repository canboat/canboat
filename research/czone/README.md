# CZone proprietary PGN capture evidence

This contribution improves the **existing provisional** BEP Marine PGN definitions in `database/pgns/` in place. It does not introduce competing variants or change manufacturer/industry matching (BEP 295, marine 4).

| PGN | Observed device | Capture source | Decoding |
| --- | --- | --- | --- |
| 130817 | SugarShack AC Output Interface (ACOI), ZCF module 0xF8 | source 11 | Page, module ID, eight 3-byte slots: UINT8 current at 0.2 A/count, followed by 10-bit percentage level (0.1% units), a six-bit unknown flag group (including bit positions 10 and 11) |
| 130822 | SugarShack DC Output Interface (COI), ZCF module 0x18 | source 5 | Module ID, page, eight 3-byte slots: UINT8 current at 0.1 A/count, followed by UINT16 state/level code |
| 130825 | Compass Rose Control X Plus, module 2 | source 2 | Module ID, page, unidentified header byte, eight packed slots of 11-bit raw current and 11-bit raw level |

The exact captured payloads and machine-checked expectations are embedded in each active YAML definition as `samples`. The sample timestamps are **synthetic test-fixture timestamps**, not the original recording times.

## Module identifier versus CZone DIP-switch

CZone's configuration software uses **DIP-switch** for a specific configured device setting. The captured module identifier should not automatically be assumed to encode that setting on every device type. SugarShack configuration-tool observations provide a cross-check:

| PGN / device | Captured module byte | CZone configured DIP-switch | Other observed identifier | NMEA source/address |
| --- | --- | --- | --- | --- |
| 130817 / ACOI 01 Stbd Aft | `0xF8` (248) | `00011111` (31) | Serial identifier `ACOI 248` | 11 |
| 130822 / COI 01 Port Engine Room | `0x18` (24) | `00011000` (24) | Serial `J315522-0008` | 5 |
| 130825 / Compass Rose Control X Plus | `0x02` (2) | Not available in SugarShack documentation | Not verified | 2 |

The ACOI module byte matches the numeric portion of the displayed serial identifier, **not** the configured DIP-switch value; the COI module byte **does** match the configured DIP-switch value. For PGN 130817 the existing field ID `dipswitch` is **retained for downstream compatibility**, while its display name `ZCF Module ID` and description clarify the actual observation. The module-byte evidence does not justify renaming a frozen field ID. The observations do not establish a universal encoding rule for all CZone devices.

## Interpretation and limits

For PGN 130817, live SugarShack load measurements support **0.2 A per raw count**: a roughly 900 W / 110 V water heater corresponds to approximately 8.2 A (raw 41); a blender/receptacle observation of approximately 1.4 A corresponds to raw 7. These nonzero-load observations are **not** embedded in the zero-current sample, so the sample validates field layout rather than independently establishing calibration. The former provisional explanation described each record as circuit ID plus packed magnitude; our measured current correlation supports the new layout for the observed AC module.

For PGNs 130817 and 130822, the 16-bit state/level word is split into a 10-bit level in tenths of a percent, six unknown upper bits (bits 10–15) in one field. The recorded words 1024 (`0x0400`) and 2024 (`0x07e8`) decode to 0% and 100%; 4072 (`0x0fe8`) decodes to 100% with bit 11 also set. In additional SugarShack recordings, bit 10 remained set, and bit 11 was observed on AC slot 7; these observations do not establish either flag's semantic meaning. For PGN 130825, the 11-bit fields are intentionally left **raw**; level code 1000 was observed at full output but is not asserted to be a universal percentage conversion. Other 16-bit state codes and the 130825 extra header byte are not exhaustively interpreted.

The unknown upper bits are deliberately grouped into one six-bit field per output: splitting them into three fields per slot would produce 45 fields per PGN, exceeding CANboat's current 33-field C analyzer limit (keel rule R11). The grouped field preserves the raw flag bits while keeping each PGN at 29 fields. The individual bit positions are documented here pending a compatible representation.\n\n## Additional dimming capture evidence

In `SugarShack-20260925-03.CAN`, five deliberately set lighting levels matched the low ten bits of PGN 130822 state/level words. These are observed raw-word/level pairs, not invented values: 

| Circuit (operator annotation) | Set level | Observed word | Low 10 bits | Raw word, little-endian |
| --- | ---: | ---: | ---: | --- |
| Bimini | 80% | 1824 (`0x0720`) | 800 | `20 07` |
| Courtesy White | 100% | 2024 (`0x07e8`) | 1000 | `e8 07` |
| Galley Lights | 90% | 1924 (`0x0784`) | 900 | `84 07` |
| Salon Lights | 75% | 1774 (`0x06ee`) | 750 | `ee 06` |
| Starboard Shower Lights | 60% | 1624 (`0x0658`) | 600 | `58 06` |

The operator reported that all five circuits started and ended OFF. The names above are associated with the observed distinct levels using that annotation; precise physical slot/terminal mapping requires separate ZCF/wiring cross-check. The existing small capture-derived YAML samples remain the reproducible decode fixtures; full `.CAN` recordings are not added to this PR.

## Validation

The earlier revision passed `keel/keel check`, `make generated`, and `make pr`. The updated split-field definitions require regeneration and revalidation before they are merged into the PR branch. Prior validation workflow: [CZone validation workflow](https://github.com/mattsmitchell/canboat/actions/workflows/czone-pgn-validation.yml). Earlier [replacement-based research validation](https://github.com/mattsmitchell/signalk-czone-circuits/actions/runs/41383317492) also passed.

No changes were made to the active Signal K CZone beta.
