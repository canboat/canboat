# CZone proprietary PGN capture evidence

This contribution improves the **existing provisional** BEP Marine PGN definitions in `database/pgns/` in place. It does not introduce competing variants or change manufacturer/industry matching (BEP 295, marine 4).

| PGN | Observed device | Capture source | Decoding |
| --- | --- | --- | --- |
| 130817 | SugarShack AC Output Interface (ACOI), ZCF module 0xF8 | source 11 | Page, module ID, eight 3-byte slots: UINT8 current at 0.2 A/count, followed by UINT16 state/level code |
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

The ACOI module byte matches the numeric portion of the displayed serial identifier, **not** the configured DIP-switch value; the COI module byte **does** match the configured DIP-switch value. `moduleId` is therefore retained as a neutral field identifier rather than reusing the older provisional `dipswitch` ID, which could be confused with CZone's specific configuration term. This is an intentional field-ID change; existing consumers using `dipswitch` may need to adapt. The observations do not establish a universal encoding rule for all CZone devices.

## Interpretation and limits

For PGN 130817, live SugarShack load measurements support **0.2 A per raw count**: a roughly 900 W / 110 V water heater corresponds to approximately 8.2 A (raw 41); a blender/receptacle observation of approximately 1.4 A corresponds to raw 7. These nonzero-load observations are **not** embedded in the zero-current sample, so the sample validates field layout rather than independently establishing calibration. The former provisional explanation described each record as circuit ID plus packed magnitude; our measured current correlation supports the new layout for the observed AC module.

For PGN 130822, observed state/level codes include 1024 (off) and 2024 (on), with intermediate values consistent with dimming. For PGN 130825, the 11-bit fields are intentionally left **raw**; level code 1000 was observed at full output but is not asserted to be a universal percentage conversion. Other 16-bit state codes and the 130825 extra header byte are not exhaustively interpreted.

## Validation

The in-place definitions pass `keel/keel check` (including embedded sample expectations), `make generated`, and `make pr` on the feature branch: [CZone validation workflow](https://github.com/mattsmitchell/canboat/actions/workflows/czone-pgn-validation.yml). Earlier [replacement-based research validation](https://github.com/mattsmitchell/signalk-czone-circuits/actions/runs/41383317492) also passed.

No changes were made to the active Signal K CZone beta.
