# CZone proprietary PGN capture evidence

This contribution improves the **existing provisional** BEP Marine PGN definitions in `database/pgns/` in place. It does not introduce competing variants or change manufacturer/industry matching (BEP 295, marine 4).

| PGN | Observed device | Capture source | Decoding |
| --- | --- | --- | --- |
| 130817 | SugarShack AC Output Interface (ACOI), ZCF module 0xF8 | source 11 | Page, module ID, eight 3-byte slots: UINT8 current at 0.2 A/count, followed by UINT16 state/level code |
| 130822 | SugarShack DC Output Interface (COI), ZCF module 0x18 | source 5 | Module ID, page, eight 3-byte slots: UINT8 current at 0.1 A/count, followed by UINT16 state/level code |
| 130825 | Compass Rose Control X Plus, module 2 | source 2 | Module ID, page, unidentified header byte, eight packed slots of 11-bit raw current and 11-bit raw level |

The exact captured payloads and machine-checked expectations are embedded in each active YAML definition as `samples`. The sample timestamps are **synthetic test-fixture timestamps**, not the original recording times.

## Interpretation and limits

For PGN 130817, live SugarShack load measurements support **0.2 A per raw count**: a roughly 900 W / 110 V water heater corresponds to approximately 8.2 A (raw 41); a blender/receptacle observation of approximately 1.4 A corresponds to raw 7. These nonzero-load observations are **not** embedded in the zero-current sample, so the sample validates field layout rather than independently establishing calibration. The former provisional explanation described each record as circuit ID plus packed magnitude; our measured current correlation supports the new layout for the observed AC module.

For PGN 130822, observed state/level codes include 1024 (off) and 2024 (on), with intermediate values consistent with dimming. For PGN 130825, the 11-bit fields are intentionally left **raw**; level code 1000 was observed at full output but is not asserted to be a universal percentage conversion. Other 16-bit state codes and the 130825 extra header byte are not exhaustively interpreted.

## Validation

The in-place definitions pass `keel/keel check` (including embedded sample expectations), `make generated`, and `make pr` on the feature branch: [CZone validation workflow](https://github.com/mattsmitchell/canboat/actions/workflows/czone-pgn-validation.yml). Earlier [replacement-based research validation](https://github.com/mattsmitchell/signalk-czone-circuits/actions/runs/41383317492) also passed.

No changes were made to the active Signal K CZone beta.
