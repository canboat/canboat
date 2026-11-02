# CZone captured-frame provenance and assertions

These YAML files in `research/czone/candidates/` are **staged review candidates**, not active definitions in `database/pgns/`. Their samples use payload bytes from recorded CZone traffic; the ISO timestamp in each `raw:` line is a **synthetic fixture timestamp** added to satisfy CANboat's input format, **not** the capture's original time.

| PGN | Vessel / device | Source address | Payload (hex) | Expected checks |
|---|---|---:|---|---|
| 130817 | SugarShack AC/ACOI, ZCF module 0xF8 | 11 | `279900f800000400000400000400000400000400000400e80700e80f` | page 0, module 248, slot 0 current 0 A |
| 130822 | SugarShack DC/COI, ZCF module 0x18 | 5 | `2799180101e8070000040000040000040000040000049be807000004` | module 24, page 1, slot 0 current 0.1 A and code 2024, slot 6 current 15.5 A |
| 130825 | Compass Rose Control X Plus, module 2 | 2 | `279902000000401f00d017000000000005405f00d0070000000000` | module 2, page 0; 11-bit raw output fields |

## Calibration evidence and limits

For **130817**, live SugarShack observations support 0.2 A per raw count: water heater approximately 900 W at 110 V (8.2 A, raw byte 41); salon receptacle/blender approximately 1.4 A (raw byte 7). These *nonzero-load observations are not embedded in the zero-current fixture above*. They must not be represented as if the fixture independently proves the scaling.

For **130822**, the current byte uses 0.1 A/count and the following little-endian 16-bit value is an output state/level code (observed 1024 OFF, 2024 ON). For **130825**, eight 22-bit slots contain unsigned 11-bit current and level values. A level raw code of 1000 was observed at full output in the application, not proven to be a standardized percentage.

## Validation and compatibility

The candidate layouts and embedded raw sample expectations passed `keel/keel check`, `make generated`, and `make pr` in an isolated checkout of upstream CANboat: [successful research CI run](https://github.com/mattsmitchell/signalk-czone-circuits/actions/runs/41383317492).

**These staged YAML files must not be copied over the existing upstream definitions without further variant-selection testing.** The generic BEP fallbacks for PGNs 130822/130825 must remain usable, and the existing 130817 record interpretation may apply to other CZone devices. See [FIELD-RECONCILIATION.md](../FIELD-RECONCILIATION.md).

Research origin: [CZone comparison draft PR #4](https://github.com/mattsmitchell/signalk-czone-circuits/pull/4). The active Signal K beta was not modified.
