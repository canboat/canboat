# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Sections can be: Added Changed Deprecated Removed Fixed Security.

## [7.2.0](https://github.com/canboat/canboat/compare/v7.1.0...v7.2.0) (2026-07-11)


### Added

* **126720:** restore shadowed variants and dedupe Maretron/BEP PGNs ([#744](https://github.com/canboat/canboat/issues/744)) ([7796fba](https://github.com/canboat/canboat/commit/7796fba0e861fc80f6cbe795c94d5b01326892ee))
* **130824:** decode B&G key-value roll/pitch/yaw rates ([#757](https://github.com/canboat/canboat/issues/757)) ([8f737e9](https://github.com/canboat/canboat/commit/8f737e93ba2a1dd8684d4bc267126c55173c3ee5))
* **130827:** decode Furuno NavPilot status ([#742](https://github.com/canboat/canboat/issues/742)) ([58a0f26](https://github.com/canboat/canboat/commit/58a0f26e4d53d5c58d0819ba0a030e035d042539))
* **analyzer:** normalize emitted timestamps to canonical ISO-8601 UTC ([#750](https://github.com/canboat/canboat/issues/750)) ([8f33ead](https://github.com/canboat/canboat/commit/8f33eadc17835df070b67cb8951118973bc98e9b)), closes [#749](https://github.com/canboat/canboat/issues/749)
* decode Garmin autopilot scalar transport over PGN 126720 ([#688](https://github.com/canboat/canboat/issues/688)) ([#692](https://github.com/canboat/canboat/issues/692)) ([3f6ce56](https://github.com/canboat/canboat/commit/3f6ce561f933c3e14769c4c6aa605c341d9f361b))
* **furuno:** correct PGN 130845 satellite record layout and decode Baseline status ([#723](https://github.com/canboat/canboat/issues/723)) ([e72b6d5](https://github.com/canboat/canboat/commit/e72b6d5c4790ff1b77f67913d299d76547be1fb7))
* **garmin:** Reactor autopilot mode state, voltage fix, and field lookups ([#753](https://github.com/canboat/canboat/issues/753)) ([a349344](https://github.com/canboat/canboat/commit/a349344fdb10402686bc43d01b99eecd396be96b))
* **navico:** add 130824 B&G start-line & MOB position keys ([#708](https://github.com/canboat/canboat/issues/708)) ([2938fd8](https://github.com/canboat/canboat/commit/2938fd80be71d3c2117c35f823007ed069c67d08))
* **navico:** add compass auto-calibration mode key; fix Local field units (130845) ([#720](https://github.com/canboat/canboat/issues/720)) ([e59571c](https://github.com/canboat/canboat/commit/e59571c5a47d3ce3eead7f2c7473913f9782a675)), closes [#718](https://github.com/canboat/canboat/issues/718)
* **navico:** correct PGN 65350 layout to normalized magnetic field vector X/Y/Z ([#719](https://github.com/canboat/canboat/issues/719)) ([4426234](https://github.com/canboat/canboat/commit/44262343b2bec327b8ad26c3eacbdaa70e149486)), closes [#716](https://github.com/canboat/canboat/issues/716)
* **navico:** decode five proprietary Navico/Simrad/Lowrance PGNs ([#743](https://github.com/canboat/canboat/issues/743)) ([6f47c34](https://github.com/canboat/canboat/commit/6f47c343b67db04c53b34b1126a3a1d04eca0768))
* **navico:** decode PGN 130822 Command 3 "Bulk Report 3" Section 10 settings ([#733](https://github.com/canboat/canboat/issues/733)) ([ac21513](https://github.com/canboat/canboat/commit/ac21513352688a90d253a3f185347b2ca086ef24))
* **navico:** decode PGN 130823 directory records, 130817 feature-unlock, and harden group-function/key-value handling ([#741](https://github.com/canboat/canboat/issues/741)) ([c41bdd5](https://github.com/canboat/canboat/commit/c41bdd5c90c454072b695014a5c9ace90ded226e))
* **navico:** decode PGN 130845 display unit-preference keys ([#729](https://github.com/canboat/canboat/issues/729)) ([dab64cd](https://github.com/canboat/canboat/commit/dab64cd0b409aa3e4c2e11ce92a3e91b00e043f1))
* **navico:** decode PGN 130845 instrument damping settings ([#730](https://github.com/canboat/canboat/issues/730)) ([#731](https://github.com/canboat/canboat/issues/731)) ([244abdf](https://github.com/canboat/canboat/commit/244abdfdcbe9aa7b8c432448f6dfd47742c4d552))
* **navico:** decode proprietary PGNs 65303/65304, 130823, 130852 ([#728](https://github.com/canboat/canboat/issues/728)) ([5003dff](https://github.com/canboat/canboat/commit/5003dff6c0d2358c14d0ae8d7c9f24a6bbfe90df))
* **navico:** decode SIMNET_ALARM_COMMAND 88 as Tack/Gybe Confirm ([#739](https://github.com/canboat/canboat/issues/739)) ([98b5240](https://github.com/canboat/canboat/commit/98b52409c879779c47c2826424dec13310591397))
* **navico:** drop PGN 130861 "Simrad: Engine Data" — no producer exists ([#721](https://github.com/canboat/canboat/issues/721)) ([28d39e7](https://github.com/canboat/canboat/commit/28d39e740e46c7aab40738e1cff85664e7046c55)), closes [#712](https://github.com/canboat/canboat/issues/712)
* **navico:** resolve PGN 65323 & 130840 Simnet Data Source Selection ([#725](https://github.com/canboat/canboat/issues/725)) ([20fee4f](https://github.com/canboat/canboat/commit/20fee4f49fc16305e1450a10808a865e11a062dc))
* **navico:** unify SIMNET_DISPLAY_GROUP into SIMNET_NETWORK_GROUP ([#732](https://github.com/canboat/canboat/issues/732)) ([1e6a7e6](https://github.com/canboat/canboat/commit/1e6a7e679e12b29f04861ee609f2e319f91f0dcd))
* **pk:** mark Function Code (126464) and Report (65305) as PartOfPrimaryKey ([#713](https://github.com/canboat/canboat/issues/713)) ([e0348ad](https://github.com/canboat/canboat/commit/e0348ade718c030c3312e6bd8542f221e76222a3))
* reassemble ISO 11783-3 Transport Protocol (PGN 60416/60160) messages ([#736](https://github.com/canboat/canboat/issues/736)) ([339669b](https://github.com/canboat/canboat/commit/339669b8e6862a859a5747a947d067215cdc372c))
* **seatalk:** add alarm IDs 112/113/122-125 and Bluetooth alarm group ([#709](https://github.com/canboat/canboat/issues/709)) ([893287b](https://github.com/canboat/canboat/commit/893287b2e93c14e6e318073a4878db5d436d9dc1)), closes [#552](https://github.com/canboat/canboat/issues/552)


### Fixed

* allow already reassembled messages to exceed the fast packet length ([#754](https://github.com/canboat/canboat/issues/754)) ([35910de](https://github.com/canboat/canboat/commit/35910de74eb12f4961579ff6f93f92b8630d38a6))
* **analyzer:** key JSON/text output on -camel mode, not camelName presence ([#746](https://github.com/canboat/canboat/issues/746)) ([f890862](https://github.com/canboat/canboat/commit/f8908620578172d9afb73cfc3f2cab8875e05b8b)), closes [#745](https://github.com/canboat/canboat/issues/745)
* decode STRING_LAU fields with 0xff encoding byte as empty ([#740](https://github.com/canboat/canboat/issues/740)) ([97881b5](https://github.com/canboat/canboat/commit/97881b53620d22913a66318aee854a1d4f41d892))
* don't let fast-packet frame 0 complete against stale frames ([#738](https://github.com/canboat/canboat/issues/738)) ([65d5599](https://github.com/canboat/canboat/commit/65d55997e2b4eeb006ded2475456503afbf9baee))
* **lookup:** reconcile lookup bit-lengths with field widths ([#751](https://github.com/canboat/canboat/issues/751)) ([1bbcb9f](https://github.com/canboat/canboat/commit/1bbcb9f9a8d44c73810d0861be91f6ee47aa6f14))
* **navico:** correct 130822 Configuration Set naming and periodicity ([#734](https://github.com/canboat/canboat/issues/734)) ([5a328c4](https://github.com/canboat/canboat/commit/5a328c46a3f0146a3c75a75160791e5d73167cdd))

## [7.1.0](https://github.com/canboat/canboat/compare/v7.0.0...v7.1.0) (2026-06-30)


### Added

* **540:** per-field-type .sentinels + lookup top-of-range markers ([#704](https://github.com/canboat/canboat/issues/704)) ([ce03421](https://github.com/canboat/canboat/commit/ce03421e1d257569808a55404cbc7e672cf74716))
* **dbc:** multiplex proprietary PGN variants; move to docs/canboat.dbc ([#707](https://github.com/canboat/canboat/issues/707)) ([859bf85](https://github.com/canboat/canboat/commit/859bf859af76c6627b4cd357718dd830f2ac7d38))


### Fixed

* **navico:** rename PGN 130846 to "Simnet: Key Value - Long" ([#705](https://github.com/canboat/canboat/issues/705)) ([a2c9a43](https://github.com/canboat/canboat/commit/a2c9a4370cc7b356f8d7a58642c177851cdee209))

## [7.0.0](https://github.com/canboat/canboat/compare/v6.2.2...v7.0.0) (2026-06-29)


### ⚠ BREAKING CHANGES

* **navico:** PGN 130845 'repeatIndicator' and the trailing reserved field become 'instance' and 'source'. PGN 130846 is restructured: 'b', 'd' and the spare are removed; 'instance', 'operation' and 'source' added; 'key' widened from 16 to 24 bits.

### Added

* **540:** expose Unknown/OutOfRange/Reserved special values per field ([#672](https://github.com/canboat/canboat/issues/672)) ([20e98c7](https://github.com/canboat/canboat/commit/20e98c75673ac6eda0676a0fde0b33cc0ea8968a))
* **654:** reconcile PGN database against NMEA 2000 PDF v1.3 ([#671](https://github.com/canboat/canboat/issues/671)) ([813bef8](https://github.com/canboat/canboat/commit/813bef8d27f54e7edda7628d476568542dc4f672))
* **693:** "NMEA 2000 gateway: network status" PGN — rename + socketcan/actisense emission ([#698](https://github.com/canboat/canboat/issues/698)) ([a6f6c78](https://github.com/canboat/canboat/commit/a6f6c780730cec3154c7ce77d244d4bd9fcbfa84))
* add 8 missing PGNs from the NMEA 3.002 field-name list ([#655](https://github.com/canboat/canboat/issues/655)) ([#673](https://github.com/canboat/canboat/issues/673)) ([3efabb1](https://github.com/canboat/canboat/commit/3efabb13075d16c2a1e0f998415f17b4a5a4220f))
* decode Discrete Status 1 bitfield in PGN 127493 ([#419](https://github.com/canboat/canboat/issues/419)) ([#667](https://github.com/canboat/canboat/issues/667)) ([bd2fb6b](https://github.com/canboat/canboat/commit/bd2fb6b92cffba3497cd4c58b1ec77cdecdd7e18))
* fold canboat-py extended-decoder notes into Mercury PGN descriptions ([#689](https://github.com/canboat/canboat/issues/689)) ([#690](https://github.com/canboat/canboat/issues/690)) ([27ca9b5](https://github.com/canboat/canboat/commit/27ca9b5f9fe129077949c4a3acd7ff29555e4e39))
* **fusion:** decode additional 130820 status messages ([#682](https://github.com/canboat/canboat/issues/682)) ([906dcde](https://github.com/canboat/canboat/commit/906dcdebc653f9cc6fac0c50e4069acc1711ca44)), closes [#678](https://github.com/canboat/canboat/issues/678)
* **mercury:** decode VesselView-Link PGNs 130824/130825/130826/130829 ([#683](https://github.com/canboat/canboat/issues/683)) ([9f380a8](https://github.com/canboat/canboat/commit/9f380a8ac5694702fc4d818264a0fabae1fe785e))
* **navico:** decode 65313 Depth Quality and 130825 NDP2k Alert ([#686](https://github.com/canboat/canboat/issues/686)) ([8df21c8](https://github.com/canboat/canboat/commit/8df21c8466d9112f24828a119ec1b7717bd80358))
* **navico:** rework Simnet 130845/130846 Key/Parameter PGNs from firmware ([#701](https://github.com/canboat/canboat/issues/701)) ([9fe36ac](https://github.com/canboat/canboat/commit/9fe36ac99ad6f562c286798daac4f661757647a8))
* read Actisense W2K-1 files in actisense-serial ([#662](https://github.com/canboat/canboat/issues/662)) ([c460636](https://github.com/canboat/canboat/commit/c460636c218cc98c4c72f12c36bedc30865f6142))
* read PCAN-View trace files in candump2analyzer ([#250](https://github.com/canboat/canboat/issues/250)) ([#665](https://github.com/canboat/canboat/issues/665)) ([1123b97](https://github.com/canboat/canboat/commit/1123b975f5e97f8932e9ed673f56e9183b1f9b78))
* **victron:** decode VE.Can VREG registers via a key/value/type table (61184) ([#681](https://github.com/canboat/canboat/issues/681)) ([99c4c0a](https://github.com/canboat/canboat/commit/99c4c0ab6fae970b79fe3884019050b526a2666d)), closes [#677](https://github.com/canboat/canboat/issues/677)


### Fixed

* **126993:** heartbeat "Data transmit offset" is in milliseconds ([#696](https://github.com/canboat/canboat/issues/696)) ([d6afb2a](https://github.com/canboat/canboat/commit/d6afb2ab457a1e23d1a16f64a2001ac08939b3fa))
* avoid signed left-shift overflow in fast-packet reassembly ([#650](https://github.com/canboat/canboat/issues/650)) ([ef5b1c5](https://github.com/canboat/canboat/commit/ef5b1c50685abe550a12df561dba8a55ab1c97cb))
* correct SIMNET Alert bits field length to 64 bits ([#657](https://github.com/canboat/canboat/issues/657)) ([c104d8e](https://github.com/canboat/canboat/commit/c104d8e7bb2d53d7bae3dc56f92aa91dfee0fb0a))
* define PGN 127751 as fast packet, flag as unconfirmed ([#655](https://github.com/canboat/canboat/issues/655)) ([#669](https://github.com/canboat/canboat/issues/669)) ([a178ddf](https://github.com/canboat/canboat/commit/a178ddf8a95ffe2992dcc9616f1323f8d84780e2))
* don't fail a PGN when only a trailing reserved/spare field is short ([#663](https://github.com/canboat/canboat/issues/663)) ([#664](https://github.com/canboat/canboat/issues/664)) ([5788c7f](https://github.com/canboat/canboat/commit/5788c7f571ebf383e61f9c369357a94fda410442))
* drop the CANboat startup record from analyzer output under -fixtime ([#668](https://github.com/canboat/canboat/issues/668)) ([27c3ef7](https://github.com/canboat/canboat/commit/27c3ef706062b7c8d6f860a9c1c202c61da8b61b))
* guard int64 conversion of field range maximum ([#651](https://github.com/canboat/canboat/issues/651)) ([0c18ddc](https://github.com/canboat/canboat/commit/0c18ddcdcd670c2fcf8f4ee6ca1f3bf31b784d18))
* **json:** export Signed for keyed (DYNAMIC_FIELD_VALUE) field types ([#691](https://github.com/canboat/canboat/issues/691)) ([f705ea4](https://github.com/canboat/canboat/commit/f705ea4d6c0d701ae61c7ab306cc2e8e57396451))
* never write to a replayed file in actisense-serial ([#660](https://github.com/canboat/canboat/issues/660)) ([#661](https://github.com/canboat/canboat/issues/661)) ([2db94ad](https://github.com/canboat/canboat/commit/2db94ade17f5b735209404bbd5d55c0e63218ef4))
* prevent out-of-bounds read for out-of-range PGNs ([#649](https://github.com/canboat/canboat/issues/649)) ([a5a22b7](https://github.com/canboat/canboat/commit/a5a22b74b9ac5688019cba62669df08562cebd6f)), closes [#644](https://github.com/canboat/canboat/issues/644)
* treat ESC as an escape byte only in .ebl files ([#431](https://github.com/canboat/canboat/issues/431)) ([#659](https://github.com/canboat/canboat/issues/659)) ([2bc71c2](https://github.com/canboat/canboat/commit/2bc71c234140bc96ea5de458e2b54e093f382398))


### Changed

* rename CAMEL macro to ID_AND_NAME ([#526](https://github.com/canboat/canboat/issues/526)) ([#670](https://github.com/canboat/canboat/issues/670)) ([a4fd69a](https://github.com/canboat/canboat/commit/a4fd69a57e91a5f55c35485a4771d55824b0a450))

## [6.2.2](https://github.com/canboat/canboat/compare/v6.2.1...v6.2.2) (2026-06-24)


### Fixed

* drop CAN 1.0 (11-bit) standard frames ([#647](https://github.com/canboat/canboat/issues/647)) ([c9f5285](https://github.com/canboat/canboat/commit/c9f5285f0618ff3722fbd5c26471449c946beff7)), closes [#400](https://github.com/canboat/canboat/issues/400)

## [Unreleased]

## [6.2.1]

### Added

- Add socketcan-serial tool: bridge a Linux SocketCAN interface to/from canboat FAST format. Reassembles
  fast packets into whole PGNs (using a single/fast lookup table generated from canboat.json at build time
  for the mixed 0x1F000-0x1FFFF range), and timestamps received frames with the kernel SO_TIMESTAMP. Acts
  as a full ISO node: claims a source address (PGN 60928, scan-then-claim with conflict back-off);
  announces and answers Product Information (PGN 126996) and the Transmit/Receive PGN List (PGN 126464);
  NAKs addressed requests for unsupported PGNs (PGN 59392); sends a Heartbeat (PGN 126993) at the default
  60000 ms interval once claimed (-hb to change, 0 disables); and honours a Request Group Function
  (PGN 126208) that changes or disables the heartbeat rate, replying with an Acknowledge Group Function.
  The PGNs it generates itself (address claim, product info, PGN list, acknowledgements, heartbeat) are
  also echoed to stdout (unless -w), so a downstream consumer sees a complete picture of the bus.
- Simnet PGN 130840 Data Source Selection: replace the empty "Data User Group Configuration" stub with the
  Navico/Simrad source-selection broadcast (Manufacturer + sequence + Data Type + selected Source as 64-bit
  NMEA NAME). Adds SIMNET_DATA_SOURCE lookup with data-type ids confirmed by live bus probe: 17 Rudder
  Feedback, 19 Position, 27 Heading, 35 Depth, 36 Boat Speed, 42 Apparent Wind, 43 Barometric Pressure.
- Simnet/Navico PGNs 65323 and 130847 decoded from Triton2/AP48 firmware: 65323 (0xFF2B, Simrad) is a
  single-frame message emitted by AP48/Triton2/ZEUS heads and received by the autopilot; 130847 (0x1FF1F,
  Navico) is a length-prefixed ASCII identifier string emitted by Triton2/AP48/WS320.
- Furuno SCX-20 satellite compass: decode the proprietary config and satellite PGNs, verified against live
  bus captures. PGN 130845 (Multi Sats In View Extended) now decodes the per-antenna satellites-in-view
  records (report/antenna/page header, then repeating PRN/elevation/azimuth/SNR/range-residual; PRN ranges
  noted in the field description, including Furuno's Galileo = PRN - 131). PGN 130846 (Motion Sensor Status
  Extended) filled in (status + 25-byte data block). Added the Furuno (manufacturer 1855) variants of the
  config target PGNs 130833 (Ship Parameters and Antenna Position), 130834 (Speed-calculation Position) and
  130819 (Dead Reckoning Configuration); these are written by the SC_Setting_Tool via PGN 126208 Command
  Group Function, and defining the manufacturer variants lets the Command decoder resolve each parameter's
  value width. 130833 carries an inline comment documenting the 126208 field-addressed write mechanism.
- BEP Marine CZone proprietary PGNs: 65295 (CZone Alarm) and 65299 (CZone Alarm String Request). PGNs
  65301 and 130819 now decode their proprietary field layouts (both round-trip byte-for-byte; neutral field
  names are used where semantics are not yet known).

### Changed

- Simnet/Simrad autopilot PGNs refined from live AC-42 and NAC-3 captures (plus their control heads).
  65340 named "Autopilot Mode State" (was "AP Unknown 2") with standby/engaged class and engaged mode
  decoded (adds SIMNET_AUTOPILOT_MODE_CLASS and SIMNET_AUTOPILOT_MODE lookups); 65302 ("AP Unknown 1")
  structure fixed; 65420 ("AP Unknown 3") byte-6 sub-index noted; 130860 ("AP Unknown 4") AC-42/NAC-3
  differences noted; and 130851 gains "AP command Reply"/"Change Course" variants for the autopilot's
  addressed-reply form. Each explanation notes which autopilot computer emits the PGN.
- BEP Marine PGN 130820 renamed from "CZone Enumeration Reply" to "CZone Alarm String Response" (the
  response to a 65299 alarm string request) with Device ID / Channel / String fields aligned to the request.

### Fixed

- Furuno PGN 130843 (Heel Angle, Roll Information): the live heel angle was split across two mislabelled
  `UINT8` fields and the trailing angles were mislabelled. Decode the first signed 16-bit value as a single
  `Heel` angle (verified equal to PGN 127257 Roll over a live capture) and relabel the remaining fields.
- ikonvert-serial: always send a reset, so a previous run with a RX/TX list will not influence this run.
- ikonvert-serial: the synthetic PGN 262400 from the ikonvert report will contain the proper source address
  (whatever the iKonvert source address is, this is reported by the same PGN).
- analyzer: silence clang `-Wimplicit-const-int-float-conversion` warnings on macOS by making the
  `ISO_NAME_FIELD` rangeMax conversion explicit (`(double) UINT64_MAX`). The value is only a non-NaN marker;
  the explain/XML output already prints the exact integer for this field.

## [6.2.0]

### Added

- Add maretron-ipg tool: bidirectional TCP client for Maretron IPG100 N2K-to-Ethernet gateway.

### Changed

- n2kd: Replace global broadcast of ISO Request for Address Claim and Product Information with per-device targeted requests, spaced 1 second apart, only for devices that haven't sent the info in 5 minutes.
- **Breaking**: actisense-serial: `-p` (passthru) now matches the ikonvert-serial / maretron-ipg semantics — stdin lines are sent to the device *and* echoed to stdout, instead of being suppressed-from-device-but-not-echoed. The old behaviour ("skip device write") was a documentation/behaviour mismatch; the help text always claimed `-p` echoed to stdout but only `-o` actually did so. `-o` is kept as an alias for backward compatibility, but `-p` is now preferred and aligned across the three device readers. Users who relied on `-p` to suppress device writes should switch to `-r`.

### Fixed

- ikonvert-serial: Fix `-w` (writeonly) silently dropping every device message, including the `$PDGY,TEXT` / `$PDGY,ACK` responses that drive the init state machine. The device stayed in `N2NET_OFFLINE` indefinitely, so PGNs queued on stdin were never actually transmitted to the bus. Now ASCII control messages are parsed unconditionally so init runs to completion; N2K data frames are still parsed for device-liveness detection but suppressed before the stdout emit. `-r` (readonly) additionally no longer polls stdin and redirects fd 0 to `/dev/null` defensively.
- actisense-serial: Fix `-w` (writeonly) opening the device `O_WRONLY` and never reading it. The NGT-1 kept emitting bytes that backed up in the FTDI/USB driver's RX buffer (`CLOCAL` = no flow-control backpressure) until the kernel started dropping data. Now the device is opened R/W and inbound bytes are drained and discarded each loop iteration.
- actisense-serial: Fix `-r` (readonly) opening `O_RDONLY`, which caused both the startup `NGT_STARTUP_SEQ` (TX-list clear) write and the 20-second keepalive `writeMessage()` to fail silently against the read-only fd. It worked in practice only because the NGT-1's TX-list config persists across power cycles, so a previously-initialised device kept emitting all PGNs. Now `-r` opens R/W so the init actually lands, and stdin is redirected to `/dev/null` defensively.
- Fix strncpy truncation warning in emitCanboatStartupRecord.
- n2kd: Fix `%1f` typo in the VHW (Water Speed) NMEA 0183 formatter. The format width specifier had no precision, so the knots field printed at the default precision of 6 (e.g. `0.000000` instead of `0.0`).
- n2kd: Fix AIS `Communication State` (and other AIS BINARY-rendered fields) silently zeroing the 19-bit slot in AIVDM payloads. `aisInteger` previously called `atol("E4 10 01")` which stops at the first non-digit and returns 0 — losing the SOTDMA / ITDMA state on every type 1/2/3/4/9/18 sentence. It also returned just the first byte for `"56 00 02"`-style inputs whose first character is a digit. Now detects the space-separated hex-byte rendering and parses it little-endian, masking to the param's max.
- analyzer: Fix `g_ftf` global state leaking across records when a `LOOKUP_TYPE_FIELDTYPE` lookup misses. The `LOOKUP_FIELDTYPE` macros only assign `g_ftf` on a matching `case`, so an unknown key (e.g. an unrecognised SIMNET_KEY_VALUE) left whatever the previous record's lookup wrote there — making the subsequent `DYNAMIC_FIELD_VALUE` decode an unrelated Value with a stale field type. Now reset `g_ftf` before each FIELDTYPE lookup so a miss is observable downstream and the Value falls through `fieldPrintKeyValue`'s `g_ftf == NULL` branch to an empty BINARY blob.

## [6.1.9]

### Added

- #573: actisense-serial and ikonvert-serial now emit a `# format=FAST` header so the analyzer auto-detects the format correctly even when the first message is a short PGN.
- #381: Serial tools emit a CANboat Startup virtual PGN record with software version, source tool name, and device path.
- #603: Yamaha Gear Status PGN 65314 with Neutral indicator field.
- #624: Garmin SteadyCast AHRS ATT proprietary PGNs (126720): COG Source Valid Flag, Device Flags, Non-default Calibration Matrix Present, Set North State.
- #628: Decode six CZone (BEP Marine) proprietary PGNs: Circuit Control (65280), Alarm Event (65282), Channel State (65283), Circuit Status (65284), Module Announce (65290), Configuration Transfer (130816), Circuit Readings (130817), Enumeration Reply (130820).
- #565: Add PhysicalQuantity for all units: new DIMENSIONLESS_RATIO (for %, m/m, s/s) and SQUARE_ROOT_LENGTH (for sqrt(m)) physical quantities; map ppt/ppm to CONCENTRATION, semi-circle to ANGLE, semi-circle/s to ANGULAR_VELOCITY; enforce that all FieldTypes with a unit must reference a PhysicalQuantity.

### Fixed

- Fix iKonvert uptime field using wrong type (TIME with 0.0001s resolution instead of DURATION with 1s resolution).
- #623: Fix analyzer reading uninitialized stack memory from short fast-packet first frames.
- #630: Fix stack buffer overflow in Garmin CSV parser when Size exceeds FASTPACKET_MAX_SIZE.
- Harden all input parsers against buffer overflows from untrusted input lengths.
- #622: extractNumberNotEmpty: skip sentinel stripping when a field's explicit `.rangeMax` exactly matches the bit-size maximum, completing the #600 fix for PGN 60928 device instance fields.
- #626: Fix NULL function pointer crash in analyzer-explain XML output for MATCH_FIELD entries without a lookup.
- #625: Update stale external documentation URLs to current or web archive snapshots.

## [6.1.8]

### Added

- #621: Add "Signal K" as a Manufacturer
- #620: fix: change SID fields from BINARY to UINT8.
- #619: Added previously unknown `SEATALK_COMMAND` values.

## [6.1.7]

### Added

- PGN 130845: B&G improvements
- 19 Maretron product codes from MConnect (SMS100, MBB200C, DST110, GPS100, CLM100, GPS200, DST100, FFM100, RAA100, J2K100, IPG100, DCM100, EMS100, CLMD16, DSM250, DSM150, FPM100, MBB300C, MConnect)
- Sea Recovery Watermaker PGN 130816 and Webasto Status 2 PGN 130818
- Xantrex proprietary PGNs 130900, 130910-130913 (AC/DC status and configuration)
- Lumishore proprietary PGNs 65403 and 130939 (light status and control)
- Mercury Marine proprietary PGNs 65280 and 130829
- Yamaha proprietary PGNs (65329, 65344, 65424, 65472, 130945-130947, 130951, 131008, 131011-131012)
- Navico proprietary PGNs (65313, 65317, 130849, 130852)
- Navico Naviop switch PGNs 65440 and 65441
- B&G proprietary PGN 65330
- Simrad proprietary PGN 130861
- Suzuki proprietary PGNs (65298-65300, 65303-65304, 65315, 130830, 130837-130838)
- Yanmar proprietary PGNs (65280-65281, 65332, 65346, 65348-65349)
- Honda Marine proprietary PGNs (65280, 65284, 130816)
- Carling switchboard PGNs 65300 and 130921
- Carling Breaker Command PGN 61184
- Carling DC Configuration Command PGN 126720
- BEP Marine / CZone single-frame PGN stubs (65281, 65283, 65294-65297, 65299-65301, 65304, 65306, 65308, 65310-65311, 65314, 65316, 65325)
- BEP Marine / CZone fast-packet PGN stubs (130816-130826)
- BEP CZone Circuit Status PGN 65284 and Configuration PGN 130820
- Lumishore PGN 126720 stub
- Webasto HVAC Command PGN 130819
- HVAC Status PGN 130329 (26 fields)
- Maretron Dometic HVAC Status PGN 130828
- J1939 Electronic Transmission Controller 1 PGN 61142
- Maretron Alert family: PGN 130819 Alert Transmission, 130820 Alert Response, 130821 Alert Text, 130822 Alert Control identity headers
- Maretron proprietary PGN field layouts (65282, 65286-65291, 130818, 130824-130826, 130830, 130843)
- Maretron Windlass Operating Status PGN 130842
- PGN 130824 B&G: new lookup key 0x49 (Current Set)
- Related projects in README

### Fixed

- #592: Actisense timestamp parsing incorrect
- PGN 130824 B&G: corrected 9 mislabeled key-value lookup names based on firmware analysis (0x1e Outside Temperature, 0x34 Magnetic Variation, 0x4f Wind Speed True, 0x51 Wind Angle True, 0x56 Water Temperature, 0x59 True Wind Direction, 0x84 Current Drift, 0x9b Attitude Roll, 0xeb Water Speed)
- Status flags updated for 24 standard PGNs confirmed by MConnect (13 PACKET_PDF_ONLY to PACKET_INCOMPLETE, 9 PACKET_NOT_SEEN to PACKET_COMPLETE)
- Resolved PACKET_RESOLUTION_UNKNOWN for multiple PGNs via Maretron MConnect protocol definitions
- Maretron proprietary PGN field resolutions and names (65286-65291)
- Maretron Windlass PGN assignments corrected (130842-130844)
- PGN 130830: corrected field sizes for Maretron Dometic HVAC Status (16-bit to 8-bit)
- BEP Marine proprietary PGN stubs given unique names to avoid duplicate Ids
- SID field changed from BINARY_FIELD to UINT8_FIELD on 5 PGNs (127750 Converter Status, 127751 DC Voltage/Current, 65288 Seatalk: Alarm, 65359 Seatalk: Pilot Heading, 65360 Seatalk: Pilot Locked Heading).

## [6.1.6]

### Fixed

- #600: use full RangeMax for PGN 60928 device instance fields

## [6.1.5]

### Fixed

- #599: Update generated files not updated in 6.1.4

## [6.1.4]

### Added

- #598: Maretron improvements

## [6.1.3]

### Fixed

- #593: Improve Simnet Autopilot support

## [6.1.2]

### Fixed

- #590: Remove `analyzer/package.json`.
- #582: Updated interval, priority, and url for PGNs 126993, 130316, and 130574
- #588: Fix executable permissions in `make install`.
- Do not overwrite config files with `make install`.
- Print unsigned and signed temperature fields with the proper sign and precision.

### Added

- Furuno PGN 130817 and PGN 130818 (SCX-20)
- #584: Add Manufacturer Code and Industry Code to 130816 Fallback
- #581: Add manufacturer code for Simarine 
- #589: Update Seatalk1: Display Brightness, Shared

## [v6.1.1]

### Added

- #579: Reworked the PGN 126720 Raymarine specific PGNs

## [v6.1.0]

### Minor schema change

The XML schema has promoted LookupEnumeration MaxValue from 'int' to 'long' because there
are two lookups that use four bytes, the maximum unsigned value of that is not a valid int.

### Fixed

- ikonvert-serial: reset connection when no actual PGNs are received
- n2kd: Fix secondary key determination when multiple { } values are present
- n2kd: Fix secondary key determination for PGN 60928
- n2kd: Do not request product information for a stuck device whenever it sends a PGN
- n2kd: simplify product info and address claim requests
- #550: FUSION_REPEAT_STATUS and FUSION_SETTING Lookup Enumerations have MaxValue of 0
- #564: PGN 128520 field 'Track Status' is not a bit lookup
- #563: PGN 65240 field 'Manufacturer Code' should not have a unit
- #554: Fix print of geo positions in DMS format
- #562: Revert kWh -> J change; kWh is the underlying resolution so a more precise translation of the data.
- #560: MissingEnumeration NoCompanyFields should be MissingCompanyFields.
- #553: INDUSTRY_CODE 4 lookup value should be "Marine Industry", not "Marine".
- #566: Manufacturer fields for proprietary PGNs should not have pk attribute set"
- #541: add some primary key indicators
- #575: fix Maretron lookup field lengths
- #578: Add Seatalk1: Pilot Hull Type

### Added

- #555: Add Magenta colour to SIMNET_MODE_COLOR.
- #556: Fusion updates
- #558: Fusion updates
- #571: Maretron PGN 126720 field decoding improvements
- #577: Run tests for canboatjs, ts-pgns and related Signal K projects

## [v6.0.1]

### Fixed

- undo incomplete primary key update to n2kd making it not use any key.

## [v6.0.0]

### Explicity garantuee on Id values

PGN field names have been left the same as in v5.1.3, and only two fallback PGN ids have been changed.

From this point forward, we will garantuee that the <Id> values do not change.
We shall be making improvements to the <Name> values.

Therefore, all downstream consumers are urged to make sure they match on PGN Id and field Id, not name.

### Changed

This is a major upgrade because the old v1 xml and json files are no longer being
supplied.

This is a minor upgrade because there is a new interesting field added to the XML,
`PartOfPrimaryKey` which is a boolean attribute; any field carrying this (have the
field present with a `true` value) contributes to the _Primary key_ of the data --
e.g. any message with a different primary key is from a different source. Fields
like `Source Id`, `Message Id` or `Instance` will have this set to true.

Also, in n2kd `-json -nv` mode and in text mode all fields that refer to a PGN will explain
the meaning of the PGN value (if it is not a proprietary PGN number.)

Also, in n2kd `json -nv` mode any NAME fields that occur (so far, only in the Alerts PGNs)
will contain in the `name:` attribute a recursive expansion of the fields contained
in that single NAME field. These subfields are the same fields as described in PGN
60928 (Address Claim.)

Further possibly breaking changes in the definitions:

  * PhysicalQuantity:
    * GEOGRAPHICAL_COORDINATE has been split into GEOGRAPHICAL_LATITUDE and GEOGRAPHICAL_LONGITUDE.
    * SIGNAL_STRENGTH has been added.
    * DURATION has been added.
  * FieldType
    * PGN, ISO_NAME, DURATION have been added. They are specialisations of NUMBER, BINARY and TIME, which is what they were before.
    * Navico specific: fieldtype FIELDTYPE_LOOKUP is now named DYNAMIC_FIELD_LOOKUP. KEY_VALUE is now named DYNAMIC_FIELD_VALUE.
      Fieldtype DYNAMIC_FIELD_LENGTH has been added.
  * PGN
    * Fusion PGNs have been reworked.
    * PrimaryKey attribute has been added.
    * Many missing attributes such as priority have been added.
    * Fields have moved from being a plain NUMBER to LOOKUP because the lookup values are now known.
  * PGN names
    * PGN fallback 59392 is now correctly named 0xe8000xee00StandardizedSingleFrameAddressed instead 
      of 0xe8000xeeffStandardizedSingleFrameAddressed (note the ff -> 00 change).
    * PGN fallback 126720 is now correctly named 0x1ef000ManufacturerProprietaryFastPacketAddressed
      instead of 0x1ef000x1efffManufacturerProprietaryFastPacketAddressed.
  * PGN fields
    * Fusion PGNs have been reworked.
    
### Fixed

- Updated copyrights to 2025.
- Updated URLs to refer to web archive where needed and available (not all...)
- #475: FieldType VARIABLE should have the VariableSize attribute set to True.
- #474: PGN updates
- #473: PGN 129546 field lengths and types
- #472: PGN 127488 field Tilt/Trim is a percentage field
- #478: Make analyzer/pgn.xml+json obsolete
- #483: PGN 127506 field ripple voltage is 1 mV precision
- #489: Raymarine sends C style strings in `STRING_FIXED` fields
- #493: Remove Fallback filter from canboat.json
- #486: Missing identity designation in the canboat schema
- #500: PGN 129540 parameter Range residuals has incorrect resolution and unit
- #506: 0x1ED00 and 0x1EE00 are not single frame but fast packet
- #504: Document handling of max and two bit numbers
- #496: Show bitoffset for first variable length field
- #498: Document new findings regarding STRING_LZ fields.
- #499: DataTransmitOffset is in centiseconds, not millis.
- #508: Offset for Peukert coefficient is wrong.
- #509: Fix fusion transport lookup
- #514: Fix lookup for OFF_ON fields that control a setting (PGN 127502.)
- #522: Fix PGNs 130064-130074
- #521: Navico PGN 130817 contains unknown data, not product information.
- #517: PGN 130323 field Mode should be lookup RESIDUAL_MODE.
- #518: PGN 127510 + 127511 revised field lengths and lookups
- #525: Sync with latest publicly known sources
- #529: FIELDTYPE_LOOKUP -> DYNAMIC_FIELD_LOOKUP, KEY_VALUE -> DYNAMIC_FIELD_VALUE
- #533: Lookup SERIAL_BIT_RATE fixed, SIMNET_KEY_VALUE 11524 data type fixed
- #539: PGN 130565 field 8 repeats as well
- #541: Check primary key fields

### Added

- #488: Added Raymarine PGNS 130848 and 130918
- #481: Add manufacturer code for Revatek
- #482: Add PGNs 127747-127749.
- Updated manufacturer codes to refer to all companies known in Oct 2024.
- #501: Split TIME into TIME and DURATION.
- #503: Introduce PGN field type.
- #502: Introduce ISO_NAME fieldtype.
- #515: Rework Fusion PGNs.
- #523: Add support for .EBL file reading to actisense-serial.
- #530: Split GEOGRAPHICAL_COORDINATE into GEOGRAPHICAL_LATITUDE and GEOGRAPHICAL_LONGITUDE.
- #532: Add RMC sentence to n2kd nmea0183 output
- #535: Improve makefiles for cross compilation

### [5.1.3]

No functional changes, release because v5.1.2 was released incompletely.

### [5.1.2]

### Fixed

- #454: Fix 129538
- #469: Fix compiler warnings
- #461: fix 128520 and 130821 max string length
- #452: Add define to compile replay binary on strict linux systems
- #450: Maretron 130836 Switch Counter Status field lengths incorrect
- #468: Remove Unicode characters in canboat.json and canboat.xml
- #467: dbc-exporter build fails on newer python installations
- #466: RangeMax is incorrect for DECIMAL fields
- #465: Document obsoleteness of pgns.xml and pgns.json
- #463: Fast packet with size 223 (32 frames) doesn't assemble properly on 32 bit platform
- #464: PGN 127508 Voltage should be signed

### Added

- #356: Add GNSS type to PGN 129810

## [5.1.1]

- #451: candump2analyzer format 3 does not handle CRLF line endings correctly.

## [5.1.0] 

### Fixed

- #439: Improve PGN 65379 Raymarine 
- #442: Fix PGN 129799 Radio Frequency/Mode/Power field lengths and types.
- #423: Add support for Garmin Backlight level + day/night mode.
- #428: PGN 127509 is FAST, not SINGLE.
- #436: Fix length of field in PGN 65379 (Seatalk Pilot Mode).

### Added

- #444: Add analyzer2csv program and analyzer `-debugdata`.
- #445: Add default priority to explanations.
- #448: Add analyzer format `PLAIN_MIX_FAST`.
- #430: Add J1939 PGNs and `analyzer-j1939` and `analyzer-explain-j1939`.

## [5.0.3]

### Added

- #425: PGN 65293: Diverse Yacht Services: Load Cell
- Raymarine Seatalk1 Brightness and Color

### Fixed

- ikonvert-serial timestamps are now ignoring time from ikonvert itself.

## [5.0.2]

### Fixed

- Print raw negative times correctly (broken in 5.0.0).

### Added

- #420: Fusion track album ID lookup added
- Simnet 130845 multi-key/value pair PGN analysis

## [5.0.1]

### Added

- A new 'replay' binary has been added for help on MS Windows without Perl.

## [5.0.0]

Note: Changes to XML v2 (canboat.xml, canboat.xsd, canboat.xsl) so this will
constitute a new major release. The changes are small, the schema for canboat.xsd
has been bumped to 2.1. The differences are:
  - Dynamic field types, where a (repeating) set of fields defines a varying 
    field, can now have a 'lookup' as well. 
  - Some fields have changed from signed to unsigned.

To see the changes, use:

    git diff v4.12.0 docs/canboat.xml
    git diff v4.12.0 docs/canboat.xsd

### Added

- #413: docs: RangeMax for ENTERTAINMENT_PLAY_STATUS_BITFIELD is incorrect
- #309: Simrad Autopilot work (phase 1)
- #401, #407: Added 7 new device function values to PGN 65240 and 60928.
- #398: Add SIMNET_AP_EVENTS codes for B&G H5000 start features
- Add command to request PGN transmit rate change
- candump2analyzer: add support for Navico TCP candump
- n2kd: Add status port to show counts and intervals of received PGNs

### Fixed

- #414: Matched fields should show name, not bare value in explanation.
- #404: analyzer/canboat.xml: remove useless duplicate values from lookups.
- #403: analyzer: unsigned fields with offset / PGN 127513 peukert exponent was wrong.
- #411: analyzer: fix formatted printing of negative time offset values.
- #410: analyzer: make -raw produce standard raw format.
- ikonvert-serial: Improve iKonvert reset handling
- Remove unicode characters from source
- n2kd: write complete JSON state (use multiple blocking writes if necessary)
- #408: n2k_monitor: avoid n2kd monitor forking every 30s


## [4.12.0]

### Fixed

- #396: Add missing Fusion message ID loopup value
- #395: Altitude in PGN 129798 (SAR AIS) should be 32 bits
- #394: v1 type for `STRING_LZ` is incorrect

### Added

## [4.11.1]

### Fixed
- #384: Updated lengths and lookup for lighting PGNs.

### Added

## [4.11.0]

### Fixed
- End all logAbort messages with a newline.
- #389: Missing fieldtypes in v1 pgns.xml
- #391: Incorrect field sizes for PGN 129285.
- #393: Make sure single frame PGNs are 8 bytes and all PGNs fill the last byte.
        This actually found a few small bugs in PGNs:
        PGN 128778: Field length for Controller Voltage was incorrect.
        PGN 126802 Airmar 34: Field names were copy/pasted and thus incorrect.
        PGN 129803: Field names changed (but this PGN is not seen yet) to match ITU.
- #387: analyzer: Don't crash when presented with PGN of 0.

### Added
- #385: Allow analyzer to read mixed `RAWFORMAT_PLAIN` and `RAWFORMAT_FAST` input.
        A new `-format <fmt>` option has been added, this new mode is only enabled when
        `-format PLAIN_OR_FAST` is selected.

## [4.10.1]

### Fixed
- #383: Add NMEA 2000 v3.002 PGNs.
- #376: Added Type field to PGN 130571.
- #377: Ensure PGNs are correct framing type (packet type) and fix PGN 130824 B&G Wind Data.
- #365: Produce a `canboat.html` file

### Changed
- #366: PGN 127500 field lengths and framing type changed

## [4.10.0]

### Fixed
- Fix #358: Repair heading and temperature lookups in NMEA0183.
- Fixed lookup types in PGN 130573 and PGN 130579.
- Fix #373: Repair `analyzer -nv` and `analyzer -debug` output.

### Changed

- #270: Added SchemaVersion element to the XSD and generated XML/JSON
  files, and removed the old Version attribute.

### Added

- #372: Add support for Actisense N2K ASCII protocol to analyzer.

## [4.9.2]

### Fixed
- Fix #350: Add ranges
- Fix #351: list-product-information update
- Fix #352: request for PGN126996 on new source

## [4.9.1]

### Fixed

- #348: analyzer -json: Fix invalid JSON when first field is missing.
- #349: n2kd: fix secondary key determination broken in v4.4.0.

## [4.9.0]

### Fixed

- #347: Repeating sets analysis bug fixes.
- #346: Fix RangeMax for lookups that interfere with "reserved" values.
- #345: Remove duplicate field length indication from PGN 129797.
- #343: Add test for repeatingfieldset without count (PGN 126464).
- #342: Remove STRING_VAR type and incorrect PGN 130323 samples.
- #334: Log error when missing fields are present in repeating set.

## [4.8.1]

### Fixed

- #338: fix reference field lookup

## [4.8.0]

### Added

- #125: Added PGNs 130052 (Loran-C TD Data), 130053 (Loran-C Range Data), and 130054 (Loran-C Signal Data)
- Added fields for PGN 126986 (Alert Configuration) with some field lengths unknown
- Added fields for PGN 126987 (Alert Threshold) and PGN 126988 (Alert Value)
- Added fields for PGN 130061 (Channel Source Configuration) and PGN 130560 (Payload Mass)
- Added STATION_STATUS bitfield lookup for Loran-C PGNs
- Added signed version of SNR field

### Changed

- Changed PGN 130560 (Payload Mass) from fast packet to single frame
- Changed simple signed fields to signed percentages for PGN 130576 (Small Craft Status)
- Set interval of PGN 130313 (Humidity) to 2000 ms
- More forgiving timeout to suit NGT-1 USB

## [4.7.0]

### Changed

- Format change: `analyzer` now passes the unformatted data for fields such as Date and Time in 
  `{"value":x,"name":"fmt"}` style so downstream code does not need to un-parse these. 
  This is triggered by `-nv` just as it is for lookup fields.
- Format change: `analyzer` output for fields is always `{"fieldname": <object>, ...}` where object
  is either a simple value or a `{ "key": "value", ...}` object that contains at least the key named
  `"value"`.
- `docs/canboat.xml`: PGNInfo element `Fallback` is now a boolean.

### Added

- #337: Add `BitLengthField` element for variable length binary fields.

### Fixed

- #336: PGN 129285 Repeating Field info incorrect.
- #335: UTF16 strings are printed incorrectly.
- #333: Repair decimal fraction printing of Time fields.
- #332: PGNInfo.Fallback should be type 'Boolean'.
- #331: Fix invalid JSON generation by `analyzer` especially when options `-nv` and/or `-debug` were passed.
- Fix regression where end-of-data was not properly handled; caused incomplete data fields to be processed
  incorrectly.

## [4.6.1]

### Fixed

- #328: PGN 129302: Fix field lengths and lookup lists for `Bearing Reference` and `Calculation Type`.
- #326: AIS PGNs now have Enum Message Id, also use this in the ais parser.

## [4.6.0]

### Fixed

- #327: Fix print of MMSI on gcc-8.30/linux-arm7l; cast required to pass correct size on stack.
- #326: Match fields should have lookup enum or be number type.

## [4.5.2]

### Fixed

- Repaired `n2kd_monitor` to pass `-nv` option.
- Improve dependencies on common files in Makefiles.

## [4.5.1]

### Changed

- Added resolution for FLOAT and POWER fields.

### Fixed

- #324: pgns: some fields are missing resolution value

## [4.5.0]

### Changed

- When a PGN contains multiple 'Reserved' or 'Spare' fields the Id has been made unique by
  appending the field order to the Id.
- Several PGN Descriptions and Id have been changed to make them unique.
  Of these, only PGN 130310 is in general use.
  The affected PGNs (with their old names) are:
  - PGN 61184 "Seatalk: Wireless Keypad Control" has been split, with one version dropping the "Control" word.
  - PGN 65325 "Simnet: Reprogram Status" has been removed.
  - PGN 126720 "Fusion: Mute" has been renamed to "Fusion: Set Mute".
  - PGN 130310 "Environmental Parameters" has been renamed to "Environmental Parameters (obsolete)".
  - PGN 130820 "Furuno: Unknown" has been renamed to "Furuno: Unknown 130820"
  - PGN 130821 "Furuno: Unknown" has been renamed to "Furuno: Unknown 130821"
- PGN 129556 "GLONASS Almanac Data" has been improved with URL reference and explanations, as well as slight changes
  to field names causing Id changes.

### Added

- `docs/canboat.xml` now contains `PGN/Fields/Field/LookupIndirectEnumerationFieldOrder`.
- A test has been added to verify that PGN Id and Field Id are unique.

### Removed

- `docs/canboat.xml` no longer contains `PGN/Fields/Field/LookupIndirectEnumerationField`.

### Fixed

- #323: Generate unique 'Id' elements for PGNs.
- #322: Generate Order for IndirectEnumeration lookups.
- #321: Generate unique 'Id' elements for fields.

## [4.4.0]

### Changed

- `n2kd` must now be fed by `analyzer -json -nv` so that it does not need to reverse the lookup numbers
  for NMEA0183 streams. It does mean that anything with a lookup value is now presented differently to
  any JSON clients. This means the protocol has changed, requiring another version bump.
- String fields that contain different 'filler' characters at the end, for instance both 'space', '@' and
  0xff will be shrunk until all such characters are removed, not just the sequence of same type fillers.

### Fixed

- #320: Update AIS lookups according to ITU M.1371-5
- #319: analyzer: fixup printing of lookup values for `-json -nv` that have no entry in the lookup table.
- #318: n2kd: NMEA0183 streams for AIS will now log errors when AIS data field names are changed,
        lookup values are extracted by number -- see above for new restriction

## [4.3.0]

### Added

- #317: Add `INDIRECT_LOOKUP` fieldtype, and XML/JSON that enumerates `EnumTriplet`s.
        This is used for enumerating DEVICE_FUNCTION, which depends on the DEVICE_CLASS,
        as used in PGNs 60928 "ISO Address Claim" and 65240 "ISO Commanded Address".

### Fixed

- #317: PGN 127489 Dynamic Engine Parameters field Percent Engine Torque is signed, not unsigned.
- #316: Regression since v2.0.0: Temperature Coefficient was dropped from PGN 127513.
- #315: Remove superfluous duplicate fallback PGN.
- Fix `RangeMax` for UINT64_MAX values.
- Fix print of bitlookup in textual analyzer.

## [4.2.2]

### Fixed

- Add/fix `RangeMin` and `RangeMax` for fields with `Offset`.
- Fix makefiles & version permeation.
- Add external documentation section.

## [4.2.1]

### Changed

- Add documentation on the fields in `canboat.xml` in `canboat.xsd`. 
  These are useful for downstream interpreters of the file.
- Make `n2kd` emit a request for PGN 126996 (Product Info) if a source 
  is found that it doesn't have product info for. This allows it to use the
  secondary keys properly, and downstream users can create device -> src mappings.
- `ikonvert-serial` will now send $PDGY strings on stdin to the iKonvert.
  This is useful in debugging scenarios.

## [4.2.0]

### Changed

- XML v2 (`canboat.xml` + `canboat.json`) changes:
  - Added LOOKUP and BITLOOKUP as base field types in the FieldList. This means they
    are easily recognised (they are no longer a NUMBER).
  - Emit `Resolution` even when it is 1. Only stringy and bitfield types now do not have a resolution.
  - Emit `LookupEnumeration` even for fields that have a `Match`.
- XML v2 `canboat.xsl` changes:
  - Show offset for number fields and add textual explanation.

### Fixed

- #283: Further improvements to v2 JSON/XML


## [4.1.0]

### Changed

- Reduce the types of fields reported in the XML and JSON to the base types. A parser needs
  a 'parser' for each FieldType, and each will be different. Copy some attributes to field
  level in the XML. Internally the derived fieldtypes are still used.
- Add an orthogonal `PhysicalQuantity` type in the canboat.xml,json that explains what is stored in the field.
- Cleaner field lists in `canboat.xsl`.

### Fixed

- #311: Fix regression in 4.0.0: PGN 126996 Version field length
- #310: Resolution of field should match field type
- #283: Further improvements to v2 JSON/XML


## [4.0.0]

### Changed

- The default `make` target on top level is now to only build the binaries, not the 
  generated files. This is seems more appropriate for most users.
  Use `make generated` to update all generated files.
- The generated `pgns-v2` files (`.xml` and `.json`) added in 3.0.0 are moved to the
  `docs` subdirectory and renamed to `canboat`.
  They are now accompanied by:
  - `canboat.xsl` for rendering `canboat.xml` in a webbrowser. This is now the main
    documentation page for humans on how the PGN list is built up.
  - `canboat.css` for helping in rendering `canboat.xml` in a webbrowser.
  - `canboat.dtd` for validating `canboat.xml`.
- The older `pgns.xml` and `pgns.json` files are, as much as possible, unchanged but there are still some (breaking) changes:
  - The `RepeatingFields` element was imprecise and was not able to cover all types and manners of repetition, and has been replaced
    by up to six elements.
    `RepeatingFieldSet1Size`
    `RepeatingFieldSet1StartField`
    `RepeatingFieldSet1CountField` (only present if there is a field that contains the number of repetitions, otherwise boundless)
    `RepeatingFieldSet2Size`
    `RepeatingFieldSet2StartField`
    `RepeatingFieldSet2CountField` (only present if there is a field that contains the number of repetitions, otherwise boundless)
  - All fields now have a `Type`.
  - PGN fields no longer have a description `PGN` (which added no value as the field name is also `PGN`.
  - PGN 60416 "ISO Transport Protocol, Connection Management - End Of Message" field id "totalNumberOfPacketsReceived" is now
    "totalNumberOfFramesReceived".
  - The `MissingAttribute` named `Precision` is now named `Resolution` as that is the field attribute that it refers to that is
    missing or uncertain.
  - Power factor fields now have a proper `Resolution` (6.10352e-5) and `Units` attribute.
  - Non-matching ''Manufacturer code'' fields now have a list of manufacturers.
  - Where `Resolution` is 0 it is never output (instead of "usually not".)
  - Small resolutions are now printed in `%g` format not `%f`, so exponential notation is used -- for example `1e-07` instead of
    `0.0000001`.
- The JSON generated by `analyzer` has the following changes and fixes:
  - The '-debug' option now implies the "-nv" option, and "-debug -json" will print the data bytes or bits as hex bytes
    for every field. This makes it very easy to correlate the field with the source data.
  - shorter binary fields were incorrectly printed as a string contain decimal number. They are now printed as a
    space separated sequence of two uppercase hex digits.
  - longer binary fields are now printed the same as above, with uppercase instead of lowercase and no trailing space.
  - Some fields have changed from being a 'number' to a 'time', causing them to be formatted as `hh:mi:ss[.milli]`.
    These are:
    - PGN 126993 "Heartbeat" field "Data transmit offset"
    - PGN 127489 "Engine Parameters, Dynamic" field "Total engine hours"
  - Some fields that are really a number are now correctly printed as a number and not as a string:
    - PGN 60928 "ISO Address Claim" field "Unique Number"
  - Empty string fields are now considered 'null', so are only output in json format when -empty is passed.
  - MMSI fields are now a string, and always 9 digits long (so base stations start with "00").
  - PGN 65408 "Airmar: Depth Quality Factor" now contains a lookup for Depth Quality Factor.
  - PGN 130836 "Maretron Proprietary Switch Status Counter" renamed to "Maretron: Switch Status Counter".
  - PGN 130837 "Maretron Proprietary Switch Status Timer" renamed to "Maretron: Switch Status Timer" and
    period fields changed from "decimal" to "time".
- The algorithm for recombining frames into a full fast-packet PGN has been refactored. It now handles out-of-order
  data as produced by the YDGW-02. Some tests have been added to verify that this works.

### Fixed

- #305: Print reserved fields only when they are not all ones.
- #304: Print spare fields only when they are not all zeroes.
- #303: Allow lookups to use the values for "Error" and "Unknown".
- #302: Field type has changed for `PGNs[].Fields[].Match` in pgn JSON/XML.
- #298: Lookup values should say "Temperature", not just "Temp".
- #293: Unify field names for "RAIM", "AIS RAIM Flag" and "AIS RAIM flag" as "RAIM".
- #292: Reactive power unit is "VAR", not "var".
- #291: Units should never contain "seconds".
- #289: Unify spelling of "Repeat Indicator" to have capital I.
- #267: Print User ID as string field, always with 9 digits.
- #283: Enumerations in pgns-v2.json use names for all attributes.
- #285: Fix `SHORT_TIME` field printing incorrect values.

## [3.1.0]

### Added

- #279: Added value 15 'Shaft Seal Temperature' to `TEMPERATURE_SOURCE` lookup.

### Changed

- #282: Lookup table values should be numbers, not strings.
- #273: Fix resolutions for PGN 129541 which contain (very small) 2^-n values.

## [3.0.0] - 2022-09-06

### Added

- #266: Major refactoring, split analyzer-explain off and add better XML and JSON output
- A `dbc-exporter` was added.

### Changed

actisense-serial:
- #255: Fix handling of timeout argument

common:
- #248: Use R bit for PGN number calculation for use with J1939.

analyzer:

- #245: Instance field doesn't need to be qualified with Bank or Inverter, which is implicit.
- #256: Fix length of COG in PGN 129028
- #252: Further fixes for PGN 127507
- #245: Updates for PGNs 126993, 127509, 129541, companylist. Added PGN 130823 Maretron.
- #243: PGN 127507 is fast packet
- #229: Default install prefix should be /usr/local
- #234: Spelling refrigeration
- #244: 127506 is fast packet and add AmpHours field to 127506
- #235: Incorrectly generated schema in pgns.json for Fields.Field values,  Windlass enum values
- #241: Leeway PGN 12800 should contain signed value for leeway

## [2.0.0] - 2021-03-10

### Changed

All software is now licensed via Apache License Version 2.0. Since this 
is a potentially breaking issue for users that can only distribute GPL v3
this is a major version change.

Changes from a contributor that objected to the change have been reverted
(Pull requests #149, #150 and #152) and have been partly rebuilt.

## [1.4.2] - 2021-02-01

### Added

analyzer:

- #223: New windlass and anchor PGNs 128776, 128777, 128778.

## [1.4.1] - 2021-01-28

### Changed

n2kd:

- #157: NMEA0183 output is independent on whether `analyzer` is in `-si` mode.

## [1.4.0] - 2021-01-27

### Added

n2kd:

- #219: New NMEA0183 UDP mode.
- #187: NMEA0183 now writes ZDA message.
- #204: New `-empty` option will show all not-set fields as `null`.
- #222: Support for Furuno GNSS PGNs (65280, 130842-130845) and Heave (127252).

n2kd_monitor:

- #221: Add support for ikonvert-serial.

make:

- #194: Generate manpages with help2man if available.

### Changed

analyzer:

- Updated various PGNs in #205, #206, #200, #197, #191, #190. Affected PGNs:
  127513, 127744, 127745, 127746, 127551, 127550.
- #192: Fixed display of PSI.
- #202: Fixed close detection of output-only streams.
- #193: Fixed compiler compatibility.
- #195: Fixed typos.

format-message:

- #220: Fix length of PGN 127488.

n2kd:

- #218: Improve write reliability and avoid hangs.

all:

- Change copyright to 2021.
- #217: Fix compiler warnings.

## [1.3.0] - 2020-03-04

### Added

analyzer:

- Support for alert PGNs 126983, 126984, 126985.
- Support for Seatalk1 Smart Remote.
- Add type 19 in AtoN type in PGN 129041 (Issue 159.)
- Add rudimentary PGN 127500 data (Issue 175.)
- Add PGN 130569
- Add Chetco dimmer control (PR 178.)
- Add Fusion audio control PGNS (PR 177.)

format-message:

- Add support for PGNs 127506, 127508, 127509, 127488, 127489.
- Add support for writing to YDWG-02.
- Add support for SonicHub audio level PGN 130816.

### Changed

- Restore compatibility with old C compilers.
- Fix makefile for Ubuntu/Debian packaging

analyzer:

- Fix analysis of PGNs with repeating fields but no "# of fields" field.
- PGN fast/single determination is no longer made by number of bytes in PGN (Issue 181.)
  This is also reflected in the XML and JSON files with a new Type attribute.
- Fix transposed Maretron PGN 126270 -> 126720.
- Improve analysis of PGN 127513. Hopefully correct now, but there are still
  conflicting sources (Issue 143.)
- Fix YDWG-02 raw format analysis.
- Improve PGN 127506 lookups.
- Fix incorrect heading/track control fields (PR 179.)
- Fix PGN 126998 (PR 174.)
- Add support for "Special Manoeuver Indicator" in AIS PGNs.
- Fix PGN 127488 (PR 171.)
- Add missing Sequence ID field to PGN 129810 (PR 168.)
- Add missing Reserved field to to PGN 127250 (PR 169.)
- Improve AIS PGNs.
- Fix PGN 127502 to have no repeating fields.

n2kd:

- Add explicit separate server port (2600) for writing to the N2K interface.
- Add explicit separate server port (2601) for AIS data from N2K bus.
- Send both 129026 and 129029 to both normal and AIS clients.
- Fix default parameters in default n2kd config (PR 150.)
- Fix n2kd_monitor forking bug (PR 152.)
- Write AIS data to NMEA0183 format.
- Write NMEA0183 GLL data if position data is only in PGN 129025.

socketcan-writer:

- Write correct can id for both PDU1 and PDU2 messages.
- Replay the frames with original timeframe interval (PR 170.)

ikonvert-serial:

- Fix writing to bus when load is high.
- Allow unlimited rate transmission mode.

actisense-serial:

- Improve writing to bus when load is medium. NGT-1 seems to be unable
  to handle full bandwidth writes.
- Fix writing of messages, in particular those with 10 bytes.

iptee:

- Make it quit again when stdout is not writable
- Fix `iptee -u` for non-listening UDP connections
- Make `iptee -s` operational


## [1.2.1] - 2019-02-01

### Changed
- Update PGN 127489 Fuel Pressure units (Issue 141.)
- Update PGN 127498 ASCII string lengths (Issue 142.)
- Update PGN 127498 Max Speed resolution and unit (Issue 140.)
- Update PGN 65026 Generator AC Real/Apparent Power values should be 4 bytes

## [1.2.0] - 2019-02-01

### Added
- support for Digital Yacht iKonvert
- actisense-serial supports higher baud rates 460800 and 921600 (where supported by OS)
- analyzer can read YDWG-02 logfiles
- replay utility to play back raw logfiles with same rate as they were recorded

### Changed
- Fixed calculation of negative numbers
- Minor fixes to some PGNs.

## [1.1.0] - 2018-10-27

### Added
- Add -version option to all C binaries
- Add hyperlinks to README.md.

### Changed
- Rename + rework README to README.md for better readability on GitHub.
- Reformat all C code with new .clang-format file.
- Refactor use of StringBuffer and logDebug in actisense-serial.

### Removed

## [1.0.0] - 2018-10-21
### Added
- Add version. Since this is a pretty mature product we start at 1.0.0.
- Add changelog (this file).

### Changed

### Removed

## Versions

[Unreleased]: https://github.com/canboat/canboat/compare/v6.1.7..HEAD
[6.1.7]: https://github.com/canboat/canboat/compare/v6.1.6...v6.1.7
[6.1.6]: https://github.com/canboat/canboat/compare/v6.1.5...v6.1.6
[6.1.5]: https://github.com/canboat/canboat/compare/v6.1.4...v6.1.5
[6.1.4]: https://github.com/canboat/canboat/compare/v6.1.3...v6.1.4
[6.1.3]: https://github.com/canboat/canboat/compare/v6.1.2...v6.1.3
[6.1.2]: https://github.com/canboat/canboat/compare/v6.1.1...v6.1.2
[6.1.1]: https://github.com/canboat/canboat/compare/v6.1.0...v6.1.1
[6.1.0]: https://github.com/canboat/canboat/compare/v6.0.1...v6.1.0
[6.0.1]: https://github.com/canboat/canboat/compare/v6.0.0...v6.0.1
[6.0.0]: https://github.com/canboat/canboat/compare/v5.1.3...v6.0.0
[5.1.3]: https://github.com/canboat/canboat/compare/v5.1.1...v5.1.3
[5.1.1]: https://github.com/canboat/canboat/compare/v5.1.0...v5.1.1
[5.1.0]: https://github.com/canboat/canboat/compare/v5.0.3...v5.1.0
[5.0.3]: https://github.com/canboat/canboat/compare/v5.0.2...v5.0.3
[5.0.2]: https://github.com/canboat/canboat/compare/v5.0.1...v5.0.2
[5.0.1]: https://github.com/canboat/canboat/compare/v5.0.0...v5.0.1
[5.0.0]: https://github.com/canboat/canboat/compare/v4.12.0...v5.0.0
[4.12.0]: https://github.com/canboat/canboat/compare/v4.11.1...v4.12.0
[4.11.1]: https://github.com/canboat/canboat/compare/v4.11.0...v4.11.1
[4.11.0]: https://github.com/canboat/canboat/compare/v4.10.1...v4.11.0
[4.10.1]: https://github.com/canboat/canboat/compare/v4.10.0...v4.10.1
[4.10.0]: https://github.com/canboat/canboat/compare/v4.9.2...v4.10.0
[4.9.2]: https://github.com/canboat/canboat/compare/v4.9.1...v4.9.2
[4.9.1]: https://github.com/canboat/canboat/compare/v4.9.0...v4.9.1
[4.9.0]: https://github.com/canboat/canboat/compare/v4.8.1...v4.9.0
[4.8.1]: https://github.com/canboat/canboat/compare/v4.8.0...v4.8.1
[4.8.0]: https://github.com/canboat/canboat/compare/v4.7.0...v4.8.0
[4.7.0]: https://github.com/canboat/canboat/compare/v4.6.1...v4.7.0
[4.6.1]: https://github.com/canboat/canboat/compare/v4.6.0...v4.6.1
[4.6.0]: https://github.com/canboat/canboat/compare/v4.5.2...v4.6.0
[4.5.2]: https://github.com/canboat/canboat/compare/v4.5.1...v4.5.2
[4.5.1]: https://github.com/canboat/canboat/compare/v4.5.0...v4.5.1
[4.5.0]: https://github.com/canboat/canboat/compare/v4.4.0...v4.5.0
[4.4.0]: https://github.com/canboat/canboat/compare/v4.3.0...v4.4.0
[4.3.0]: https://github.com/canboat/canboat/compare/v4.2.2...v4.3.0
[4.2.2]: https://github.com/canboat/canboat/compare/v4.2.1...v4.2.2
[4.2.1]: https://github.com/canboat/canboat/compare/v4.2.0...v4.2.1
[4.2.0]: https://github.com/canboat/canboat/compare/v4.1.0...v4.2.0
[4.1.0]: https://github.com/canboat/canboat/compare/v4.0.0...v4.1.0
[4.0.0]: https://github.com/canboat/canboat/compare/v3.1.0...v4.0.0
[3.1.0]: https://github.com/canboat/canboat/compare/v3.0.0...v3.1.0
[3.0.0]: https://github.com/canboat/canboat/compare/v2.0.0...v3.0.0
[2.0.0]: https://github.com/canboat/canboat/compare/v1.4.2...v2.0.0
[1.4.2]: https://github.com/canboat/canboat/compare/v1.4.1...v1.4.2
[1.4.1]: https://github.com/canboat/canboat/compare/v1.4.0...v1.4.1
[1.4.0]: https://github.com/canboat/canboat/compare/v1.3.0...v1.4.0
[1.3.0]: https://github.com/canboat/canboat/compare/v1.2.1...v1.3.0
[1.2.1]: https://github.com/canboat/canboat/compare/v1.2.0...v1.2.1
[1.2.0]: https://github.com/canboat/canboat/compare/v1.1.0...v1.2.0
[1.1.0]: https://github.com/canboat/canboat/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/canboat/canboat/compare/v0.1.0...v1.0.0
