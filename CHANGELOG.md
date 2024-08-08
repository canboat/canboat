# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Sections can be: Added Changed Deprecated Removed Fixed Security.

## [Unreleased]

### Fixed

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

- #125: Added PGNs 130052 (Loran-C TD Data), 130054 (Loran-C Range Data), and 130054 (Loran-C Signal Data)
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

[Unreleased]: https://github.com/canboat/canboat/compare/v5.1.1...HEAD
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
