# Changelog
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## Wip

analyzer:

- #229: Default install prefix should be /usr/local
- #234: Spelling refrigeration
- #244: Add AmpHours field to 127506
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

[Unreleased]: https://github.com/canboat/canboat/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/canboat/canboat/compare/v0.1.0...v1.0.0
