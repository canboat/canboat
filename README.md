# CANBOAT

A small but effective set of command-line utilities to work with CAN networks on BOATs.  The most common version of CAN networks on board, and in fact at the moment the only ones that this suite can analyse, is NMEA 2000.

The NMEA 2000 database and implementation is copyrighted by the NMEA (National Marine Electronics Association). Access is restricted to members and parties that pay for it. If they do so they are not able to divulge the content of the database, thus making it impossible for open source developers to get access to it.

For this reason we have reverse engineered the NMEA 2000 database by network observation and assembling data from public sources.

## Quick reference

If you just want to know how the NMEA 2000 protocol works, with an explanation of all reverse engineered data, please go to the
[documentation](https://canboat.github.io/canboat) page.

To use the programs included in this project you may need a supported CAN interface. This can be a marketed-as-such NMEA 2000 Gateway or a non NMEA specific CAN interface. 

For more information go to the [CANBoat Wiki](http://github.com/canboat/canboat/wiki).

## File formats

`analyzer` reads NMEA 2000 data from a number of text formats on stdin and turns
each message into human readable text or JSON. Some formats it understands
directly; capture formats from specific gateways or CAN tools are first turned
into the analyzer's `PLAIN` format by a small helper program that you pipe into
`analyzer`:

- **Directly by `analyzer`** — text logs that already contain decoded
  CAN identifiers. The format is auto-detected, or can be forced with
  `-format <NAME>`.
- **Via `actisense-serial`** — Actisense NGT-1 binary streams and log files.
  `actisense-serial -r <file> | analyzer`
- **Via `candump2analyzer`** — Linux can-utils dumps and other raw CAN captures.
  `candump2analyzer <file> | analyzer`

The full list of understood formats:

| Format | Origin | Example line | How to read |
| --- | --- | --- | --- |
| `PLAIN` | canboat native, one CAN frame per line | `2016-02-28T19:57:41.000Z,3,126208,40,72,8,01,0d,f2,01,f8,01,03,01` | `analyzer` (auto, or `-format PLAIN`) |
| `FAST` | canboat native, a whole fast-packet message per line | `2021-01-18T01:23:06.960Z,6,129540,26,255,216,ec,ff,…` | `analyzer` (auto, or `-format FAST`) |
| `AIRMAR` | Airmar weather/depth devices | `0123 -8 09F80 …` | `analyzer` (auto) |
| `CHETCO` | Chetco devices | `$PCDIN,01F119,00000000,0F,FFFFFFFFFFFFFFFF*21` | `analyzer` (auto) |
| `GARMIN_CSV1` / `GARMIN_CSV2` | Garmin CSV export | `0,486942,127508,Battery Status,Garmin,6,255,2,1,8,0x017505FF7FFFFFFF` | `analyzer` (auto) |
| `YDWG02` | Yacht Devices RAW (YDWG-02 / YDNU-02) | `19:07:21.014 R 09F8017F 50 C3 …` | `analyzer` (auto) |
| `ACTISENSE_N2K_ASCII` | Actisense N2K ASCII | `A173321.107 23FF7 1F513 012F…` | `analyzer` (auto) |
| Actisense NGT-1 binary | Actisense NGT-1 USB / serial / TCP gateway | _(binary)_ | `actisense-serial -r <dev> \| analyzer` |
| Actisense EBL | Actisense `.ebl` log files | _(binary)_ | `actisense-serial -r <file.ebl> \| analyzer` |
| Actisense W2K-1 | W2K-1 gateway capture | `{"pgn":60928,"payload":[147,19,6,0,238,…]}` | `actisense-serial -r <file> \| analyzer` |
| candump (Angstrom) | Linux can-utils | `<0x18eeff01> [8] 05 a0 be 1c 00 a0 a0 c0` | `candump2analyzer <file> \| analyzer` |
| candump (Debian) | Linux can-utils | `can0  09F8027F  [8]  00 FC FF FF 00 00 FF FF` | `candump2analyzer <file> \| analyzer` |
| candump log | Linux can-utils (`candump -l`) | `(1502979132.106111) slcan0 09F50374#000A00FFFF00FFFF` | `candump2analyzer <file> \| analyzer` |
| tshark / pcap | Wireshark text export of a CAN pcap | `… CAN 16 XTD: 0x09fd0223   00 49 02 1c a7 fa ff ff` | `candump2analyzer <file> \| analyzer` |
| Navico (TCP port 8086) | Navico raw dump | `0021200 0e 1d ff 9d 08 00 00 00 80 df 3f 9f 34 12 ff 0d` | `candump2analyzer <file> \| analyzer` |
| PCAN-View | PEAK PCAN-View v1.1 trace | `1)  2.7  Rx  09F11324  8  53 84 9E 01 00 FF FF FF` | `candump2analyzer <file> \| analyzer` |

`pcap2candump` can additionally turn a raw `.pcap` capture into the candump log
format for `candump2analyzer`. Example captures for many of these formats live in
the [`samples/`](./samples) directory.

## Building, Development and Testing

In [Wiki](https://github.com/canboat/canboat/wiki) you can find instructions on how to build the programs on your own computer 
and how to start extending the PGN database. Short instructions are also found in [BUILDING.md](./BUILDING.md).

## Using the definitions in your own project

If you just want to use the definitions in XML or JSON format, use the versions in the `docs` directory, e.g. (./docs/canboat.xml) or (./docs/canboat.json).

## Version history

See [Changelog](CHANGELOG.md).

## Related Projects

- [canboatjs](https://github.com/canboat/canboatjs) Pure JavaScript NMEA 2000 decoder and encoder
- [nmea2000](https://github.com/tomer-w/nmea2000) Pure Python NMEA 2000 decoder and encoder library based on canboat
- [nmea2000 Home Assistant custom integration](https://github.com/tomer-w/ha-nmea2000) Expose NMEA2000 PGNs as Home Assistant devices and entities

---

(C) 2009-2025, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
