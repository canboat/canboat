# File formats

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
| `SEASMART` (or `CHETCO`) | `$PCDIN` sentences: Chetco SeaSmart.Net, Quark-elec (QK-A032, QK-A034, QK-A036), Yacht Devices in NMEA 0183 mode (YDWG-02, YDNG-03) | `$PCDIN,01F119,00000000,0F,FFFFFFFFFFFFFFFF*58` | `analyzer` (auto), `canboat convert --from seasmart` |
| `GARMIN_CSV1` / `GARMIN_CSV2` | Garmin CSV export | `0,486942,127508,Battery Status,Garmin,6,255,2,1,8,0x017505FF7FFFFFFF` | `analyzer` (auto) |
| `MXPGN` | ShipModul MiniPlex `$MXPGN` sentences, one CAN frame each, and its `$PSMDNC` / `$PSMDNP` (Address Claim, Product Information), among other NMEA 0183 (`canboat convert --from mxpgn`) | `$MXPGN,01F112,2807,FC7FFF7FFF168012*11` | `analyzer` (auto) |
| `ACTISENSE_MXPGN` | `$MXPGN` as an Actisense gateway sends it (a W2K-2's CANDIRECT in NMEA 0183): the data in bus order (`canboat convert --from actisense-mxpgn`) | `$MXPGN,01F214,6842,00B004FF7FFFFFFF*1F` | `analyzer -format ACTISENSE_MXPGN` |
| `YDWG02` | RAW ASCII: Yacht Devices RAW (YDWG-02 / YDNU-02), an Actisense W2K-1 or PRO-NDC-1E2K in RAW ASCII mode (`canboat convert --from raw-ascii`) | `19:07:21.014 R 09F8017F 50 C3 …` | `analyzer` (auto) |
| `ACTISENSE_N2K_ASCII` | Actisense N2K ASCII (`canboat convert --from n2k-ascii`) | `A173321.107 23FF7 1F513 012F…` | `analyzer` (auto) |
| Actisense NGT-1 binary | Actisense NGT-1 USB / serial / TCP gateway | _(binary)_ | `actisense-serial -r <dev> \| analyzer` |
| Actisense EBL | Actisense `.ebl` log files | _(binary)_ | `actisense-serial -r <file.ebl> \| analyzer` |
| Actisense W2K-1 | W2K-1 gateway capture | `{"pgn":60928,"payload":[147,19,6,0,238,…]}` | `actisense-serial -r <file> \| analyzer` |
| Waveshare USB-CAN-A | The 20-byte frames the adapter sends on its serial port | _(binary)_ | `canboat convert --from usb-can-a <file>` |
| candump (Angstrom) | Linux can-utils | `<0x18eeff01> [8] 05 a0 be 1c 00 a0 a0 c0` | `candump2analyzer <file> \| analyzer` |
| candump (Debian) | Linux can-utils | `can0  09F8027F  [8]  00 FC FF FF 00 00 FF FF` | `candump2analyzer <file> \| analyzer` |
| candump log | Linux can-utils (`candump -l`) | `(1502979132.106111) slcan0 09F50374#000A00FFFF00FFFF` | `candump2analyzer <file> \| analyzer` |
| tshark / pcap | Wireshark text export of a CAN pcap | `… CAN 16 XTD: 0x09fd0223   00 49 02 1c a7 fa ff ff` | `candump2analyzer <file> \| analyzer` |
| Navico (TCP port 8086) | Navico raw dump | `0021200 0e 1d ff 9d 08 00 00 00 80 df 3f 9f 34 12 ff 0d` | `candump2analyzer <file> \| analyzer` |
| PCAN-View | PEAK PCAN-View v1.1 trace | `1)  2.7  Rx  09F11324  8  53 84 9E 01 00 FF FF FF` | `candump2analyzer <file> \| analyzer` |

`pcap2candump` can additionally turn a raw `.pcap` capture into the candump log
format for `candump2analyzer`. Example captures for many of these formats live in
the [`samples/`](../samples) directory.
