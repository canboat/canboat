# CANBOAT

A small but effective set of command-line utilities to work with CAN networks on BOATs.  The most common version of CAN networks on board, and in fact at the moment the only ones that this suite can analyse, is NMEA 2000.

The NMEA 2000 database and implementation is copyrighted by the NMEA (National Marine Electronics Association). Access is restricted to members and parties that pay for it. If they do so they are not able to divulge the content of the database, thus making it impossible for open source developers to get access to it.

For this reason we have reverse engineered the NMEA 2000 database by network observation and assembling data from public sources.

## Quick reference

If you just want to know how the NMEA 2000 protocol works, with an explanation of all reverse engineered data, please go to the
[documentation](https://canboat.github.io/canboat) page.

To use the programs included in this project on a live bus you need a supported CAN interface: a marketed-as-such NMEA 2000
gateway, or a plain CAN adapter. The [gateways](./doc/gateways.md) page lists the ones canboat drives.

## The programs

CANboat ships two implementations. Both are **tier 1** — supported, tested and
released together — and they are interchangeable for the jobs they both do.

**C tools.** `analyzer` and the `*-serial` gateway drivers are the originals and
stay tier 1. They are small, dependency-light and build with nothing but `make`
and a C compiler.

| | |
| --- | --- |
| `analyzer` | decode NMEA 2000 into text or JSON |
| `actisense-serial` | Actisense NGT-1 / W2K-1 gateways |
| `ikonvert-serial` | Digital Yacht iKonvert |
| `socketcan-serial` | Linux SocketCAN |
| `maretron-ipg` | Maretron IPG100 |
| `candump2analyzer`, `replay`, `iptee`, `nmea0183-serial`, `analyzer-j1939` | converters and helpers |

**`canboat`, the Rust binary.** One executable with subcommands. `convert` and
the gateway support are tier 1 alongside their C counterparts; everything with
no C equivalent is tier 1 as the only implementation.

| | |
| --- | --- |
| `canboat convert` | decode a capture between any supported formats |
| `canboat interface` | bridge a live gateway (NGT-1 / iKonvert / Maretron / Yacht Devices / W2K-1 / PRO-NDC / SocketCAN / CANalyst-II / USB-CAN-A) |
| `canboat n2kd` | multiplex an analyzer-JSON stream to TCP clients |
| `canboat server` | the whole device → analyzer → n2kd pipeline in one process |
| `canboat tui` | interactive terminal browser for a live stream or a capture |
| `canboat format-message` | build a single frame from field values, for any PGN |
| `canboat replay` | pace a capture at its original wall-clock rhythm |

`canboat install-shims` drops symlinks so the subcommands answer to the old
names (`analyzer`, `n2kd`, `actisense-serial`, …). It never takes a name that
another program already holds, so it is safe to run in a directory that has the
C tools installed — it reports what it left alone.

The remaining C converters, the helper scripts in `util/` (PHP, perl and ksh)
and the Python tooling will most likely move to Rust in time; the C `n2kd` and
`n2kd_monitor` already have, and were removed in v8.

## Documentation

| | |
| --- | --- |
| [doc/gateways.md](./doc/gateways.md) | The supported gateways and CAN adapters: `--kind` and `--data-format`, what each reads and writes, J1939 and 11-bit, device notes, the AOS simulator, the CANalyst-II on Windows, the transmit and receive PGN lists and `--rx-filter` |
| [doc/server.md](./doc/server.md) | `canboat server`: its ports, the device quirks, a capture as a live bus (`--replay`), performance |
| [doc/file-formats.md](./doc/file-formats.md) | The capture formats `analyzer` and `canboat convert` read |
| [crates/canboat/README.md](./crates/canboat/README.md) | The `canboat` Rust library: quick start and features. The API reference is on [docs.rs](https://docs.rs/canboat), its design in [crates/doc/library-api-plan.md](./crates/doc/library-api-plan.md) |
| [BUILDING.md](./BUILDING.md) | Building the C tools and the Rust binary, and testing |
| [CONTRIBUTING.md](./CONTRIBUTING.md) | Pull request titles, adding or fixing a PGN, the evidence a database change needs |

For more information go to the [CANBoat Wiki](http://github.com/canboat/canboat/wiki).

## Contributing

Pull requests are welcome! See [CONTRIBUTING.md](./CONTRIBUTING.md) for how to write your PR title
(it becomes the release changelog entry), the source/generated lockstep rule, how to add or fix a PGN with
`keel edit`, and what evidence a database change needs. [BUILDING.md](./BUILDING.md) has the build and test
instructions.

## Using the definitions in your own project

If you just want to use the definitions in XML or JSON format, use the versions in the `docs` directory, e.g. (./docs/canboat.xml) or (./docs/canboat.json).
We recommend that you pin your retrieval to a particular release. Semantic versioning is used to warn you of major (x.0.0) or minor (0.x.0) changes to the 
definitions.

## Version history

See [Changelog](CHANGELOG.md).

## Related Projects

### Sibling projects (same authors)

- [canboatjs](https://github.com/canboat/canboatjs) — pure JavaScript NMEA 2000 decoder and encoder

(The Rust library used to live separately as `canboat-rs`; it was merged into
this repository in v8 and is the [`canboat` crate](./crates/canboat/README.md).)

### Other projects using the CANboat PGN definitions

> **Snapshot of early October 2026.** The CANboat version each project uses and
> when it was last updated were taken from GitHub on 7 October 2026 and will
> drift. The current CANboat release is v8.3.0. The survey behind this table,
> with more projects, is in [research/downstream-survey-2026-10.md](research/downstream-survey-2026-10.md).
> Maintain one of these, or another project built on CANboat?
> [Open an issue](https://github.com/canboat/canboat/issues/new?template=release-notifications.yml)
> to correct your entry or to be notified of new major and minor releases.

| Project | What it is | Uses CANboat as | CANboat version | Project last updated |
|---|---|---|---|---|
| [AvNav](https://github.com/wellenvogel/avnav) | Chart plotter | NMEA 2000 data from the `n2kd` server | binaries | 2026-10 |
| [boatkit n2k](https://github.com/boatkit-io/n2k) | Go NMEA 2000 library | Go types generated from canboat.json at a pinned release | v8.1.0 | 2026-10 |
| [CanLab](https://github.com/Sherin-SEF-AI/CanLab) | CAN, J1939 and NMEA 2000 analysis workbench (Python) | PGN table built from canboat.json | v8.2.1 | 2026-09 |
| [canboat_vendor](https://github.com/robotic-esp/canboat_vendor) | ROS 2 package | builds the CANboat tools | 2026-08 (pre-v8.0.0-beta3) | 2026-08 |
| [go-nmea-client](https://github.com/aldas/go-nmea-client) | Go NMEA 2000 library | canboat.json loaded at runtime | whichever file is supplied | 2023-07 |
| [korri-n2k](https://github.com/fard-draf/korri-n2k) | NMEA 2000 stack for embedded Rust | code generated from canboat.json | v7.1.0 | 2026-09 |
| [maritime-dissector](https://github.com/fkie-cad/maritime-dissector) | Wireshark dissectors for maritime protocols | NMEA 2000 dissector generated from canboat.json | latest, at generation time | 2026-03 |
| [n2k](https://github.com/mbj4668/n2k) | Erlang NMEA 2000 library | vendored CANboat definitions | v4.12.0 | 2026-04 |
| [nmea2000](https://github.com/tomer-w/nmea2000) | Python NMEA 2000 decoder and encoder | vendored canboat.json, synced weekly | v8.1.0 | 2026-10 |
| [nmea2000 Home Assistant integration](https://github.com/tomer-w/ha-nmea2000) | NMEA 2000 PGNs as Home Assistant devices and entities | through nmea2000 | v8.1.0 | 2026-10 |
| [NMEA2000-Analyzer](https://github.com/negrusti/NMEA2000-Analyzer) | Windows GUI | canboat.json downloaded from master at runtime | latest | 2026-07 |
| [nmea2000-rs](https://github.com/herostrat/nmea2000-rs) | Rust NMEA 2000 decoder and encoder | code generated from canboat.json at build time | v6.1.3 | 2026-02 |
| [open-ships n2k](https://github.com/open-ships/n2k) | Go NMEA 2000 library and bus node | decoders generated from a vendored canboat.json | v7.1.0 | 2026-09 |
| [pgntui](https://github.com/phobicdotno/pgntui) | Terminal UI for NMEA 2000 (Python) | vendored canboat.json | v6.2.0 | 2026-06 |
| [signalk-server-go](https://github.com/wdantuma/signalk-server-go) | Signal K server in Go | decodes NMEA 2000 with a vendored canboat.xml | v6.2.2 | 2026-09 |
| [Smart Boat Innovations integrations](https://github.com/SmartBoatInnovations) | [ha-smart2000usb](https://github.com/SmartBoatInnovations/ha-smart2000usb) and [ha-smart2000esp](https://github.com/SmartBoatInnovations/ha-smart2000esp): NMEA 2000 into Home Assistant over USB-CAN or an ESP32 | decoders generated from canboat.json | v5.0.3 | 2026-04 |

### For non-technical sailors

CANboat and its sibling [canboatjs](https://github.com/canboat/canboatjs) are command-line tools. If you
just want to use your NMEA 2000 network, these projects build on them and provide a friendlier experience:

- [OpenPlotter](https://openplotter.org/)
- [Signal K](https://signalk.org/)

## Sponsorship

If you find CANboat useful, please consider supporting its development via
[GitHub Sponsors](https://github.com/sponsors/keesverruijt).

---

(C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

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
