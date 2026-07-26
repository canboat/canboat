# canboat-rs

Advanced tooling for NMEA 2000. Sister project to
[canboat](https://github.com/canboat/canboat) — same PGN database, same wire
formats.

The project serves two purposes. First, a more composable way of working with
the CANboat database, built with modern tools: the PGN database is compiled
into a sans-I/O core library, with thin sync and async adapters above it, so
you can embed NMEA 2000 decoding in your own application instead of parsing
another program's output. Second, more polished end-to-end solutions —
`canboat server` runs the whole device-to-services chain in one process,
and `canboat tui` puts an interactive monitor on top of it.

## The library side

- **`canboat-core`** — sans-I/O. PGN database, format parsers, reassembly,
  decoder, encoder, output formatters. No `std::io`, no `tokio`, no threads.
  The schema (`canboat.json`) is compiled in at build time; there is nothing
  to load or distribute at runtime.
- **`canboat-io`** — sync `std::io` adapters (stdin, serial, `std::net`).
  Used by the standalone binaries.
- **`canboat-tokio`** — async tokio adapters, for embedding the decode
  pipeline in a tokio application.
- **`canboat-cli`**, **`canboat-schema`** — shared CLI plumbing and schema
  types.

Note: as this is not hitting crates.io yet, we may decide that a different
crate structure is better. This is still v0.x, so expect breakage!

## The tool side

`canboat` is the single front-end binary. Its subcommands cover what used to
be a fistful of separate tools:

- `canboat server` — the flagship: one process that reads a device
  (iKonvert, Actisense, SocketCAN, Maretron IPG, or a log file), decodes, and
  serves snapshot / analyzer-JSON / NMEA 0183 / CSV over TCP, with
  supervisor-based device reconnect.
- `canboat tui` — a text user interface à la `top` for viewing messages live
  or from a log file. The go-to way of reading log files, monitoring a live
  bus, and configuring devices.
- `canboat convert` — decode/convert any capture (PLAIN/FAST/Actisense/
  YDWG02/iKonvert, and `.pcap`/`.pcap.gz`/`.nif` containers unwrapped
  automatically) to PLAIN, JSON, or text. Drop-in for the C `analyzer` and
  the old `candump2analyzer` / `pcap2candump` / `nif2analyzer` shunts.
- `canboat interface` — bridge a live gateway (NGT-1 / iKonvert / SocketCAN /
  Maretron IPG) to and from stdout.
- `canboat replay` — pace a captured PLAIN/FAST stream at its original
  wall-clock rhythm.
- `canboat n2kd` — multiplex an analyzer-JSON stdin stream to TCP clients
  (snapshot / stream / NMEA 0183 / AIS / status ports).

The retired standalone names still work: `canboat` inspects `argv[0]`, so a
symlink named `analyzer`, `actisense-serial`, `ikonvert-serial`,
`socketcan-serial`, `maretron-ipg`, `replay`, `n2kd`, `canboat-pipeline`, or
`canboat-tui` dispatches into the right subcommand. `canboat install-shims`
creates them. (`socketcan-writer` is gone too: its stdin-PLAIN → CAN-bus job
is `canboat interface --kind socketcan -w`.)

Everything is now one binary — `canboat` — plus those compatibility shims.

The decode path, the device bridges, `n2kd`, and the `server` pipeline are
exercised against real hardware and pass a byte-for-byte golden-test suite
against canboat C's output.

## Installation

Each [release](https://github.com/canboat/canboat-rs/releases) ships all the
binaries as one archive per platform: static musl Linux builds for x86_64,
aarch64, and armv7 (Raspberry Pi class), a macOS universal binary, and
Windows x86_64.

Or build from source (Rust 1.88+):

```
git clone https://github.com/canboat/canboat-rs
cd canboat-rs
cargo build --release
```

## The PGN database

The schema is read straight from the repository's own `docs/canboat.json`,
which keel generates from `database/`. There is no vendored copy and no pin:
the build scripts in `canboat-core` and `canboat-io` read the real file, so
the runtime can never be a schema behind the analyzer. Regenerate it the
normal way — `make generated` from the repository root — and the Rust test
suite tells you whether the decoder needs work for the new schema.

The `CANBOAT_BEM` pseudo-PGNs (0x40000+) that `analyzer/pgn.h` defines
directly never appear in `canboat.json`, so `crates/canboat-core/data/
synthetic-pgns.json` still mirrors those by hand. Every binary reports the embedded
schema version in `--help` and in its startup log line.

## Performance

Same decode work — `-json -nv` over 1.26 M PGN frames (canboat's
`dirona-actisense-serial.raw` × 50) on an M4 Pro, release build:

| Implementation         | Wall time | vs canboat-rs |
|------------------------|----------:|--------------:|
| canboatjs (Node 25)    |  27.8 s   |   **8.1 ×**   |
| canboat C              |   9.1 s   |   **2.6 ×**   |
| canboat-rs `canboat convert` |   3.4 s   |       1.0 ×   |

`canboat server` goes one step further and collapses the
`canboat convert | canboat n2kd` pipeline into a single process with no
JSON text serialisation between stages:

| Pipeline                                          | Wall time | Throughput          |
|---------------------------------------------------|----------:|---------------------|
| `canboat convert` alone (PGN decode only)         |   3.3 s   | 380 k frames / s    |
| `canboat convert \| canboat n2kd` (piped, 2 proc) |   6.5 s   | 194 k sentences / s |
| `canboat server` (single proc)                    |   3.5 s   | 360 k sentences / s |

That's 46 % less wall time than the piped setup while doing strictly more
work (it's a long-running service with TCP fan-out); on CPU time the ratio
is closer to 3.4 ×.

## Contributing

`make precommit` runs what CI checks (fmt, clippy, tests). PR titles follow
[Conventional Commits](https://www.conventionalcommits.org/) — releases and
the changelog are derived from them. The golden tests need a canboat
checkout: a sibling `../canboat`, or point `CANBOAT_DIR` at one.

## License

Apache-2.0. See [LICENSE](LICENSE).

(C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
