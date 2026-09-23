# canboat

NMEA 2000 decoding, encoding and live-bus access, as a Rust library and a
command-line tool. The whole [CANboat](https://github.com/canboat/canboat)
PGN database — some 600 message definitions, reverse-engineered on the water
since 2009 — is compiled in: nothing to load at runtime, and every PGN both
decodes and encodes.

## The tool

```
cargo install canboat
```

installs one binary, `canboat`, whose subcommands replace the historical
scatter of CANboat utilities:

| subcommand | |
| --- | --- |
| `convert` | decode any capture (PLAIN/FAST, Actisense, YDWG-02, iKonvert, `.pcap`, `.nif`, …) to PLAIN, JSON or text; the successor of `analyzer` |
| `interface` | bridge a live gateway — Actisense NGT-1, Digital Yacht iKonvert, Maretron IPG, Linux SocketCAN — to and from stdout |
| `server` | one process that reads a device, decodes, applies quirks and serves snapshot / analyzer-JSON / NMEA 0183 / AIS / raw ports over TCP |
| `tui` | a `top`-like terminal browser for a live bus or a capture |
| `format-message` | build one frame from `FIELD=VALUE` pairs, for any PGN |
| `replay`, `n2kd` | pace a capture at wall-clock rhythm; multiplex analyzer JSON to TCP clients |

Pre-built binaries for Linux (static musl, x86_64 / aarch64 / armv7), macOS
and Windows are on the
[releases page](https://github.com/canboat/canboat/releases).

## The library

```toml
[dependencies]
canboat = { version = "8", default-features = false, features = ["decode"] }
```

The public API is deliberately small: think in terms of a `Frame` (a raw PGN
on the wire) and a `DecodedPgn`. Fields are addressed by generated,
compile-checked constants — a typo, or a field from the wrong PGN, fails the
build instead of erroring at runtime.

```rust
use canboat::{Database, EncodeValue, FieldValue, Units};
use canboat::ids::field::wind_data as wd;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let db = Database::embedded(Units::Metric);

    // Encode PGN 130306 "Wind Data": 5.23 m/s at 1.5 rad, apparent wind.
    let frame = db
        .encode("windData")?
        .push(wd::WIND_SPEED, 5.23)?
        .push(wd::WIND_ANGLE, 1.5)?
        .push(wd::REFERENCE, EncodeValue::Lookup("Apparent".into()))?
        .build()?;

    // Decode it back — the read side mirrors the write side.
    let decoded = db.decode(&frame).expect("valid frame decodes");
    match &decoded.field(wd::REFERENCE).unwrap().value {
        FieldValue::Lookup { name, .. } => assert_eq!(*name, Some("Apparent")),
        other => panic!("unexpected: {other:?}"),
    }
    Ok(())
}
```

Features enable you to choose what you want to do and not pay for (in terms of binary size and dependencies) what you do not need.

| feature | |
| --- | --- |
| `decode` | the baseline: schema, decode, encode, JSON output. No threads, no sockets, no I/O |
| `io` | readers for capture files, and `bus::open_ngt1` / `open_ikonvert` / `open_socketcan` for a live link |
| `node` | be a compliant NMEA 2000 node without owning a transport: ISO NAME, address claim, the standard responses |
| `bridge` | the whole `canboat server` pipeline as a library: `Bridge` owns the bus, hands you a `Receiver<DecodedPgn>`, and optionally re-serves the TCP ports |
| `nmea0183`, `ais`, `json-input` | output and input formatter sub-features |
| `cli` | the binary (the default; library users turn it off) |

The full API reference is on [docs.rs](https://docs.rs/canboat).

## Where the schema comes from

The tables are generated from the YAML database in the CANboat repository
by its `keel` tool — the same source the C `analyzer` and
[canboat.json](https://canboat.github.io/canboat) are generated from — and
are committed into this crate, so the published crate is self-contained. To
change a PGN, edit the database upstream by forking the `canboat` github
repository and running `keel`, followed by a PR back upstream, not this crate.

## License

Apache-2.0. (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
