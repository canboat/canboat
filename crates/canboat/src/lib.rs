// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat` — NMEA 2000 (canboat) decoding, encoding, and live-bus access.
//!
//! This is the single crate an external consumer depends on. Its public
//! surface is deliberately small and feature-gated; think in terms of
//! [`Frame`] ⇆ [`DecodedPgn`] (decode) and [`PgnBuilder`] → [`Frame`]
//! (encode). See `docs/library-api-plan.md` for the full design.
//!
//! canboat is the fruit of reverse-engineering NMEA 2000 — a closed,
//! pay-to-read standard — out on the water since 2009. This crate embeds the
//! whole distilled result: **nearly 600 PGN message definitions** across some
//! 350 PGNs — over 220 standardized, plus 370-odd *company-specific* variants
//! spanning ~25 manufacturers (Simrad, Furuno, Garmin, Raymarine, B&G, …) —
//! and it both decodes **and** encodes every one of them.
//!
//! # Quick start
//!
//! Everything below is in the baseline `decode` feature — no I/O, no threads.
//! The pivot is [`Frame`] (a raw PGN on the wire) and [`Database`] (the
//! compiled canboat schema). Fields are addressed by a generated **`FieldId`
//! constant** ([`ids::field`]) — the recommended path on both sides:
//! [`PgnBuilder::push`] to encode, [`DecodedPgn::field`] to read back. It is
//! `O(1)` (a single array index by the field's schema position) and
//! compile-checked: a typo, or a field from the wrong PGN, fails the build
//! instead of erroring at runtime.
//!
//! ```
//! use canboat::{Database, EncodeValue, FieldValue, Units};
//! use canboat::ids::field::wind_data as wd;
//!
//! let db = Database::embedded(Units::Metric);
//!
//! // Encode PGN 130306 "Wind Data": 5.23 m/s at 1.5 rad, apparent wind.
//! // Unset fields (and proprietary manufacturer/industry selectors) fall
//! // back to their schema defaults.
//! let frame: canboat::Frame = db
//!     .encode("windData")?
//!     .push(wd::WIND_SPEED, 5.23)?
//!     .push(wd::WIND_ANGLE, 1.5)?
//!     .push(wd::REFERENCE, EncodeValue::Lookup("Apparent".into()))?
//!     .build()?;
//! assert_eq!(frame.pgn, 130306);
//!
//! // Decode it back — the read side mirrors the write side.
//! let decoded = db.decode(&frame).expect("valid frame decodes");
//! assert_eq!(decoded.id, "windData");
//! match &decoded.field(wd::REFERENCE).unwrap().value {
//!     FieldValue::Lookup { name, .. } => assert_eq!(*name, Some("Apparent")),
//!     other => panic!("unexpected: {other:?}"),
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! No constant to hand (the field is chosen at runtime — a config-driven
//! encoder, a CLI arg)? Use the `*_by_name` twins,
//! [`PgnBuilder::push_by_name`] and [`DecodedPgn::field_by_name`] — an `O(n)`
//! scan by schema name, resolved at runtime rather than compile time:
//!
//! ```
//! # use canboat::{Database, EncodeValue, Units};
//! # let db = Database::embedded(Units::Metric);
//! let frame = db
//!     .encode("windData")?
//!     .push_by_name("Wind Speed", 5.23)?
//!     .push_by_name("Reference", EncodeValue::Lookup("Apparent".into()))?
//!     .build()?;
//! assert_eq!(db.decode(&frame).unwrap().field_by_name("Wind Speed").is_some(), true);
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! ## Lookup (enum) fields
//!
//! A `LOOKUP` field carries a raw integer that maps to a label. Encode it by
//! **label** ([`EncodeValue::Lookup`], shown above) or by **raw value**
//! ([`EncodeValue::Int`]); on decode, [`FieldValue::Lookup`] gives you both
//! the number and its resolved name, so you can match on whichever you have:
//!
//! ```
//! # use canboat::{Database, EncodeValue, FieldValue, Units};
//! # use canboat::ids::field::wind_data as wd;
//! # let db = Database::embedded(Units::Metric);
//! // Reference = 2 is "Apparent" — set the enum's raw value directly.
//! let frame = db.encode("windData")?.push(wd::REFERENCE, EncodeValue::Int(2))?.build()?;
//!
//! let decoded = db.decode(&frame).unwrap();
//! match &decoded.field(wd::REFERENCE).unwrap().value {
//!     FieldValue::Lookup { value, name } => {
//!         assert_eq!(*value, 2);                 // the raw enum value
//!         assert_eq!(*name, Some("Apparent"));   // its schema label, if known
//!     }
//!     _ => unreachable!(),
//! }
//! # Ok::<(), Box<dyn std::error::Error>>(())
//! ```
//!
//! For a live bus, add the `io` feature: `bus::open_ngt1` / `bus::open_ikonvert`
//! / `bus::open_socketcan` (Linux) open a gateway and hand back a bidirectional
//! `bus::DeviceHandle` (a [`read::FrameSource`] plus a `bus::FrameSender`). To
//! own the bus with the address-claim, quirks, and the 2597–2606 TCP serving
//! layer already assembled, add the `bridge` feature and drive a
//! `bridge::Bridge` instead.
//!
//! # Features
//!
//! * `decode` (baseline) — sans-I/O core: schema, decode, encode, formatters,
//!   and `codec`: the NGT-1 / iKonvert / Maretron wire protocols without a port.
//!   No threads, no async, no sockets.
//! * `io` — byte-source readers (`read::*Reader`) for files / text, plus
//!   `bus::open_*` to read+write a live NGT-1 / iKonvert / SocketCAN link.
//! * `node` — `device`: be a compliant N2K node (address claim + responder).
//! * `bridge` — `bridge`: the live CAN bus, fully assembled.
//! * `nmea0183`, `ais` — output-formatter sub-features.
//!
//! The `cli` feature (the package default) builds the `canboat` binary and is
//! not part of this library's public contract. Library consumers select
//! `default-features = false` plus the features they need.
#![cfg_attr(feature = "decode", deny(missing_docs))]
// The public items are re-exported from private modules whose docs
// naturally cross-reference their private neighbours; rustdoc renders such
// links as plain code, which is the right thing for the facade's readers.
// Genuinely broken links stay fatal (`rustdoc::broken_intra_doc_links`).
#![allow(rustdoc::private_intra_doc_links)]

// ───────────────────────── implementation (private) ─────────────────────────
// Everything below the facade lives in private modules: the compiler, not
// convention, keeps it off the public surface. Two lints are relaxed for
// them:
//
// * `missing_docs` applies at the definition site, so it would otherwise
//   fire on every field and method of a re-exported type.
// * `dead_code` / `unused_imports` are only meaningful under the full
//   feature set: a `decode`-only build legitimately leaves the format
//   parsers, text formatter and snapshot cache unreachable, since only the
//   `io` / `bridge` / `cli` layers call them. The default (`cli`) build,
//   which CI's clippy job checks, still enforces both.

/// Build-time version + git commit for the analyzer JSON banner.
#[cfg(feature = "bridge")]
#[allow(missing_docs)]
#[cfg_attr(not(feature = "cli"), allow(dead_code, unused_imports))]
mod build_info;
/// The `canboat` command-line tool (subcommands and their plumbing).
#[cfg(feature = "cli")]
#[allow(missing_docs)]
mod cli;
/// The sans-I/O engine: schema tables, decode, encode, formatters.
#[cfg(feature = "decode")]
#[allow(missing_docs)]
#[cfg_attr(not(feature = "cli"), allow(dead_code, unused_imports))]
mod engine;
/// Sync I/O adapters and device drivers (`io`), plus the transport-free
/// node pieces (`node`).
#[cfg(any(feature = "io", feature = "node"))]
#[allow(missing_docs)]
#[cfg_attr(not(feature = "cli"), allow(dead_code, unused_imports))]
mod io;
/// The n2kd daemon: TCP serving, NMEA 0183 / AIS conversion, request engine.
#[cfg(feature = "bridge")]
#[allow(missing_docs)]
#[cfg_attr(not(feature = "cli"), allow(dead_code, unused_imports))]
mod n2kd;
/// The live-bus pipeline: device → decode → quirks → serving.
#[cfg(feature = "bridge")]
#[allow(missing_docs)]
#[cfg_attr(not(feature = "cli"), allow(dead_code, unused_imports))]
mod server;

/// Entry point of the `canboat` binary; `src/main.rs` is a one-line call
/// to this. Not part of the library contract.
#[cfg(feature = "cli")]
#[doc(hidden)]
pub fn cli_main() -> std::process::ExitCode {
    cli::app::main()
}

// ─────────────────────────── decode (baseline) ───────────────────────────
// Engine types, re-exported at the crate root under their locked public names.
// The internal engine names (PgnDatabase, RawFrame, FieldRef) are mapped to
// the public names here; FieldHandle was collapsed into FieldId in Phase 1.

#[cfg(feature = "decode")]
pub use crate::engine::{
    ADDR_GLOBAL,
    ADDR_NULL,
    CANBOAT_JSON_VERSION as CANBOAT_VERSION,
    DecodeError,
    DecodedField,
    // decode (inbound)
    DecodedPgn,
    EncodeError,
    EncodeValue,
    FASTPACKET_MAX_SIZE,
    // the single field-addressing type (FieldHandle was removed in Phase 1)
    FieldRef as FieldId,
    FieldValue,
    FramePacketType,
    // encode (outbound)
    PgnBuilder,
    // schema & database
    PgnDatabase as Database,
    RAWFRAME_MAX_SIZE as FRAME_MAX_SIZE,
    // frames — the wire-side pivot
    RawFrame as Frame,
    Reassembled,
    // fast-packet reassembly
    Reassembler,
    ReassemblyError,
    // versions / identity
    SCHEMA_HASH,
    Units,
};

/// Schema introspection: the read-facing shape of the PGN/field definitions,
/// for consumers that treat the database as metadata (codegen, UI forms).
#[cfg(feature = "decode")]
pub mod schema {
    pub use crate::engine::db::PgnVariants;
    pub use crate::engine::types::LookupFieldTypeValue;
    pub use crate::engine::{
        BitLookupTable, BitLookupValue, FieldInfo, FieldType, IndirectLookupTable,
        IndirectLookupValue, LookupTable, LookupValue, PacketType, PgnInfo,
    };
}

/// Generated compile-time identity constants: `ids::pgn::WIND_DATA`,
/// `ids::field::wind_data::WIND_ANGLE`. Resolve a field once, at build time.
#[cfg(feature = "decode")]
pub mod ids {
    pub use crate::engine::{field, pgn};
}

/// Turn a [`DecodedPgn`] back into bytes/text. `write_nmea0183` / `write_ais`
/// land here when the `nmea0183` / `ais` features are wired.
#[cfg(feature = "decode")]
pub mod output {
    pub use crate::engine::output::{CamelCase, JsonOptions, write_json};
}

/// Inbound: turn a byte source into a stream of [`DecodedPgn`].
///
/// [`FrameSource`](read::FrameSource) is the bring-your-own-transport seam —
/// implement it over your own CAN driver, then feed it to
/// [`Decoder`](read::Decoder) to get decoded records. Ready-made readers for
/// ASCII line formats, `.ebl` logs, and `.pcap`/`.nif` captures arrive with the
/// `io` feature. [`from_analyzer_json`](read::from_analyzer_json) rehydrates an
/// already-decoded analyzer-JSON line.
#[cfg(feature = "decode")]
pub mod read {
    /// The ASCII line formats `PlainReader` (feature `io`) understands; pass
    /// one to `PlainReader::with_format` to skip autodetection.
    pub use crate::engine::format::InputFormat;
    pub use crate::engine::json_to_decoded as from_analyzer_json;
    pub use crate::engine::{Decoder, FrameSource};

    /// A [`FrameSource`] over an Actisense `.ebl` binary log.
    #[cfg(feature = "io")]
    pub use crate::io::EblReader;
    /// A [`FrameSource`] over any canboat ASCII line format (PLAIN / FAST /
    /// Actisense / YDWG-02 / iKonvert): honours `# format=` headers, otherwise
    /// autodetects. Wrap a file, a stdin lock, or [`open_capture`].
    #[cfg(feature = "io")]
    pub use crate::io::LineFrameReader as PlainReader;

    /// Open a capture as a [`PlainReader`] ready for a [`Decoder`]: a plain
    /// PLAIN/FAST text log, or a `.pcap` / `.pcap.gz` / `.nif` container
    /// (auto-detected and unwrapped to its PLAIN payload).
    #[cfg(feature = "io")]
    pub fn open_capture(
        path: &std::path::Path,
    ) -> std::io::Result<PlainReader<Box<dyn std::io::BufRead>>> {
        let br: Box<dyn std::io::BufRead> = if crate::io::container::is_container(path) {
            crate::io::container::plain_reader(path, crate::io::container::Options::default())?
        } else {
            Box::new(std::io::BufReader::new(std::fs::File::open(path)?))
        };
        Ok(PlainReader::new(br))
    }
}

// ──────────────────────────────── bus ────────────────────────────────────

/// Talk to a live NMEA 2000 link directly — an Actisense **NGT-1**, a Digital
/// Yacht **iKonvert**, or a Linux **SocketCAN** interface — reading a stream of
/// frames while writing frames back, without the full `bridge` pipeline or its
/// TCP serving layer. This is the middle rung between bring-your-own
/// [`read::FrameSource`] and the fully-assembled `bridge::Bridge`.
///
/// Each opener spawns the device's reader/writer threads and returns a
/// [`DeviceHandle`](bus::DeviceHandle) that is bidirectional and composes with
/// the rest of the crate:
///
/// * **read** — `handle.frames_rx` is an `mpsc::Receiver<Frame>`, hence a
///   [`read::FrameSource`]; feed it to a [`read::Decoder`] for decoded records
///   (fast-packet reassembly included).
/// * **write** — [`frame_sender`](bus::DeviceHandle::frame_sender) hands out a
///   cloneable [`FrameSender`](bus::FrameSender); build frames with
///   [`Database::encode`] and
///   [`send_frame`](bus::DeviceHandle::send_frame) them onto the bus.
///
/// ```no_run
/// use canboat::{Database, Units};
/// use canboat::ids::field::iso_request;
///
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let db = Database::embedded(Units::Metric);
/// let link = canboat::bus::open_ngt1("/dev/ttyUSB0", 115_200)?;
///
/// // Write: ask every device for its Product Information (PGN 126996).
/// let tx = link.frame_sender();
/// let request = db
///     .encode("isoRequest")?
///     .destination(255)
///     .push(iso_request::PGN, 126_996u32)?
///     .build()?;
/// tx.send_frame(request)?;
///
/// // Read: decode the live stream.
/// for pgn in canboat::read::Decoder::new(link.frames_rx, db) {
///     let pgn = pgn?;
///     println!("{} from {}", pgn.pgn, pgn.src);
/// }
/// # Ok(())
/// # }
/// ```
#[cfg(feature = "io")]
pub mod bus {
    use std::io;

    pub use crate::io::device::{
        Closed, DeviceCloser, DeviceHandle, DeviceWriterGone, FrameSender,
    };

    /// Open an Actisense NGT-1 / NGT-1-USB on a serial port (typically
    /// `115_200` baud), speaking the Actisense binary protocol.
    pub fn open_ngt1(path: &str, baud: u32) -> io::Result<DeviceHandle> {
        let (reader, writer) = crate::io::open_serial_rw(path, baud)?;
        Ok(crate::io::device::ngt1::run(reader, writer))
    }

    /// Open a Digital Yacht iKonvert on a serial port (typically `230_400`
    /// baud). Runs the iKonvert init handshake in all-PGN receive mode.
    pub fn open_ikonvert(path: &str, baud: u32) -> io::Result<DeviceHandle> {
        let (reader, writer) = crate::io::open_serial_rw(path, baud)?;
        let config = crate::io::device::ikonvert::Config::default();
        Ok(crate::io::device::ikonvert::run(reader, writer, config))
    }

    /// Open a Linux SocketCAN interface (e.g. `"can0"`), claiming ISO source
    /// `address` (the ISO 11783-5 address-claim handshake runs in the
    /// background). SocketCAN is Linux only — on other platforms this returns
    /// an [`io::ErrorKind::Unsupported`] error.
    pub fn open_socketcan(iface: &str, address: u8) -> io::Result<DeviceHandle> {
        use std::sync::Arc;
        use std::sync::atomic::AtomicU8;
        let config = crate::io::device::socketcan::Config {
            address,
            ..Default::default()
        };
        crate::io::device::socketcan::run(iface, config, Arc::new(AtomicU8::new(address)))
    }
}

// ─────────────────────────────── codec ───────────────────────────────────

/// The gateways' wire protocols without the I/O: bytes in, [`Frame`]s out,
/// and back. For a transport the `bus` openers cannot reach — WebSerial or a
/// WebSocket in a browser, an async runtime, an embedded UART.
///
/// Each gateway is one stateful object implementing [`Codec`](codec::Codec)
/// — [`Ngt1`](codec::ngt1::Ngt1), [`Ikonvert`](codec::ikonvert::Ikonvert),
/// [`Maretron`](codec::maretron::Maretron) — that holds everything its
/// protocol needs: the partial message between reads, the init handshake,
/// the transmit list. It never touches a port, a thread or the clock; the
/// caller writes what it returns and passes the time in. The `io` feature's
/// `bus::open_*` openers are these same codecs on a pair of threads.
///
/// ```
/// use canboat::codec::{Codec, Event};
/// use canboat::codec::ikonvert::{Config, Ikonvert};
///
/// let mut gw = Ikonvert::new(Config::default());
/// let mut to_port: Vec<Vec<u8>> = vec![gw.open()]; // $PDGY,N2NET_OFFLINE
///
/// // Whatever the port delivers, fed in with the current time.
/// let now_ms = 1_780_082_164_826;
/// let mut events = Vec::new();
/// gw.receive(b"$PDGY,TEXT,Digital_Yacht_iKonvert\r\n", now_ms, &mut events);
/// for ev in events.drain(..) {
///     match ev {
///         Event::Frame(frame) => println!("PGN {} from {}", frame.pgn, frame.src),
///         Event::Send(bytes) => to_port.push(bytes), // the next handshake step
///         Event::Error(e) => eprintln!("{e}"),
///     }
/// }
/// assert_eq!(to_port[1], b"$PDGY,N2NET_RESET\r\n");
///
/// // Closing takes the iKonvert off the bus.
/// to_port.push(gw.close());
/// ```
///
/// To speak raw CAN over a byte pipe instead (SLCAN, candump over a socket),
/// [`can_id`](codec::can_id) maps the 29-bit identifier and
/// [`fastpacket`](codec::fastpacket) splits an outgoing message into its
/// CAN frames; [`Reassembler`] joins incoming ones.
#[cfg(feature = "decode")]
pub mod codec {
    pub use crate::engine::codec::{Codec, Event, Refused};
    /// The PGNs a gateway is told the application sends and reads.
    pub use crate::engine::pgn_list::PgnLists;

    /// Actisense NGT-1 (Actisense binary protocol, typically 115 200 baud).
    pub mod ngt1 {
        pub use crate::engine::codec::ngt1::{Config, KEEPALIVE_INTERVAL, Ngt1};
        pub use crate::engine::codec::ngt1_tx_list::TxListRecord;
        /// What an NGT-1 sends for a frame it received — to simulate one.
        /// `None` for more than 244 bytes of data.
        pub use crate::engine::format::ngt1::encode_n2k_received_frame as encode_received;
    }

    /// Digital Yacht iKonvert (`$PDGY` / `!PDGY` ASCII, typically 230 400
    /// baud).
    pub mod ikonvert {
        pub use crate::engine::codec::ikonvert::{Config, Ikonvert};
    }

    /// Maretron IPG100 / IPG200 (TCP).
    pub mod maretron {
        pub use crate::engine::codec::maretron::{Config, Maretron};
    }

    /// The 29-bit ISO 11783 CAN identifier ⇄ priority, PGN, source and
    /// destination.
    pub mod can_id {
        /// `(prio, pgn, src, dst)` → CAN identifier. For a PDU2 PGN the
        /// destination is part of the PGN and `dst` is ignored.
        pub use crate::engine::format::iso11783_compose as compose;
        /// CAN identifier → `(prio, pgn, src, dst)`; `dst` is 255 for a
        /// PDU2 PGN.
        pub use crate::engine::format::iso11783_decompose as decompose;
    }

    /// Splitting an outgoing message into CAN frames.
    pub mod fastpacket {
        pub use crate::engine::fastpacket::{fragment, packet_type};
    }

    /// The text line dialects, one [`Frame`](crate::Frame) per line: canboat
    /// PLAIN / FAST, Actisense N2K ASCII, YDWG-02 RAW, candump, iKonvert
    /// `!PDGY`, Airmar, Chetco, Garmin CSV. For a gateway that speaks one of
    /// them over a socket, or a capture read without the `io` feature.
    pub mod line {
        /// Which dialect a line is in; the same type as `read::InputFormat`.
        pub use crate::engine::format::InputFormat;
        /// Why a line did not parse.
        pub use crate::engine::format::PlainError as ParseError;
        /// Frame → an Actisense N2K ASCII (W2K-1) line.
        pub use crate::engine::format::actisense_ascii::write_line as write_actisense_ascii;
        /// The dialect of a line, or `None` when nothing matches (canboat
        /// then reads it as PLAIN).
        pub use crate::engine::format::detect;
        /// One line in the given dialect → a frame; `Ok(None)` for a line
        /// that carries no frame (an iKonvert control sentence).
        pub use crate::engine::format::parse_with as parse;
        /// Frame → a canboat PLAIN line.
        pub use crate::engine::format::plain::write_line as write_plain;
    }
}

// ─────────────────────────────── device ──────────────────────────────────

/// Be a compliant N2K node without owning a transport, all [`Frame`]-in /
/// [`Frame`]-out — the caller owns the bus.
///
/// [`Name`](device::Name) builds the 64-bit ISO NAME; [`Claimer`](device::Claimer)
/// is the address-claim state machine (NAME arbitration). The response builders —
/// [`ProductInfo`](device::ProductInfo) (PGN 126996) and the
/// [`pgn_list_frames`](device::pgn_list_frames) /
/// [`iso_ack_frame`](device::iso_ack_frame) /
/// [`heartbeat_frame`](device::heartbeat_frame) helpers — emit the frames a node
/// answers discovery with; the caller drives them from its own event loop (as the
/// socketcan gateway and the motion quirk both do).
#[cfg(feature = "node")]
pub mod device {
    pub use crate::engine::pgn_list::{MAX_PGN_LIST_LEN, PgnListStatus, PgnListSupport, PgnLists};
    pub use crate::io::address_claim::{AddressClaim as Claimer, ClaimState};
    pub use crate::io::name::Name;
    pub use crate::io::nmea_responder::{
        ProductInfo, heartbeat_frame, iso_ack_frame, pgn_list_frames,
    };
}

// ─────────────────────────────── bridge ──────────────────────────────────

/// The live CAN bus, fully assembled: own a socketcan interface (or another
/// [`FrameSource`](read) via a device backend), claim an address, apply
/// quirks, and expose an in-process [`DecodedPgn`] stream plus an optional
/// TCP serving layer (the 2597–2606 ports the `canboat server` daemon opens).
///
/// [`Bridge::new`](bridge::Bridge::new) builds the core;
/// [`Bridge::decoded`](bridge::Bridge::decoded) taps the decoded stream,
/// [`Bridge::serve`](bridge::Bridge::serve) opens the TCP ports, and
/// [`Bridge::spawn`](bridge::Bridge::spawn) (or the blocking
/// [`Bridge::run`](bridge::Bridge::run)) starts the bus.
/// [`BridgeConfig`](bridge::BridgeConfig) is the plain, clap-free config;
/// [`Quirk`](bridge::Quirk) selects a bus value-add.
///
/// **Backend selection.** The bus source is whichever
/// [`BridgeConfig`](bridge::BridgeConfig) field is set, in precedence order:
/// `actisense` (an Actisense NGT-1 serial gateway),
/// `ikonvert` (a Digital Yacht iKonvert), `maretron`, `canboat_csv`, then
/// `socketcan` — each a device path or interface name, with `baud` setting the
/// serial rate. So to run off an NGT-1 instead of SocketCAN, set
/// `config.actisense = Some("/dev/ttyUSB0".into())` (baud `Some(115_200)`) and
/// leave `socketcan` `None`:
///
/// ```
/// # use canboat::bridge::BridgeConfig;
/// let config = BridgeConfig {
///     actisense: Some("/dev/ttyUSB0".into()), // NGT-1 instead of socketcan
///     baud: Some(115_200),
///     ..BridgeConfig::default()
/// };
/// ```
///
/// The `scx20` and `motion` quirks need SocketCAN specifically — they preserve
/// a frame's source address, which the serial gateways rewrite on write — so
/// [`Bridge::new`](bridge::Bridge::new) rejects them on other backends; `wmm`
/// works on any writable backend.
///
/// ```no_run
/// use canboat::bridge::{Bridge, BridgeConfig};
///
/// # fn main() -> anyhow::Result<()> {
/// // Own a SocketCAN interface, tap the decoded stream, and re-serve the
/// // 2597–2606 TCP ports so other LAN consumers keep working.
/// let mut config = BridgeConfig::default();
/// config.socketcan = Some("can0".into());
///
/// let mut bridge = Bridge::new(config)?;
/// let decoded = bridge.decoded(); // Receiver<Arc<DecodedPgn>>
/// bridge.serve()?;
/// bridge.spawn()?; // pipeline runs on a background thread
///
/// for pgn in decoded.iter() {
///     println!("{} from {}", pgn.pgn, pgn.src);
/// }
/// bridge.wait();
/// # Ok(())
/// # }
/// ```
#[cfg(feature = "bridge")]
pub mod bridge {
    /// [`BridgeConfig::pgn_lists`] and [`Bridge::pgn_list_status`]'s types.
    pub use crate::engine::pgn_list::{PgnListStatus, PgnListSupport, PgnLists};
    /// Which dates [`Quirk::GpsRollover`] corrects, and how a device on
    /// that list is named. Both parse from the `gps-rollover=…` CLI syntax.
    pub use crate::engine::quirk::{Device as GpsRolloverDevice, Target as GpsRolloverTarget};
    pub use crate::server::{Bridge, BridgeConfig, QuirkKind as Quirk, Transmitter};
}

// ───────────────────────────── json input ────────────────────────────────

/// Analyzer/canboatjs JSON records → wire frames (`convert --from json`,
/// the gateway bridges' stdin TX, and the wasm bindings). Behind its own
/// feature because it is the one place a real JSON parser (serde_json)
/// is warranted.
#[cfg(feature = "json-input")]
pub mod json_input;

// ─────────────────────────────── prelude ─────────────────────────────────

/// The 90% path in one glob import: `use canboat::prelude::*;`.
#[cfg(feature = "decode")]
pub mod prelude {
    pub use crate::{
        Database, DecodeError, DecodedField, DecodedPgn, FieldId, FieldValue, Frame, Units,
    };
}
