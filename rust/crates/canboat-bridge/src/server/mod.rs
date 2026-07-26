// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat-pipeline` — single-process N2K → NMEA 0183 pipeline.
//!
//! Combines the work that the C canboat stack normally splits across
//! three processes (a device reader like `actisense-serial`, the
//! `analyzer`, and `n2kd`) into one binary. `RawFrame`s flow between
//! stages as structs over mpsc channels — no PLAIN/FAST text
//! serialization between stages — so the formatter/parser tax pays
//! out only at the human-readable boundaries (NMEA 0183 stdout and
//! the optional CSV TCP server).
//!
//! Input modes:
//!
//! * **stdin** (no `--actisense` / `--ikonvert` / `--maretron`): the
//!   binary parses PLAIN/FAST lines off stdin. Compatible with
//!   `analyzer < file.raw | canboat-pipeline` style pipelines that used
//!   to exist with `n2kd-inproc`.
//! * **device**: a [`canboat_io::device`] reader stage opens the
//!   chosen serial port / TCP socket and emits `RawFrame`s
//!   directly. In this mode, the write-only input port and the lazy
//!   output TCP servers are also wired up.
//!
//! Output: NMEA 0183 sentences on stdout.
//!
//! ## TCP port layout
//!
//! Base-relative offsets match canboat C `n2kd` (and the `n2kd`
//! crate) one-for-one, so no port number ever means two different
//! things across the three implementations. Every output stream is
//! read-only; the single write path is the input port.
//!
//! | Port | Flag | Dir | Purpose |
//! |------|------|-----|---------|
//! | 2597 | `--snapshot-port`       | out | JSON snapshot per `(pgn, src)`, then close |
//! | 2598 | `--analyzer-port`       | out | analyzer JSON stream (read-only) |
//! | 2599 | `--nmea0183-port`       | out | NMEA 0183 stream |
//! | 2600 | `--input-port`          | in  | **write-only** raw PLAIN/FAST → bus |
//! | 2601 | `--ais-port`            | out | AIS snapshot, then close |
//! | 2602 | *(reserved)*            | —   | future status stream (n2kd `port+5`) |
//! | 2603 | `--raw-port`            | out | raw frame output stream |
//! | 2605 | `--nmea0183-filter-port`| **i/o** | PGN 262657 filter control (with `--nmea0183-filter`) |
//!
//! The lone exception to "every output is read-only" is 2605: the
//! filter control channel is request/response and needs to answer the
//! client on the same socket, so it is deliberately bidirectional. It
//! carries nothing but PGN 262657 and never touches the bus.

mod bridge;
mod pipeline;
mod quirks;
mod snapshot;
mod tcp;

pub use bridge::{Bridge, Transmitter};
pub use quirks::QuirkKind;

use std::io::{self, BufRead, BufReader, Read, Write};
use std::net::Ipv4Addr;
use std::path::PathBuf;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::mpsc;
use std::thread;

use anyhow::{Result, bail};

use canboat_core::RawFrame;
use canboat_core::format::{
    InputFormat, detect, header_implies_coalesced, parse_format_header, parse_with,
};
use canboat_io::device::{self, FrameSender, Supervisor};
use canboat_io::open_serial_rw;

/// Clap front-end for the `canboat server` CLI. Gated behind the `cli` feature
/// so the library path ([`BridgeConfig`] + [`run`]) stays clap-free; convert
/// with [`BridgeConfig::from`].
#[cfg(feature = "cli")]
#[derive(Debug, clap::Args)]
#[command(after_help = canboat_cli::help_footer())]
pub struct Args {
    /// Read frames from an Actisense NGT-1 / W2K-1 on this serial
    /// device path (e.g. `/dev/ttyUSB0`).
    #[arg(
        long,
        value_name = "DEVICE",
        conflicts_with_all = ["ikonvert", "maretron", "canboat_csv", "socketcan"]
    )]
    actisense: Option<String>,

    /// Read frames from a Digital Yacht iKonvert on this serial
    /// device path.
    #[arg(
        long,
        value_name = "DEVICE",
        conflicts_with_all = ["actisense", "maretron", "canboat_csv", "socketcan"]
    )]
    ikonvert: Option<String>,

    /// Read frames from a Maretron IPG100/200 over TCP. Accepts either
    /// `host:port` or `tcp://host[:port]`. Default port 6543 (bus 0).
    #[arg(
        long,
        value_name = "URL",
        conflicts_with_all = ["actisense", "ikonvert", "canboat_csv", "socketcan"]
    )]
    maretron: Option<String>,

    /// Read frames from a Linux SocketCAN interface (e.g. `can0`,
    /// `nmea2000`). The pipeline becomes a full NMEA 2000 bus
    /// participant: it claims an ISO address, answers ISO Requests and
    /// Group Functions, and sends Heartbeats. Linux-only.
    #[arg(
        long,
        value_name = "IFACE",
        conflicts_with_all = ["actisense", "ikonvert", "maretron", "canboat_csv"]
    )]
    socketcan: Option<String>,

    /// Preferred ISO source address to claim on the SocketCAN bus.
    /// Defaults to 0; the claim machine will pick a free address if
    /// this one is taken. Ignored without `--socketcan`.
    #[arg(
        long = "socketcan-address",
        value_name = "ADDR",
        default_value_t = 0,
        requires = "socketcan"
    )]
    socketcan_address: u8,

    /// Chain into another `canboat-pipeline` instance over its
    /// bidirectional Raw CSV port (default 2603). Accepts
    /// `host:port` or `tcp://host[:port]`. Wire format is
    /// canboat PLAIN/FAST CSV in both directions, so all frames
    /// and any client writes flow end-to-end.
    ///
    /// When `--canboat-csv-write` is also given, this URL is used
    /// only as a read source (e.g. an iptee'd raw stream), and
    /// outbound writes are diverted to the write URL instead.
    #[arg(
        long,
        value_name = "URL",
        conflicts_with_all = ["actisense", "ikonvert", "maretron", "socketcan"]
    )]
    canboat_csv: Option<String>,

    /// Optional separate sink for outbound PLAIN/FAST frames when
    /// chaining via `--canboat-csv`. Use this when the read source
    /// is a one-way feed (e.g. iptee on actisense-serial output)
    /// and writes need to go to a different endpoint (e.g. n2kd's
    /// input-stream port). Without this flag, reads and writes
    /// share the single `--canboat-csv` socket.
    #[arg(long, value_name = "URL", requires = "canboat_csv")]
    canboat_csv_write: Option<String>,

    /// Baud rate for serial devices. Defaults: 115200 for NGT-1,
    /// 230400 for iKonvert. Ignored for Maretron.
    #[arg(long, value_name = "BAUD")]
    baud: Option<u32>,

    /// Maretron login password (default: empty).
    #[arg(long, default_value = "")]
    maretron_password: String,

    /// iKonvert RX filter list (`<pgn>,<pgn>,...`). When set, init
    /// brings the device online in `NORMAL` mode and runs the
    /// `N2NET_RESET` + `RX_LIST` handshake steps.
    #[arg(long, value_name = "PGN,PGN,...")]
    ikonvert_rx: Option<String>,

    /// iKonvert TX filter list (`<pgn>,<pgn>,...`). Triggers the
    /// `N2NET_RESET` + `TX_LIST` handshake steps.
    #[arg(long, value_name = "PGN,PGN,...")]
    ikonvert_tx: Option<String>,

    /// Disable the iKonvert TX rate limit. Use at your own risk.
    #[arg(long)]
    ikonvert_rate_limit_off: bool,

    /// Suppress the periodic ISO Address Claim / Product Info request
    /// engine. By default it runs whenever a device writer is wired up
    /// (matching canboat C `n2kd`'s default). Stdin-only mode always
    /// skips the engine — there's nowhere to send the requests.
    #[arg(long)]
    no_request_claims: bool,

    /// Apply a quirk / value-add on the bus (see the possible values).
    /// Repeatable — pass `--quirk` once per quirk you want, e.g.
    /// `--quirk scx20 --quirk wmm`.
    ///
    /// `scx20` and `motion` impersonate/claim a device and need
    /// `--socketcan`; `wmm` emits canboat's own PGN and works with any
    /// writable backend (socketcan / NGT-1 / iKonvert).
    #[arg(long, value_enum, value_name = "NAME")]
    quirk: Vec<quirks::QuirkKind>,

    /// Bind address for all TCP listeners. Defaults to `0.0.0.0` so
    /// clients on the LAN (chartplotters, OpenCPN, etc.) can
    /// connect. Pass `127.0.0.1` to restrict access to the local
    /// host.
    #[arg(long, default_value = "0.0.0.0")]
    bind: Ipv4Addr,

    /// Port for the snapshot server — on connect, dumps the latest
    /// analyzer JSON line per `(pgn, src, secondary)` then closes.
    /// Matches canboat C `n2kd`'s base port (`-p`). AIS-described
    /// PGNs are filtered out; they go to `--ais-port` instead, except
    /// PGN 129026 and 129029 which appear in both. Enabling this
    /// forces JSON serialization for every decoded record so the
    /// cache stays current; disable with `0` if you don't need it.
    #[arg(long, default_value_t = 2597)]
    snapshot_port: u16,

    /// Port for the analyzer JSON server — one decoded PGN per line.
    /// Matches canboat C `n2kd`'s `port+1` stream port: read-only.
    /// Injection goes through `--input-port` instead; this port is
    /// kept free for a future "analyzed write" feature that would
    /// accept JSON here. Lazy: formatting is skipped when no client is
    /// subscribed. `0` disables.
    #[arg(long, default_value_t = 2598)]
    analyzer_port: u16,

    /// Port for the read-only NMEA 0183 server (includes AIVDM).
    /// Matches canboat C `n2kd`'s `port+2` NMEA 0183 port. `0`
    /// disables.
    #[arg(long, default_value_t = 2599)]
    nmea0183_port: u16,

    /// Write-only raw input port — the canonical `SERVER_INPUT_STREAM`
    /// slot (canboat C `n2kd` `port+3`). Clients connect and write
    /// canboat PLAIN/FAST lines that are injected onto the bus
    /// (device mode only); nothing is streamed back, so it never
    /// forces JSON/CSV serialization. This is the cheap write path for
    /// a consumer that reads its data on another port. `0` disables.
    #[arg(long, default_value_t = 2600)]
    input_port: u16,

    /// Read-only raw N2K output stream. Clients receive every frame as
    /// a `# format=FAST` PLAIN line (already-coalesced fast-packet
    /// payloads — never the per-CAN-frame PLAIN format). Reading is
    /// lazy (skipped with no client). Injection is NOT accepted here —
    /// use `--input-port`. `0` disables. (Previously this port was
    /// bidirectional at 2600 under `--csv-port` / `--raw-input-port`;
    /// the write side moved to `--input-port` and the read side moved
    /// here to keep the input slot write-only.)
    #[arg(long, default_value_t = 2603)]
    raw_port: u16,

    /// Port for the AIS snapshot server — on connect, dumps the
    /// latest analyzer JSON line for every AIS-described PGN (plus
    /// PGN 129026 and 129029) then closes. Matches canboat C
    /// `n2kd`'s `port+4` AIS port. `0` disables.
    #[arg(long, default_value_t = 2601)]
    ais_port: u16,

    /// Also write NMEA 0183 sentences (including AIVDM) to stdout —
    /// mirrors canboat C `n2kd`'s `--nmea0183` flag. Off by default,
    /// matching n2kd's TCP-multiplex behaviour; subscribers should
    /// connect to `--nmea0183-port` (2599) instead.
    #[arg(long)]
    nmea0183_stdout: bool,

    /// Disable the 1 Hz NMEA 0183 rate limit. By default each
    /// `(source, quantity)` is limited to one sentence per second on
    /// the NMEA 0183 outputs (matching canboat C `n2kd`), so a bus
    /// with several devices reporting the same measurement doesn't
    /// flood downstream 0183 consumers. AIS (`!AI…`) is never rate-
    /// limited. Pass this to emit every converted sentence unthrottled
    /// (e.g. for byte-for-byte parity captures).
    #[arg(long = "no-nmea0183-rate-limit")]
    no_nmea0183_rate_limit: bool,

    /// Override the per-device NMEA 0183 filter mute-rules file. The
    /// filter is **always on**, reading `nmea0183-filter.json` from the
    /// config dir (see `--config-dir`) by default; a rule-less file is a
    /// no-op (every converted sentence is emitted). Once ≥1 mute rule
    /// exists, the 0183 outputs only carry sentences from devices whose
    /// NAME (PGN 60928) the pipeline has learned and that aren't muted.
    /// The N2K bus is never affected. Use this to point at a specific
    /// file instead of the config dir's.
    #[arg(long = "nmea0183-filter", value_name = "PATH")]
    nmea0183_filter: Option<PathBuf>,

    /// Directory holding the server's persistent state files —
    /// `overrides.json` (PGN-rate overrides) and `nmea0183-filter.json`
    /// (0183 mute rules). Both features are on by default. When unset the
    /// dir is resolved automatically: `/etc/default/canboat` if writable
    /// (the systemd/root case), otherwise `$HOME/.local/canboat`.
    #[arg(long = "config-dir", value_name = "DIR")]
    config_dir: Option<PathBuf>,

    /// Port for the bidirectional NMEA 0183 filter control channel (PGN
    /// 262657). This is the one write-capable output port: the TUI
    /// connects, is sent the current filter state, and thereafter writes
    /// `Set` / `Request` frames and reads `Report` frames on the same
    /// socket. Kept off the read-only analyzer stream so no other
    /// consumer ever sees the control PGN. Only opened when
    /// `--nmea0183-filter` is set (there's nothing to control otherwise);
    /// `0` disables. Slot 2605.
    #[arg(long, default_value_t = 2605)]
    nmea0183_filter_port: u16,

    /// Port for the bidirectional PGN-rate-override control channel (PGN
    /// 262658). Like the filter port: the TUI connects, receives the
    /// current override state, then writes `Set` / `Delete` / `Request`
    /// frames and reads `Report` frames on the same socket. `0` disables.
    /// Slot 2606, after the filter control port.
    #[arg(long, default_value_t = 2606)]
    overrides_port: u16,

    /// Emit field keys + PGN descriptions as camelCase
    /// identifiers (`"uniqueNumber"` instead of `"Unique Number"`)
    /// on the analyzer JSON / snapshot TCP ports. Matches canboat
    /// C's `-camel`.
    #[arg(long, conflicts_with = "upper_camel")]
    camel: bool,

    /// Same as `--camel` but UpperCamelCase (`"UniqueNumber"`).
    /// Matches canboat C's `-upper-camel`.
    #[arg(long = "upper-camel")]
    upper_camel: bool,

    /// Decode numeric fields into strict SI base units (`rad`, `K`,
    /// `Pa`, …) instead of canboat's humanized Metric (`deg`, `°C`,
    /// `bar`, …) on the analyzer JSON / snapshot TCP ports. Matches
    /// canboat C's `-si`. Orthogonal to `--camel`. NMEA 0183 / AIS
    /// output is unaffected — those always emit their spec units.
    #[arg(long)]
    si: bool,

    /// Verbose logging.
    #[arg(short = 'v', long)]
    verbose: bool,

    /// Quiet — only errors.
    #[arg(short = 'q', long)]
    quiet: bool,
}

/// Plain, clap-free configuration for the pipeline [`run`]. Construct with
/// [`BridgeConfig::default`] and set the fields you need — or, under the `cli`
/// feature, `BridgeConfig::from(args)`. Field meanings mirror the `canboat
/// server` flags one-for-one.
#[derive(Debug, Clone)]
pub struct BridgeConfig {
    pub actisense: Option<String>,
    pub ikonvert: Option<String>,
    pub maretron: Option<String>,
    pub socketcan: Option<String>,
    pub socketcan_address: u8,
    pub canboat_csv: Option<String>,
    pub canboat_csv_write: Option<String>,
    pub baud: Option<u32>,
    pub maretron_password: String,
    pub ikonvert_rx: Option<String>,
    pub ikonvert_tx: Option<String>,
    pub ikonvert_rate_limit_off: bool,
    pub no_request_claims: bool,
    pub quirk: Vec<quirks::QuirkKind>,
    pub bind: Ipv4Addr,
    pub snapshot_port: u16,
    pub analyzer_port: u16,
    pub nmea0183_port: u16,
    pub input_port: u16,
    pub raw_port: u16,
    pub ais_port: u16,
    pub nmea0183_stdout: bool,
    pub no_nmea0183_rate_limit: bool,
    pub nmea0183_filter: Option<PathBuf>,
    /// Directory holding the persistent state files (`overrides.json`,
    /// `nmea0183-filter.json`). Unlike the CLI flag, the library does **not**
    /// auto-discover a dir: `Some(path)` uses it, `None` disables persistence
    /// (no overrides; no filter unless `nmea0183_filter` gives an explicit
    /// path). The `canboat` binary resolves the systemd/home default and
    /// passes an explicit path here.
    pub config_dir: Option<PathBuf>,
    pub nmea0183_filter_port: u16,
    pub overrides_port: u16,
    pub camel: bool,
    pub upper_camel: bool,
    pub si: bool,
    pub verbose: bool,
    pub quiet: bool,
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            actisense: None,
            ikonvert: None,
            maretron: None,
            socketcan: None,
            socketcan_address: 0,
            canboat_csv: None,
            canboat_csv_write: None,
            baud: None,
            maretron_password: String::new(),
            ikonvert_rx: None,
            ikonvert_tx: None,
            ikonvert_rate_limit_off: false,
            no_request_claims: false,
            quirk: Vec::new(),
            bind: Ipv4Addr::new(0, 0, 0, 0),
            snapshot_port: 2597,
            analyzer_port: 2598,
            nmea0183_port: 2599,
            input_port: 2600,
            raw_port: 2603,
            ais_port: 2601,
            nmea0183_stdout: false,
            no_nmea0183_rate_limit: false,
            nmea0183_filter: None,
            config_dir: None,
            nmea0183_filter_port: 2605,
            overrides_port: 2606,
            camel: false,
            upper_camel: false,
            si: false,
            verbose: false,
            quiet: false,
        }
    }
}

#[cfg(feature = "cli")]
impl From<Args> for BridgeConfig {
    fn from(a: Args) -> Self {
        Self {
            actisense: a.actisense,
            ikonvert: a.ikonvert,
            maretron: a.maretron,
            socketcan: a.socketcan,
            socketcan_address: a.socketcan_address,
            canboat_csv: a.canboat_csv,
            canboat_csv_write: a.canboat_csv_write,
            baud: a.baud,
            maretron_password: a.maretron_password,
            ikonvert_rx: a.ikonvert_rx,
            ikonvert_tx: a.ikonvert_tx,
            ikonvert_rate_limit_off: a.ikonvert_rate_limit_off,
            no_request_claims: a.no_request_claims,
            quirk: a.quirk,
            bind: a.bind,
            snapshot_port: a.snapshot_port,
            analyzer_port: a.analyzer_port,
            nmea0183_port: a.nmea0183_port,
            input_port: a.input_port,
            raw_port: a.raw_port,
            ais_port: a.ais_port,
            nmea0183_stdout: a.nmea0183_stdout,
            no_nmea0183_rate_limit: a.no_nmea0183_rate_limit,
            nmea0183_filter: a.nmea0183_filter,
            config_dir: a.config_dir,
            nmea0183_filter_port: a.nmea0183_filter_port,
            overrides_port: a.overrides_port,
            camel: a.camel,
            upper_camel: a.upper_camel,
            si: a.si,
            verbose: a.verbose,
            quiet: a.quiet,
        }
    }
}

/// Run the single-process pipeline to completion (blocks until the frame
/// source ends). Thin wrapper over [`Bridge`]: build the core, spawn the
/// TCP serving layer, then drive the pipeline in place — so the CLI
/// `canboat server` and an embedding library share one code path. The host
/// owns logger setup; this no longer initialises `env_logger`.
pub fn run(config: BridgeConfig) -> Result<()> {
    let mut bridge = Bridge::new(config)?;
    bridge.serve()?;
    bridge.run()
}

/// Open whichever input source the CLI selected. Returns
/// `(frames_rx, optional_supervisor, pre_coalesced)`.
///
/// * `pre_coalesced` is `true` when each `RawFrame` produced by the
///   source is already a complete PGN payload — the iKonvert and the
///   Maretron IPG do their own fast-packet coalescing on the wire,
///   so the pipeline must skip the reassembler for them. Sending
///   small coalesced payloads through the reassembler causes their
///   leading bytes to be misread as fast-packet headers.
/// * `supervisor` is `None` only in stdin mode; the caller skips
///   wiring the write-only TCP port and the supervisor shutdown.
///
/// In device mode the supervisor's manager thread handles reconnect
/// on disconnect/EOF with exponential backoff up to 30 s, so a flaky
/// serial port or TCP gateway doesn't take the pipeline down.
/// What `open_source` hands back to `run`. Separate struct (rather
/// than a return tuple) because the field count grew past what
/// clippy's `type_complexity` lint will tolerate.
struct OpenedSource {
    frames_rx: mpsc::Receiver<RawFrame>,
    supervisor: Option<Supervisor>,
    pre_coalesced: Arc<AtomicBool>,
    /// Live claim address of the device backend, when known (today
    /// only `--socketcan` exposes one). Read by the CSV-port
    /// injector to rewrite client-supplied default-`src` frames.
    claim_addr: Option<Arc<std::sync::atomic::AtomicU8>>,
}

// Config-dir *discovery* (the `/etc/default/canboat` vs `~/.local/canboat`
// filesystem probe) is deliberately NOT here: it's a binary concern, so it
// lives in the `canboat` bin's `server` handler. The library takes an
// explicit `BridgeConfig.config_dir` (or `None` to disable persistence).

fn open_source(config: &BridgeConfig) -> Result<OpenedSource> {
    if let Some(path) = config.actisense.as_deref() {
        let baud = config.baud.unwrap_or(115_200);
        let path = path.to_string();
        let factory = NamedFactory::new("ngt1", move || {
            let (reader, writer) = open_serial_rw(&path, baud)?;
            Ok(device::ngt1::run(reader, writer))
        });
        let sup = Supervisor::new(factory);
        let (rx, sup) = split_supervisor(sup);
        // NGT-1's `N2K_MSG_RECEIVED` (0x93) frames carry full PGN
        // payloads — fast-packet coalescing happens inside the
        // device, not on the wire we receive. Skip the reassembler.
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(sup),
            pre_coalesced: Arc::new(AtomicBool::new(true)),
            claim_addr: None,
        });
    }
    if let Some(path) = config.ikonvert.as_deref() {
        let baud = config.baud.unwrap_or(230_400);
        let path = path.to_string();
        let rx_list = config.ikonvert_rx.clone();
        let tx_list = config.ikonvert_tx.clone();
        let rate_limit_off = config.ikonvert_rate_limit_off;
        let factory = NamedFactory::new("ikonvert", move || {
            let (reader, writer) = open_serial_rw(&path, baud)?;
            let config = device::ikonvert::Config {
                rx_list: rx_list.clone(),
                tx_list: tx_list.clone(),
                rate_limit_off,
                ..Default::default()
            };
            Ok(device::ikonvert::run(reader, writer, config))
        });
        let sup = Supervisor::new(factory);
        let (rx, sup) = split_supervisor(sup);
        // iKonvert coalesces fast-packets internally; skip the
        // reassembler entirely.
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(sup),
            pre_coalesced: Arc::new(AtomicBool::new(true)),
            claim_addr: None,
        });
    }
    if let Some(url) = config.maretron.as_deref() {
        let url = url.to_string();
        let password = config.maretron_password.clone();
        let factory = NamedFactory::new("maretron", move || {
            let (reader, writer) = open_maretron_pair(&url)?;
            let cfg = device::maretron::Config {
                password: password.clone(),
                fixtime: None,
            };
            Ok(device::maretron::run(reader, writer, cfg))
        });
        let sup = Supervisor::new(factory);
        let (rx, sup) = split_supervisor(sup);
        // Maretron IPG ships full PGN payloads per 0xA5 frame.
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(sup),
            pre_coalesced: Arc::new(AtomicBool::new(true)),
            claim_addr: None,
        });
    }
    if let Some(read_url) = config.canboat_csv.as_deref() {
        let read_url = read_url.to_string();
        let write_url = config.canboat_csv_write.clone();
        let factory = NamedFactory::new("canboat-csv", move || {
            let (reader, writer) = match &write_url {
                Some(w) => open_canboat_csv_split(&read_url, w)?,
                None => open_canboat_csv_pair(&read_url)?,
            };
            Ok(device::canboat_csv::run(reader, writer))
        });
        let sup = Supervisor::new(factory);
        let (rx, sup) = split_supervisor(sup);
        // Wire format is canboat PLAIN/FAST — each line is already
        // a complete PGN payload, so skip the reassembler.
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(sup),
            pre_coalesced: Arc::new(AtomicBool::new(true)),
            claim_addr: None,
        });
    }
    if let Some(iface) = config.socketcan.as_deref() {
        let iface = iface.to_string();
        let config = device::socketcan::Config {
            address: config.socketcan_address,
            model_version: Some("canboat-pipeline-rs"),
            ..device::socketcan::Config::default()
        };
        // Shared across factory reconnects so the live claim address
        // survives supervisor-driven device-session restarts.
        let claim_addr = Arc::new(std::sync::atomic::AtomicU8::new(
            device::socketcan::CLAIM_UNCLAIMED,
        ));
        let claim_for_factory = Arc::clone(&claim_addr);
        let factory = NamedFactory::new("socketcan", move || {
            device::socketcan::run(&iface, config.clone(), Arc::clone(&claim_for_factory))
        });
        let sup = Supervisor::new(factory);
        let (rx, sup) = split_supervisor(sup);
        // The SocketCAN adapter reassembles internally (driven by the
        // `canboat-io::fastpacket` table) and hands us coalesced
        // `RawFrame`s, matching the NGT-1 / iKonvert contract — skip
        // the pipeline's own reassembler.
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(sup),
            pre_coalesced: Arc::new(AtomicBool::new(true)),
            claim_addr: Some(claim_addr),
        });
    }
    // stdin fallback — no device, no reconnect logic needed. We
    // can't know in advance whether the upstream uses PLAIN
    // (per-CAN-frame, needs reassembly) or FAST (already coalesced).
    // The stdin pump watches for `# format=<NAME>` headers and flips
    // the flag when it sees one declaring a coalesced format; the
    // pipeline itself also flips it on the first >8-byte payload.
    let pre_coalesced = Arc::new(AtomicBool::new(false));
    let (tx, rx) = mpsc::channel::<RawFrame>();
    let coalesced_for_pump = pre_coalesced.clone();
    thread::Builder::new()
        .name("stdin-pump".into())
        .spawn(move || stdin_pump(tx, coalesced_for_pump, None))
        .expect("spawn stdin-pump");
    Ok(OpenedSource {
        frames_rx: rx,
        supervisor: None,
        pre_coalesced,
        claim_addr: None,
    })
}

/// Swap the `Supervisor::frames_rx` out so the pipeline can own it
/// while we keep the supervisor for shutdown / frame_sender access.
fn split_supervisor(mut sup: Supervisor) -> (mpsc::Receiver<RawFrame>, Supervisor) {
    let (_dummy_tx, dummy_rx) = mpsc::channel::<RawFrame>();
    let rx = std::mem::replace(&mut sup.frames_rx, dummy_rx);
    (rx, sup)
}

/// A [`device::DeviceFactory`] that carries a static name and a
/// closure. We can't `impl DeviceFactory for FnMut()` directly because
/// the blanket name() default returns `"device"`; this wrapper plumbs
/// the device-kind label through to the supervisor's log lines.
struct NamedFactory<F> {
    name: &'static str,
    open: F,
}

impl<F> NamedFactory<F>
where
    F: FnMut() -> io::Result<device::DeviceHandle> + Send + 'static,
{
    fn new(name: &'static str, open: F) -> Self {
        Self { name, open }
    }
}

impl<F> device::DeviceFactory for NamedFactory<F>
where
    F: FnMut() -> io::Result<device::DeviceHandle> + Send + 'static,
{
    fn open(&mut self) -> io::Result<device::DeviceHandle> {
        (self.open)()
    }
    fn name(&self) -> &str {
        self.name
    }
}

fn open_maretron_pair(url: &str) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    open_tcp_pair(url, 6543, "maretron")
}

fn open_canboat_csv_pair(url: &str) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    open_tcp_pair(url, 2603, "canboat-csv")
}

/// Split-mode: read frames from `read_url`, send frames out to
/// `write_url`. Two independent TCP connections; the unused
/// directions of each are dropped (their kernel-side socket
/// references remain alive via the other half, but we never
/// touch them).
///
/// Use case: chaining into a pipeline whose read source is a
/// one-way feed (e.g. `iptee` mirroring the raw output of
/// `actisense-serial`) while the write sink is a separate
/// endpoint (e.g. `n2kd`'s input-stream port on `port + 3`).
fn open_canboat_csv_split(
    read_url: &str,
    write_url: &str,
) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    let (read_half, _unused_write) = open_tcp_pair(read_url, 2603, "canboat-csv read")?;
    let (_unused_read, write_half) = open_tcp_pair(write_url, 2603, "canboat-csv write")?;
    Ok((read_half, write_half))
}

/// Open a TCP socket and return cloned read/write halves. Mirrors
/// what the device codecs need: a separate `Box<dyn Read>` and
/// `Box<dyn Write>` over the same underlying connection. Tolerates
/// `tcp://` URL prefix, fills in `default_port` when the user
/// didn't include one.
fn open_tcp_pair(
    url: &str,
    default_port: u16,
    log_label: &'static str,
) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    use std::net::{TcpStream, ToSocketAddrs};
    use std::time::Duration;
    let raw = url.strip_prefix("tcp://").unwrap_or(url);
    let with_port = if raw.contains(':') {
        raw.to_string()
    } else {
        format!("{raw}:{default_port}")
    };
    let resolved = with_port
        .to_socket_addrs()?
        .next()
        .ok_or_else(|| io::Error::other(format!("no addresses for {with_port}")))?;
    log::info!("{log_label}: connecting to {resolved}");
    let stream = TcpStream::connect_timeout(&resolved, Duration::from_secs(10))?;
    let read_clone = stream.try_clone()?;
    Ok((Box::new(read_clone), Box::new(stream)))
}

/// Read PLAIN/FAST lines off stdin, parse to `RawFrame`, push onto
/// `tx`. Auto-detects between PLAIN and FAST on the first non-empty
/// line. When `device_sender` is `Some`, each parsed frame is also
/// forwarded to the device — mirroring `actisense-serial -p`'s
/// "send to device AND echo to stdout" behaviour, where the echo
/// half is implemented here as a loopback into the pipeline source.
fn stdin_pump(
    tx: mpsc::Sender<RawFrame>,
    pre_coalesced: Arc<AtomicBool>,
    device_sender: Option<FrameSender>,
) {
    let stdin = io::stdin();
    let mut reader = BufReader::new(stdin.lock());
    let mut line = String::with_capacity(1024);
    let mut active_format: Option<InputFormat> = None;
    loop {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => return,
            Ok(_) => {}
            Err(e) => {
                log::error!("stdin read: {e}");
                return;
            }
        }
        let trimmed = line.trim_end_matches(['\r', '\n']);
        if trimmed.is_empty() {
            continue;
        }
        if trimmed.starts_with('#') {
            // `# format=<NAME>` headers — emitted by the canboat C
            // reader binaries — pin both the parser format and (for
            // any coalesced format) the pipeline's reassembly bypass.
            if let Some(fmt) = parse_format_header(trimmed) {
                if active_format.is_none() {
                    active_format = Some(fmt);
                    log::info!("input format set by header: {:?}", fmt);
                }
                if header_implies_coalesced(trimmed) && !pre_coalesced.swap(true, Ordering::Relaxed)
                {
                    log::debug!(
                        "stdin header declares coalesced format; \
                         pipeline will skip reassembly"
                    );
                }
            }
            continue;
        }
        if active_format.is_none() {
            active_format = detect(trimmed).or(Some(InputFormat::Plain));
        }
        let Ok(Some(frame)) = parse_with(active_format.unwrap(), trimmed) else {
            continue;
        };
        if let Some(sender) = &device_sender {
            // Drop on the floor if the device is currently
            // disconnected; the supervisor will reconnect, but stdin
            // injection is best-effort.
            let _ = sender.send_frame(frame.clone());
        }
        if tx.send(frame).is_err() {
            return;
        }
    }
}

/// Spawn a forwarder thread that pulls from `device_frames_rx` into a
/// fresh merge channel, plus a stdin pump that parses PLAIN/FAST
/// lines from stdin and pushes them onto the same merge channel
/// *and* writes them to the device via `sender`. Returns
/// `(merge_rx, loopback_tx)` — the receiver becomes the pipeline's
/// source and the sender is handed to TCP servers so client writes
/// can join the same loopback.
fn install_stdin_loopback(
    device_frames_rx: mpsc::Receiver<RawFrame>,
    sender: FrameSender,
    pre_coalesced: Arc<AtomicBool>,
) -> (mpsc::Receiver<RawFrame>, mpsc::Sender<RawFrame>) {
    let (merge_tx, merge_rx) = mpsc::channel::<RawFrame>();
    let merge_tx_device = merge_tx.clone();
    let merge_tx_stdin = merge_tx.clone();
    thread::Builder::new()
        .name("device-forward".into())
        .spawn(move || {
            while let Ok(f) = device_frames_rx.recv() {
                if merge_tx_device.send(f).is_err() {
                    return;
                }
            }
        })
        .expect("spawn device-forward");
    thread::Builder::new()
        .name("stdin-pump".into())
        .spawn(move || stdin_pump(merge_tx_stdin, pre_coalesced, Some(sender)))
        .expect("spawn stdin-pump");
    (merge_rx, merge_tx)
}

// Suppress an unused-import warning when no device flag is built (we
// rely on `bail!` for the future "no source AND not interactive"
// branch — keeping the import wired up here documents the intent).
#[allow(dead_code)]
fn _no_source() -> Result<()> {
    bail!("no input source");
}
