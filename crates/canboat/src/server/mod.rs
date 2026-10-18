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
//! * **stdin** (no `--kind` gateway, `--canboat-csv` or `--replay`): the
//!   binary parses PLAIN/FAST lines off stdin. Compatible with
//!   `analyzer < file.raw | canboat-pipeline` style pipelines that used
//!   to exist with `n2kd-inproc`.
//! * **device**: a [`crate::io::device`] reader stage opens the
//!   chosen serial port / TCP socket and emits `RawFrame`s
//!   directly. In this mode, the write-only input port and the lazy
//!   output TCP servers are also wired up.
//! * **replay** (`--replay <FILE>`): a [`crate::io::device::replay`]
//!   device plays a capture as a live bus — looped, paced and stamped
//!   with the current time — with the input port wired up as for a
//!   device, so a consumer can be tested end to end without a boat.
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

use anyhow::{Context, Result, bail};

#[cfg(feature = "cli")]
use crate::cli::device::{self as device_args, DeviceArgs, KindArg};
use crate::engine::RawFrame;
use crate::engine::format::{
    InputFormat, detect, header_implies_coalesced, parse_for, parse_format_header,
};
use crate::engine::pgn_list::{PgnListStatus, PgnListSupport};
pub use crate::io::device::kind::{ActisenseFormat, Kind};
pub use crate::io::device::open::Device;
use crate::io::device::supervisor::{Named, supervise};
use crate::io::device::{self, DeviceHandle, FrameSender};

/// Clap front-end for the `canboat server` CLI. Gated behind the `cli` feature
/// so the library path ([`BridgeConfig`] + [`Bridge`]) stays clap-free; convert
/// with [`BridgeConfig::from`].
#[cfg(feature = "cli")]
#[derive(Debug, clap::Args)]
#[command(after_help = crate::cli::help_footer())]
#[command(after_long_help = format!(
    "{}\n\n{}",
    crate::cli::device::GATEWAY_HELP,
    crate::cli::help_footer()
))]
#[command(group(clap::ArgGroup::new("source").multiple(false)))]
pub struct Args {
    /// The kind of gateway; without one, frames are read from stdin.
    ///
    /// With socketcan, canalyst or usb-can-a canboat becomes a full NMEA 2000 bus
    /// participant: it claims an address, answers ISO Requests and Group
    /// Functions, and sends Heartbeats.
    #[arg(long, value_enum, value_name = "KIND", requires = "device", help_heading = device_args::GATEWAY)]
    kind: Option<KindArg>,

    /// Where it is: a serial port, `usb`, `tcp://host[:port]`,
    /// `udp://host:port`, `host:port` or a CAN interface such as `can0`.
    ///
    /// Which one depends on the gateway; see the table at the end.
    #[arg(
        long,
        value_name = "DEVICE",
        group = "source",
        requires = "kind",
        help_heading = device_args::GATEWAY
    )]
    device: Option<String>,

    // The old one-flag-per-gateway spellings, each `--kind <kind> --device
    // <DEVICE>`.
    #[arg(long, value_name = "DEVICE", group = "source", hide = true)]
    actisense: Option<String>,
    #[arg(long, value_name = "DEVICE", group = "source", hide = true)]
    ikonvert: Option<String>,
    #[arg(long, value_name = "URL", group = "source", hide = true)]
    maretron: Option<String>,
    #[arg(long, value_name = "IFACE", group = "source", hide = true)]
    socketcan: Option<String>,

    /// Read from another `canboat server`'s raw port (`host:port` or
    /// `tcp://host[:port]`, 2603).
    ///
    /// The stream is canboat PLAIN/FAST CSV. The raw output port is
    /// read-only, so on its own this only receives frames; anything this
    /// instance sends is written to the same socket and rejected by the
    /// peer. To also send onto the upstream bus, give --canboat-csv-write
    /// the peer's input port. The URL can equally be a one-way feed such
    /// as an iptee'd raw stream.
    #[arg(long, value_name = "URL", group = "source", help_heading = OTHER_SOURCES)]
    canboat_csv: Option<String>,

    /// With --canboat-csv: where to send frames, the peer's input port (2600).
    ///
    /// The same slot as n2kd's `port+3`. Without it, writes go to the
    /// --canboat-csv socket, which a `canboat server` raw port does not
    /// accept.
    #[arg(long, value_name = "URL", requires = "canboat_csv", help_heading = OTHER_SOURCES)]
    canboat_csv_write: Option<String>,

    /// Play this capture as if it were a live bus, looped, at its own pace.
    ///
    /// Any format canboat reads (PLAIN, FAST, candump, N2K ASCII, RAW
    /// ASCII, …). Frames carry the current time. The input port accepts
    /// writes as for a real gateway; they go to --replay-sent, or are
    /// dropped. For testing a consumer without a boat.
    #[arg(long, value_name = "FILE", group = "source", help_heading = OTHER_SOURCES)]
    replay: Option<PathBuf>,

    /// With --replay: append the frames sent to the bus to this file.
    ///
    /// As canboat PLAIN/FAST lines, so a test can check what a consumer
    /// sent.
    #[arg(long, value_name = "FILE", requires = "replay", help_heading = OTHER_SOURCES)]
    replay_sent: Option<PathBuf>,

    #[command(flatten)]
    gateway: DeviceArgs,

    /// Apply a quirk to the bus; repeatable (see the possible values).
    ///
    /// `scx20` and `motion` impersonate or claim a device and need
    /// `--kind socketcan`, `canalyst` or `usb-can-a`; `wmm` emits canboat's own PGN and
    /// works with any gateway that can send; `gps-rollover` only rewrites
    /// decoded dates and needs no gateway at all.
    /// `gps-rollover=<device>[,<device>...]` also corrects every date the
    /// listed devices stamp from the broken clock; a device is a source
    /// address (`4`), manufacturer code and unique number (`1851:491603`),
    /// or a hex ISO NAME (`0x…`); `gps-rollover=all` corrects every date
    /// on the bus. `gps-relay` then re-sends what the listed devices
    /// broadcast — all their data PGNs, but not network management nor
    /// anything addressed — from canboat's own address, dates corrected,
    /// so other devices can select canboat as their GPS / time source;
    /// it needs a gateway that can send and a device list (not `all`).
    #[arg(long, value_name = "NAME", help_heading = ON_THE_BUS)]
    quirk: Vec<quirks::QuirkKind>,

    /// Do not request every device's Address Claim and Product Information.
    ///
    /// By default canboat asks periodically whenever it can send, as
    /// canboat C `n2kd` does. On stdin it never asks: there is nowhere to
    /// send the requests.
    #[arg(long, help_heading = ON_THE_BUS)]
    no_request_claims: bool,

    /// The address every TCP port listens on; 127.0.0.1 for this host only.
    #[arg(long, default_value = "0.0.0.0", help_heading = PORTS)]
    bind: Ipv4Addr,

    /// Snapshot: on connect, the latest JSON record of every PGN, then close.
    ///
    /// One per `(pgn, src, secondary)`. canboat C `n2kd`'s base port
    /// (`-p`). AIS PGNs go to --ais-port instead, apart from 129026 and
    /// 129029, which are on both. It keeps every record decoded to JSON;
    /// `0` turns it off when you don't need it.
    #[arg(long, value_name = "PORT", default_value_t = 2597, help_heading = PORTS)]
    snapshot_port: u16,

    /// Decoded JSON, one record per line; read-only.
    ///
    /// n2kd's `port+1`. Formatting is skipped while no client is
    /// connected. `0` turns it off.
    #[arg(long, value_name = "PORT", default_value_t = 2598, help_heading = PORTS)]
    analyzer_port: u16,

    /// NMEA 0183 sentences, AIVDM included; read-only.
    ///
    /// n2kd's `port+2`. `0` turns it off.
    #[arg(long, value_name = "PORT", default_value_t = 2599, help_heading = PORTS)]
    nmea0183_port: u16,

    /// Input: PLAIN/FAST lines written here are sent onto the bus.
    ///
    /// Write-only: nothing is streamed back. n2kd's `port+3`. The cheap
    /// way to send for a client that reads its data on another port. `0`
    /// turns it off.
    #[arg(long, value_name = "PORT", default_value_t = 2600, help_heading = PORTS)]
    input_port: u16,

    /// AIS snapshot: on connect, the latest record of every AIS PGN, then close.
    ///
    /// Plus PGN 129026 and 129029. n2kd's `port+4`. `0` turns it off.
    #[arg(long, value_name = "PORT", default_value_t = 2601, help_heading = PORTS)]
    ais_port: u16,

    /// Raw frames, as `# format=FAST` PLAIN lines; read-only.
    ///
    /// Fast-packets come already joined. Skipped while no client is
    /// connected. Sending goes through --input-port. `0` turns it off.
    #[arg(long, value_name = "PORT", default_value_t = 2603, help_heading = PORTS)]
    raw_port: u16,

    /// NMEA 0183 filter control (PGN 262657), for the TUI; read and write.
    ///
    /// The client is sent the filter state, then writes `Set` / `Request`
    /// and reads `Report` frames on the same socket. `0` turns it off.
    #[arg(long, value_name = "PORT", default_value_t = 2605, help_heading = PORTS)]
    nmea0183_filter_port: u16,

    /// PGN rate override control (PGN 262658), for the TUI; read and write.
    ///
    /// As the filter port: the client is sent the override state, then
    /// writes `Set` / `Delete` / `Request` and reads `Report` frames. `0`
    /// turns it off.
    #[arg(long, value_name = "PORT", default_value_t = 2606, help_heading = PORTS)]
    overrides_port: u16,

    /// Also write the NMEA 0183 sentences to stdout, as n2kd --nmea0183 does.
    #[arg(long, help_heading = NMEA0183)]
    nmea0183_stdout: bool,

    /// Do not limit NMEA 0183 output to one sentence per source and quantity a second.
    ///
    /// The limit (canboat C `n2kd`'s) keeps a bus with several devices
    /// reporting the same measurement from flooding 0183 clients. AIS is
    /// never limited. Turn it off for byte-for-byte parity captures.
    #[arg(long = "no-nmea0183-rate-limit", help_heading = NMEA0183)]
    no_nmea0183_rate_limit: bool,

    /// The NMEA 0183 filter's mute rules file [default: in --config-dir].
    ///
    /// The filter is always on; a file without rules lets everything
    /// through. Once there is a mute rule, the 0183 outputs only carry
    /// sentences from devices whose NAME (PGN 60928) canboat has learned
    /// and that aren't muted. The NMEA 2000 bus is never affected.
    #[arg(long = "nmea0183-filter", value_name = "PATH", help_heading = NMEA0183)]
    nmea0183_filter: Option<PathBuf>,

    /// Where the state files are kept [default: /etc/default/canboat or ~/.local/canboat].
    ///
    /// `overrides.json` (PGN rate overrides) and `nmea0183-filter.json`
    /// (0183 mute rules). Without it, `/etc/default/canboat` if writable
    /// (the systemd/root case), otherwise `$HOME/.local/canboat`.
    #[arg(long = "config-dir", value_name = "DIR", help_heading = STATE)]
    config_dir: Option<PathBuf>,

    /// The JSON on the analyzer and snapshot ports: field names, units and
    /// wrapping. NMEA 0183 and AIS always use their own units.
    #[command(flatten, next_help_heading = JSON)]
    shape: crate::cli::ShapeArgs,

    /// Log more.
    #[arg(short = 'v', long, help_heading = LOGGING)]
    verbose: bool,

    /// Log only errors.
    #[arg(short = 'q', long, help_heading = LOGGING)]
    quiet: bool,
}

/// `canboat server`'s own help headings, after the gateway's.
#[cfg(feature = "cli")]
const OTHER_SOURCES: &str = "Other sources";
#[cfg(feature = "cli")]
const ON_THE_BUS: &str = "On the bus";
#[cfg(feature = "cli")]
const PORTS: &str = "TCP ports";
#[cfg(feature = "cli")]
const NMEA0183: &str = "NMEA 0183";
#[cfg(feature = "cli")]
const STATE: &str = "State";
#[cfg(feature = "cli")]
const JSON: &str = "JSON output";
#[cfg(feature = "cli")]
const LOGGING: &str = "Logging";

/// Plain, clap-free configuration for a [`Bridge`]. Construct with
/// [`BridgeConfig::default`] and set the fields you need — or, under the `cli`
/// feature, `BridgeConfig::from(args)`. Field meanings mirror the `canboat
/// server` flags one-for-one.
#[derive(Debug, Clone)]
pub struct BridgeConfig {
    /// The gateway, and how to run it: see [`Device`]. The bridge opens
    /// it with [`reconnect`](Device::reconnect), so a gateway that goes
    /// away is opened again. `None` for one of the other sources below, or
    /// stdin.
    pub device: Option<Device>,
    /// What the bus carries (`--protocol`): picks the decode table and, on a
    /// gateway where canboat is the node, whether that is an NMEA 2000 or a
    /// J1939 node.
    /// Defaults to [`BusProtocol::Nmea2000`](crate::engine::BusProtocol).
    pub protocol: crate::engine::BusProtocol,
    pub canboat_csv: Option<String>,
    pub canboat_csv_write: Option<String>,
    /// Play this capture as the bus, looped and restamped with the current
    /// time (`--replay`).
    pub replay: Option<PathBuf>,
    /// Where frames written to a `replay` bus go (`--replay-sent`);
    /// `None` drops them.
    pub replay_sent: Option<PathBuf>,
    /// Pass on only these PGNs (`--rx-filter`), and always the
    /// network-management PGNs and canboat's own: canboat drops the rest
    /// itself, whatever the source. (An iKonvert can also filter in the
    /// gateway: [`Device::rx_filter`].) Empty: nothing is filtered.
    pub rx_filter: Vec<u32>,
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
    /// How field keys and PGN descriptions are spelled on the analyzer
    /// JSON / snapshot ports (`--id`). Defaults to
    /// [`CamelCase::Lower`](crate::engine::output::CamelCase::Lower).
    pub camel_case: crate::engine::output::CamelCase,
    /// Unit system those ports decode into (`--units`). Defaults to
    /// [`Units::Si`](crate::engine::Units::Si). NMEA 0183 / AIS output is
    /// unaffected — those always emit their spec units.
    pub units: crate::engine::Units,
    /// Nest each JSON record in `{"<pgnId>":{…}}` (`--wrap`). Off by
    /// default; canboat C ties this to `-camel`, we don't.
    pub wrap: bool,
    pub verbose: bool,
    pub quiet: bool,
}

impl BridgeConfig {
    /// Whether the backend runs canboat's own node on a raw CAN link
    /// (`socketcan`, `canalyst` or `usb-can-a`): it claims an address and
    /// sends a frame from the source it names.
    pub(crate) fn is_node(&self) -> bool {
        self.device.as_ref().is_some_and(|dev| {
            let caps = dev.capabilities();
            caps.node && caps.raw
        })
    }
}

impl Default for BridgeConfig {
    fn default() -> Self {
        Self {
            device: None,
            protocol: crate::engine::BusProtocol::Nmea2000,
            canboat_csv: None,
            canboat_csv_write: None,
            replay: None,
            replay_sent: None,
            rx_filter: Vec::new(),
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
            // Machine-facing defaults, matching `convert` / `analyzer`
            // and what canboatjs consumers expect.
            camel_case: crate::engine::output::CamelCase::Lower,
            units: crate::engine::Units::Si,
            wrap: false,
            verbose: false,
            quiet: false,
        }
    }
}

#[cfg(feature = "cli")]
impl Args {
    /// Nudge users off canboat C's `--camel` / `--upper-camel` / `--si`
    /// spellings. The host binary calls this before converting to a
    /// [`BridgeConfig`], which only carries the resolved shape.
    pub fn warn_deprecated(&self) {
        self.shape.warn_deprecated();
    }

    /// The gateway the flags name, `--kind` (with `--data-format`) and
    /// `--device`, or one of the old one-flag-per-gateway spellings.
    fn source(&self) -> anyhow::Result<Option<(Kind, String)>> {
        if let (Some(kind), Some(device)) = (self.kind, &self.device) {
            let kind = kind.resolve(self.gateway.data_format)?;
            return Ok(Some((kind, device.clone())));
        }
        if let Some(format) = self.gateway.data_format {
            anyhow::bail!("--data-format {} goes with --kind", format.name());
        }
        Ok([
            (Kind::NGT1, &self.actisense),
            (Kind::Ikonvert, &self.ikonvert),
            (Kind::Maretron, &self.maretron),
            (Kind::SocketCan, &self.socketcan),
        ]
        .into_iter()
        .find_map(|(kind, device)| Some((kind, device.clone()?))))
    }

    /// Reject what clap cannot: a gateway option the chosen gateway does
    /// not have, a protocol it cannot carry, more than one `--protocol`.
    /// Call before converting to a [`BridgeConfig`].
    pub fn check(&self) -> anyhow::Result<()> {
        if self.kind.is_some() && self.device.is_none() {
            anyhow::bail!(
                "--kind needs --device, where the gateway is (not an old --<gateway> flag)"
            );
        }
        self.gateway.check(self.source()?.map(|(kind, _)| kind))
    }
}

#[cfg(feature = "cli")]
impl From<Args> for BridgeConfig {
    fn from(a: Args) -> Self {
        let source = a
            .source()
            .expect("Args::check rejects a --kind this cannot resolve");
        let g = &a.gateway;
        Self {
            device: source.map(|(kind, address)| g.device(kind, &address)),
            protocol: g.protocol(),
            canboat_csv: a.canboat_csv,
            canboat_csv_write: a.canboat_csv_write,
            replay: a.replay,
            replay_sent: a.replay_sent,
            rx_filter: g.rx_filter.clone(),
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
            camel_case: a.shape.camel_case(),
            units: a.shape.units(),
            wrap: a.shape.wrap(),
            verbose: a.verbose,
            quiet: a.quiet,
        }
    }
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
    supervisor: Option<DeviceHandle>,
    pre_coalesced: Arc<AtomicBool>,
    /// Live claim address of the device backend, when known
    /// (`--socketcan` and `--canalyst` expose one). Read by the CSV-port
    /// injector to rewrite client-supplied default-`src` frames.
    claim_addr: Option<Arc<std::sync::atomic::AtomicU8>>,
    /// What the backend does with `BridgeConfig::pgn_lists`; `None` when
    /// none were given.
    pgn_list_status: Option<PgnListStatus>,
}

// Config-dir *discovery* (the `/etc/default/canboat` vs `~/.local/canboat`
// filesystem probe) is deliberately NOT here: it's a binary concern, so it
// lives in the `canboat` bin's `server` handler. The library takes an
// explicit `BridgeConfig.config_dir` (or `None` to disable persistence).

fn open_source(config: &BridgeConfig) -> Result<OpenedSource> {
    if let Some(dev) = &config.device {
        let dev = dev
            .clone()
            // What the quirks send from the gateway's own address.
            .extra_tx_pgns(quirk_tx_pgns(config))
            .node_defaults("canboat-pipeline-rs", config.config_dir.clone())
            .reconnect(true);
        let pgn_list_status = dev.pgn_list_status();
        // What the gateway lacks says which, already; and the bridge
        // reconnects, so nothing else fails here.
        let handle = dev.open(config.protocol)?;
        let claim_addr = handle.claim_atom();
        let (rx, handle) = split_supervisor(handle);
        // Every gateway hands over whole messages: it joins fast-packets
        // itself, or canboat's node or codec does. Skip the reassembler.
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(handle),
            pre_coalesced: Arc::new(AtomicBool::new(true)),
            claim_addr,
            pgn_list_status,
        });
    }
    if let Some(read_url) = config.canboat_csv.as_deref() {
        let read_url = read_url.to_string();
        let write_url = config.canboat_csv_write.clone();
        let handle = supervise(Named {
            name: "canboat-csv".into(),
            open: move || {
                let (reader, writer) = match &write_url {
                    Some(w) => open_canboat_csv_split(&read_url, w)?,
                    None => open_canboat_csv_pair(&read_url)?,
                };
                Ok(device::canboat_csv::run(reader, writer))
            },
        });
        let (rx, handle) = split_supervisor(handle);
        // Wire format is canboat PLAIN/FAST — each line is already
        // a complete PGN payload, so skip the reassembler.
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(handle),
            pre_coalesced: Arc::new(AtomicBool::new(true)),
            claim_addr: None,
            pgn_list_status: unsupported_pgn_lists(config),
        });
    }
    if let Some(path) = config.replay.as_deref() {
        let scan = device::replay::scan(path, config.protocol)
            .with_context(|| format!("--replay {}", path.display()))?;
        let replay = device::replay::Config {
            path: path.to_path_buf(),
            protocol: config.protocol,
            sent: config.replay_sent.clone(),
            ..device::replay::Config::new(path)
        };
        let handle = supervise(Named {
            name: "replay".into(),
            open: move || device::replay::run(replay.clone()),
        });
        let (rx, handle) = split_supervisor(handle);
        return Ok(OpenedSource {
            frames_rx: rx,
            supervisor: Some(handle),
            pre_coalesced: Arc::new(AtomicBool::new(scan.coalesced)),
            claim_addr: None,
            pgn_list_status: unsupported_pgn_lists(config),
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
        .spawn({
            let protocol = config.protocol;
            move || stdin_pump(tx, coalesced_for_pump, None, protocol)
        })
        .expect("spawn stdin-pump");
    Ok(OpenedSource {
        frames_rx: rx,
        supervisor: None,
        pre_coalesced,
        claim_addr: None,
        pgn_list_status: unsupported_pgn_lists(config),
    })
}

/// The PGNs the configured quirks transmit from the gateway's own address.
fn quirk_tx_pgns(config: &BridgeConfig) -> Vec<u32> {
    let mut pgns = Vec::new();
    if config.quirk.contains(&quirks::QuirkKind::Wmm) {
        pgns.push(quirks::wmm::PGN_MAGNETIC_VARIATION);
    }
    pgns
}

/// The status for a source that is not a gateway, which advertises no PGN
/// lists: `None` unless a quirk transmits from its own address.
fn unsupported_pgn_lists(config: &BridgeConfig) -> Option<PgnListStatus> {
    let pgns = quirk_tx_pgns(config);
    if pgns.is_empty() {
        return None;
    }
    // The user asked for no lists, so say so without a warning.
    log::info!(
        "this source cannot advertise PGN lists; the quirks' transmit PGNs {pgns:?} are not advertised"
    );
    Some(PgnListStatus {
        tx: PgnListSupport::Unsupported,
        rx: PgnListSupport::Unsupported,
        dropped: Vec::new(),
    })
}

/// Swap the supervised handle's `frames_rx` out so the pipeline can own
/// it while we keep the handle for shutdown / frame_sender access.
fn split_supervisor(mut sup: DeviceHandle) -> (mpsc::Receiver<RawFrame>, DeviceHandle) {
    let (_dummy_tx, dummy_rx) = mpsc::channel::<RawFrame>();
    let rx = std::mem::replace(&mut sup.frames_rx, dummy_rx);
    (rx, sup)
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
/// This is the normal way to chain into another `canboat server`:
/// its raw output port (2603) is read-only and its input port
/// (2600) is write-only, so the two directions live on different
/// sockets. The read URL can also be a one-way feed such as
/// `iptee` mirroring the raw output of `actisense-serial`.
fn open_canboat_csv_split(
    read_url: &str,
    write_url: &str,
) -> io::Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    let (read_half, _unused_write) = open_tcp_pair(read_url, 2603, "canboat-csv read")?;
    let (_unused_read, write_half) = open_tcp_pair(write_url, 2600, "canboat-csv write")?;
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
    crate::io::set_tcp_keepalive(&stream)?;
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
    protocol: crate::engine::BusProtocol,
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
        let Ok(Some(frame)) = parse_for(active_format.unwrap(), protocol, trimmed) else {
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
/// *and* writes them to the device via `sender`. Returns the merge
/// receiver, which becomes the pipeline's source. Only stdin joins
/// this loopback; the TCP input port writes to the device alone.
fn install_stdin_loopback(
    device_frames_rx: mpsc::Receiver<RawFrame>,
    sender: FrameSender,
    pre_coalesced: Arc<AtomicBool>,
    protocol: crate::engine::BusProtocol,
) -> mpsc::Receiver<RawFrame> {
    let (merge_tx, merge_rx) = mpsc::channel::<RawFrame>();
    let merge_tx_device = merge_tx.clone();
    let merge_tx_stdin = merge_tx;
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
        .spawn(move || stdin_pump(merge_tx_stdin, pre_coalesced, Some(sender), protocol))
        .expect("spawn stdin-pump");
    merge_rx
}

// Suppress an unused-import warning when no device flag is built (we
// rely on `bail!` for the future "no source AND not interactive"
// branch — keeping the import wired up here documents the intent).
#[allow(dead_code)]
fn _no_source() -> Result<()> {
    bail!("no input source");
}
