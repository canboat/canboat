// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat interface` — bridge a live NMEA 2000 gateway to/from stdout.
//!
//! Supersedes the standalone `actisense-serial`, `ikonvert-serial`,
//! `maretron-ipg` and `socketcan-serial` binaries. Each spoke shares
//! one shape: a device codec from [`crate::io::device`] turns the wire
//! protocol into a [`DeviceHandle`], whose `frames_rx` is a
//! [`FrameReader`] and whose `FrameSender` is a [`FrameWriter`]. So both
//! directions are a single [`crate::io::copy`]:
//!
//! - **read**  device `frames_rx` → PLAIN on stdout
//! - **write** PLAIN on stdin → device `FrameSender`
//!
//! Output leads with the canboat `# format=FAST` header and a synthetic
//! startup record, matching the retired binaries byte for byte (the
//! per-kind producer name is preserved so the n2kd parity harness keeps
//! comparing like with like).

use std::io::{self, BufRead, BufWriter, Read, Write};
use std::sync::Arc;
use std::sync::atomic::AtomicU8;
use std::thread;

use anyhow::{Context, Result};

use crate::engine::format::ngt1::OperatingMode;
use crate::engine::{BusProtocol, RawFrame};
use crate::io::device::gateway::GatewayKind;
use crate::io::device::{self, DeviceHandle};
use crate::io::{FrameWriter, PlainWriter, copy, open_serial_rw};

/// `# format=FAST` tag every canboat reader prepends to its PLAIN
/// stream so a downstream analyzer knows the frames are coalesced.
const FORMAT_FAST_HEADER: &[u8] = b"# format=FAST\n";

/// Which gateway to talk to.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Kind {
    /// Actisense NGT-1 (binary), over a serial port.
    Ngt1,
    /// Digital Yacht iKonvert (ASCII), over a serial port.
    Ikonvert,
    /// Maretron IPG100/200, over TCP.
    Maretron,
    /// Linux SocketCAN interface (e.g. `can0`).
    Socketcan,
    /// Yacht Devices RAW gateway: YDWG-02 / YDEN / NavLink2 RAW port over
    /// TCP (`tcp://host[:port]`) or UDP (`udp://gateway:port` to send and
    /// receive, `udp://[bind:]port` to receive only), or the YDNU-02 USB
    /// gateway over its serial port (38400 baud).
    Ydwg,
    /// Actisense W2K-1 in N2K ASCII mode, over TCP or UDP, as ydwg.
    #[value(name = "w2k-ascii")]
    W2kAscii,
    /// Actisense BST-D0 (binary, reassembled NMEA 2000) over TCP: a W2K-1
    /// or PRO-NDC-1E2K in Actisense mode.
    #[value(name = "bst-d0", alias = "w2k-actisense")]
    BstD0,
    /// Actisense BST-95 (binary, raw CAN frames): a PRO-NDC-1E2K or W2K-1
    /// in "CAN Actisense" mode over TCP (`tcp://host[:port]`), or an NGX in
    /// CAN Packet mode over a serial port. canboat joins and splits
    /// fast-packets itself.
    #[value(name = "bst-95")]
    Bst95,
    /// CANalyst-II over USB (`usb`): the Waveshare USB-CAN-B. Like
    /// socketcan, a full NMEA 2000 node — it claims an address — on every
    /// platform.
    #[value(name = "canalyst", alias = "usb-can-b")]
    Canalyst,
}

impl Kind {
    /// Producer name stamped into the startup record. Kept identical
    /// to the retired standalone binary so parity output is unchanged.
    fn producer(self) -> &'static str {
        match self {
            Kind::Ngt1 => "actisense-serial",
            Kind::Ikonvert => "ikonvert-serial",
            Kind::Maretron => "maretron-ipg",
            Kind::Socketcan => "socketcan-serial",
            Kind::Ydwg => "ydwg-gateway",
            Kind::W2kAscii | Kind::BstD0 | Kind::Bst95 => "w2k-gateway",
            Kind::Canalyst => "canalyst",
        }
    }

    /// Default serial baud of the NGT-1 and the iKonvert.
    fn default_baud(self) -> u32 {
        match self {
            Kind::Ngt1 => 115_200,
            Kind::Ikonvert => 230_400,
            _ => self.gateway().map_or(0, GatewayKind::default_baud),
        }
    }

    /// The gateways `canboat server` opens the same way.
    fn gateway(self) -> Option<GatewayKind> {
        match self {
            Kind::Ydwg => Some(GatewayKind::Ydwg),
            Kind::W2kAscii => Some(GatewayKind::W2kAscii),
            Kind::BstD0 => Some(GatewayKind::BstD0),
            Kind::Bst95 => Some(GatewayKind::Bst95),
            _ => None,
        }
    }
}

/// The NGT-1 operating modes `--mode` offers.
#[derive(Debug, Clone, Copy, PartialEq, Eq, clap::ValueEnum)]
enum Ngt1Mode {
    RxAll,
    Normal,
}

impl From<Ngt1Mode> for OperatingMode {
    fn from(mode: Ngt1Mode) -> Self {
        match mode {
            Ngt1Mode::RxAll => OperatingMode::NgTransferRxAll,
            Ngt1Mode::Normal => OperatingMode::NgTransferNormal,
        }
    }
}

#[derive(Debug, clap::Args)]
pub struct Args {
    /// Gateway type.
    #[arg(long, value_enum)]
    kind: Kind,

    /// Endpoint: serial path (ngt1/ikonvert, ydwg for a YDNU-02, bst-95
    /// for an NGX), `tcp://host[:port]` (ydwg, w2k-*, bst-d0, bst-95, and
    /// ngt1/ikonvert through a network bridge), `udp://gateway:port` (ydwg,
    /// w2k-ascii; `udp://[bind:]port` only receives), `host:port` (maretron), or CAN interface name such as `can0`
    /// (socketcan), or `usb[:VVVV:PPPP]` (canalyst). An FTDI-based
    /// gateway such as the NGT-1 can also be opened directly over USB as
    /// `usb`, `usb:SERIAL` or `usb:VVVV:PPPP[:SERIAL]` — for macOS, whose
    /// serial driver does not recognise the NGT-1.
    #[arg(value_name = "DEVICE")]
    device: String,

    /// Serial baud rate. Defaults to 115200 (ngt1, bst-95) / 230400
    /// (ikonvert) / 38400 (ydwg).
    /// `-s` is the C actisense-serial/ikonvert-serial spelling; the
    /// argv[0] shims must stay drop-in compatible with it.
    #[arg(short = 'b', long, short_alias = 's')]
    baud: Option<u32>,

    /// Read-only: emit received frames, ignore stdin.
    #[arg(short = 'r', long = "read-only", conflicts_with = "write_only")]
    read_only: bool,

    /// Write-only: send stdin frames to the device, drop received ones.
    #[arg(short = 'w', long = "write-only")]
    write_only: bool,

    /// NGT-1: the operating mode to put the gateway in. `rx-all` ("NGT
    /// Transfer Rx All Mode") forwards every PGN received; `normal` ("NGT
    /// Transfer Normal Mode") only those on the gateway's Receive PGN
    /// Enable list.
    #[arg(long, value_enum, default_value_t = Ngt1Mode::RxAll)]
    mode: Ngt1Mode,

    /// iKonvert: comma-separated receive PGN allow-list.
    #[arg(long, value_name = "PGN,...")]
    rx: Option<String>,

    /// iKonvert: comma-separated transmit PGN allow-list. WARNING — ONCE
    /// GIVEN, EVERY PGN NOT ON IT IS REFUSED (NOT SENT), apart from network
    /// management. List every PGN you will send.
    #[arg(long, value_name = "PGN,...")]
    tx: Option<String>,

    /// iKonvert: disable the device's TX rate limit.
    #[arg(long)]
    rate_limit_off: bool,

    /// Maretron: IPG login password (default: empty).
    #[arg(long, value_name = "PASSWORD")]
    password: Option<String>,

    /// SocketCAN, CANalyst, AOS simulator (on ngt1): preferred source address
    /// to claim.
    #[arg(short = 'a', long, value_name = "ADDR", default_value_t = 0)]
    address: u8,

    /// SocketCAN, CANalyst: passive sniff — skip the ISO address-claim handshake.
    /// AOS simulator (on ngt1): no node, so it sends from an address it never
    /// claims.
    #[arg(short = 'n', long)]
    no_claim: bool,

    /// SocketCAN, CANalyst, AOS simulator: unique number for the ISO NAME (default derived from
    /// the machine id, stable per-host across restarts; on a machine
    /// that can't be identified, a random one stored with the server's
    /// state files).
    #[arg(short = 'u', long, value_name = "N", default_value_t = 0)]
    unique: u32,

    /// SocketCAN, CANalyst, AOS simulator: manufacturer code for the ISO NAME (999 = Signal K).
    #[arg(short = 'm', long, value_name = "N", default_value_t = 999)]
    manufacturer: u16,

    /// SocketCAN, CANalyst, AOS simulator: Heartbeat (PGN 126993) interval in ms, up to 65532;
    /// 0 disables.
    #[arg(
        long,
        alias = "hb",
        value_name = "MS",
        default_value_t = 60_000,
        value_parser = clap::value_parser!(u64).range(0..=65_532)
    )]
    heartbeat: u64,

    /// SocketCAN, CANalyst, AOS simulator: ISO NAME System Instance, 0..15. Default 15 (max) so
    /// our NAME yields to real hardware rather than stealing addresses.
    #[arg(long, alias = "si", value_name = "N", default_value_t = 15)]
    system_instance: u8,

    /// CANalyst-II: the CAN channel, 0 or 1.
    #[arg(long, value_name = "N", default_value_t = 0)]
    channel: u8,

    /// The CAN bus bit rate, for the gateways canboat drives as the CAN
    /// controller: the CANalyst-II (250 000 if not given) and SocketCAN,
    /// where it brings the interface down, sets the rate and brings it up
    /// again (needs root or CAP_NET_ADMIN). Without it a SocketCAN
    /// interface is used as it was configured. 250 000 for NMEA 2000,
    /// 500 000 for a J1939-14 bus. Not the serial speed to a gateway such
    /// as the NGT-1: that is `--baud`.
    #[arg(long, value_name = "BIT/S")]
    bitrate: Option<u32>,

    /// Quit, with an error, if no frame comes off the bus for this many
    /// seconds (0 disables): a gateway that went away without closing the
    /// connection is then noticed, and a supervisor can restart canboat.
    #[arg(short = 't', long, value_name = "SECONDS", default_value_t = 0)]
    timeout: u64,

    /// `--protocol j1939` frames the traffic as J1939 (single frames and ISO
    /// TP, no fast-packet) and, on SocketCAN, leaves out the NMEA 2000
    /// Heartbeat, Product Information and PGN lists. SocketCAN, YDWG,
    /// BST-95 and CANalyst only: the other gateways do NMEA 2000 framing
    /// themselves. ISO TP messages (longer than 8 bytes) are received on all
    /// four but sent on SocketCAN only; the others refuse to send them.
    #[command(flatten)]
    protocol: crate::cli::protocol::ProtocolArgs,
}

pub fn run(args: Args) -> Result<()> {
    // On stderr, at the C gateway programs' default level; the device
    // drivers' warnings went nowhere without it. `RUST_LOG` overrides.
    let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info"))
        .try_init();
    let handle = open_device(&args)?;
    // Close the device on purpose — when we are stopped, and when the
    // stream ends — so a gateway is taken off the bus (iKonvert).
    let closer = handle.closer();
    {
        let closer = closer.clone();
        super::stop_signal::on_stop(move || {
            // A device that has gone already has nothing left to close.
            let closed = closer.close(device::CLOSE_TIMEOUT);
            if closed == device::Closed::Unconfirmed {
                log::error!(
                    "the device did not confirm closing (a gateway may still be on the bus)"
                );
            }
            closed != device::Closed::Unconfirmed
        });
    }

    let result = stream(&args, handle);
    // Also when a stream step failed: the gateway still has to leave the bus.
    let closed = closer.close(device::CLOSE_TIMEOUT);
    // A stream error is the one to report; otherwise an unconfirmed close.
    // A device that went away first (unplugged, timed out) is not one: its
    // writer had already stopped, so there was nothing to close.
    result?;
    if closed == device::Closed::Unconfirmed {
        anyhow::bail!("the device did not confirm closing (a gateway may still be on the bus)");
    }
    Ok(())
}

/// Move frames between the device and stdin/stdout until a stream ends.
fn stream(args: &Args, mut handle: DeviceHandle) -> Result<()> {
    let protocol = args.protocol.resolve()?;
    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    write_prologue(&mut out, args).context("writing prologue")?;

    match (args.read_only, args.write_only) {
        // Read-only: device → PLAIN stdout, ignore stdin.
        (true, _) => {
            let mut writer = PlainWriter::new(&mut out);
            let mut rx = Idle::new(&mut handle.frames_rx, args.timeout);
            copy(&mut rx, &mut writer).context("reading from device")?;
        }
        // Write-only: stdin PLAIN → device on the main thread (so we
        // exit when stdin ends); received frames are drained and
        // discarded so the channel can't back up.
        // The end of stdin ends the stream, and so do a device that went
        // away and a bus quiet for `--timeout`.
        (false, true) => {
            let (done_tx, done_rx) = std::sync::mpsc::channel::<Result<()>>();
            let mut sender = handle.frame_sender();
            let pump_done = done_tx.clone();
            thread::Builder::new()
                .name("stdin-pump".into())
                .spawn(move || {
                    let result = pump_stdin(&mut sender, protocol).context("sending stdin frames");
                    let _ = pump_done.send(result);
                })
                .expect("spawn stdin pump");
            let mut rx = handle.frames_rx;
            let timeout = args.timeout;
            thread::Builder::new()
                .name("frame-drain".into())
                .spawn(move || {
                    let mut rx = Idle::new(&mut rx, timeout);
                    let result = copy(&mut rx, &mut DiscardWriter)
                        .map(drop)
                        .context("reading from device");
                    let _ = done_tx.send(result);
                })
                .expect("spawn frame drain");
            // Don't wait on a blocked stdin read.
            done_rx.recv().expect("a stream thread reports")?;
        }
        // Bidirectional (default): stdin → device on a background
        // thread, device → PLAIN stdout on the main thread.
        (false, false) => {
            let mut sender = handle.frame_sender();
            let pump = thread::Builder::new()
                .name("stdin-pump".into())
                .spawn(move || {
                    if let Err(e) = pump_stdin(&mut sender, protocol) {
                        log::warn!("stdin pump stopped: {e}");
                    }
                })
                .expect("spawn stdin pump");
            let mut writer = PlainWriter::new(&mut out);
            let mut rx = Idle::new(&mut handle.frames_rx, args.timeout);
            copy(&mut rx, &mut writer).context("reading from device")?;
            drop(pump); // device closed; don't wait on a blocked stdin read
        }
    }
    Ok(())
}

/// The device's frames, ended with an error once none has come off the
/// bus for `timeout_secs` (0: never). A frame canboat makes itself (a
/// gateway status on a timer) does not count: it comes even from a
/// gateway that has gone.
struct Idle<'a> {
    rx: &'a mut std::sync::mpsc::Receiver<RawFrame>,
    timeout: Option<std::time::Duration>,
    last: std::time::Instant,
}

impl<'a> Idle<'a> {
    fn new(rx: &'a mut std::sync::mpsc::Receiver<RawFrame>, timeout_secs: u64) -> Self {
        Self {
            rx,
            timeout: (timeout_secs > 0).then(|| std::time::Duration::from_secs(timeout_secs)),
            last: std::time::Instant::now(),
        }
    }
}

impl crate::io::FrameReader for Idle<'_> {
    fn read_frame(&mut self) -> io::Result<Option<RawFrame>> {
        use std::sync::mpsc::RecvTimeoutError;
        let Some(timeout) = self.timeout else {
            return Ok(self.rx.recv().ok());
        };
        loop {
            let left = timeout.saturating_sub(self.last.elapsed());
            match self.rx.recv_timeout(left) {
                Ok(frame) => {
                    if frame.pgn < crate::engine::codec::SYNTHETIC_PGN_START {
                        self.last = std::time::Instant::now();
                    }
                    return Ok(Some(frame));
                }
                Err(RecvTimeoutError::Disconnected) => return Ok(None),
                Err(RecvTimeoutError::Timeout) if self.last.elapsed() >= timeout => {
                    return Err(io::Error::new(
                        io::ErrorKind::TimedOut,
                        format!("no data received for {} seconds", timeout.as_secs()),
                    ));
                }
                Err(RecvTimeoutError::Timeout) => {}
            }
        }
    }
}

/// Pump PLAIN frames from stdin into `sender` until stdin ends.
fn pump_stdin(sender: &mut device::FrameSender, protocol: BusProtocol) -> io::Result<()> {
    // Two stdin dialects, dispatched per line: canboat PLAIN/FAST CSV
    // (the historical C contract — coalesced messages, the device layer
    // fragments fast-packets) and analyzer JSON records ('{'-prefixed),
    // encoded via the schema. The JSON path lets a driving process
    // (e.g. signalk-server) hand decoded PGN objects straight to the
    // bridge without running a separate encoder; both kinds may be
    // interleaved on one stream. Physical values in JSON are taken as
    // SI, the unit system canboatjs-style producers emit, and encoded
    // against the table of the protocol on the bus they go out on.
    let db = protocol.database(crate::engine::Units::Si);
    let stdin = io::stdin();
    for line in stdin.lock().lines() {
        let line = line?;
        let t = line.trim();
        if t.is_empty() || t.starts_with('#') {
            continue;
        }
        if t.starts_with('{') {
            match crate::json_input::frame_from_json(db, t) {
                Ok(Some(frame)) => sender.write_frame(&frame)?,
                Ok(None) => {}
                Err(e) => log::warn!("stdin json: {e:#}"),
            }
        } else {
            match crate::engine::format::plain::parse_line(t) {
                Ok(frame) => sender.write_frame(&frame)?,
                Err(e) => log::warn!("stdin: {e}"),
            }
        }
    }
    Ok(())
}

/// A [`FrameWriter`] that throws frames away — used to keep a device's
/// receive channel drained in write-only mode.
struct DiscardWriter;

impl FrameWriter for DiscardWriter {
    fn write_frame(&mut self, _frame: &RawFrame) -> io::Result<()> {
        Ok(())
    }
}

/// Emit the `# format=FAST` header + the synthetic startup record.
fn write_prologue<W: Write>(out: &mut W, args: &Args) -> io::Result<()> {
    out.write_all(FORMAT_FAST_HEADER)?;
    let rec = crate::engine::startup_record(
        env!("CARGO_PKG_VERSION"),
        args.kind.producer(),
        &args.device,
    );
    let mut line = String::with_capacity(160);
    crate::engine::format::plain::write_line(&mut line, &rec).ok();
    out.write_all(line.as_bytes())?;
    out.write_all(b"\n")?;
    out.flush()
}

/// Open the selected transport and start its device codec.
fn open_device(args: &Args) -> Result<DeviceHandle> {
    let protocol = args.protocol.resolve()?;
    // Quick's 11-bit frames: only SocketCAN and the CANalyst-II pass them on.
    if protocol == BusProtocol::Quick && !matches!(args.kind, Kind::Socketcan | Kind::Canalyst) {
        anyhow::bail!(
            "--protocol quick needs socketcan or canalyst: the other gateways pass only 29-bit frames"
        );
    }
    if protocol != BusProtocol::Nmea2000
        && !matches!(args.kind, Kind::Socketcan | Kind::Canalyst)
        && !args
            .kind
            .gateway()
            .is_some_and(GatewayKind::passes_raw_frames)
    {
        anyhow::bail!(
            "--protocol {protocol} needs a gateway that passes raw CAN frames (socketcan, ydwg, bst-95 or canalyst); \
             this one does NMEA 2000 framing itself"
        );
    }
    if args.bitrate.is_some() && !matches!(args.kind, Kind::Socketcan | Kind::Canalyst) {
        anyhow::bail!(
            "--bitrate sets the CAN bus rate of socketcan or canalyst; this gateway runs its own CAN side \
             (for the serial speed to it, use --baud)"
        );
    }
    match args.kind {
        Kind::Ngt1 => {
            let (r, w) = open_stream(args)?;
            let config = device::ngt1::Config {
                operating_mode: args.mode.into(),
                ..Default::default()
            };
            // An AOS simulator, which claims no address, gets a node.
            let node = device::ngt1::NodeConfig {
                model_version: Some("canboat-aos"),
                ..node_config(args, protocol)
            };
            let claim = Arc::new(AtomicU8::new(device::socketcan::CLAIM_UNCLAIMED));
            Ok(device::ngt1::run_with_node(r, w, config, node, claim))
        }
        Kind::Ikonvert => {
            let (r, w) = open_stream(args)?;
            let config = device::ikonvert::Config {
                rx_list: args.rx.clone(),
                tx_list: args.tx.clone(),
                rate_limit_off: args.rate_limit_off,
                skip_init: false,
                ..Default::default()
            };
            Ok(device::ikonvert::run(r, w, config))
        }
        Kind::Maretron => {
            // The C maretron-ipg takes `tcp://<host>[:<port>]` with the
            // port defaulting to 6543; the argv[0] shim must stay
            // drop-in compatible with that syntax (signalk-server
            // spawns it exactly so), and the bare `host:port` form
            // keeps working.
            let (reader, writer) = crate::io::net::open_tcp(&args.device, 6543)?;
            let config = device::maretron::Config {
                password: args.password.clone().unwrap_or_default(),
                fixtime: None,
            };
            Ok(device::maretron::run(reader, writer, config))
        }
        Kind::Ydwg | Kind::W2kAscii | Kind::BstD0 | Kind::Bst95 => {
            let gateway = device::gateway::Gateway {
                kind: args.kind.gateway().expect("a gateway kind"),
                device: args.device.clone(),
            };
            Ok(device::gateway::open(&gateway, args.baud, protocol)?)
        }
        Kind::Canalyst => open_canalyst(args, protocol),
        Kind::Socketcan => {
            let config = device::socketcan::Config {
                // Configure the link only when asked to; otherwise use the
                // interface as it was set up.
                configure_link: args.bitrate.is_some(),
                bitrate: args.bitrate.unwrap_or(250_000),
                ..node_config(args, protocol)
            };
            let claim = Arc::new(AtomicU8::new(config.address));
            // On non-Linux this returns ErrorKind::Unsupported.
            device::socketcan::run(&args.device, config, claim)
                .with_context(|| format!("opening SocketCAN interface {}", args.device))
        }
    }
}

/// The node configuration of a raw CAN gateway (socketcan, canalyst), or
/// of an AOS simulator on `ngt1`.
fn node_config(args: &Args, protocol: BusProtocol) -> device::socketcan::Config {
    device::socketcan::Config {
        address: args.address,
        unique: args.unique,
        manufacturer: args.manufacturer,
        system_instance: args.system_instance,
        heartbeat_ms: args.heartbeat,
        no_claim: args.no_claim,
        timeout_secs: args.timeout,
        protocol,
        // Probing creates the dir, so only when it may be needed.
        state_dir: (args.unique == 0).then(super::app::resolve_config_dir),
        ..device::socketcan::Config::default()
    }
}

/// Open a CANalyst-II over USB and run a node on it, as for SocketCAN.
fn open_canalyst(args: &Args, protocol: BusProtocol) -> Result<DeviceHandle> {
    let selector = crate::io::usb::Selector::parse(&args.device)
        .ok_or_else(|| anyhow::anyhow!("a CANalyst-II is named `usb` or `usb:VVVV:PPPP`"))??;
    let config = device::socketcan::Config {
        model_version: Some("canboat-canalyst"),
        bitrate: args.bitrate.unwrap_or(250_000),
        ..node_config(args, protocol)
    };
    let (reader, writer) = crate::io::canalyst::open_rw(&selector, args.channel, config.bitrate)
        .with_context(|| format!("opening CANalyst-II {}", args.device))?;
    let claim = Arc::new(AtomicU8::new(config.address));
    Ok(device::canalyst::run(reader, writer, config, claim)?)
}

/// Open the serial transport as an independent `(reader, writer)` pair.
/// `interface` is live-device only — captured byte streams are decoded
/// with `canboat convert`, not here.
fn open_stream(args: &Args) -> Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    // A serial-protocol gateway reached over the network (the NavLink2
    // serves the iKonvert protocol on TCP 6001; ser2net setups do the
    // same for an NGT-1): same codec, different transport.
    if args.device.starts_with("tcp://") {
        return Ok(crate::io::net::open_tcp(&args.device, 6001)?);
    }
    let baud = args.baud.unwrap_or_else(|| args.kind.default_baud());
    open_serial_rw(&args.device, baud)
        .with_context(|| format!("opening serial port {}", args.device))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::FrameReader;
    use std::sync::mpsc;
    use std::time::{Duration, Instant};

    fn frame(pgn: u32) -> RawFrame {
        RawFrame::new(None, 6, pgn, 1, 255, [0u8; 8])
    }

    /// A bus frame resets the clock; a frame canboat made does not, and
    /// once the time is up reading ends with an error.
    #[test]
    fn a_silent_bus_times_out() {
        let (tx, mut rx) = mpsc::channel();
        let mut idle = Idle::new(&mut rx, 1);
        tx.send(frame(127508)).unwrap();
        assert_eq!(idle.read_frame().unwrap().unwrap().pgn, 127508);
        let start = Instant::now();
        let sender = thread::spawn(move || {
            for _ in 0..3 {
                thread::sleep(Duration::from_millis(200));
                let _ = tx.send(frame(262400));
            }
            thread::sleep(Duration::from_secs(2));
            drop(tx);
        });
        let mut synthetic = 0;
        let err = loop {
            match idle.read_frame() {
                Ok(Some(_)) => synthetic += 1,
                Ok(None) => panic!("ended without a timeout"),
                Err(e) => break e,
            }
        };
        assert_eq!(synthetic, 3);
        assert_eq!(err.kind(), io::ErrorKind::TimedOut);
        let waited = start.elapsed();
        assert!(waited >= Duration::from_millis(900) && waited < Duration::from_millis(1500));
        sender.join().unwrap();
    }

    /// Without a timeout, and when the device goes, reading just ends.
    #[test]
    fn a_closed_device_ends_the_stream() {
        let (tx, mut rx) = mpsc::channel::<RawFrame>();
        drop(tx);
        assert!(Idle::new(&mut rx, 0).read_frame().unwrap().is_none());
        assert!(Idle::new(&mut rx, 5).read_frame().unwrap().is_none());
    }
}
