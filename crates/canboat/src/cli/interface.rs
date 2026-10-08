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
use std::net::TcpStream;
use std::sync::Arc;
use std::sync::atomic::AtomicU8;
use std::thread;

use anyhow::{Context, Result};

use crate::engine::{BusProtocol, RawFrame};
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
    /// TCP (`tcp://host[:port]`) or receive-only UDP (`udp://[bind:]port`),
    /// or the YDNU-02 USB gateway over its serial port (38400 baud).
    Ydwg,
    /// Actisense W2K-1 in N2K ASCII mode, over TCP.
    #[value(name = "w2k-ascii")]
    W2kAscii,
    /// Actisense BST-D0 (binary, reassembled NMEA 2000) over TCP: a W2K-1
    /// or PRO-NDC-1E2K in Actisense mode.
    #[value(name = "bst-d0", alias = "w2k-actisense")]
    BstD0,
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
            Kind::W2kAscii | Kind::BstD0 => "w2k-gateway",
        }
    }

    /// Default serial baud (0 for non-serial transports).
    fn default_baud(self) -> u32 {
        match self {
            Kind::Ngt1 => 115_200,
            Kind::Ikonvert => 230_400,
            Kind::Ydwg => 38_400,
            Kind::Maretron | Kind::Socketcan | Kind::W2kAscii | Kind::BstD0 => 0,
        }
    }
}

#[derive(Debug, clap::Args)]
pub struct Args {
    /// Gateway type.
    #[arg(long, value_enum)]
    kind: Kind,

    /// Endpoint: serial path (ngt1/ikonvert, and ydwg for a YDNU-02),
    /// `tcp://host[:port]` (ydwg, w2k-*, and ngt1/ikonvert through a
    /// network bridge), `udp://[bind:]port` (ydwg, receive only),
    /// `host:port` (maretron), or CAN interface name such as `can0`
    /// (socketcan). An FTDI-based
    /// gateway such as the NGT-1 can also be opened directly over USB as
    /// `usb`, `usb:SERIAL` or `usb:VVVV:PPPP[:SERIAL]` — for macOS, whose
    /// serial driver does not recognise the NGT-1.
    #[arg(value_name = "DEVICE")]
    device: String,

    /// Serial baud rate. Defaults to 115200 (ngt1) / 230400 (ikonvert) /
    /// 38400 (ydwg).
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

    /// SocketCAN: preferred source address to claim.
    #[arg(short = 'a', long, value_name = "ADDR", default_value_t = 0)]
    address: u8,

    /// SocketCAN: passive sniff — skip the ISO address-claim handshake.
    #[arg(short = 'n', long)]
    no_claim: bool,

    /// SocketCAN: unique number for the ISO NAME (default derived from
    /// the machine id, stable per-host across restarts; on a machine
    /// that can't be identified, a random one stored with the server's
    /// state files).
    #[arg(short = 'u', long, value_name = "N", default_value_t = 0)]
    unique: u32,

    /// SocketCAN: manufacturer code for the ISO NAME (999 = Signal K).
    #[arg(short = 'm', long, value_name = "N", default_value_t = 999)]
    manufacturer: u16,

    /// SocketCAN: Heartbeat (PGN 126993) interval in ms, up to 65532;
    /// 0 disables.
    #[arg(
        long,
        alias = "hb",
        value_name = "MS",
        default_value_t = 60_000,
        value_parser = clap::value_parser!(u64).range(0..=65_532)
    )]
    heartbeat: u64,

    /// SocketCAN: ISO NAME System Instance, 0..15. Default 15 (max) so
    /// our NAME yields to real hardware rather than stealing addresses.
    #[arg(long, alias = "si", value_name = "N", default_value_t = 15)]
    system_instance: u8,

    /// Quit if no frame is received for this many seconds (0 disables).
    /// SocketCAN only.
    #[arg(short = 't', long, value_name = "SECONDS", default_value_t = 0)]
    timeout: u64,

    /// `--protocol j1939` frames the traffic as J1939 (single frames and ISO
    /// TP, no fast-packet) and, on SocketCAN, leaves out the NMEA 2000
    /// Heartbeat, Product Information and PGN lists. SocketCAN and YDWG
    /// only: the other gateways do NMEA 2000 framing themselves.
    #[command(flatten)]
    protocol: crate::cli::protocol::ProtocolArgs,
}

pub fn run(args: Args) -> Result<()> {
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
            copy(&mut handle.frames_rx, &mut writer).context("reading from device")?;
        }
        // Write-only: stdin PLAIN → device on the main thread (so we
        // exit when stdin ends); received frames are drained and
        // discarded so the channel can't back up.
        (false, true) => {
            let mut sender = handle.frame_sender();
            let mut rx = handle.frames_rx;
            let drain = thread::Builder::new()
                .name("frame-drain".into())
                .spawn(move || {
                    let _ = copy(&mut rx, &mut DiscardWriter);
                })
                .expect("spawn frame drain");
            pump_stdin(&mut sender, protocol).context("sending stdin frames")?;
            drop(drain);
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
            copy(&mut handle.frames_rx, &mut writer).context("reading from device")?;
            drop(pump); // device closed; don't wait on a blocked stdin read
        }
    }
    Ok(())
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
    // Quick's 11-bit frames: only SocketCAN is known to pass them on.
    if protocol == BusProtocol::Quick && !matches!(args.kind, Kind::Socketcan) {
        anyhow::bail!(
            "--protocol quick needs socketcan: the other gateways pass only 29-bit frames"
        );
    }
    if protocol != BusProtocol::Nmea2000 && !matches!(args.kind, Kind::Socketcan | Kind::Ydwg) {
        anyhow::bail!(
            "--protocol {protocol} needs a gateway that passes raw CAN frames (socketcan or ydwg); \
             this one does NMEA 2000 framing itself"
        );
    }
    match args.kind {
        Kind::Ngt1 => {
            let (r, w) = open_stream(args)?;
            Ok(device::ngt1::run(r, w))
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
            let endpoint = args.device.strip_prefix("tcp://").unwrap_or(&args.device);
            let endpoint = if endpoint.contains(':') {
                endpoint.to_string()
            } else {
                format!("{endpoint}:6543")
            };
            let stream = TcpStream::connect(&endpoint)
                .with_context(|| format!("connecting to {endpoint}"))?;
            let reader: Box<dyn Read + Send> =
                Box::new(stream.try_clone().context("cloning TCP stream")?);
            let writer: Box<dyn Write + Send> = Box::new(stream);
            let config = device::maretron::Config {
                password: args.password.clone().unwrap_or_default(),
                fixtime: None,
            };
            Ok(device::maretron::run(reader, writer, config))
        }
        Kind::Ydwg => {
            // A network endpoint is a YDWG-02 / YDEN; anything else is the
            // serial port of a YDNU-02, which is first put in RAW mode.
            let network = ["tcp://", "udp://"]
                .iter()
                .any(|p| args.device.starts_with(p));
            let (reader, writer, init) = if network {
                let (r, w) = open_line_endpoint(&args.device, 1457)?;
                (r, w, &[][..])
            } else {
                let baud = args.baud.unwrap_or_else(|| args.kind.default_baud());
                let (r, w) = open_serial_rw(&args.device, baud)
                    .with_context(|| format!("opening serial port {}", args.device))?;
                (r, w, device::line_gateway::YDNU_RAW_MODE)
            };
            Ok(device::line_gateway::run(
                reader,
                writer,
                device::line_gateway::LineFormat::YdwgRaw,
                protocol,
                init,
            ))
        }
        Kind::W2kAscii => {
            let (reader, writer) = open_line_endpoint(&args.device, 60002)?;
            Ok(device::line_gateway::run(
                reader,
                writer,
                device::line_gateway::LineFormat::N2kAscii,
                protocol,
                &[],
            ))
        }
        Kind::BstD0 => {
            let (reader, writer) = open_tcp(&args.device, 60002)?;
            Ok(device::bst_d0::run(reader, writer))
        }
        Kind::Socketcan => {
            let config = device::socketcan::Config {
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
            };
            let claim = Arc::new(AtomicU8::new(config.address));
            // On non-Linux this returns ErrorKind::Unsupported.
            device::socketcan::run(&args.device, config, claim)
                .with_context(|| format!("opening SocketCAN interface {}", args.device))
        }
    }
}

/// Open the serial transport as an independent `(reader, writer)` pair.
/// `interface` is live-device only — captured byte streams are decoded
/// with `canboat convert`, not here.
fn open_stream(args: &Args) -> Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    // A serial-protocol gateway reached over the network (the NavLink2
    // serves the iKonvert protocol on TCP 6001; ser2net setups do the
    // same for an NGT-1): same codec, different transport.
    if args.device.starts_with("tcp://") {
        return open_tcp(&args.device, 6001);
    }
    let baud = args.baud.unwrap_or_else(|| args.kind.default_baud());
    open_serial_rw(&args.device, baud)
        .with_context(|| format!("opening serial port {}", args.device))
}

/// Open `tcp://host[:port]` as a cloned read/write stream pair.
fn open_tcp(
    device: &str,
    default_port: u16,
) -> Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    let endpoint = device.strip_prefix("tcp://").unwrap_or(device);
    let endpoint = if endpoint.contains(':') {
        endpoint.to_string()
    } else {
        format!("{endpoint}:{default_port}")
    };
    let stream =
        TcpStream::connect(&endpoint).with_context(|| format!("connecting to {endpoint}"))?;
    let reader: Box<dyn Read + Send> = Box::new(stream.try_clone().context("cloning TCP stream")?);
    let writer: Box<dyn Write + Send> = Box::new(stream);
    Ok((reader, writer))
}

/// Open a line-gateway endpoint: `tcp://host[:port]` bidirectional, or
/// `udp://[bind:]port` receive-only (the YDWG-02's UDP mode has no
/// transmit path — sent frames are dropped with a warning).
fn open_line_endpoint(
    device: &str,
    default_tcp_port: u16,
) -> Result<(Box<dyn Read + Send>, Box<dyn Write + Send>)> {
    if let Some(rest) = device.strip_prefix("udp://") {
        let bind = if rest.contains(':') {
            rest.to_string()
        } else {
            format!("0.0.0.0:{rest}")
        };
        let sock =
            std::net::UdpSocket::bind(&bind).with_context(|| format!("binding UDP {bind}"))?;
        log::warn!("UDP gateway mode is receive-only; transmitted frames are dropped");
        let reader: Box<dyn Read + Send> = Box::new(UdpRead(sock));
        let writer: Box<dyn Write + Send> = Box::new(std::io::sink());
        return Ok((reader, writer));
    }
    open_tcp(device, default_tcp_port)
}

/// [`Read`] over a bound UDP socket: each `read` yields one datagram.
struct UdpRead(std::net::UdpSocket);

impl Read for UdpRead {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        let (n, _peer) = self.0.recv_from(buf)?;
        Ok(n)
    }
}
