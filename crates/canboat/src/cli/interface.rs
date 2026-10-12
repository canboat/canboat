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

use crate::cli::device::{self as gateway_args, DeviceArgs, Kind};
use crate::engine::{BusProtocol, RawFrame};
use crate::io::device::{self, DeviceHandle};
use crate::io::{FrameWriter, PlainWriter, copy, open_serial_rw};

/// `# format=FAST` tag every canboat reader prepends to its PLAIN
/// stream so a downstream analyzer knows the frames are coalesced.
const FORMAT_FAST_HEADER: &[u8] = b"# format=FAST\n";

/// Producer name stamped into the startup record. Kept identical to the
/// retired standalone binary so parity output is unchanged.
fn producer(kind: Kind) -> &'static str {
    match kind {
        Kind::Ngt1 => "actisense-serial",
        Kind::Ikonvert => "ikonvert-serial",
        Kind::Maretron => "maretron-ipg",
        Kind::Socketcan => "socketcan-serial",
        Kind::Ydwg => "ydwg-gateway",
        Kind::W2kAscii | Kind::BstD0 | Kind::Bst95 => "w2k-gateway",
        Kind::Canalyst => "canalyst",
    }
}

#[derive(Debug, clap::Args)]
#[command(after_long_help = gateway_args::GATEWAY_HELP)]
pub struct Args {
    /// The kind of gateway.
    #[arg(long, value_enum, help_heading = gateway_args::GATEWAY)]
    kind: Kind,

    /// Where it is: a serial port, `usb`, `tcp://host[:port]`,
    /// `udp://host:port`, `host:port` or a CAN interface such as `can0`.
    ///
    /// Which one depends on the gateway; see the table at the end. An
    /// FTDI-based gateway such as the NGT-1 can also be opened directly
    /// over USB as `usb`, `usb:SERIAL` or `usb:VVVV:PPPP[:SERIAL]`: on
    /// macOS, whose serial driver does not recognise the NGT-1. `udp://[bind:]port` only receives.
    #[arg(value_name = "DEVICE")]
    device: String,

    /// Only read: write the bus to stdout, ignore stdin.
    #[arg(short = 'r', long = "read-only", conflicts_with = "write_only", help_heading = STREAM)]
    read_only: bool,

    /// Only write: send stdin to the bus, drop what is received.
    #[arg(short = 'w', long = "write-only", help_heading = STREAM)]
    write_only: bool,

    /// Quit with an error after this many seconds without a frame; 0: never.
    ///
    /// A gateway that went away without closing the connection is then
    /// noticed, and a supervisor can restart canboat.
    #[arg(short = 't', long, value_name = "SECONDS", default_value_t = 0, hide_default_value = true, help_heading = STREAM)]
    timeout: u64,

    #[command(flatten)]
    gateway: DeviceArgs,

    /// `--system-instance`, as the C socketcan-serial spells it (not on
    /// `canboat server`, where `--si` picks SI units).
    #[arg(long = "si", hide = true, value_name = "N", value_parser = clap::value_parser!(u8).range(0..=15))]
    si: Option<u8>,
}

/// The help heading of the stdin/stdout options.
const STREAM: &str = "stdin and stdout";

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
    let protocol = args.gateway.protocol();
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
    let rec =
        crate::engine::startup_record(env!("CARGO_PKG_VERSION"), producer(args.kind), &args.device);
    let mut line = String::with_capacity(160);
    crate::engine::format::plain::write_line(&mut line, &rec).ok();
    out.write_all(line.as_bytes())?;
    out.write_all(b"\n")?;
    out.flush()
}

/// Open the selected transport and start its device codec.
fn open_device(args: &Args) -> Result<DeviceHandle> {
    let gateway = &args.gateway;
    gateway.check(Some(args.kind))?;
    if args.si.is_some() && !matches!(args.kind, Kind::Socketcan | Kind::Canalyst | Kind::Ngt1) {
        anyhow::bail!("--si: --kind {} does not have it", args.kind.name());
    }
    let protocol = gateway.protocol();
    let lists = gateway.pgn_lists();
    let unsupported = |what: &str, pgns: &[u32]| {
        if !pgns.is_empty() {
            log::warn!(
                "--kind {} has no {what} list; ignoring {pgns:?}",
                args.kind.name()
            );
        }
    };
    match args.kind {
        Kind::Ngt1 => {
            unsupported("receive PGN", &lists.rx);
            let (r, w) = open_stream(args)?;
            let config = device::ngt1::Config {
                operating_mode: gateway.operating_mode(),
                pgn_lists: crate::engine::pgn_list::PgnLists {
                    tx: lists.tx.clone(),
                    rx: Vec::new(),
                },
                ..Default::default()
            };
            // An AOS simulator, which claims no address, gets a node.
            let node = device::ngt1::NodeConfig {
                model_version: Some("canboat-aos"),
                ..node_config(args)
            };
            let claim = Arc::new(AtomicU8::new(device::socketcan::CLAIM_UNCLAIMED));
            Ok(device::ngt1::run_with_node(r, w, config, node, claim))
        }
        Kind::Ikonvert => {
            let (r, w) = open_stream(args)?;
            let config = device::ikonvert::Config {
                rx_list: gateway.ikonvert_rx_list(),
                pgn_lists: lists,
                rate_limit_off: gateway.no_tx_rate_limit,
                skip_init: false,
                ..Default::default()
            };
            Ok(device::ikonvert::run(r, w, config))
        }
        Kind::Maretron => {
            unsupported("receive PGN", &lists.rx);
            unsupported("transmit PGN", &lists.tx);
            // The C maretron-ipg takes `tcp://<host>[:<port>]` with the
            // port defaulting to 6543; the argv[0] shim must stay
            // drop-in compatible with that syntax (signalk-server
            // spawns it exactly so), and the bare `host:port` form
            // keeps working.
            let (reader, writer) = crate::io::net::open_tcp(&args.device, 6543)?;
            let config = device::maretron::Config {
                password: gateway.password.clone().unwrap_or_default(),
                fixtime: None,
            };
            Ok(device::maretron::run(reader, writer, config))
        }
        Kind::Ydwg | Kind::W2kAscii | Kind::BstD0 | Kind::Bst95 => {
            unsupported("receive PGN", &lists.rx);
            unsupported("transmit PGN", &lists.tx);
            let gateway_kind = args.kind.gateway().expect("a gateway kind");
            let target = device::gateway::Gateway {
                kind: gateway_kind,
                device: args.device.clone(),
            };
            Ok(device::gateway::open(&target, gateway.baud, protocol)?)
        }
        Kind::Canalyst => open_canalyst(args),
        Kind::Socketcan => {
            let config = device::socketcan::Config {
                // Configure the link only when asked to; otherwise use the
                // interface as it was set up.
                configure_link: gateway.bitrate.is_some(),
                bitrate: gateway.bitrate.unwrap_or(250_000),
                ..node_config(args)
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
fn node_config(args: &Args) -> device::socketcan::Config {
    let node = args.gateway.node_config();
    device::socketcan::Config {
        system_instance: args.si.unwrap_or(node.system_instance),
        timeout_secs: args.timeout,
        // Probing creates the dir, so only when it may be needed.
        state_dir: (node.unique == 0).then(super::app::resolve_config_dir),
        ..node
    }
}

/// Open a CANalyst-II over USB and run a node on it, as for SocketCAN.
fn open_canalyst(args: &Args) -> Result<DeviceHandle> {
    let selector = crate::io::usb::Selector::parse(&args.device)
        .ok_or_else(|| anyhow::anyhow!("a CANalyst-II is named `usb` or `usb:VVVV:PPPP`"))??;
    let config = device::socketcan::Config {
        model_version: Some("canboat-canalyst"),
        bitrate: args.gateway.bitrate.unwrap_or(250_000),
        ..node_config(args)
    };
    let channel = args.gateway.channel.unwrap_or(0);
    let (reader, writer) = crate::io::canalyst::open_rw(&selector, channel, config.bitrate)
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
    let baud = args
        .gateway
        .baud
        .unwrap_or_else(|| args.kind.default_baud());
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
