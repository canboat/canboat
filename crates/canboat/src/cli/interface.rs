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

use std::io::{self, BufRead, BufWriter, Write};
use std::thread;
use std::time::Duration;

use anyhow::{Context, Result};

use crate::cli::device::{self as gateway_args, DeviceArgs, KindArg};
use crate::engine::pgn_list::RxFilter;
use crate::engine::{BusProtocol, RawFrame};
use crate::io::device::kind::{ActisenseFormat, Kind, Setting};
use crate::io::device::{self, DeviceHandle};
use crate::io::{FrameWriter, PlainWriter, copy};

/// `# format=FAST` tag every canboat reader prepends to its PLAIN
/// stream so a downstream analyzer knows the frames are coalesced.
const FORMAT_FAST_HEADER: &[u8] = b"# format=FAST\n";

/// Producer name stamped into the startup record. Kept identical to the
/// retired standalone binary so parity output is unchanged.
fn producer(kind: Kind) -> &'static str {
    match kind {
        Kind::Actisense(ActisenseFormat::NgtActisense) => "actisense-serial",
        Kind::Actisense(_) => "w2k-gateway",
        Kind::Aos => "aos",
        Kind::Ikonvert => "ikonvert-serial",
        Kind::Maretron => "maretron-ipg",
        Kind::SocketCan => "socketcan-serial",
        Kind::YachtDevices => "ydwg-gateway",
        Kind::Miniplex => "miniplex",
        Kind::SeaSmart => "seasmart",
        Kind::Canalyst => "canalyst",
        Kind::UsbCanA => "usb-can-a",
    }
}

#[derive(Debug, clap::Args)]
#[command(after_long_help = gateway_args::GATEWAY_HELP)]
pub struct Args {
    /// The kind of gateway.
    #[arg(long = "kind", value_enum, value_name = "KIND", help_heading = gateway_args::GATEWAY)]
    kind_arg: KindArg,

    /// The gateway `--kind` and `--data-format` name, resolved by `run`.
    #[arg(skip = Kind::NGT1)]
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

pub fn run(mut args: Args) -> Result<()> {
    args.kind = args.kind_arg.resolve(args.gateway.data_format)?;
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
    // Why the device was given up on, or went away, if its frames stop
    // for that: then exit with an error, so a service manager restarts us.
    let failure = handle.failure();
    let lost = handle.lost();
    // `--rx-filter`, for what the gateway does not filter itself; an
    // iKonvert has done it already, which this does not undo.
    let filter = RxFilter::new(&args.gateway.rx_filter);
    let stdout = io::stdout();
    let mut out = BufWriter::new(stdout.lock());
    write_prologue(&mut out, args).context("writing prologue")?;

    match (args.read_only, args.write_only) {
        // Read-only: device → PLAIN stdout, ignore stdin.
        (true, _) => {
            let mut writer = PlainWriter::new(&mut out);
            let mut rx = Idle::new(&mut handle.frames_rx, args.timeout).filtered(filter.clone());
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
                    let mut rx = Idle::new(&mut rx, timeout).filtered(filter);
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
            let mut rx = Idle::new(&mut handle.frames_rx, args.timeout).filtered(filter.clone());
            copy(&mut rx, &mut writer).context("reading from device")?;
            drop(pump); // device closed; don't wait on a blocked stdin read
        }
    }
    match failure.get().or(lost.get()) {
        Some(why) => anyhow::bail!("{why}"),
        None => Ok(()),
    }
}

/// The device's frames, ended with an error once none has come off the
/// bus for `timeout_secs` (0: never). A frame canboat makes itself (a
/// gateway status on a timer) does not count: it comes even from a
/// gateway that has gone. With a filter (`--rx-filter`), the frames it
/// does not pass are dropped, though they still show the bus is there.
struct Idle<'a> {
    rx: &'a mut std::sync::mpsc::Receiver<RawFrame>,
    timeout: Option<std::time::Duration>,
    last: std::time::Instant,
    filter: Option<RxFilter>,
}

impl<'a> Idle<'a> {
    fn new(rx: &'a mut std::sync::mpsc::Receiver<RawFrame>, timeout_secs: u64) -> Self {
        Self {
            rx,
            timeout: (timeout_secs > 0).then(|| std::time::Duration::from_secs(timeout_secs)),
            last: std::time::Instant::now(),
            filter: None,
        }
    }

    /// Pass on only what `filter` passes.
    fn filtered(self, filter: Option<RxFilter>) -> Self {
        Self { filter, ..self }
    }

    fn passes(&self, frame: &RawFrame) -> bool {
        self.filter.as_ref().is_none_or(|f| f.passes(frame.pgn))
    }
}

impl crate::io::FrameReader for Idle<'_> {
    fn read_frame(&mut self) -> io::Result<Option<RawFrame>> {
        use std::sync::mpsc::RecvTimeoutError;
        let Some(timeout) = self.timeout else {
            return Ok(self.rx.iter().find(|frame| self.passes(frame)));
        };
        loop {
            let left = timeout.saturating_sub(self.last.elapsed());
            match self.rx.recv_timeout(left) {
                Ok(frame) => {
                    if frame.pgn < crate::engine::codec::SYNTHETIC_PGN_START {
                        self.last = std::time::Instant::now();
                    }
                    if self.passes(&frame) {
                        return Ok(Some(frame));
                    }
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
    let (gateway, kind) = (&args.gateway, args.kind);
    gateway.check(Some(kind))?;
    if args.si.is_some() && !kind.takes(Setting::Node) {
        anyhow::bail!("--si: --kind {kind} does not have it");
    }
    let mut dev = gateway.device(kind, &args.device);
    if kind.takes(Setting::Node) {
        if let Some(instance) = args.si {
            dev = dev.system_instance(instance);
        }
        dev = dev.timeout(Duration::from_secs(args.timeout));
        // Probing creates the dir, so only when it may be needed.
        if gateway.unique.is_none() {
            dev = dev.state_dir(super::app::resolve_config_dir());
        }
    }
    if args.write_only && !dev.capabilities().sends {
        anyhow::bail!(
            "--write-only: --kind {kind} at {} only receives; use tcp:// to send",
            args.device
        );
    }
    dev.open(gateway.protocol()).map_err(|e| {
        // What the gateway lacks says which, already.
        if e.kind() == io::ErrorKind::InvalidInput {
            anyhow::Error::new(e)
        } else {
            anyhow::Error::new(e).context(format!("opening --kind {kind} {}", args.device))
        }
    })
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

    /// With --rx-filter, only the named PGNs, network management and
    /// canboat's own come through; with a timeout too, a dropped frame still
    /// shows the bus is there.
    #[test]
    fn the_rx_filter_drops_what_was_not_asked_for() {
        for timeout in [0, 5] {
            let (tx, mut rx) = mpsc::channel();
            for pgn in [130306, 127250, 60928, 129029, 262400] {
                tx.send(frame(pgn)).unwrap();
            }
            drop(tx);
            let mut idle = Idle::new(&mut rx, timeout).filtered(RxFilter::new(&[127250]));
            let mut pgns = Vec::new();
            while let Some(f) = idle.read_frame().unwrap() {
                pgns.push(f.pgn);
            }
            assert_eq!(pgns, [127250, 60928, 262400]);
        }
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
