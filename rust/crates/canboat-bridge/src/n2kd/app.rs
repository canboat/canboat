// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `n2kd`: TCP multiplexer for an analyzer JSON stream.
//!
//! Mirrors `canboat/n2kd/main.c`. The analyzer feeds JSON-per-line on
//! stdin; we fan that out to several TCP ports of clients:
//!
//! | Port (default)   | Behaviour                                                                      |
//! |------------------|--------------------------------------------------------------------------------|
//! | `port`     (2597) | **JSON snapshot** — on connect, send the latest line per `(pgn, src)`, close. |
//! | `port+1`   (2598) | **JSON stream** — each connected client receives every line as it arrives.    |
//! | `port+2`   (2599) | **NMEA 0183 stream** — converted sentences (HDG / MWV / DPT / RSA / VTG / …). |
//! | `port+3`   (2600) | **Raw input** (write-only) — clients write canboat PLAIN/FAST; we forward to stdout. |
//! | `port+4`   (2601) | **AIS-only stream** — the AIS-related PGNs in JSON, unconverted.              |
//! | `port+5`   (2602) | **Status stream** — periodic `{"clients":N,"pgns":[…]}` snapshots.            |
//!
//! These offsets match `canboat/n2kd/main.c` (`SERVER_JSON`,
//! `SERVER_JSON_STREAM`, `SERVER_NMEA0183_STREAM`, `SERVER_INPUT_STREAM`,
//! `SERVER_AIS`, `SERVER_STATUS`) one-for-one.
//!
//! Plus an optional UDP NMEA 0183 broadcast (`--udp183 <host:port>`)
//! and a periodic device-claim / product-info auto-request engine
//! (canboat's `requestAddressClaimAndProductInfo`) that emits ISO
//! request PLAIN lines on stdout for devices we've seen.
//!
//! AIS NMEA 0183 conversion (AIVDM bit-packing) is deferred — those
//! PGNs flow unchanged through the AIS port; the NMEA stream simply
//! doesn't emit AIVDM sentences yet.

use crate::n2kd::nmea_filter::NmeaFilter;
use crate::n2kd::request_engine::{self, RequestEngine};
use crate::n2kd::serving::{Hub as WireHub, tcp as serving_tcp};
use crate::n2kd::{json, nmea0183};

use std::io::{self, BufRead, BufReader, Write};
use std::net::{Ipv4Addr, SocketAddrV4, TcpListener, TcpStream, ToSocketAddrs, UdpSocket};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{SystemTime, UNIX_EPOCH};

use anyhow::{Context, Result};
use canboat_core::PgnDatabase;
use canboat_core::format::days_to_ymd;
use canboat_core::snapshot::SnapshotStore;

use crate::n2kd::nmea0183::RateLimiter;

/// Default TCP base port.
const DEFAULT_PORT: u16 = 2597;

#[derive(Debug, clap::Args)]
#[command(after_help = canboat_cli::help_footer())]
pub struct Args {
    /// Base TCP port. `+1`=stream, `+2`=nmea0183, `+3`=raw input, `+4`=ais, `+5`=status. Matches canboat C n2kd.
    #[arg(short = 'p', long, default_value_t = DEFAULT_PORT)]
    port: u16,

    /// Filter incoming JSON by source address. Comma-separated list
    /// of `u8`; prepend `!` for a negative match (e.g. `!1,2`
    /// allows everything except sources 1 and 2). Mirrors canboat
    /// `srcFilter`.
    #[arg(long)]
    src_filter: Option<String>,

    /// Rate-limit each (src, kind) of NMEA 0183 sentence to at most
    /// one per second. Mirrors canboat's `--rate-limit`.
    #[arg(long)]
    rate_limit: bool,

    /// Restrict mode: suppress all stdout output (claim requests in
    /// normal mode, NMEA sentences in `--nmea0183` mode). Matches
    /// canboat's `-r`.
    #[arg(short = 'r', long)]
    restrict: bool,

    /// Bind on `0.0.0.0` instead of `127.0.0.1`.
    #[arg(long)]
    public: bool,

    /// UDP-broadcast each NMEA 0183 sentence. Canboat's C tool takes
    /// two positional args (`-u <host> <port>`); we accept that form
    /// for drop-in script compatibility, plus a single-arg
    /// `<host:port>` shorthand. Either way, the value lands in the
    /// LAN where OpenCPN / Navionics receivers can pick it up.
    #[arg(
        short = 'u',
        long = "udp183",
        value_names = &["HOST", "PORT"],
        num_args = 1..=2,
    )]
    udp183: Vec<String>,

    /// Start no TCP servers and send NMEA 0183 / AIVDM data on
    /// stdout. Mirrors canboat's `--nmea0183` flag — mainly for
    /// pipeline / debug use.
    #[arg(long)]
    nmea0183: bool,

    /// Copy data received from TCP clients on the raw-input port back
    /// to stdout (as well as forwarding to the analyzer pipeline).
    /// Matches canboat C's `-o` flag. Off by default to keep stdout
    /// clean for downstream consumers.
    #[arg(short = 'o', long = "output")]
    output_copy: bool,

    /// Pin the log timestamp string. Matches canboat C's `-fixtime`,
    /// used by deterministic-output tests. Accepted but currently
    /// inert in Rust's `env_logger` setup.
    #[arg(long = "fixtime", value_name = "STR")]
    fixtime: Option<String>,

    /// Suppress the periodic ISO Address Claim / Product Info request
    /// engine. By default it's on (matching canboat C); `-r` /
    /// `--restrict` also turns it off.
    #[arg(long)]
    no_request_claims: bool,

    /// Verbose / debug logging — alias of `-d`.
    #[arg(short = 'v', long)]
    verbose: bool,

    /// Debug logging.
    #[arg(short = 'd', long)]
    debug: bool,

    /// Quiet — only show errors.
    #[arg(short = 'q', long)]
    quiet: bool,

    /// Mute redundant NMEA 0183 output per device NAME using the rules
    /// in this JSON file: when several devices report the same
    /// measurement, keep the one the rules name and drop the rest.
    /// Devices with no rule pass through. The `src → NAME` mapping is
    /// learned from ISO Address Claims on the stream.
    #[arg(long = "nmea0183-filter", value_name = "PATH")]
    nmea0183_filter: Option<std::path::PathBuf>,
}

pub fn run(args: Args) -> Result<()> {
    let cli = args;
    let level = if cli.quiet {
        "error"
    } else if cli.debug || cli.verbose {
        "debug"
    } else {
        "info"
    };
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(level)).init();
    canboat_cli::log_startup(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));

    let src_filter = parse_src_filter(cli.src_filter.as_deref())?;
    let bind_addr: Ipv4Addr = if cli.public {
        Ipv4Addr::UNSPECIFIED
    } else {
        Ipv4Addr::LOCALHOST
    };
    // `-u host port` (canboat C) and `-u host:port` (ergonomic) both
    // arrive as `Vec<String>` here. Re-stitch into a single
    // `host:port` for `open_udp_broadcast` to resolve.
    let udp = match cli.udp183.as_slice() {
        [] => None,
        [one] => Some(open_udp_broadcast(one)?),
        [host, port] => Some(open_udp_broadcast(&format!("{host}:{port}"))?),
        _ => anyhow::bail!("--udp183 / -u accepts at most 2 args (host [port])"),
    };
    if let Some(ts) = cli.fixtime.as_deref() {
        log::debug!("--fixtime accepted but not applied to logging: {ts}");
    }

    let filter = match cli.nmea0183_filter.as_deref() {
        Some(path) => Some(NmeaFilter::load(path)?),
        None => None,
    };
    let engine = Arc::new(RequestEngine::new());
    let mut hub = Hub::new(src_filter, cli.rate_limit, udp, Arc::clone(&engine), filter);
    hub.nmea_to_stdout = cli.nmea0183 && !cli.restrict;
    let hub = Arc::new(hub);

    if !cli.nmea0183 {
        // Live streams + connect-and-dump snapshot/AIS ports are the
        // shared `n2kd::serving::tcp` listeners (identical to `server`'s
        // read-only ports), fed off the same hubs / snapshot cache. No
        // format header on the JSON or 0183 streams. The status and
        // raw-input ports stay n2kd-local: no `serving` equivalent for
        // status, and raw-input carries the filter control channel.
        // The daemon runs until process exit, so its listeners pass
        // `stop = None`: block on accept forever, no shutdown handle.
        serving_tcp::spawn_stream_server(
            "json-stream",
            bind_addr,
            cli.port + 1,
            Arc::clone(&hub.json_hub),
            None,
            None,
        )?;
        serving_tcp::spawn_stream_server(
            "nmea0183-stream",
            bind_addr,
            cli.port + 2,
            Arc::clone(&hub.nmea_hub),
            None,
            None,
        )?;
        spawn_raw_input_listener(bind_addr, cli.port + 3, cli.output_copy && !cli.restrict)?;
        serving_tcp::spawn_ais_snapshot(bind_addr, cli.port + 4, Arc::clone(&hub.cache), None)?;
        serving_tcp::spawn_snapshot(bind_addr, cli.port, Arc::clone(&hub.cache), None, None)?;
        spawn_status_listener(bind_addr, cli.port + 5, Arc::clone(&hub))?;
    }
    // Default-on like canboat C; `-r` / `--restrict` and the explicit
    // `--no-request-claims` opt-out both turn it off. Skip in
    // `--nmea0183` debug mode too — stdout is already busy.
    if !cli.restrict && !cli.no_request_claims && !cli.nmea0183 {
        spawn_claim_request_engine(Arc::clone(&engine));
    }

    run_stdin_pump(&hub)
}

fn parse_src_filter(arg: Option<&str>) -> Result<Option<SrcFilter>> {
    let Some(s) = arg else { return Ok(None) };
    let s = s.trim();
    let (negate, body) = match s.strip_prefix('!') {
        Some(rest) => (true, rest),
        None => (false, s),
    };
    let mut srcs = Vec::new();
    for tok in body.split(',') {
        let tok = tok.trim();
        if tok.is_empty() {
            continue;
        }
        let v: u8 = tok
            .parse()
            .with_context(|| format!("--src-filter token {tok:?}"))?;
        srcs.push(v);
    }
    Ok(Some(SrcFilter { srcs, negate }))
}

#[derive(Debug, Clone)]
struct SrcFilter {
    srcs: Vec<u8>,
    /// `true` → match `!N` style: allow everything except listed.
    negate: bool,
}

impl SrcFilter {
    fn allows(&self, src: u8) -> bool {
        let hit = self.srcs.contains(&src);
        if self.negate { !hit } else { hit }
    }
}

/// Open a UDP socket bound to an ephemeral local port; we'll
/// `send_to` the target on each broadcast. The string is `host:port`.
fn open_udp_broadcast(target: &str) -> Result<UdpBroadcast> {
    let mut iter = target
        .to_socket_addrs()
        .with_context(|| format!("resolving udp183 target {target:?}"))?;
    let addr = iter
        .next()
        .with_context(|| format!("no addresses for {target:?}"))?;
    let sock = UdpSocket::bind("0.0.0.0:0").context("binding ephemeral UDP socket")?;
    // Many embedded receivers listen on the broadcast address — enable
    // SO_BROADCAST so a `255.255.255.255` target works too.
    sock.set_broadcast(true).ok();
    Ok(UdpBroadcast { sock, addr })
}

struct UdpBroadcast {
    sock: UdpSocket,
    addr: std::net::SocketAddr,
}
impl UdpBroadcast {
    fn send(&self, bytes: &[u8]) {
        let _ = self.sock.send_to(bytes, self.addr);
    }
}

fn spawn_raw_input_listener(bind: Ipv4Addr, port: u16, copy_to_stdout: bool) -> Result<()> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding raw-input on {bind}:{port}"))?;
    log::info!("listening on {bind}:{port} (raw-input)");
    thread::Builder::new()
        .name("n2kd-raw-input".into())
        .spawn(move || {
            loop {
                let stream = match listener.accept() {
                    Ok((s, _)) => s,
                    Err(e) => {
                        log::warn!("accept on raw-input: {e}");
                        continue;
                    }
                };
                thread::Builder::new()
                    .spawn(move || run_raw_input_client(stream, copy_to_stdout))
                    .ok();
            }
        })
        .context("spawning raw-input listener")?;
    Ok(())
}

/// Status port — connect-and-dump, just like the JSON snapshot port,
/// but emits the canboat C `{<pgn>:{description,<src>:{last,interval,
/// count}}}` shape. Mirrors `n2kd/main.c`'s `CLIENT_STATUS_STREAM`
/// behavior.
fn spawn_status_listener(bind: Ipv4Addr, port: u16, hub: Arc<Hub>) -> Result<()> {
    let listener = TcpListener::bind(SocketAddrV4::new(bind, port))
        .with_context(|| format!("binding status on {bind}:{port}"))?;
    log::info!("listening on {bind}:{port} (status)");
    thread::Builder::new()
        .name("n2kd-status".into())
        .spawn(move || {
            loop {
                let stream = match listener.accept() {
                    Ok((s, _)) => s,
                    Err(e) => {
                        log::warn!("accept on status: {e}");
                        continue;
                    }
                };
                let hub2 = Arc::clone(&hub);
                thread::Builder::new()
                    .spawn(move || run_status_client(stream, hub2))
                    .ok();
            }
        })
        .context("spawning status listener")?;
    Ok(())
}

/// Spawn the library `request_engine` with an emit closure that
/// writes canboat PLAIN-format ISO requests to stdout — a downstream
/// writer (e.g. `actisense-serial`) puts them on the bus.
fn spawn_claim_request_engine(engine: Arc<RequestEngine>) {
    request_engine::spawn(engine, |dst, pgn| {
        println!(
            "{}",
            request_engine::format_iso_request_plain(&now_iso(), dst, pgn)
        );
    });
}

fn now_iso() -> String {
    let secs = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let days = secs.div_euclid(86_400);
    let day_secs = secs.rem_euclid(86_400) as u32;
    let h = day_secs / 3600;
    let m = (day_secs / 60) % 60;
    let s = day_secs % 60;
    let (y, mo, d) = days_to_ymd(days);
    format!("{y:04}-{mo:02}-{d:02}-{h:02}:{m:02}:{s:02}.000")
}

/// Per-stream-client handler.
fn run_status_client(mut stream: TcpStream, hub: Arc<Hub>) {
    let dump = hub.status_snapshot();
    let _ = stream.write_all(dump.as_bytes());
    let _ = stream.shutdown(std::net::Shutdown::Both);
}

fn run_raw_input_client(stream: TcpStream, copy_to_stdout: bool) {
    let reader = BufReader::new(stream);
    let stdout = io::stdout();
    for line in reader.lines().map_while(|r| r.ok()) {
        // Default (canboat C): forward to stdout only with `-o`, else
        // drain and discard to keep the client side healthy.
        if copy_to_stdout {
            let mut lock = stdout.lock();
            let mut line = line;
            line.push('\n');
            if lock.write_all(line.as_bytes()).is_err() {
                return;
            }
            let _ = lock.flush();
        }
    }
}

fn run_stdin_pump(hub: &Hub) -> Result<()> {
    let stdin = io::stdin();
    let mut lock = stdin.lock();
    let mut line = String::with_capacity(4096);
    // Unit system of the incoming stream, learned from the analyzer
    // version banner (`units:"si"` → true). Governs which schema we
    // rebuild each DecodedPgn against, so `as_f64_in(...)` in the 0183 /
    // AIS converters sees the right source unit. Defaults to Metric —
    // the same assumption as a stream with no banner.
    let mut unit_si = false;
    let mut saw_banner = false;
    let mut warned_no_banner = false;
    loop {
        line.clear();
        let n = lock.read_line(&mut line).context("reading stdin")?;
        if n == 0 {
            break;
        }
        let trimmed = line.trim_end_matches(['\r', '\n']);
        if trimmed.is_empty() {
            continue;
        }
        if trimmed.starts_with("{\"version\"") {
            // Analyzer banner — broadcast on JSON stream, skip cache.
            // Learn the stream's unit system so the converters below
            // request the units they need from the right schema.
            saw_banner = true;
            unit_si = trimmed.contains("\"units\":\"si\"");
            log::info!(
                "analyzer stream units: {}",
                if unit_si {
                    "SI (rad/K/Pa)"
                } else {
                    "Metric (deg/°C/bar)"
                }
            );
            hub.json_hub.broadcast(&line);
            continue;
        }
        if !trimmed.starts_with("{\"timestamp\"") {
            log::debug!("ignoring non-PGN line: {trimmed:.80}");
            continue;
        }
        // canboat C aborts here unless the first line was the banner; we
        // are lenient (a bannerless stream is assumed Metric), but warn
        // once so a misconfigured producer is visible. `-si` streams
        // *must* carry the banner or their radians decode as degrees.
        if !saw_banner && !warned_no_banner {
            log::warn!(
                "input has no analyzer version banner; assuming Metric units. \
                 Pipe from `analyzer -json -nv` (add `-si` for SI units) so the \
                 unit system is declared."
            );
            warned_no_banner = true;
        }
        // canboat C requires the line to end with `}}` (the closing
        // brace of `fields` plus the outer brace) — see
        // `n2kd/main.c:982`. Empty-fields records like `{"pgn":...,
        // "description":"..."}` (no fields block) drop out; they
        // would otherwise create cache entries with no payload and
        // diverge from canboat C's view of the bus.
        if !trimmed.ends_with("}}") {
            log::debug!("ignoring fieldless record: {trimmed:.80}");
            continue;
        }
        let Some(meta) = extract_meta(trimmed) else {
            hub.json_hub.broadcast(&line);
            continue;
        };
        if !hub.src_allowed(meta.src) {
            continue;
        }
        hub.note_device_seen(meta.pgn, meta.src);
        // `line` carries the trailing `\n` from `read_line`; the
        // snapshot's nested wrapper would print that as a blank line
        // after every entry. Strip it before stashing.
        //
        // For PGNs whose primary key lives inside a repeating set
        // (PGN 130824 et al.) `store` walks the `"list":[…]` array
        // and emits one cache entry per iteration, each with a
        // spliced-down single-element `"list":[…]` so subsequent
        // records refresh the matching iteration in place.
        hub.store(trimmed);
        hub.json_hub.broadcast(&line);
        // The AIS port is a one-shot snapshot now, not a live stream —
        // see `spawn_ais_listener`. No per-line broadcast.
        // NMEA 0183 conversion. Rebuild the line into a DecodedPgn
        // once, then run the struct-path converters directly (the same
        // code the live `server` pipeline uses) — AIS PGNs through the
        // AIVDM encoder, everything else through the simple-sentence
        // table. The JSON-parsing `convert` wrappers are only the
        // .j2k-input adapters now; the daemon works on DecodedPgn.
        let mut nmea = String::new();
        // Rebuild against the schema matching the stream's declared units
        // so each field's `info.unit` agrees with its value; the 0183 /
        // AIS converters then ask `as_f64_in("deg")` etc. and the core
        // converts from rad/K only when the stream is SI.
        let units = if unit_si {
            canboat_core::Units::Si
        } else {
            canboat_core::Units::Metric
        };
        let decoded = canboat_core::json_to_decoded(trimmed, PgnDatabase::embedded(units));
        // Learn `src → NAME` for the per-device 0183 filter from ISO
        // Address Claims — `iso_name()` re-packs it from the decoded
        // fields (the same helper the live pipeline uses), returning
        // `None` for any non-claim PGN.
        if hub.has_filter()
            && let Some(name) = decoded.as_ref().and_then(|d| d.iso_name())
        {
            hub.note_address_claim(meta.src, name);
        }
        if let Some(decoded) = decoded.as_ref() {
            let mut rl = hub.rate_limiter.lock().unwrap();
            // One dispatcher for both `$` sentences and AIS `!AIVDM`
            // (folded into `convert_nmea0183`).
            nmea0183::convert_decoded(&mut nmea, decoded, &mut rl);
        }
        // Mute redundant devices' 0183 by NAME before it goes out. When
        // the filter empties the buffer (every sentence muted) there's
        // nothing to broadcast.
        if hub.has_filter() {
            hub.apply_filter(meta.src, &mut nmea);
        }
        if !nmea.is_empty() {
            hub.nmea_hub.broadcast(&nmea);
            hub.udp_broadcast(nmea.as_bytes());
            if hub.nmea_to_stdout {
                let stdout = io::stdout();
                let mut lock = stdout.lock();
                let _ = lock.write_all(nmea.as_bytes());
                let _ = lock.flush();
            }
        }
    }
    Ok(())
}

/// Minimal routing header pulled off an analyzer-JSON line — just
/// what `run_stdin_pump` needs to decide src filtering, device-seen
/// bookkeeping, and which NMEA-0183 converter to run.
///
/// All snapshot keying (top-level composite PK, per-iteration PK
/// inside a repeating set) is done inside
/// [`canboat_core::snapshot::classify_json_line`], which the hub
/// invokes via [`Hub::store`] — `Meta` itself doesn't carry a
/// secondary anymore. AIS vs non-AIS is decided inside
/// [`crate::n2kd::decoded::convert_nmea0183`] (which folds in the AIS
/// `!AIVDM` path), so `Meta` doesn't need to carry an `is_ais` flag.
#[derive(Debug, Clone, Copy)]
struct Meta {
    pgn: u32,
    src: u8,
}

fn extract_meta(line: &str) -> Option<Meta> {
    let pgn = json::int(line, "pgn")? as u32;
    let src = json::int(line, "src")? as u8;
    Some(Meta { pgn, src })
}

struct Hub {
    /// Per-`(pgn, src)` snapshot cache, shared (via `Arc`) with the
    /// connect-and-dump snapshot / AIS listeners in `n2kd::serving::tcp`
    /// and this crate's status listener. The pump owns the writes.
    cache: Arc<SnapshotStore>,
    engine: Arc<RequestEngine>,
    /// Live JSON stream (`port+1`) and NMEA 0183 stream (`port+2`),
    /// served by `n2kd::serving::tcp::spawn_stream_server`. The pump
    /// broadcasts each line / sentence into them.
    json_hub: Arc<WireHub>,
    nmea_hub: Arc<WireHub>,
    src_filter: Option<SrcFilter>,
    rate_limiter: Mutex<RateLimiter>,
    udp: Option<UdpBroadcast>,
    /// `--nmea0183` mode: NMEA / AIVDM sentences are also written to
    /// the process's stdout. Off in the normal TCP-multiplex mode.
    nmea_to_stdout: bool,
    /// Per-device NMEA 0183 filter (`--nmea0183-filter`), shared between
    /// the stdin pump (learn NAME from claims, mute output) and the
    /// raw-input port (apply runtime PGN 262657 Set frames). `has_filter`
    /// is a cheap gate so the pump's hot path skips the lock entirely
    /// when no filter is configured.
    filter: Mutex<Option<NmeaFilter>>,
    has_filter: bool,
}

impl Hub {
    fn new(
        src_filter: Option<SrcFilter>,
        rate_limit: bool,
        udp: Option<UdpBroadcast>,
        engine: Arc<RequestEngine>,
        filter: Option<NmeaFilter>,
    ) -> Self {
        Self {
            cache: Arc::new(SnapshotStore::new()),
            engine,
            json_hub: Arc::new(WireHub::new()),
            nmea_hub: Arc::new(WireHub::new()),
            src_filter,
            rate_limiter: Mutex::new(RateLimiter::new(rate_limit)),
            udp,
            nmea_to_stdout: false,
            has_filter: filter.is_some(),
            filter: Mutex::new(filter),
        }
    }

    /// Whether a per-device 0183 filter is configured. Cheap enough to
    /// call per line — lets the pump avoid taking [`Self::filter`]'s
    /// lock on the hot path when the feature is off.
    fn has_filter(&self) -> bool {
        self.has_filter
    }

    /// Learn `src → NAME` from an ISO Address Claim, for the filter.
    fn note_address_claim(&self, src: u8, name: u64) {
        if let Some(f) = self.filter.lock().unwrap().as_mut() {
            f.note_address_claim(src, name);
        }
    }

    /// Mute the just-converted 0183 buffer per device NAME in place.
    fn apply_filter(&self, src: u8, buf: &mut String) {
        if let Some(f) = self.filter.lock().unwrap().as_mut() {
            f.apply(src, buf);
        }
    }

    fn src_allowed(&self, src: u8) -> bool {
        match &self.src_filter {
            None => true,
            Some(f) => f.allows(src),
        }
    }

    fn note_device_seen(&self, pgn: u32, src: u8) {
        self.engine.note_device_seen(pgn, src);
    }

    fn store(&self, line: &str) {
        canboat_core::snapshot::classify_json_line(line, |input| self.cache.store(input));
    }

    /// Canboat C–compatible per-PGN status dump
    /// (`{<pgn>:{description,<src>:{last,interval,count}}}`).
    fn status_snapshot(&self) -> String {
        self.cache.status_snapshot()
    }

    fn udp_broadcast(&self, bytes: &[u8]) {
        if let Some(udp) = &self.udp {
            udp.send(bytes);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn extracts_pgn_src() {
        let line = r#"{"timestamp":"2026-01-01T00:00:00","prio":2,"src":7,"dst":255,"pgn":127251,"description":"Rate of Turn","fields":{"Rate":0}}"#;
        let meta = extract_meta(line).unwrap();
        assert_eq!(meta.pgn, 127251);
        assert_eq!(meta.src, 7);
    }

    /// Helper: classify and return the single emitted secondary.
    /// Panics when the classifier emits 0 or >1 records — the cases
    /// below are all top-level-PK PGNs.
    fn classify_secondary(line: &str) -> Option<String> {
        let mut got = Vec::new();
        canboat_core::snapshot::classify_json_line(line, |i| got.push(i));
        assert_eq!(got.len(), 1, "expected one record, got {got:?}");
        got.remove(0).secondary
    }

    #[test]
    fn ais_message_marks_is_ais_and_keys_on_user_id() {
        // Schema 2.5.0 declares only `User ID` as PartOfPrimaryKey on
        // PGN 129039 — Message ID is NOT a discriminator here.
        let line = r#"{"timestamp":"…","src":23,"pgn":129039,"fields":{"Message ID":18,"User ID":"244180106"}}"#;
        assert!(canboat_core::snapshot::is_ais_pgn(129039));
        assert_eq!(classify_secondary(line).as_deref(), Some("244180106"));
    }

    #[test]
    fn classify_uses_lookup_numeric_value() {
        // -nv mode wraps a lookup as `{"value":1,"name":"Outside
        // Temperature"}`; the composite snapshot key uses the raw
        // numeric `value` (short, stable across schema label edits).
        let line = r#"{"src":7,"pgn":130312,"fields":{"Instance":0,"Source":{"value":1,"name":"Outside Temperature"},"Actual Temperature":18.5}}"#;
        assert_eq!(classify_secondary(line).as_deref(), Some("0_1"));
    }

    #[test]
    fn classify_handles_key_true_lookup_objects() {
        // PGN 127501 Instance is a `{"value":N,"key":true}` lookup —
        // no name to consider, value extraction still works.
        let line = r#"{"src":17,"pgn":127501,"fields":{"Instance":{"value":0,"key":true}}}"#;
        assert_eq!(classify_secondary(line).as_deref(), Some("0"));
    }

    #[test]
    fn composite_pk_keys_distinguish_otherwise_identical_records() {
        // PGN 127509 (Inverter Status) declares `Instance + AC Instance
        // + DC Instance` as a composite PK. Two records that share
        // Instance=0 but differ in AC Instance must get distinct cache
        // keys.
        let a =
            r#"{"src":35,"pgn":127509,"fields":{"Instance":0,"AC Instance":0,"DC Instance":1}}"#;
        let b =
            r#"{"src":35,"pgn":127509,"fields":{"Instance":0,"AC Instance":1,"DC Instance":0}}"#;
        let sa = classify_secondary(a);
        let sb = classify_secondary(b);
        assert_eq!(sa.as_deref(), Some("0_0_1"));
        assert_eq!(sb.as_deref(), Some("0_1_0"));
        assert_ne!(sa, sb);
    }

    /// Build a Hub with no source filter / rate limit / UDP — just
    /// enough surface to exercise `Hub::store` and the snapshot dump.
    fn test_hub() -> Hub {
        let engine = Arc::new(RequestEngine::new());
        Hub::new(None, false, None, engine, None)
    }

    #[test]
    fn store_pgn_130824_emits_one_entry_per_iteration() {
        // Hand-built analyzer-JSON line for PGN 130824 src=27 with
        // three Key/Value pairs (`Polar Speed`, `Polar Performance`,
        // `Opposite Tack COG`). Each Key field is the `-nv` lookup
        // object whose `value` becomes the per-iteration cache key.
        let line = r#"{"timestamp":"…","prio":3,"src":27,"dst":255,"pgn":130824,"description":"B&G: key-value data","fields":{"Manufacturer Code":{"value":381,"name":"B & G"},"Industry Code":{"value":4,"name":"Marine Industry"},"list":[{"Key":{"value":126,"name":"Polar Speed"},"Length":2,"Value":2.03},{"Key":{"value":124,"name":"Polar Performance"},"Length":2,"Value":128.0},{"Key":{"value":306,"name":"Opposite Tack COG"},"Length":2,"Value":103.6}]}}"#;
        let hub = test_hub();
        // PGN 130824's primary key lives inside the repeating set,
        // so the per-iteration `27_<key>` entries asserted below
        // verify the classifier emits them correctly.
        hub.store(line);
        let dump = hub.cache.snapshot();
        assert!(
            dump.contains("\"27_126\":"),
            "missing src_<key=126> (Polar Speed):\n{dump}"
        );
        assert!(
            dump.contains("\"27_124\":"),
            "missing src_<key=124> (Polar Performance):\n{dump}"
        );
        assert!(
            dump.contains("\"27_306\":"),
            "missing src_<key=306> (Opposite Tack COG):\n{dump}"
        );
        // Each spliced line keeps a single-element list so subsequent
        // records refresh that iteration in place.
        assert_eq!(
            dump.matches("\"list\":[{").count(),
            3,
            "expected 3 single-element list payloads:\n{dump}"
        );
    }

    #[test]
    fn store_pgn_130824_preserves_other_iterations_on_partial_update() {
        let initial = r#"{"src":27,"pgn":130824,"description":"B&G: key-value data","fields":{"list":[{"Key":{"value":126},"Length":2,"Value":2.03},{"Key":{"value":124},"Length":2,"Value":128.0}]}}"#;
        let hub = test_hub();
        hub.store(initial);

        // Second record carries only key 126 with a refreshed value.
        let update = r#"{"src":27,"pgn":130824,"description":"B&G: key-value data","fields":{"list":[{"Key":{"value":126},"Length":2,"Value":4.00}]}}"#;
        hub.store(update);

        let dump = hub.cache.snapshot();
        // Both Keys still present — partial update didn't blow away
        // the non-mentioned iteration.
        assert!(dump.contains("\"27_126\":"), "Polar Speed missing:\n{dump}");
        assert!(
            dump.contains("\"27_124\":"),
            "Polar Performance dropped on partial update:\n{dump}"
        );
        // And the refreshed value made it into the cached line.
        assert!(
            dump.contains("\"Value\":4"),
            "expected refreshed Polar Speed value=4 in dump:\n{dump}"
        );
    }

    #[test]
    fn negative_src_filter() {
        let f = parse_src_filter(Some("!1,2")).unwrap().unwrap();
        assert!(f.allows(3));
        assert!(!f.allows(1));
        assert!(!f.allows(2));
    }

    #[test]
    fn positive_src_filter() {
        let f = parse_src_filter(Some("7,8")).unwrap().unwrap();
        assert!(f.allows(7));
        assert!(!f.allows(9));
    }
}
