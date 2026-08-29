// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The [`Bridge`]: the pipeline core split out from the serving layer.
//!
//! A `Bridge` owns the frame source, the fast-packet → decode → quirk →
//! transmit pipeline, and the broadcast hubs. It exposes three things an
//! embedder wants:
//!
//! * an **in-process decoded stream** ([`Bridge::decoded`]) — a
//!   `Receiver<Arc<DecodedPgn>>` fed straight off the pipeline, so a host
//!   process reads N2K without a TCP hop;
//! * a **transmit path** ([`Bridge::transmit`] / [`Bridge::encode`]) that
//!   injects a frame with the bridge's claimed source address;
//! * the **optional TCP serving layer** ([`Bridge::serve`]) — the same
//!   2597–2606 ports the standalone `canboat server` opens, so other
//!   consumers on the LAN keep working unchanged.
//!
//! The lifecycle is: [`Bridge::new`] builds the core (opens the source,
//! loads the config-dir state, wires the request engine) but starts
//! nothing on the bus; then the caller optionally taps [`Bridge::decoded`]
//! and [`Bridge::serve`]s the ports, and finally [`Bridge::spawn`]s the
//! pipeline on a background thread (leaving the `Bridge` alive to transmit
//! and shut down) or drives it to completion in place with [`Bridge::run`].
//!
//! The CLI `canboat server` is exactly `Bridge::new(config).serve().run()`,
//! so the daemon and an embedding library share one code path.

use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicBool, AtomicU8};
use std::sync::{Arc, Mutex, mpsc};
use std::thread;

use anyhow::{Result, anyhow};

use canboat_core::output::JsonOptions;
use canboat_core::{DecodedPgn, PgnDatabase, RawFrame};
use canboat_io::device::{FrameSender, Supervisor};

use crate::n2kd::request_engine::{self, RequestEngine};
use crate::n2kd::serving::Hub;
use crate::n2kd::serving::tcp as serving_tcp;
use crate::server::pipeline::{self, Hubs, Nmea0183Options};
use crate::server::snapshot::SnapshotStore;
use crate::server::{BridgeConfig, OpenedSource, quirks, tcp};

/// An assembled live-bus pipeline: source → reassembly → decode → quirks →
/// transmit, with an optional in-process decoded tap and an optional TCP
/// serving layer. See the [module docs](self) for the lifecycle.
pub struct Bridge {
    config: BridgeConfig,
    db: &'static PgnDatabase,
    json_opts: JsonOptions,
    banner: &'static [u8],

    /// The pipeline's frame source. Taken (`Option::take`) when the
    /// pipeline starts, so `new`→`spawn` is a one-shot.
    frames_rx: Option<mpsc::Receiver<RawFrame>>,
    supervisor: Option<Supervisor>,
    pre_coalesced: Arc<AtomicBool>,
    device_sender: Option<FrameSender>,
    claim_addr: Option<Arc<AtomicU8>>,

    // Broadcast hubs — held as clonable `Arc`s so `serve` can attach TCP
    // listeners and the pipeline can consume its own clones.
    raw_hub: Arc<Hub>,
    nmea_hub: Arc<Hub>,
    analyzer_hub: Arc<Hub>,
    snapshot: Option<Arc<SnapshotStore>>,
    engine: Arc<RequestEngine>,
    nmea_filter: Option<Arc<Mutex<crate::n2kd::nmea_filter::NmeaFilter>>>,
    overrides: Option<Arc<Mutex<crate::n2kd::overrides::OverrideEngine>>>,

    /// Injection point for the write-only input port and the client-write
    /// loopback. `None` in stdin-only mode (no device writer).
    inject: Option<tcp::InjectPoint>,

    /// Optional in-process decoded tap (installed by [`Bridge::decoded`]).
    decoded_tx: Option<mpsc::Sender<Arc<DecodedPgn>>>,

    /// Shutdown flag shared with every [`Bridge::serve`] accept loop. Set by
    /// [`Bridge::shutdown`] (and `Drop`); each loop then drops its listener,
    /// releasing the port so a later `Bridge` can re-bind it.
    serve_stop: Arc<AtomicBool>,
    /// TCP accept threads spawned by [`Bridge::serve`], and the pipeline
    /// thread spawned by [`Bridge::spawn`].
    tcp_joins: Vec<thread::JoinHandle<()>>,
    pipeline_join: Option<thread::JoinHandle<()>>,
}

/// Decide which persistent-state files to read from the config. Pure: no
/// filesystem access, no probing — that stays in the binary. Returns
/// `(nmea0183-filter path, overrides path)`, each `None` when that feature
/// is off:
///
/// * an explicit `nmea0183_filter` path always wins for the filter, even
///   with no config dir;
/// * otherwise both files live under `config_dir`;
/// * `config_dir == None` (and no explicit filter) means persistence is off.
fn state_file_paths(
    config_dir: Option<&Path>,
    explicit_filter: Option<&Path>,
) -> (Option<PathBuf>, Option<PathBuf>) {
    let filter = explicit_filter
        .map(Path::to_path_buf)
        .or_else(|| config_dir.map(|d| d.join("nmea0183-filter.json")));
    let overrides = config_dir.map(|d| d.join("overrides.json"));
    (filter, overrides)
}

impl Bridge {
    /// Build the pipeline core from `config`: validate the quirk/backend
    /// combination, open the frame source, wire the stdin loopback, load
    /// the config-dir state (0183 filter + PGN-rate overrides), and spawn
    /// the periodic ISO-request engine. Starts **nothing** on the bus —
    /// call [`Bridge::serve`] and then [`Bridge::spawn`]/[`Bridge::run`].
    pub fn new(config: BridgeConfig) -> Result<Self> {
        // The scx20 and motion quirks impersonate a device (motion claims a
        // whole new virtual node), so they need --socketcan — the one backend
        // that preserves a frame's src on outbound. The wmm quirk emits
        // canboat's own PGN 127258, so it only needs *some* writable backend
        // — that check happens below once one is known.
        for kind in [quirks::QuirkKind::Scx20, quirks::QuirkKind::Motion] {
            if config.quirk.contains(&kind) && config.socketcan.is_none() {
                let name = match kind {
                    quirks::QuirkKind::Scx20 => "scx20",
                    quirks::QuirkKind::Motion => "motion",
                    quirks::QuirkKind::Wmm | quirks::QuirkKind::GpsRollover => unreachable!(),
                };
                anyhow::bail!(
                    "--quirk {name} only works with --socketcan; it claims/impersonates \
                     a device, and other backends rewrite src on outbound writes so the \
                     frame cannot reach the bus with the claimed source address"
                );
            }
        }

        // The schema is compiled into the binary; no JSON loading, no
        // path discovery, no synthetic-PGN merge — `canboat-core/build.rs`
        // already folded `data/synthetic-pgns.json` into the static tables.
        let units = config.units;
        let db = PgnDatabase::embedded(units);

        // JsonOptions mirror the pipeline's per-record serializer settings
        // so per-iteration snapshot lines (PGN 130824 etc.) come out
        // byte-identical to the regular `analyzer` port stream.
        let json_opts = JsonOptions {
            include_empty: false,
            name_value: true,
            debug: false,
            camel_case: config.camel_case,
            wrap: config.wrap,
        };

        // The analyzer version banner (version, commit, units,
        // showLookupValues) leads the analyzer stream (2598) and the
        // snapshot (2597) so consumers can detect the unit system on
        // connect. Computed once and leaked: it lives for the whole
        // process, so a `&'static` is honest and lets the accept threads
        // hold it without an Arc.
        let banner: &'static [u8] = Box::leak(
            format!(
                "{}\n",
                crate::build_info::version_banner(
                    units == canboat_core::Units::Si,
                    json_opts.name_value,
                )
            )
            .into_bytes()
            .into_boxed_slice(),
        );

        let snapshot = if config.snapshot_port != 0 {
            Some(Arc::new(SnapshotStore::new(json_opts.clone())))
        } else {
            None
        };
        let engine = Arc::new(RequestEngine::new());

        // Pick the frame source and (if a device) its writer handle. In
        // device mode the source is a Supervisor that survives serial /
        // TCP disconnects with exponential backoff.
        let OpenedSource {
            frames_rx,
            supervisor,
            pre_coalesced,
            claim_addr,
        } = super::open_source(&config)?;
        let device_sender = supervisor.as_ref().map(|s| s.frame_sender());

        // The wmm quirk emits PGN 127258 onto the bus, so it needs a
        // writable device backend (any of socketcan / NGT-1 / iKonvert).
        // Refuse it in stdin- or log-only mode where there is no bus.
        if config.quirk.contains(&quirks::QuirkKind::Wmm) && device_sender.is_none() {
            anyhow::bail!(
                "--quirk wmm needs a writable device backend (e.g. --socketcan or an \
                 NGT-1/iKonvert gateway) to emit PGN 127258; there is no bus to write \
                 to in stdin/log-only mode"
            );
        }
        // (motion is already gated to --socketcan above, which guarantees a
        // writable backend, so it needs no separate device_sender check.)

        // In device mode treat stdin like `actisense-serial -p`: parse
        // PLAIN/FAST lines, write the resulting frames to the device, AND
        // loop them back into the pipeline source so they show up in NMEA
        // 0183 / TCP outputs alongside device-originated frames. TCP
        // read/write ports get the same loopback channel.
        let (frames_rx, inject) = match device_sender.clone() {
            Some(sender) => {
                let (rx, loopback) =
                    super::install_stdin_loopback(frames_rx, sender, pre_coalesced.clone());
                let inject = tcp::InjectPoint {
                    device: device_sender.clone().expect("device_sender Some"),
                    loopback,
                    claim_addr: claim_addr.clone(),
                };
                (rx, Some(inject))
            }
            None => (frames_rx, None),
        };

        // Mirror canboat C `n2kd`'s periodic ISO claim / product-info
        // auto-request. Only meaningful with a device writer to put the
        // requests on the bus; stdin-only mode has no sink, so skip it.
        // `--no-request-claims` disables it explicitly.
        if !config.no_request_claims
            && let Some(sender) = device_sender.clone()
        {
            request_engine::spawn(Arc::clone(&engine), move |dst, pgn| {
                let _ = sender.send_frame(request_engine::iso_request_frame(0, dst, pgn));
            });
        }

        // Persistent server state (0183 mute rules + PGN-rate overrides)
        // lives in the caller-supplied config dir. The library never probes
        // the filesystem to *find* a dir — that is the binary's job (the CLI
        // resolves `/etc/default/canboat` vs `~/.local/canboat` before
        // constructing the config). `config_dir == None` here means
        // persistence is off: no overrides, and no filter unless an explicit
        // `nmea0183_filter` path is given. An empty/absent file is a no-op,
        // so a default install with a dir set is unaffected.
        let (filter_path, overrides_path) = state_file_paths(
            config.config_dir.as_deref(),
            config.nmea0183_filter.as_deref(),
        );
        let nmea_filter = match filter_path {
            Some(path) => match crate::n2kd::nmea_filter::NmeaFilter::load(&path) {
                Ok(f) => {
                    log::info!("NMEA 0183 per-device filter loaded from {}", path.display());
                    Some(Arc::new(Mutex::new(f)))
                }
                Err(e) => {
                    log::warn!("NMEA 0183 filter disabled: {e:#}");
                    None
                }
            },
            None => None,
        };
        let overrides = match overrides_path {
            Some(path) => match crate::n2kd::overrides::OverrideEngine::load(&path) {
                Ok(engine) => {
                    log::info!("PGN-rate overrides loaded from {}", path.display());
                    Some(Arc::new(Mutex::new(engine)))
                }
                Err(e) => {
                    log::warn!("PGN-rate overrides disabled: {e:#}");
                    None
                }
            },
            None => None,
        };

        Ok(Self {
            config,
            db,
            json_opts,
            banner,
            frames_rx: Some(frames_rx),
            supervisor,
            pre_coalesced,
            device_sender,
            claim_addr,
            raw_hub: Arc::new(Hub::new()),
            nmea_hub: Arc::new(Hub::new()),
            analyzer_hub: Arc::new(Hub::new()),
            snapshot,
            engine,
            nmea_filter,
            overrides,
            inject,
            decoded_tx: None,
            serve_stop: Arc::new(AtomicBool::new(false)),
            tcp_joins: Vec::new(),
            pipeline_join: None,
        })
    }

    /// Install the in-process decoded tap and return the receiving end.
    /// Every record the pipeline decodes is forwarded here (an `Arc`
    /// refcount bump), so a host process consumes N2K without a TCP hop.
    /// Call before [`Bridge::spawn`]/[`Bridge::run`]. Calling twice
    /// replaces the tap (the earlier receiver then sees end-of-stream).
    pub fn decoded(&mut self) -> mpsc::Receiver<Arc<DecodedPgn>> {
        let (tx, rx) = mpsc::channel();
        self.decoded_tx = Some(tx);
        rx
    }

    /// A cloneable sender for the raw N2K output stream, if this bridge has
    /// a writable device backend. Prefer [`Bridge::transmit`].
    pub fn frame_sender(&self) -> Option<FrameSender> {
        self.device_sender.clone()
    }

    /// The embedded schema database this bridge decodes and encodes with.
    pub fn database(&self) -> &'static PgnDatabase {
        self.db
    }

    /// Inject a frame onto the bus with the bridge's claimed source
    /// address. A frame left at `src` 0 / [`ADDR_GLOBAL`](canboat_core::ADDR_GLOBAL)
    /// is stamped with the live claimed address (as the pipeline does for a
    /// quirk's own-node emission); an explicit `src` passes through
    /// untouched. Errors when the bridge has no writable device backend.
    pub fn transmit(&self, frame: RawFrame) -> Result<()> {
        self.transmitter()
            .ok_or_else(|| anyhow!("bridge has no writable device backend to transmit on"))?
            .transmit(frame)
    }

    /// A cloneable, `Send` handle that transmits onto the bus with the
    /// same claimed-source stamping as [`Bridge::transmit`]. Extract it
    /// before moving the `Bridge` elsewhere (e.g. into a thread that keeps
    /// the pipeline alive) so another task can still inject frames.
    /// `None` when the bridge has no writable device backend.
    pub fn transmitter(&self) -> Option<Transmitter> {
        self.device_sender.as_ref().map(|sender| Transmitter {
            sender: sender.clone(),
            claim_addr: self.claim_addr.clone(),
        })
    }

    /// Start encoding a message for `pgn`, to be finished with `.build()`
    /// and handed to [`Bridge::transmit`]. Convenience for
    /// `bridge.database().encode_by_pgn(pgn)`.
    pub fn encode(&self, pgn: u32) -> Result<canboat_core::PgnBuilder, canboat_core::EncodeError> {
        self.db.encode_by_pgn(pgn)
    }

    /// The live claimed ISO source address, if the backend exposes one
    /// (today only `--socketcan`) and it currently holds a valid unicast
    /// address. `None` otherwise.
    pub fn claimed_address(&self) -> Option<u8> {
        self.claim_addr
            .as_ref()
            .map(|a| a.load(std::sync::atomic::Ordering::Relaxed))
            .filter(|&a| a != canboat_core::ADDR_GLOBAL && a != canboat_core::ADDR_NULL)
    }

    /// Spawn the TCP serving layer — the 2597–2606 ports named in the
    /// config (any port `0` is skipped). Call once. Each listener is wired
    /// to the bridge's shutdown flag, so [`Bridge::shutdown`] (or dropping
    /// the bridge) closes them all and frees the ports. Safe to call before
    /// or after [`Bridge::spawn`]; the CLI serves first so no early frames
    /// are dropped for a client that connects at startup.
    pub fn serve(&mut self) -> Result<()> {
        let config = &self.config;
        let stop = || Some(self.serve_stop.clone());
        if let Some(store) = self.snapshot.as_ref() {
            self.tcp_joins.push(serving_tcp::spawn_snapshot(
                config.bind,
                config.snapshot_port,
                store.core(),
                Some(self.banner),
                stop(),
            )?);
        }
        // Write-only input port (`SERVER_INPUT_STREAM`): clients write
        // PLAIN/FAST lines encoded and forwarded onto the bus. Nothing is
        // streamed back, so it adds no serialization cost.
        if config.input_port != 0
            && let Some(inject) = self.inject.clone()
        {
            self.tcp_joins.push(tcp::spawn_input_server(
                "input",
                config.bind,
                config.input_port,
                Some(inject),
                stop(),
            )?);
        }
        // Raw output stream: every coalesced frame as a PLAIN line under a
        // `# format=FAST` header. Read-only — writes go to `--input-port`.
        if config.raw_port != 0 {
            self.tcp_joins.push(serving_tcp::spawn_stream_server(
                "raw",
                config.bind,
                config.raw_port,
                self.raw_hub.clone(),
                Some(serving_tcp::CANBOAT_FORMAT_FAST_HEADER),
                stop(),
            )?);
        }
        if config.nmea0183_port != 0 {
            // NMEA 0183 is strictly read-only — clients trying to write
            // get an immediate FIN on the read direction.
            self.tcp_joins.push(serving_tcp::spawn_stream_server(
                "nmea0183",
                config.bind,
                config.nmea0183_port,
                self.nmea_hub.clone(),
                None,
                stop(),
            )?);
        }
        if config.analyzer_port != 0 {
            // Analyzer-JSON stream is read-only, matching canboat C n2kd's
            // `port+1` stream port. Injection lives on the input port.
            self.tcp_joins.push(serving_tcp::spawn_stream_server(
                "analyzer",
                config.bind,
                config.analyzer_port,
                self.analyzer_hub.clone(),
                Some(self.banner),
                stop(),
            )?);
        }
        if config.ais_port != 0 {
            if let Some(store) = self.snapshot.as_ref() {
                self.tcp_joins.push(serving_tcp::spawn_ais_snapshot(
                    config.bind,
                    config.ais_port,
                    store.core(),
                    stop(),
                )?);
            } else {
                log::warn!(
                    "--ais-port {} ignored: snapshot port is disabled, no AIS cache to dump",
                    config.ais_port,
                );
            }
        }
        // Dedicated bidirectional control port for the PGN 262657 filter
        // channel (always on now, like the filter itself).
        if let Some(filter) = self.nmea_filter.as_ref()
            && config.nmea0183_filter_port != 0
        {
            self.tcp_joins.push(tcp::spawn_filter_control_server(
                config.bind,
                config.nmea0183_filter_port,
                filter.clone(),
                self.json_opts.clone(),
                stop(),
            )?);
        }
        // Dedicated bidirectional control port for the PGN 262658 override
        // channel. The control server applies a `Set` immediately by
        // injecting the PGN 126208 Request through the device sender.
        if let Some(engine) = self.overrides.as_ref()
            && config.overrides_port != 0
        {
            self.tcp_joins.push(tcp::spawn_overrides_control_server(
                config.bind,
                config.overrides_port,
                engine.clone(),
                self.device_sender.clone(),
                self.json_opts.clone(),
                stop(),
            )?);
        }
        Ok(())
    }

    /// Assemble the pipeline's [`Hubs`] from the bridge's shared state.
    fn build_hubs(&self) -> Hubs {
        Hubs {
            raw: self.raw_hub.clone(),
            nmea: self.nmea_hub.clone(),
            analyzer: self.analyzer_hub.clone(),
            snapshot: self.snapshot.clone(),
            engine: Arc::clone(&self.engine),
            quirks: quirks::Quirks::new(self.config.quirk.clone()),
            device_sender: self.device_sender.clone(),
            claim_addr: self.claim_addr.clone(),
            overrides: self.overrides.clone(),
            decoded_tx: self.decoded_tx.clone(),
        }
    }

    /// Drive the pipeline on the *current* thread until the frame source
    /// closes, then tear everything down (stop the reconnect supervisor and
    /// close the served ports). This is the blocking entry the CLI `canboat
    /// server` uses.
    pub fn run(mut self) -> Result<()> {
        let frames_rx = self.take_source()?;
        let hubs = self.build_hubs();
        let (db, pre_coalesced, json_opts, nmea) = self.pipeline_inputs();
        pipeline::run(db, frames_rx, hubs, pre_coalesced, json_opts, nmea);
        self.shutdown();
        Ok(())
    }

    /// Start the pipeline on a background thread, leaving this `Bridge`
    /// alive to [`transmit`](Bridge::transmit) and [`wait`](Bridge::wait).
    /// Use this from an embedding host (e.g. a tokio service) that reads
    /// the [`decoded`](Bridge::decoded) stream while the bus runs.
    pub fn spawn(&mut self) -> Result<()> {
        let frames_rx = self.take_source()?;
        let hubs = self.build_hubs();
        let (db, pre_coalesced, json_opts, nmea) = self.pipeline_inputs();
        let join = thread::Builder::new()
            .name("bridge-pipeline".into())
            .spawn(move || pipeline::run(db, frames_rx, hubs, pre_coalesced, json_opts, nmea))
            .map_err(|e| anyhow!("spawning the bridge pipeline thread: {e}"))?;
        self.pipeline_join = Some(join);
        Ok(())
    }

    /// Block until a [`spawn`](Bridge::spawn)ed pipeline finishes (the frame
    /// source closed), then tear everything down. No-op if the pipeline was
    /// never spawned.
    pub fn wait(&mut self) {
        if let Some(join) = self.pipeline_join.take() {
            let _ = join.join();
        }
        self.shutdown();
    }

    /// Stop the bus and release the served ports: signal the reconnect
    /// supervisor to stop (which closes the frame source, draining a
    /// [`spawn`](Bridge::spawn)ed pipeline), trip every [`serve`](Bridge::serve)
    /// accept loop, and join the accept threads so their listeners are
    /// dropped and the ports can be re-bound. Idempotent; also run on `Drop`.
    /// Does not join a spawned pipeline thread — use [`wait`](Bridge::wait)
    /// for that.
    pub fn shutdown(&mut self) {
        if let Some(s) = self.supervisor.take() {
            s.shutdown();
        }
        self.serve_stop
            .store(true, std::sync::atomic::Ordering::Relaxed);
        for join in self.tcp_joins.drain(..) {
            let _ = join.join();
        }
    }

    /// Take the frame source exactly once; a second start is a bug.
    fn take_source(&mut self) -> Result<mpsc::Receiver<RawFrame>> {
        self.frames_rx
            .take()
            .ok_or_else(|| anyhow!("bridge pipeline already started"))
    }

    /// The by-value inputs `pipeline::run` needs, cloned off the bridge so
    /// the source (moved) and these can go to a thread together.
    fn pipeline_inputs(
        &self,
    ) -> (
        &'static PgnDatabase,
        Arc<AtomicBool>,
        JsonOptions,
        Nmea0183Options,
    ) {
        (
            self.db,
            self.pre_coalesced.clone(),
            self.json_opts.clone(),
            Nmea0183Options {
                emit_stdout: self.config.nmea0183_stdout,
                rate_limit: !self.config.no_nmea0183_rate_limit,
                filter: self.nmea_filter.clone(),
            },
        )
    }
}

/// A cloneable transmit handle detached from a [`Bridge`] (see
/// [`Bridge::transmitter`]). Holds the device writer and the live claimed
/// address, so it can inject frames with the bridge's source-stamping
/// after the `Bridge` itself has been moved away.
#[derive(Clone)]
pub struct Transmitter {
    sender: FrameSender,
    claim_addr: Option<Arc<AtomicU8>>,
}

impl Transmitter {
    /// Inject a frame onto the bus. A frame left at `src` 0 /
    /// [`ADDR_GLOBAL`](canboat_core::ADDR_GLOBAL) is stamped with the live
    /// claimed address; an explicit `src` passes through untouched. See
    /// [`Bridge::transmit`], whose behaviour this mirrors exactly.
    pub fn transmit(&self, mut frame: RawFrame) -> Result<()> {
        if (frame.src == 0 || frame.src == canboat_core::ADDR_GLOBAL)
            && let Some(addr) = self.claimed_address()
        {
            frame.src = addr;
        }
        self.sender
            .send_frame(frame)
            .map_err(|_| anyhow!("device writer is gone (backend disconnected)"))
    }

    /// The live claimed ISO source address, if currently a valid unicast
    /// address. `None` otherwise. Mirrors [`Bridge::claimed_address`].
    pub fn claimed_address(&self) -> Option<u8> {
        self.claim_addr
            .as_ref()
            .map(|a| a.load(std::sync::atomic::Ordering::Relaxed))
            .filter(|&a| a != canboat_core::ADDR_GLOBAL && a != canboat_core::ADDR_NULL)
    }
}

impl Drop for Bridge {
    /// A dropped `Bridge` releases its served ports: [`Bridge::shutdown`] is
    /// idempotent, so this is a safety net for the case where the caller
    /// dropped the bridge without an explicit shutdown (e.g. a test binding
    /// a second `Bridge` on the same ports).
    fn drop(&mut self) {
        self.shutdown();
    }
}

#[cfg(test)]
mod tests {
    use super::state_file_paths;
    use std::path::{Path, PathBuf};

    #[test]
    fn no_config_dir_disables_persistence() {
        // The library never probes: `None` means off — no filter, no overrides.
        let (filter, overrides) = state_file_paths(None, None);
        assert_eq!(filter, None);
        assert_eq!(overrides, None);
    }

    #[test]
    fn config_dir_places_both_files() {
        let (filter, overrides) = state_file_paths(Some(Path::new("/etc/default/canboat")), None);
        assert_eq!(
            filter,
            Some(PathBuf::from("/etc/default/canboat/nmea0183-filter.json"))
        );
        assert_eq!(
            overrides,
            Some(PathBuf::from("/etc/default/canboat/overrides.json"))
        );
    }

    #[test]
    fn explicit_filter_wins_and_stands_alone() {
        // An explicit filter path loads even with no config dir; overrides
        // stay off (they have no explicit-path escape hatch).
        let (filter, overrides) = state_file_paths(None, Some(Path::new("/custom/f.json")));
        assert_eq!(filter, Some(PathBuf::from("/custom/f.json")));
        assert_eq!(overrides, None);

        // With both, the explicit filter still wins; overrides come from the dir.
        let (filter, overrides) =
            state_file_paths(Some(Path::new("/cfg")), Some(Path::new("/custom/f.json")));
        assert_eq!(filter, Some(PathBuf::from("/custom/f.json")));
        assert_eq!(overrides, Some(PathBuf::from("/cfg/overrides.json")));
    }
}
