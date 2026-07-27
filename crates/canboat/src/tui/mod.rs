// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat-tui` — interactive terminal UI for an NMEA 2000 bus.
//!
//! Connects to either `canboat-pipeline` or `n2kd`. On startup the
//! TUI loads the one-shot snapshot from the status port (default
//! 2597) and then stays subscribed to the live stream port (default
//! 2598). Outgoing writes (ISO Requests, NMEA-0183-filter edits, and
//! PGN-rate-override edits) go to dedicated ports derived from the
//! stream port (see [`write_ports`]) — those work against
//! canboat-pipeline but are absent on a plain n2kd.
//!
//! The state model is shared with the snapshot module
//! ([`canboat_core::snapshot::classify_json_line`]) so the keys we
//! show in the TUI are byte-identical to the ones stored on the
//! snapshot port.

use std::io;
use std::sync::Arc;
use std::time::Duration;

use anyhow::Result;
use crossterm::event::{Event, EventStream};
use crossterm::execute;
use crossterm::terminal::{
    EnterAlternateScreen, LeaveAlternateScreen, disable_raw_mode, enable_raw_mode,
};
use futures::StreamExt;
use ratatui::Terminal;
use ratatui::backend::CrosstermBackend;
use tokio::sync::Mutex;
use tokio::task::JoinHandle;

mod client;
mod device_cache;
mod filebrowser;
mod iso;
mod menu;
mod state;
mod ui;

use crate::tui::state::{AppState, Status};

/// Interactive terminal UI for an NMEA 2000 bus, fed by n2kd or the server.
#[derive(clap::Args, Debug)]
#[command(after_help = canboat_cli::help_footer())]
pub struct Args {
    /// Hostname or IP of the n2kd / canboat-pipeline endpoint. Optional:
    /// with neither `--host` nor `--log` the TUI starts idle and opens
    /// the File ▸ Load dialog so you can pick a capture (or Connect from
    /// the menu). There is no implicit localhost default.
    #[arg(long, conflicts_with = "log")]
    host: Option<String>,
    /// Snapshot port (default 2597 — canboat-pipeline `--snapshot-port`
    /// or n2kd JSON snapshot). Ignored when `--log` is set.
    #[arg(long, default_value_t = 2597)]
    snapshot_port: u16,
    /// Live JSON stream port (default 2598 — canboat-pipeline
    /// `--analyzer-port` or n2kd JSON stream). Ignored when `--log`
    /// is set.
    #[arg(long, default_value_t = 2598)]
    stream_port: u16,
    /// Replay a captured log file instead of connecting to a live
    /// endpoint. Accepts any analyzer-readable format (PLAIN, FAST,
    /// Actisense, iKonvert, YDWG02 — auto-detected on the first
    /// content line). Mutually exclusive with `--host`. The `o`
    /// (override) and `i` (ISO request) keys are disabled in this
    /// mode since they need a live bus.
    #[arg(long, value_name = "PATH")]
    log: Option<std::path::PathBuf>,
}

pub async fn run(args: Args) -> Result<()> {
    // The TUI takes over the terminal via crossterm's alt-screen, so
    // any log line emitted on stderr paints garbage over the UI. Route
    // logs to a file instead, and quiet the default filter (WARN) so a
    // capture with a handful of decode warnings doesn't produce a
    // multi-MB session log. Users who need verbose diagnostics can
    // still opt in via `RUST_LOG=debug` — the file grows accordingly.
    if let Ok(log_path) = std::env::var("CANBOAT_TUI_LOG").or_else(|_| {
        std::env::var("TMPDIR")
            .map(|d| format!("{d}/canboat-tui.log"))
            .or_else(|_| Ok::<_, std::env::VarError>("/tmp/canboat-tui.log".to_string()))
    }) && let Ok(file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(&log_path)
    {
        let _ = env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("warn"))
            .target(env_logger::Target::Pipe(Box::new(file)))
            .try_init();
        canboat_cli::log_startup(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
    }

    // `--host` and `--log` are mutually exclusive (clap); with neither
    // we start idle and prompt for a capture via File ▸ Load.
    let idle = args.log.is_none() && args.host.is_none();
    let status = match (&args.log, &args.host) {
        (Some(path), _) => Status::new_log(path.display().to_string()),
        (None, Some(host)) => Status::new_live(host.clone(), args.snapshot_port, args.stream_port),
        (None, None) => Status::new_idle(),
    };
    // Load the persistent NAME → info device cache. Enriches the
    // device list with anything we've learned across past sessions.
    let device_cache_path = device_cache::default_path();
    let device_cache = device_cache::load_or_default(&device_cache_path);
    let state = Arc::new(Mutex::new(AppState::new(status, device_cache)));

    // Writer channel is created up front in both modes — in log mode
    // nothing ever sends through it (the `o` / `i` handlers are gated
    // on Mode::Live), but having the same `Writer` shape keeps the
    // UI code path uniform.
    let (writer, writer_rx) = client::make_writer();
    // Track the background task handles so File ▸ Connect / Load can
    // abort the current connection before starting a new one.
    let mut tasks: Vec<JoinHandle<()>> = Vec::new();
    match (args.log.clone(), args.host.clone()) {
        (Some(path), _) => {
            // Log mode: drive the analyzer's library decode pipeline on
            // a blocking thread; no snapshot/stream sockets. Drop
            // `writer_rx` — nothing will write to the channel.
            drop(writer_rx);
            tasks.extend(client::spawn_log_load(path, state.clone()));
        }
        (None, Some(host)) => {
            // Live mode: kick off both network tasks in the background;
            // the UI loop surfaces their progress / errors via the
            // status bar.
            tasks.push(client::spawn_snapshot_load(
                host.clone(),
                args.snapshot_port,
                state.clone(),
            ));
            spawn_live_connections(&mut tasks, host, args.stream_port, &state, writer_rx);
        }
        (None, None) => {
            // Idle: nothing to read from yet. Drop the writer receiver;
            // File ▸ Connect / Load will start a real source later.
            drop(writer_rx);
        }
    }

    let mut app = ui::App::new();
    if idle {
        // No source chosen — greet the user with the Load dialog once
        // the intro animation finishes.
        app.prompt_load();
    }

    let mut tty = setup_tty()?;
    let res = run_loop(&mut tty, &mut app, state.clone(), writer, tasks).await;
    restore_tty(&mut tty)?;
    // Persist the freshly-enriched NAME → info cache so a subsequent
    // log-mode session sees whatever this run learned. Failures
    // aren't fatal — a full disk shouldn't crash the exit path — but
    // do surface on stderr so the user notices.
    {
        let s = state.lock().await;
        if let Err(e) = device_cache::save(&s.device_cache, &device_cache_path) {
            eprintln!("canboat-tui: failed to save device cache: {e:#}");
        }
    }
    res
}

async fn run_loop(
    tty: &mut ui::Tty,
    app: &mut ui::App,
    state: Arc<Mutex<AppState>>,
    mut writer: client::Writer,
    mut tasks: Vec<JoinHandle<()>>,
) -> Result<()> {
    let mut events = EventStream::new();
    // Whether a capture save is streaming out (seen on the previous
    // frame). While it is, tick faster so the progress bar animates.
    let mut saving = false;
    loop {
        // Tick fast while the intro animation is playing so the zoom is
        // smooth (~17 fps) or a save is running so the bar advances;
        // fall back to a lazy 250 ms once idle.
        let frame_delay = if app.splash.is_some() {
            Duration::from_millis(60)
        } else if saving {
            Duration::from_millis(100)
        } else {
            Duration::from_millis(250)
        };
        tokio::select! {
            biased;
            ev = events.next() => {
                match ev {
                    Some(Ok(Event::Key(key))) => {
                        let s = state.lock().await;
                        app.handle_key(key, &s, &writer);
                    }
                    Some(Ok(_)) => {}
                    Some(Err(e)) => {
                        let mut s = state.lock().await;
                        s.status.last_error = Some(format!("terminal: {e}"));
                    }
                    None => break,
                }
            }
            _ = tokio::time::sleep(frame_delay) => {}
        }
        if app.should_quit {
            break;
        }
        // Enact any File-menu command the UI recorded this iteration
        // (it owns none of the runtime plumbing — we do).
        if let Some(cmd) = app.pending_command.take() {
            execute_command(cmd, &mut writer, &mut tasks, &state, app).await;
        }
        let mut s = state.lock().await;
        // Drop an override the user just deleted on the Overrides screen
        // right away, rather than waiting for the server's next report to
        // stop listing it.
        if let Some((src, pgn)) = app.pending_override_forget.take() {
            s.forget_override(src, pgn);
        }
        // Age out overrides the server has stopped reporting (deleted or
        // NAK-forgotten), so the Overrides view reflects removals.
        s.prune_overrides();
        // Surface a one-shot completion notice (load / save done) as a
        // toast the user can dismiss with any key.
        if let Some(notice) = s.notice.take() {
            app.toast = Some(notice);
        }
        saving = s.save_progress.is_some();
        ui::draw(tty, app, &s)?;
    }
    Ok(())
}

/// Carry out a File-menu command: reconnect to a new endpoint, load a
/// capture file, or save the current capture. Connect / Load tear down
/// the running tasks and swap in a fresh [`AppState`] (preserving the
/// persistent device cache) so no stale data or task bleeds across.
/// Derive the two write-capable ports from the JSON stream port, using
/// the server's canonical n2kd-relative offsets: stream = base+1,
/// input = base+3, filter control = base+8, override control = base+9.
/// The Connect dialog only collects host + snapshot + stream ports, so
/// the write ports follow the stream port rather than being entered
/// separately. Returns `(input, filter, overrides)`.
fn write_ports(stream_port: u16) -> (u16, u16, u16) {
    let base = stream_port.wrapping_sub(1);
    (
        base.wrapping_add(3),
        base.wrapping_add(8),
        base.wrapping_add(9),
    )
}

/// Spawn the three live-mode connections that back the shared
/// [`client::Writer`]: the read-only bus JSON stream, the write-only
/// input port (bus injection), and the bidirectional filter control
/// port. The input / filter ports are derived from `stream_port` via
/// [`write_ports`].
fn spawn_live_connections(
    tasks: &mut Vec<JoinHandle<()>>,
    host: String,
    stream_port: u16,
    state: &Arc<Mutex<AppState>>,
    writer_rx: client::WriterRx,
) {
    let (input_port, filter_port, overrides_port) = write_ports(stream_port);
    tasks.push(client::spawn_stream_connection(
        host.clone(),
        stream_port,
        state.clone(),
    ));
    tasks.push(client::spawn_input_connection(
        host.clone(),
        input_port,
        state.clone(),
        writer_rx.input,
    ));
    tasks.push(client::spawn_filter_connection(
        host.clone(),
        filter_port,
        state.clone(),
        writer_rx.filter,
    ));
    tasks.push(client::spawn_overrides_connection(
        host,
        overrides_port,
        state.clone(),
        writer_rx.overrides,
    ));
}

async fn execute_command(
    cmd: ui::PendingCommand,
    writer: &mut client::Writer,
    tasks: &mut Vec<JoinHandle<()>>,
    state: &Arc<Mutex<AppState>>,
    app: &mut ui::App,
) {
    // Connect / Load reset the whole AppState; refuse them while a save
    // is streaming out of `history` / `raw_lines` so it can't read into
    // a swapped-out buffer.
    if matches!(
        cmd,
        ui::PendingCommand::Connect { .. } | ui::PendingCommand::Load { .. }
    ) && state.lock().await.save_progress.is_some()
    {
        app.toast = Some("Wait for the save to finish".to_string());
        return;
    }
    match cmd {
        ui::PendingCommand::Connect {
            host,
            snapshot_port,
            stream_port,
        } => {
            for t in tasks.drain(..) {
                t.abort();
            }
            {
                let mut s = state.lock().await;
                let cache = std::mem::take(&mut s.device_cache);
                *s = AppState::new(
                    Status::new_live(host.clone(), snapshot_port, stream_port),
                    cache,
                );
            }
            let (new_writer, rx) = client::make_writer();
            tasks.push(client::spawn_snapshot_load(
                host.clone(),
                snapshot_port,
                state.clone(),
            ));
            spawn_live_connections(tasks, host, stream_port, state, rx);
            *writer = new_writer;
            app.reset_views();
            app.connecting_dismissed = false;
        }
        ui::PendingCommand::Load { path } => {
            for t in tasks.drain(..) {
                t.abort();
            }
            {
                let mut s = state.lock().await;
                let cache = std::mem::take(&mut s.device_cache);
                *s = AppState::new(Status::new_log(path.display().to_string()), cache);
            }
            tasks.extend(client::spawn_log_load(path, state.clone()));
            app.reset_views();
            // Log mode has no connection to wait for; the draw path
            // auto-dismisses the connecting overlay immediately.
            app.connecting_dismissed = true;
        }
        ui::PendingCommand::Save { path, format } => {
            // Refuse while a load is still in flight (would race the
            // save task's history indices) or another save is running.
            let (loading, saving) = {
                let s = state.lock().await;
                (!s.status.snapshot_loaded, s.save_progress.is_some())
            };
            if loading {
                app.toast = Some("Still loading — try Save again once it's done".to_string());
            } else if saving {
                app.toast = Some("A save is already in progress".to_string());
            } else {
                // Runs in the background with a progress bar; the write
                // never holds the state lock long enough to stall the UI.
                tasks.push(client::spawn_save(path, format, state.clone()));
            }
        }
    }
}

fn setup_tty() -> Result<ui::Tty> {
    enable_raw_mode()?;
    let mut out = io::stdout();
    execute!(out, EnterAlternateScreen)?;
    let backend = CrosstermBackend::new(out);
    Ok(Terminal::new(backend)?)
}

fn restore_tty(tty: &mut ui::Tty) -> Result<()> {
    disable_raw_mode()?;
    execute!(tty.backend_mut(), LeaveAlternateScreen)?;
    tty.show_cursor()?;
    Ok(())
}
