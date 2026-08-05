// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! `canboat`: the single-binary front end to the canboat-rs toolkit.
//!
//! Subcommands replace the historical scatter of standalone binaries:
//!
//! - `convert` — translate a capture between any supported formats
//!   (subsumes `analyzer`, `nif2analyzer`, the various `*2*` shunts).
//! - `interface` — bridge a live gateway (NGT-1, iKonvert, SocketCAN,
//!   Maretron IPG) to/from stdout.
//! - `server` — the device → analyzer → n2kd pipeline daemon.
//! - `tui` — the terminal browser.
//! - `replay` — pace a captured stream at its original wall-clock rhythm.
//! - `n2kd` — multiplex an analyzer-JSON stdin stream to TCP clients.
//!
//! Retired tool names keep working via argv[0] multiplexing — see
//! [`legacy`]. `canboat install-shims` creates the symlinks.

use std::path::PathBuf;
use std::process::ExitCode;

use clap::{Parser, Subcommand};

mod convert;
mod format_message;
mod interface;
mod legacy;
mod replay;
mod tui;

// The server / n2kd pipeline implementations now live in `canboat-bridge`.
use canboat_bridge::{n2kd, server};

#[derive(Debug, Parser)]
#[command(
    name = "canboat",
    about = "The canboat NMEA 2000 toolkit",
    version,
    after_help = canboat_cli::help_footer()
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Debug, Subcommand)]
enum Command {
    /// Convert a capture between formats (any input → PLAIN / JSON / text).
    #[command(long_about = convert::LONG_ABOUT)]
    Convert(convert::Args),

    /// Build one NMEA 2000 frame from field values (any PGN). See `--list` / `<PGN> --help`.
    FormatMessage(format_message::Args),

    /// Bridge a live gateway (NGT-1 / iKonvert / Maretron / SocketCAN) to/from stdout.
    Interface(interface::Args),

    /// Run the single-process device → analyzer → n2kd pipeline server.
    Server(Box<server::Args>),

    /// Interactive terminal browser for a live n2kd/server stream or a capture.
    Tui(tui::Args),

    /// Pace a captured PLAIN/FAST stream at its original wall-clock rhythm.
    Replay(replay::Args),

    /// Multiplex an analyzer-JSON stdin stream to TCP clients (the n2kd daemon).
    N2kd(Box<n2kd::app::Args>),

    /// Install symlinks for the retired tool names (analyzer, *-serial, …).
    InstallShims {
        /// Directory to create the symlinks in. Defaults to the
        /// directory containing the `canboat` executable.
        #[arg(long, value_name = "DIR")]
        dir: Option<PathBuf>,
    },
}

fn main() -> ExitCode {
    // argv[0] multiplexing: a legacy name either runs its own
    // alias-module or rewrites into a `canboat <subcommand>` invocation.
    let argv = match legacy::route(canboat_cli::canboat_argv()) {
        legacy::Route::Alias(result) => return finish(result),
        legacy::Route::Canboat(argv) => argv,
    };
    let cli = Cli::parse_from(argv);
    let result = match cli.command {
        Command::Convert(args) => convert::run(args),
        Command::FormatMessage(args) => format_message::run(args),
        Command::Interface(args) => interface::run(args),
        Command::Server(args) => {
            // The pipeline library no longer initialises logging — the host
            // (this binary) owns it. Convert the clap Args to the plain config,
            // then set up env_logger from its verbosity before running.
            let mut config: server::BridgeConfig = (*args).into();
            // Config-dir *discovery* is a binary concern, not the library's:
            // when the user didn't pass `--config-dir`, probe for the
            // conventional location so the daemon's overrides / 0183-filter
            // persistence stays on by default. The library itself never
            // touches the filesystem to find a dir.
            if config.config_dir.is_none() {
                config.config_dir = Some(resolve_config_dir());
            }
            let level = if config.quiet {
                "error"
            } else if config.verbose {
                "debug"
            } else {
                "info"
            };
            env_logger::Builder::from_env(env_logger::Env::default().default_filter_or(level))
                .init();
            canboat_cli::log_startup(env!("CARGO_PKG_NAME"), env!("CARGO_PKG_VERSION"));
            server::run(config)
        }
        Command::Tui(args) => run_tui(args),
        Command::Replay(args) => replay::run(args),
        Command::N2kd(args) => n2kd::app::run(*args),
        Command::InstallShims { dir } => install_shims(dir),
    };
    finish(result)
}

/// Print any error and translate to a process exit code.
fn finish(result: anyhow::Result<()>) -> ExitCode {
    if let Err(e) = result {
        eprintln!("canboat: {e:#}");
        return ExitCode::from(1);
    }
    ExitCode::SUCCESS
}

/// Resolve the shim install directory (default: the binary's own dir)
/// and create the symlinks.
fn install_shims(dir: Option<PathBuf>) -> anyhow::Result<()> {
    use anyhow::Context;
    let dir = match dir {
        Some(d) => d,
        None => std::env::current_exe()
            .context("locating the canboat executable")?
            .parent()
            .context("canboat executable has no parent directory")?
            .to_path_buf(),
    };
    legacy::install_shims(&dir)
}

/// Resolve the directory holding the server's persistent state files
/// (`overrides.json`, `nmea0183-filter.json`) when the user didn't pass
/// `--config-dir`. Prefer `/etc/default/canboat` when it's writable (the
/// systemd/root case), otherwise fall back to `$HOME/.local/canboat`. This
/// filesystem probe is a *binary* concern — the library takes an explicit
/// path — so it lives here rather than in `canboat-bridge`.
fn resolve_config_dir() -> PathBuf {
    let system = PathBuf::from("/etc/default/canboat");
    if dir_is_writable(&system) {
        return system;
    }
    if let Ok(home) = std::env::var("HOME") {
        return PathBuf::from(home).join(".local/canboat");
    }
    system
}

/// `true` when `dir` can be created (if needed) and written to — probed
/// with a temp file that's removed immediately.
fn dir_is_writable(dir: &std::path::Path) -> bool {
    if std::fs::create_dir_all(dir).is_err() {
        return false;
    }
    let probe = dir.join(".canboat-write-test");
    match std::fs::OpenOptions::new()
        .write(true)
        .create(true)
        .truncate(false)
        .open(&probe)
    {
        Ok(_) => {
            let _ = std::fs::remove_file(&probe);
            true
        }
        Err(_) => false,
    }
}

/// The TUI is async; the rest of `canboat` is not. Spin up a dedicated
/// multi-threaded runtime (matching the old `#[tokio::main]` shape) just
/// for this subcommand rather than making the whole binary async.
fn run_tui(args: tui::Args) -> anyhow::Result<()> {
    let rt = tokio::runtime::Builder::new_multi_thread()
        .worker_threads(2)
        .enable_all()
        .build()?;
    rt.block_on(tui::run(args))
}
