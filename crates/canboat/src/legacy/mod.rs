// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! argv[0] multiplexing — the compat layer that lets the retired
//! standalone tools keep working as names that dispatch into `canboat`.
//!
//! Two styles (see [`route`]):
//!
//! - **Prefix-translate**: the legacy CLI is a subset of a `canboat`
//!   subcommand, so we just rewrite argv to
//!   `canboat <subcommand> <fixed args> <rest>` and let clap parse it.
//!   Used for the device bridges → `interface`, `canboat-pipeline` →
//!   `server`, `canboat-tui` → `tui`.
//! - **Alias-module**: the legacy CLI is too different to translate
//!   (e.g. `analyzer`'s golden-tested flag surface), so its parser +
//!   glue live verbatim in a submodule, dispatched by name.
//!
//! Install the names as symlinks with [`install_shims`] (or
//! `canboat install-shims`).

mod analyzer;

use std::ffi::{OsStr, OsString};
use std::path::Path;

use anyhow::Result;

/// Every legacy name `canboat` answers to. Kept in sync with [`route`]
/// so [`install_shims`] never creates a symlink `canboat` can't serve.
pub const LEGACY_NAMES: &[&str] = &[
    // prefix-translate → interface
    "actisense-serial",
    "ikonvert-serial",
    "socketcan-serial",
    "maretron-ipg",
    // prefix-translate → other subcommands
    "canboat-pipeline",
    "canboat-tui",
    "replay",
    "n2kd",
    // alias-module
    "analyzer",
];

/// What to do with a process invocation.
pub enum Route {
    /// argv[0] named a legacy alias-module tool, which has already run;
    /// here is its result.
    Alias(Result<()>),
    /// Parse this (possibly rewritten) argv as a normal `canboat`
    /// invocation.
    Canboat(Vec<OsString>),
}

/// Inspect argv[0] and decide how to dispatch. Called before clap.
pub fn route(argv: Vec<OsString>) -> Route {
    let prog = argv.first().map(|a| program_name(a)).unwrap_or_default();

    // Alias-modules run their own parser and return.
    if prog == "analyzer" {
        return Route::Alias(analyzer::run(argv));
    }

    // Prefix-translate: name → `canboat <subcommand> <fixed args>`.
    if let Some(prefix) = prefix_subcommand(&prog) {
        return Route::Canboat(rewrite(prefix, argv));
    }

    // Not a legacy name — run as `canboat …` unchanged.
    Route::Canboat(argv)
}

/// The subcommand + fixed leading args a prefix-translate name maps to.
fn prefix_subcommand(prog: &str) -> Option<&'static [&'static str]> {
    Some(match prog {
        "actisense-serial" => &["interface", "--kind", "ngt1"],
        "ikonvert-serial" => &["interface", "--kind", "ikonvert"],
        "socketcan-serial" => &["interface", "--kind", "socketcan"],
        "maretron-ipg" => &["interface", "--kind", "maretron"],
        "canboat-pipeline" => &["server"],
        "canboat-tui" => &["tui"],
        "replay" => &["replay"],
        "n2kd" => &["n2kd"],
        _ => return None,
    })
}

/// Build `["canboat", <prefix…>, <original args after argv[0]>]`.
fn rewrite(prefix: &[&str], argv: Vec<OsString>) -> Vec<OsString> {
    let mut out = Vec::with_capacity(prefix.len() + argv.len());
    out.push(OsString::from("canboat"));
    out.extend(prefix.iter().map(OsString::from));
    out.extend(argv.into_iter().skip(1));
    out
}

/// The invoked program name: argv[0]'s basename with any `.exe` dropped.
fn program_name(argv0: &OsStr) -> String {
    let base = Path::new(argv0)
        .file_name()
        .unwrap_or(argv0)
        .to_string_lossy();
    base.strip_suffix(".exe").unwrap_or(&base).to_string()
}

/// Create a symlink for every [`LEGACY_NAMES`] entry in `dir`, pointing
/// at the running `canboat` binary. If `dir` is the binary's own
/// directory the link is relative (`analyzer -> canboat`) so the set
/// survives being moved; otherwise it is absolute.
#[cfg(unix)]
pub fn install_shims(dir: &Path) -> Result<()> {
    use anyhow::Context;
    use std::os::unix::fs::symlink;

    let exe = std::env::current_exe().context("locating the canboat executable")?;
    let exe_name = exe
        .file_name()
        .context("canboat executable has no file name")?;
    let same_dir = exe.parent() == Some(dir);
    let exe_canon = exe.canonicalize().unwrap_or_else(|_| exe.clone());

    let mut installed = 0usize;
    let mut kept = 0usize;
    for name in LEGACY_NAMES {
        let link = dir.join(name);
        if link == exe {
            continue; // never clobber the real binary
        }
        // Something is already called this. Only ever replace one of our
        // own shims -- most of these names are C programs that are still
        // supported and shipped (analyzer, the *-serial gateways,
        // maretron-ipg, replay), and installing over one would delete it.
        if let Ok(md) = std::fs::symlink_metadata(&link) {
            let ours = md.file_type().is_symlink()
                && match link.canonicalize() {
                    // Resolves to this very binary: a shim we made before.
                    Ok(p) => p == exe_canon,
                    // Dangling: whatever it pointed at is gone, so it is
                    // not a working program either way. Safe to refresh.
                    Err(_) => true,
                };
            if !ours {
                let what = if md.file_type().is_symlink() {
                    "a symlink to something else"
                } else {
                    "a file"
                };
                println!("{}: left alone, {what} is already there", link.display());
                kept += 1;
                continue;
            }
            std::fs::remove_file(&link)
                .with_context(|| format!("removing existing {}", link.display()))?;
        }
        let target: &Path = if same_dir { Path::new(exe_name) } else { &exe };
        symlink(target, &link).with_context(|| format!("symlinking {}", link.display()))?;
        println!("{} -> {}", link.display(), target.display());
        installed += 1;
    }
    if kept > 0 {
        println!(
            "\n{installed} shim(s) installed, {kept} name(s) left alone because a program of \
             that name is already installed there. Remove those by hand first if you do want \
             canboat to take the name over."
        );
    }
    Ok(())
}

#[cfg(not(unix))]
pub fn install_shims(_dir: &Path) -> Result<()> {
    anyhow::bail!("install-shims is only supported on Unix");
}

#[cfg(all(test, unix))]
mod shim_tests {
    use super::*;

    /// A name already taken by a real program must survive. Most of
    /// `LEGACY_NAMES` are C binaries that are still shipped and still
    /// supported — `analyzer`, the `*-serial` gateways, `maretron-ipg`,
    /// `replay` — so installing over one would delete a working tool.
    #[test]
    fn install_shims_does_not_clobber_an_existing_program() {
        let dir = std::env::temp_dir().join(format!("canboat-shim-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).expect("temp dir");

        // Stand in for the C analyzer, and for a foreign symlink.
        let c_analyzer = dir.join("analyzer");
        std::fs::write(&c_analyzer, b"#!/bin/sh\necho C analyzer\n").expect("write");
        let elsewhere = dir.join("some-other-tool");
        std::fs::write(&elsewhere, b"x").expect("write");
        std::os::unix::fs::symlink(&elsewhere, dir.join("replay")).expect("symlink");

        install_shims(&dir).expect("install");

        // Untouched: still a regular file with its own content.
        let md = std::fs::symlink_metadata(&c_analyzer).expect("analyzer still there");
        assert!(md.file_type().is_file(), "the C analyzer was replaced");
        assert_eq!(
            std::fs::read(&c_analyzer).expect("read"),
            b"#!/bin/sh\necho C analyzer\n"
        );
        // A symlink pointing somewhere else is left pointing there.
        assert_eq!(
            std::fs::read_link(dir.join("replay")).expect("read_link"),
            elsewhere
        );
        // A free name did get a shim.
        assert!(
            std::fs::symlink_metadata(dir.join("n2kd"))
                .expect("n2kd shim")
                .file_type()
                .is_symlink()
        );

        // Idempotent: a second run refreshes our own shim rather than
        // deciding it is somebody else's.
        install_shims(&dir).expect("re-install");
        assert!(
            std::fs::symlink_metadata(dir.join("n2kd"))
                .expect("n2kd shim still there")
                .file_type()
                .is_symlink()
        );

        let _ = std::fs::remove_dir_all(&dir);
    }
}
