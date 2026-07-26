// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Shared CLI plumbing for the canboat-rs binaries.
//!
//! At the moment the only thing here is [`canboat_argv`], an argv
//! pre-processor that lets clap accept canboat's single-dash long
//! options (`-json`, `-rx`, `-fixtime`, …). canboat's C tools use
//! that form throughout; clap defaults to requiring a double-dash
//! for any option longer than one character. Each binary calls
//! `canboat_argv()` to normalise its own argv before handing it to
//! `Cli::parse_from`.

use std::env;
use std::ffi::OsString;

/// The canboat copyright line and schema version, extracted from
/// `crates/canboat-core/data/canboat.json` at build time by
/// `canboat-core/build.rs`. Every binary shows them in `--help` (via
/// clap `after_help`) and logs them at startup when verbose logging
/// is on.
pub use canboat_core::{CANBOAT_JSON_VERSION, COPYRIGHT_ID};

/// `--help` footer shared by all binaries: embedded schema version +
/// the canboat copyright line.
pub fn help_footer() -> String {
    format!("canboat.json v{CANBOAT_JSON_VERSION}\n{COPYRIGHT_ID}")
}

/// Log the standard startup banner at info level: binary name and
/// version, embedded canboat.json version, copyright. Callers pass
/// `env!("CARGO_PKG_NAME")` / `env!("CARGO_PKG_VERSION")` so the
/// values come from the binary crate, not from canboat-cli.
pub fn log_startup(name: &str, version: &str) {
    log::info!("{name} v{version} (canboat.json v{CANBOAT_JSON_VERSION}) {COPYRIGHT_ID}");
}

/// Read the process argv and translate any single-dash long options
/// to double-dash form. This lets users invoke a canboat-rs tool the
/// same way they'd invoke its C counterpart:
///
/// ```text
///   analyzer -json -nv -fixtime pgn-test < frames.in
///   ikonvert-serial -rx 127250,127251 -reset 30 /dev/ttyUSB0
/// ```
///
/// Rules:
///   - `-` (lone dash) is left alone — it's our stdin-as-device marker.
///   - `--` is left alone (clap uses it to stop option parsing).
///   - `-X` (a single char after the dash) is a short option, left alone.
///   - Anything longer that starts with a single dash and an alphabetic
///     character is rewritten to start with `--`. `-fixtime=foo` and
///     `-fixtime` are both translated.
///   - Bare negative numbers (`-42`) are left alone.
///
/// Returns an iterator yielding `OsString`s, suitable for
/// `Cli::parse_from`.
pub fn canboat_argv() -> Vec<OsString> {
    normalize(env::args_os())
}

/// Public for unit tests; identical to [`canboat_argv`] but takes an
/// arbitrary input iterator instead of `env::args_os()`.
pub fn normalize<I: IntoIterator<Item = OsString>>(args: I) -> Vec<OsString> {
    args.into_iter().map(rewrite_one).collect()
}

fn rewrite_one(arg: OsString) -> OsString {
    let Some(s) = arg.to_str() else {
        return arg;
    };
    // `--anything` or `-` or `--` or `-X` (X = one char): leave alone.
    if !s.starts_with('-') || s.starts_with("--") {
        return arg;
    }
    if s.len() <= 2 {
        return arg;
    }
    // First char after the dash must be alphabetic for this to be a
    // long-form option. Skip `-42`, `-0.5`, etc.
    let mut chars = s[1..].chars();
    let Some(c) = chars.next() else {
        return arg;
    };
    if !c.is_ascii_alphabetic() {
        return arg;
    }
    OsString::from(format!("-{s}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn norm(input: &[&str]) -> Vec<String> {
        normalize(input.iter().map(|s| OsString::from(*s)))
            .into_iter()
            .map(|s| s.to_string_lossy().into_owned())
            .collect()
    }

    #[test]
    fn translates_canboat_long_options() {
        assert_eq!(
            norm(&["analyzer", "-json", "-nv", "-fixtime", "pgn-test"]),
            vec!["analyzer", "--json", "--nv", "--fixtime", "pgn-test"]
        );
    }

    #[test]
    fn leaves_short_options_alone() {
        assert_eq!(
            norm(&["bin", "-r", "-w", "-d"]),
            vec!["bin", "-r", "-w", "-d"]
        );
    }

    #[test]
    fn leaves_double_dash_alone() {
        assert_eq!(
            norm(&["bin", "--rx", "12,34", "--"]),
            vec!["bin", "--rx", "12,34", "--"]
        );
    }

    #[test]
    fn leaves_single_dash_alone() {
        // `-` is our stdin marker.
        assert_eq!(norm(&["bin", "-"]), vec!["bin", "-"]);
    }

    #[test]
    fn leaves_negative_numbers_alone() {
        assert_eq!(norm(&["bin", "-42", "-0.5"]), vec!["bin", "-42", "-0.5"]);
    }

    #[test]
    fn translates_with_equals() {
        assert_eq!(norm(&["bin", "-fixtime=foo"]), vec!["bin", "--fixtime=foo"]);
    }
}
