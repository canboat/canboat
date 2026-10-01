// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Build-time version + commit info, and the analyzer JSON version
//! banner that consumers (n2kd) key off.
//!
//! The git SHA is captured by `build.rs` (vergen) into `VERGEN_GIT_SHA` /
//! `VERGEN_GIT_DIRTY`. They are read with `option_env!` because a build
//! outside a git worktree — a crates.io tarball, or `cargo package`'s
//! verification build — has neither.

/// The canboat-rs release version (the human version).
pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Short git SHA of the build, `"-dirty"`-suffixed when the working
/// tree had uncommitted tracked changes. `"unknown"` for a build with
/// no `.git` to ask (a crates.io or source tarball).
pub fn commit() -> String {
    let sha = option_env!("VERGEN_GIT_SHA").unwrap_or("unknown");
    if option_env!("VERGEN_GIT_DIRTY") == Some("true") {
        format!("{sha}-dirty")
    } else {
        sha.to_string()
    }
}

/// The one-line analyzer version banner, e.g.
/// `{"version":"0.5.0","commit":"08f08eb","units":"std","protocol":"nmea2000","showLookupValues":true}`.
///
/// Superset of canboat C's banner (`analyzer.c:395`): it carries the
/// extra `commit` and `protocol` fields, but keeps `version` + `units` +
/// `showLookupValues` byte-compatible so C's n2kd — which requires
/// `"version":` and `"showLookupValues":true` in the first line — still
/// accepts a Rust-produced stream, and vice versa.
///
/// `protocol` names the table the records were decoded against
/// (`nmea2000` or `j1939`): the same PGN number means different things
/// on the two, so a consumer has to know. A banner without it (canboat
/// C, Rust before 8.4) is NMEA 2000.
pub fn version_banner(si: bool, name_value: bool, protocol: crate::engine::BusProtocol) -> String {
    format!(
        "{{\"version\":\"{}\",\"commit\":\"{}\",\"units\":\"{}\",\"protocol\":\"{}\",\"showLookupValues\":{}}}",
        VERSION,
        commit(),
        if si { "si" } else { "std" },
        protocol,
        if name_value { "true" } else { "false" },
    )
}
