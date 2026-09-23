// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Build-time version + commit info, and the analyzer JSON version
//! banner that consumers (n2kd) key off.
//!
//! The git SHA is captured by this crate's `build.rs` (vergen) into
//! `VERGEN_GIT_SHA` / `VERGEN_GIT_DIRTY`; those env vars only resolve
//! inside this crate, so the banner is built here rather than in
//! canboat-core or canboat-cli. They are read with `option_env!`
//! because a build outside a git worktree — a crates.io tarball, or
//! `cargo package`'s verification build — has neither.

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
/// `{"version":"0.5.0","commit":"08f08eb","units":"std","showLookupValues":true}`.
///
/// Superset of canboat C's banner (`analyzer.c:395`): it carries the
/// extra `commit` field, but keeps `version` + `units` +
/// `showLookupValues` byte-compatible so C's n2kd — which requires
/// `"version":` and `"showLookupValues":true` in the first line — still
/// accepts a Rust-produced stream, and vice versa.
pub fn version_banner(si: bool, name_value: bool) -> String {
    format!(
        "{{\"version\":\"{}\",\"commit\":\"{}\",\"units\":\"{}\",\"showLookupValues\":{}}}",
        VERSION,
        commit(),
        if si { "si" } else { "std" },
        if name_value { "true" } else { "false" },
    )
}
