// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! canboat-bridge: the live-bus pipeline — device → decode → quirks →
//! serving — hosting the `server` and `n2kd` subcommand implementations.
//!
//! This crate currently exposes the `server` and `n2kd` modules as they were
//! lifted out of the `canboat` binary (behaviour-preserving). The curated
//! `Bridge` library API (a plain `BridgeConfig`, an in-process
//! `Receiver<DecodedPgn>`, an optional serving layer with a real shutdown
//! handle) is built on top of these in the following phases — see
//! `docs/library-api-plan.md`.

pub mod n2kd;
pub mod server;

/// Build-time version + commit info and the analyzer JSON version banner
/// (also used by the binary's legacy `analyzer` shim).
pub mod build_info;
