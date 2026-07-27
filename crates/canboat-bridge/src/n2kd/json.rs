// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Compatibility shim — the analyzer-JSON value extractor lives in
//! `canboat_core::analyzer_json` so the snapshot ingest path and the
//! TUI viewer can share it. `crate::n2kd::json::*` calls inside this crate
//! keep working unchanged via this re-export.

pub use canboat_core::analyzer_json::*;
