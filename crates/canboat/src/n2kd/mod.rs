// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! n2kd — the NMEA 2000 daemon modules, folded into the `canboat`
//! binary (formerly the standalone `n2kd` crate).
//!
//! Two consumers share this code: the `canboat n2kd` subcommand
//! ([`app::run`], the analyzer-JSON stdin → TCP multiplexer) and the
//! `canboat server` pipeline ([`crate::server`]), which reuses the
//! `serving` TCP layer, the `request_engine`, the NMEA 0183 / AIS
//! converters (`nmea0183`, `ais_decoded`, `decoded`), and the
//! `nmea_filter`.

pub mod ais;
pub mod ais_decoded;
// The standalone `n2kd` daemon CLI (clap + logger init) — a `cli`-only entry.
#[cfg(feature = "cli")]
pub mod app;
pub mod decoded;
pub mod json;
pub mod nmea0183;
pub mod nmea_filter;
pub mod overrides;
pub mod request_engine;
pub mod serving;
