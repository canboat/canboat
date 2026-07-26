// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Shared TCP serving layer for the canboat daemons.
//!
//! Both `n2kd` (JSON in on stdin) and `canboat server` (device in via
//! the pipeline) fan the same records out to the canboat n2kd TCP port
//! layout — JSON snapshot / JSON stream / NMEA 0183 / raw input / AIS /
//! status. The read-side machinery is identical, so it lives here once:
//!
//! - [`hub`] — the lazy per-stream broadcast [`Hub`] / [`BinHub`].
//! - [`tcp`] — the read-only TCP listeners (snapshot / AIS / stream).
//!
//! The snapshot *store* is already shared (`canboat_core::snapshot`);
//! each daemon keeps its own input glue (DecodedPgn → `store_lazy` for
//! the pipeline, JSON line → `store` for n2kd) and its own listeners
//! only for the parts that genuinely differ.

pub mod hub;
pub mod tcp;

pub use hub::{BinHub, Hub};
