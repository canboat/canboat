// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Static schema tables, generated at build time from the repository's own
//! YAML database (`database/**`) by `canboat-core/build.rs`, via keel.
//!
//! This file is hand-written -- it is the documented wrapper that `include!`s
//! the generated tables. The generated part is not in the repository; it is
//! rebuilt into `OUT_DIR` whenever the database changes. To change the
//! schema, edit `database/pgns/*.yaml` and rebuild.
//!
//! Exposes:
//! - [`PGNS`] — `&[PgnInfo]` in JSON declaration order.
//! - [`PGN_INDEX`] — `&[(u32, &[u32])]`, sorted by PGN number, value
//!   is a slice of indices into [`PGNS`] (variants in declaration order).
//! - [`LOOKUPS`], [`BIT_LOOKUPS`], [`INDIRECT_LOOKUPS`],
//!   [`FIELD_TYPE_LOOKUPS`] — sorted alphabetically by name for
//!   binary-search lookup.
//! - [`SCHEMA_VERSION`], [`VERSION`] — strings from canboat.json.
//! - [`COPYRIGHT_ID`] — the `(C) ...` line from canboat.json's
//!   Copyright banner, for help text and verbose logging.

// The SI schema carries canboat.json's raw radian ranges (±π), so the
// generated literals legitimately sit near `f64::consts::PI`. They are
// verbatim schema data, not an inexact use of the constant.
#![allow(clippy::approx_constant)]

include!(concat!(env!("OUT_DIR"), "/schema_generated.rs"));
