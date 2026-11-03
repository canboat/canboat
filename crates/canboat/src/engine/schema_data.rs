// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Static schema tables, generated at build time from the repository's own
//! YAML database (`database/**`) by `keel generate`.
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
//!   [`FIELD_TYPE_LOOKUPS_SI`], [`FIELD_TYPE_LOOKUPS_METRIC`] — sorted
//!   alphabetically by name for binary-search lookup.
//! - [`SCHEMA_VERSION`], [`VERSION`] — strings from canboat.json.
//! - [`COPYRIGHT_ID`] — the `(C) ...` line from canboat.json's
//!   Copyright banner, for help text and verbose logging.

// The SI schema carries canboat.json's raw radian ranges (±π), so the
// generated literals legitimately sit near `f64::consts::PI`. They are
// verbatim schema data, not an inexact use of the constant.
#![allow(clippy::approx_constant)]

include!("schema_generated.rs");

/// Content hash of the generated schema tables, computed by build.rs. Two
/// processes compare it to prove they were built from the same database
/// before trusting each other's field indices.
pub const SCHEMA_HASH: u64 = include!(concat!(env!("OUT_DIR"), "/schema_hash.rs"));
