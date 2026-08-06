// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Static J1939 schema tables, generated from `database/j1939/pgns/*.yaml`
//! by keel — the Rust mirror of the C `pgn-j1939-generated-data.h` that
//! `analyzer-j1939` is built from.
//!
//! Same shape as [`crate::schema_data`], but PGN tables only:
//! [`PGNS_SI`] / [`PGNS_METRIC`], [`PGN_INDEX`], `dispatch` and
//! `find_catchall`. The version constants and every lookup table are
//! shared with the main schema — the J1939 definitions reference the
//! same enumerations, so [`crate::db::PgnDatabase::embedded_j1939`]
//! wires this module's PGN tables to `schema_data`'s lookups.
//!
//! Table choice is exclusive, matching keel's `decode` and the C
//! analyzer pair: a J1939 database decodes *only* against the J1939
//! definitions (which carry their own ISO PGNs), never a merge.

#![allow(clippy::approx_constant)]

include!("schema_generated_j1939.rs");
