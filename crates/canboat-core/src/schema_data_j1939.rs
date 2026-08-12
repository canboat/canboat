// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Static J1939 schema tables, generated from `database/j1939/pgns/*.yaml`
//! by keel — the Rust mirror of the C `pgn-j1939-generated-data.h` that
//! `analyzer-j1939` is built from.
//!
//! Same shape as [`crate::schema_data`]: [`PGNS_SI`] / [`PGNS_METRIC`],
//! [`PGN_INDEX`], `dispatch`, `find_catchall` and this tree's lookup
//! tables. Only the version constants are shared with the main schema.
//! The lookups are *not*: the two trees draw on different manufacturer
//! registries (`MANUFACTURER_CODE` vs `J1939_MANUFACTURER_CODE`), so
//! each module carries just the enumerations its own PGNs reference.
//!
//! Table choice is exclusive, matching keel's `decode` and the C
//! analyzer pair: a J1939 database decodes *only* against the J1939
//! definitions (which carry their own ISO PGNs), never a merge.

#![allow(clippy::approx_constant)]

include!("schema_generated_j1939.rs");
