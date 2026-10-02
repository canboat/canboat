// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Static Quick PCS schema tables, generated from
//! `database/quick/pgns/*.yaml` by keel. Quick has no C analyzer.
//!
//! Same shape as [`crate::engine::schema_data`]: [`PGNS_SI`] / [`PGNS_METRIC`],
//! [`PGN_INDEX`], `dispatch`, `find_catchall` and this tree's lookup
//! tables. Only the version constants are shared with the main schema;
//! the module carries just the enumerations its own definitions reference.
//!
//! Table choice is exclusive: a Quick database decodes *only* against the
//! Quick definitions, whose "PGNs" are 11-bit message types.

#![allow(clippy::approx_constant)]

include!("schema_generated_quick.rs");
