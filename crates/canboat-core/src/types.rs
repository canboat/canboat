// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Canboat PGN database types.
//!
//! These are defined in [`canboat_schema`]; this module re-exports them
//! so existing internal call sites keep compiling unchanged. The schema
//! itself (the data) is generated at build time by `build.rs` and lives
//! in [`crate::schema_data`].

pub use canboat_schema::{
    BitLookupTable, BitLookupValue, FieldInfo, FieldRef, FieldType, IndirectLookupTable,
    IndirectLookupValue, LookupFieldTypeTable, LookupFieldTypeValue, LookupTable, LookupValue,
    PacketType, PgnInfo,
};
