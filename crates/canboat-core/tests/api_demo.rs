// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! API surface walkthrough — a single decoded PGN, then the consumer
//! patterns that touch every accessor on `DecodedField` / `DecodedPgn` /
//! `FieldValue` we expose today.
//!
//! Phase 5: schema metadata (`id`, `name`, `order`, `unit`, `resolution`,
//! `precision`, `part_of_primary_key`) reads through accessor methods
//! that delegate to `info: &'static FieldInfo`, with VARIABLE /
//! DYNAMIC_FIELD_VALUE overrides on a boxed sidecar. Runtime values
//! that the decoder owns — `value`, `bit_offset`, `bit_length`,
//! `repeat_index`, `repeat_set` — stay as direct fields.

use canboat_core::{FieldValue, PgnDatabase, RawFrame};
use smallvec::smallvec;

/// PGN 60928 (ISO Address Claim) — same fixture every other test in
/// the workspace uses. 10 fields, one each of NUMBER / LOOKUP /
/// INDIRECT_LOOKUP / RESERVED, so it exercises most accessor paths.
fn decode_iso_address_claim() -> canboat_core::DecodedPgn {
    let frame = RawFrame {
        timestamp: Some("2022-09-10T12:10:16.614Z".into()),
        prio: 6,
        pgn: 60928,
        src: 5,
        dst: 255,
        data: smallvec![0xfb, 0x9b, 0x70, 0x22, 0x00, 0x9b, 0x50, 0xc0],
    };
    PgnDatabase::embedded(canboat_core::Units::Metric)
        .decode(&frame)
        .expect("decode")
}

#[test]
fn decoded_pgn_top_level_accessors() {
    let pgn = decode_iso_address_claim();

    // Header — DecodedPgn's shape isn't part of Phase 5.
    assert_eq!(pgn.pgn, 60928);
    assert_eq!(pgn.prio, 6);
    assert_eq!(pgn.src, 5);
    assert_eq!(pgn.dst, 255);
    assert_eq!(pgn.id, "isoAddressClaim");
    assert_eq!(pgn.description, "ISO Address Claim");
    assert_eq!(pgn.timestamp.as_deref(), Some("2022-09-10T12:10:16.614Z"));
    assert_eq!(pgn.data.len(), 8);
    assert_eq!(pgn.fields.len(), 10);
    assert!(!pgn.has_repeating_set[0]);
}

#[test]
fn decoded_field_metadata_accessors() {
    let pgn = decode_iso_address_claim();

    // Field 1: Unique Number (NUMBER, 21-bit unsigned at offset 0).
    // Schema metadata reads through accessor methods.
    let f = &pgn.fields[0];
    assert_eq!(f.order(), 1);
    assert_eq!(f.id(), "uniqueNumber");
    assert_eq!(f.name(), "Unique Number");
    assert_eq!(f.unit(), None);
    assert_eq!(f.resolution(), Some(1.0));
    assert_eq!(f.precision(), 0);
    assert!(!f.part_of_primary_key());

    // Runtime fields stay as direct accessors.
    assert_eq!(f.bit_offset, Some(0));
    assert_eq!(f.bit_length, Some(21));
    assert_eq!(f.repeat_index, None);
    assert_eq!(f.repeat_set, 0);

    // The raw FieldInfo is reachable for callers who want the full
    // schema record (range_min, range_max, lookup_enumeration,
    // physical_quantity — fields the flat shape didn't expose at all).
    assert_eq!(f.info.id, "uniqueNumber");

    let f = &pgn.fields[1];
    assert_eq!(f.id(), "manufacturerCode");
    assert_eq!(f.bit_length, Some(11));
    assert_eq!(f.info.lookup_enumeration, Some("MANUFACTURER_CODE"));
}

#[test]
fn field_value_consumer_patterns() {
    let pgn = decode_iso_address_claim();

    // FieldValue's shape is orthogonal to Phase 5.
    match &pgn.fields[0].value {
        FieldValue::Integer(v) => assert_eq!(*v, 1_088_507),
        other => panic!("expected Integer, got {other:?}"),
    }
    match &pgn.fields[1].value {
        FieldValue::Lookup { value, name } => {
            assert_eq!(*value, 275);
            assert_eq!(*name, Some("Navico"));
        }
        other => panic!("expected Lookup, got {other:?}"),
    }
    assert_eq!(pgn.fields[0].value.as_i64(), Some(1_088_507));
    assert_eq!(pgn.fields[1].value.as_str(), Some("Navico"));
    assert!(!pgn.fields[0].value.is_not_available());
}

#[test]
fn iterate_all_top_level_fields() {
    let pgn = decode_iso_address_claim();

    // Real consumer pattern: walk only top-level (non-repeating)
    // fields, render `name = value [unit]`. The text formatter does
    // exactly this.
    let mut rendered = Vec::new();
    for f in &pgn.fields {
        if f.repeat_set != 0 {
            continue;
        }
        let unit = f.unit().map(|u| format!(" {u}")).unwrap_or_default();
        let value = match &f.value {
            FieldValue::Integer(v) => v.to_string(),
            FieldValue::Number(v) => format!("{v:.*}", f.precision() as usize),
            FieldValue::Lookup { name: Some(n), .. } => (*n).to_string(),
            FieldValue::Lookup { value, name: None } => value.to_string(),
            _ => "<other>".into(),
        };
        rendered.push(format!("{} = {}{}", f.name(), value, unit));
    }
    assert_eq!(rendered.len(), 10);
    assert_eq!(rendered[0], "Unique Number = 1088507");
    assert_eq!(rendered[1], "Manufacturer Code = Navico");
}

#[test]
fn field_handle_indexed_access() {
    // "Resolve a handle once, reuse for millions of records" pattern.
    let db = PgnDatabase::embedded(canboat_core::Units::Metric);
    let handle = db.field("isoAddressClaim", "uniqueNumber").expect("handle");
    let pgn = decode_iso_address_claim();
    let f = pgn.field(handle).expect("field present");
    assert_eq!(f.value.as_i64(), Some(1_088_507));
}
