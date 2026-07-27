// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Struct-path equivalent of [`crate::n2kd::ais`] — converts AIS PGNs to
//! NMEA 0183 `!AIVDM` sentences by reading fields directly from a
//! `DecodedPgn` instead of round-tripping through analyzer JSON.
//!
//! The bit-packing semantics are identical to `ais.rs` (which is the
//! canboat C reference). What changed with the id-constants refactor:
//! every PGN-specific field is read through its generated
//! [`canboat_core::FieldRef`] constant
//! (`field::ais_class_aposition_report::LONGITUDE`) via
//! [`DecodedPgn::field_ref`] — an `O(1)`, compile-checked lookup, so a
//! rename in canboat.json breaks the build instead of silently dropping
//! a field. Each per-message encoder pulls in its PGN's field module as
//! `f`, so the field set is self-documenting and variant-precise.
//!
//! Two fields are read by *name* on purpose: `"Message ID"` and
//! `"AIS Transceiver information"` are the PGN-independent AIS header —
//! present on every AIS variant with the same name — so a single
//! name-keyed read spans all of them (the same reasoning as the TUI's
//! proprietary-header codes). Everything else is a constant.

use canboat_core::{DecodedPgn, FieldRef, FieldValue, field, pgn};

use crate::n2kd::ais::{
    AIS_TRANSCEIVER_NAMES, BitVector, POSITION_ACCURACY_NAMES, RAIM_NAMES, REPEAT_NAMES,
    emit_sentences,
};

/// Convert a decoded AIS PGN into one or more `!AIVDM,…` sentences.
/// Returns the number of sentences emitted. Mirrors the string-input
/// `ais::convert`.
pub fn convert(out: &mut String, decoded: &DecodedPgn, seq_counter: &mut u8) -> usize {
    let msgid = match pgn_to_msgid(decoded) {
        Some(m) => m,
        None => return 0,
    };
    let channel = ais_channel(decoded);
    let talker = ais_talker(decoded);
    let mut bv = BitVector::new();
    if !encode_payload(&mut bv, msgid, decoded) {
        return 0;
    }
    emit_sentences(out, &bv, talker, channel, seq_counter)
}

/// The AIVDM message type. Prefers the decoded `Message ID` (present on
/// every AIS PGN — read by name as the common header), falling back to
/// the variant's canboat `id` when the field is missing or carries a
/// bare label string instead of the numeric code.
fn pgn_to_msgid(d: &DecodedPgn) -> Option<i64> {
    if let Some(f) = d.field_by_name("Message ID") {
        if let Some(n) = f.value.lookup_value() {
            return Some(n as i64);
        }
        if let Some(n) = f.value.as_i64() {
            return Some(n);
        }
        if let Some(s) = f.value.as_str() {
            match s {
                "Scheduled Class A position report"
                | "Assigned scheduled Class A position report"
                | "Interrogated Class A position report" => return Some(1),
                "Standard Class B position report" => return Some(18),
                "AIS UTC and Date Report" => return Some(4),
                "Static and Voyage Related Data" => return Some(5),
                "Standard SAR Aircraft Position Report" => return Some(9),
                "Addressed safety related message" => return Some(12),
                "Safety related broadcast message" => return Some(14),
                "Extended Class B position report" => return Some(19),
                "ATON report" => return Some(21),
                "Static data report" => return Some(24),
                _ => {}
            }
        }
    }
    let id = d.id;
    Some(if id == pgn::AIS_CLASS_APOSITION_REPORT.id {
        1
    } else if id == pgn::AIS_CLASS_BPOSITION_REPORT.id {
        18
    } else if id == pgn::AIS_CLASS_BEXTENDED_POSITION_REPORT.id {
        19
    } else if id == pgn::AIS_AIDS_TO_NAVIGATION_ATON_REPORT.id {
        21
    } else if id == pgn::AIS_UTC_AND_DATE_REPORT.id {
        4
    } else if id == pgn::AIS_CLASS_ASTATIC_AND_VOYAGE_RELATED_DATA.id {
        5
    } else if id == pgn::AIS_SAR_AIRCRAFT_POSITION_REPORT.id {
        9
    } else if id == pgn::AIS_ADDRESSED_SAFETY_RELATED_MESSAGE.id {
        12
    } else if id == pgn::AIS_SAFETY_RELATED_BROADCAST_MESSAGE.id {
        14
    } else if id == pgn::AIS_CLASS_BSTATIC_DATA_MSG24_PART_A.id
        || id == pgn::AIS_CLASS_BSTATIC_DATA_MSG24_PART_B.id
    {
        24
    } else {
        return None;
    })
}

/// AIS radio channel ('A' / 'B') from the common `AIS Transceiver
/// information` header field.
fn ais_channel(decoded: &DecodedPgn) -> char {
    if matches!(transceiver_info(decoded), 1 | 3) {
        'B'
    } else {
        'A'
    }
}

/// Talker id (`VDM` own-vessel-relayed vs `VDO` own-vessel) from the
/// common `AIS Transceiver information` header field.
fn ais_talker(decoded: &DecodedPgn) -> &'static str {
    match transceiver_info(decoded) {
        2..=4 => "VDO",
        _ => "VDM",
    }
}

/// `AIS Transceiver information` — the PGN-independent AIS header field,
/// read by name (every AIS variant carries it under the same name).
fn transceiver_info(d: &DecodedPgn) -> i64 {
    enum_by_name(d, "AIS Transceiver information", AIS_TRANSCEIVER_NAMES)
}

fn encode_payload(bv: &mut BitVector, msgid: i64, decoded: &DecodedPgn) -> bool {
    match msgid {
        1..=3 => encode_class_a_position(bv, msgid, decoded),
        4 => encode_utc_date(bv, msgid, decoded),
        5 => encode_class_a_static(bv, msgid, decoded),
        9 => encode_sar_aircraft(bv, msgid, decoded),
        12 => encode_addressed_safety(bv, msgid, decoded),
        14 => encode_broadcast_safety(bv, msgid, decoded),
        18 => encode_class_b_position(bv, msgid, decoded),
        19 => encode_class_b_extended(bv, msgid, decoded),
        21 => encode_aton(bv, msgid, decoded),
        24 => encode_class_b_static(bv, msgid, decoded),
        _ => return false,
    }
    true
}

// ---------- per-message-type encoders ----------

fn encode_class_a_position(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_class_aposition_report as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::USER_ID, 0), 30);
    bv.add_int(enum_field(d, f::NAV_STATUS, &[]), 4);
    bv.add_int(rate_of_turn(d, f::RATE_OF_TURN), 8);
    bv.add_int(sog(d, f::SOG), 10);
    bv.add_int(
        enum_field(d, f::POSITION_ACCURACY, POSITION_ACCURACY_NAMES),
        1,
    );
    bv.add_int(longitude(d, f::LONGITUDE), 28);
    bv.add_int(latitude(d, f::LATITUDE), 27);
    bv.add_int(cog(d, f::COG), 12);
    bv.add_int(heading(d, f::HEADING), 9);
    bv.add_int(int_field(d, f::TIME_STAMP).unwrap_or(60).clamp(0, 63), 6);
    bv.add_int(enum_field(d, f::SPECIAL_MANEUVER_INDICATOR, &[]), 2);
    bv.add_int(0, 3); // Spare
    bv.add_int(enum_field(d, f::RAIM, RAIM_NAMES), 1);
    bv.add_int(comm_state(d, f::COMMUNICATION_STATE), 19);
}

fn encode_utc_date(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_utc_and_date_report as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::USER_ID, 0), 30);
    bv.add_int(ais_date(d, f::POSITION_DATE), 23);
    bv.add_int(ais_time(d, f::POSITION_TIME), 17);
    bv.add_int(
        enum_field(d, f::POSITION_ACCURACY, POSITION_ACCURACY_NAMES),
        1,
    );
    bv.add_int(longitude(d, f::LONGITUDE), 28);
    bv.add_int(latitude(d, f::LATITUDE), 27);
    bv.add_int(enum_field(d, f::GNSS_TYPE, &[]), 4);
    bv.add_int(0, 10); // Spare
    bv.add_int(enum_field(d, f::RAIM, RAIM_NAMES), 1);
    bv.add_int(comm_state(d, f::COMMUNICATION_STATE), 19);
}

fn encode_class_a_static(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_class_astatic_and_voyage_related_data as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::USER_ID, 0), 30);
    bv.add_int(enum_field(d, f::AIS_VERSION_INDICATOR, &[]), 2);
    bv.add_int(user_id(d, f::IMO_NUMBER, 0), 30);
    bv.add_string(str_field(d, f::CALLSIGN).unwrap_or(""), 42);
    bv.add_string(str_field(d, f::NAME).unwrap_or(""), 120);
    bv.add_int(enum_field(d, f::TYPE_OF_SHIP, &[]), 8);
    bv.add_int(
        ship_dimensions(
            d,
            f::LENGTH,
            f::BEAM,
            f::POSITION_REFERENCE_FROM_BOW,
            f::POSITION_REFERENCE_FROM_STARBOARD,
        ),
        30,
    );
    bv.add_int(enum_field(d, f::GNSS_TYPE, &[]), 4);
    bv.add_int(ais_eta(d, f::ETA_DATE, f::ETA_TIME), 20);
    bv.add_int(draft(d, f::DRAFT), 8);
    bv.add_string(str_field(d, f::DESTINATION).unwrap_or(""), 120);
    bv.add_int(enum_field(d, f::DTE, &[]), 1);
    bv.add_int(0, 1); // Spare
}

fn encode_sar_aircraft(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_sar_aircraft_position_report as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::USER_ID, 0), 30);
    bv.add_int(altitude(d, f::ALTITUDE), 12);
    bv.add_int((sog(d, f::SOG) + 5) / 10, 10);
    bv.add_int(
        enum_field(d, f::POSITION_ACCURACY, POSITION_ACCURACY_NAMES),
        1,
    );
    bv.add_int(longitude(d, f::LONGITUDE), 28);
    bv.add_int(latitude(d, f::LATITUDE), 27);
    bv.add_int(cog(d, f::COG), 12);
    bv.add_int(int_field(d, f::TIME_STAMP).unwrap_or(60).clamp(0, 63), 6);
    bv.add_int(0, 8); // Regional reserved
    bv.add_int(enum_field(d, f::DTE, &[]), 1);
    bv.add_int(0, 3); // Spare
    // PGN 129798 has no "AIS mode" field — the JSON path read nothing
    // here too, so the slot keeps its all-ones "not available" default.
    bv.add_int(-1, 1);
    bv.add_int(enum_field(d, f::RAIM, RAIM_NAMES), 1);
    // Likewise no "AIS communication state" field on 129798.
    bv.add_int(-1, 1);
    bv.add_int(comm_state(d, f::COMMUNICATION_STATE), 19);
}

fn encode_addressed_safety(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_addressed_safety_related_message as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::SOURCE_ID, 0), 30);
    bv.add_int(int_field(d, f::SEQUENCE_NUMBER).unwrap_or(0).clamp(0, 3), 2);
    bv.add_int(user_id(d, f::DESTINATION_ID, 0), 30);
    bv.add_int(int_field(d, f::RETRANSMIT_FLAG).unwrap_or(0).clamp(0, 1), 1);
    bv.add_int(0, 1); // Spare
    let text = str_field(d, f::SAFETY_RELATED_TEXT).unwrap_or("");
    let mut bits = text.chars().count() * 6;
    if bits > 156 {
        bits = 156;
    }
    if !bits.is_multiple_of(8) {
        bits += 8 - bits % 8;
    }
    bv.add_string(text, bits);
}

fn encode_broadcast_safety(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_safety_related_broadcast_message as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::SOURCE_ID, 0), 30);
    bv.add_int(0, 2); // Spare
    let text = str_field(d, f::SAFETY_RELATED_TEXT).unwrap_or("");
    let mut bits = text.chars().count() * 6;
    if bits > 161 {
        bits = 161;
    }
    if !bits.is_multiple_of(8) {
        bits += 8 - bits % 8;
    }
    bv.add_string(text, bits);
}

fn encode_class_b_position(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_class_bposition_report as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::USER_ID, 0), 30);
    bv.add_int(0, 8); // Regional reserved
    bv.add_int(sog(d, f::SOG), 10);
    bv.add_int(
        enum_field(d, f::POSITION_ACCURACY, POSITION_ACCURACY_NAMES),
        1,
    );
    bv.add_int(longitude(d, f::LONGITUDE), 28);
    bv.add_int(latitude(d, f::LATITUDE), 27);
    bv.add_int(cog(d, f::COG), 12);
    bv.add_int(heading(d, f::HEADING), 9);
    bv.add_int(int_field(d, f::TIME_STAMP).unwrap_or(60).clamp(0, 63), 6);
    bv.add_int(0, 2); // Regional reserved
    bv.add_int(enum_field(d, f::UNIT_TYPE, &[]), 1);
    bv.add_int(enum_field(d, f::INTEGRATED_DISPLAY, &[]), 1);
    bv.add_int(enum_field(d, f::DSC, &[]), 1);
    bv.add_int(enum_field(d, f::BAND, &[]), 1);
    bv.add_int(enum_field(d, f::CAN_HANDLE_MSG22, &[]), 1);
    bv.add_int(enum_field(d, f::AIS_MODE, &[]), 1);
    bv.add_int(enum_field(d, f::RAIM, RAIM_NAMES), 1);
    bv.add_int(enum_field(d, f::AIS_COMMUNICATION_STATE, &[]), 1);
    bv.add_int(comm_state(d, f::COMMUNICATION_STATE), 19);
}

fn encode_class_b_extended(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_class_bextended_position_report as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::USER_ID, 0), 30);
    bv.add_int(0, 8); // Regional reserved
    bv.add_int(sog(d, f::SOG), 10);
    bv.add_int(
        enum_field(d, f::POSITION_ACCURACY, POSITION_ACCURACY_NAMES),
        1,
    );
    bv.add_int(longitude(d, f::LONGITUDE), 28);
    bv.add_int(latitude(d, f::LATITUDE), 27);
    bv.add_int(cog(d, f::COG), 12);
    bv.add_int(heading(d, f::TRUE_HEADING), 9);
    bv.add_int(int_field(d, f::TIME_STAMP).unwrap_or(60).clamp(0, 63), 6);
    bv.add_int(0, 4); // Regional reserved
    bv.add_string(str_field(d, f::NAME).unwrap_or(""), 120);
    bv.add_int(enum_field(d, f::TYPE_OF_SHIP, &[]), 8);
    bv.add_int(
        ship_dimensions(
            d,
            f::LENGTH,
            f::BEAM,
            f::POSITION_REFERENCE_FROM_BOW,
            f::POSITION_REFERENCE_FROM_STARBOARD,
        ),
        30,
    );
    bv.add_int(enum_field(d, f::GNSS_TYPE, &[]), 4);
    bv.add_int(enum_field(d, f::RAIM, RAIM_NAMES), 1);
    bv.add_int(enum_field(d, f::DTE, &[]), 1);
    bv.add_int(enum_field(d, f::AIS_MODE, &[]), 1);
    bv.add_int(0, 4); // Spare
}

fn encode_aton(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    use field::ais_aids_to_navigation_aton_report as f;
    bv.add_int(msgid, 6);
    bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
    bv.add_int(user_id(d, f::USER_ID, 0), 30);
    bv.add_int(enum_field(d, f::ATON_TYPE, &[]), 5);
    let aton_name = str_field(d, f::ATON_NAME).unwrap_or("");
    let (name20, ext) = split_aton_name(aton_name);
    bv.add_string(name20, 120);
    bv.add_int(
        enum_field(d, f::POSITION_ACCURACY, POSITION_ACCURACY_NAMES),
        1,
    );
    bv.add_int(longitude(d, f::LONGITUDE), 28);
    bv.add_int(latitude(d, f::LATITUDE), 27);
    bv.add_int(
        ship_dimensions(
            d,
            f::LENGTH_DIAMETER,
            f::BEAM_DIAMETER,
            f::POSITION_REFERENCE_FROM_TRUE_NORTH_FACING_EDGE,
            f::POSITION_REFERENCE_FROM_STARBOARD_EDGE,
        ),
        30,
    );
    // PGN 129041 has no "GNSS type" field (its EPFD is "Position Fixing
    // Device Type"); the JSON path read nothing here too, so keep the
    // slot at its all-ones "not available" default.
    bv.add_int(-1, 4);
    bv.add_int(int_field(d, f::TIME_STAMP).unwrap_or(60).clamp(0, 63), 6);
    bv.add_int(enum_field(d, f::OFF_POSITION_INDICATOR, &[]), 1);
    bv.add_int(0, 8); // Regional reserved
    bv.add_int(enum_field(d, f::RAIM, RAIM_NAMES), 1);
    bv.add_int(enum_field(d, f::VIRTUAL_ATON_FLAG, &[]), 1);
    bv.add_int(enum_field(d, f::ASSIGNED_MODE_FLAG, &[]), 1);
    bv.add_int(0, 1); // Spare
    let mut ext_bits = ext.chars().count() * 6;
    if ext_bits % 8 != 0 {
        ext_bits += 8 - ext_bits % 8;
    }
    bv.add_string(ext, ext_bits);
}

fn encode_class_b_static(bv: &mut BitVector, msgid: i64, d: &DecodedPgn) {
    bv.add_int(msgid, 6);
    if d.id == pgn::AIS_CLASS_BSTATIC_DATA_MSG24_PART_A.id {
        use field::ais_class_bstatic_data_msg24_part_a as f;
        bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
        bv.add_int(user_id(d, f::USER_ID, 0), 30);
        bv.add_int(0, 2); // Part number = A
        bv.add_string(str_field(d, f::NAME).unwrap_or(""), 120);
        bv.add_int(0, 8); // Spare
    } else if d.id == pgn::AIS_CLASS_BSTATIC_DATA_MSG24_PART_B.id {
        use field::ais_class_bstatic_data_msg24_part_b as f;
        bv.add_int(repeat_indicator(d, f::REPEAT_INDICATOR), 2);
        bv.add_int(user_id(d, f::USER_ID, 0), 30);
        bv.add_int(1, 2); // Part number = B
        bv.add_int(enum_field(d, f::TYPE_OF_SHIP, &[]), 8);
        bv.add_string(str_field(d, f::VENDOR_ID).unwrap_or(""), 42);
        bv.add_string(str_field(d, f::CALLSIGN).unwrap_or(""), 42);
        bv.add_int(
            ship_dimensions(
                d,
                f::LENGTH,
                f::BEAM,
                f::POSITION_REFERENCE_FROM_BOW,
                f::POSITION_REFERENCE_FROM_STARBOARD,
            ),
            30,
        );
        bv.add_int(user_id(d, f::MOTHERSHIP_USER_ID, 0), 30);
        bv.add_int(0, 6); // Spare
    }
}

// ---------- helpers ----------

fn int_field(d: &DecodedPgn, f: FieldRef) -> Option<i64> {
    d.field(f).and_then(|x| x.value.as_i64())
}

fn num_field(d: &DecodedPgn, f: FieldRef) -> Option<f64> {
    d.field(f).and_then(|x| x.value.as_f64())
}

/// Like [`num_field`] but in a requested unit — the AIS bit layouts are
/// defined in degrees (COG/heading) and deg/s (rate of turn), so ask for
/// those and let the core convert from the stream's schema unit (Metric
/// or SI). See [`canboat_core::DecodedField::as_f64_in`].
fn num_field_in(d: &DecodedPgn, f: FieldRef, unit: &str) -> Option<f64> {
    d.field(f).and_then(|x| x.as_f64_in(unit))
}

fn str_field(d: &DecodedPgn, f: FieldRef) -> Option<&str> {
    d.field(f).and_then(|x| x.value.as_str())
}

fn repeat_indicator(d: &DecodedPgn, f: FieldRef) -> i64 {
    enum_field(d, f, REPEAT_NAMES).clamp(0, 3)
}

/// Read an MMSI or 30-bit user ID. canboat-core decodes MMSI fields
/// to `FieldValue::Mmsi(u32)` which `as_i64()` widens cleanly.
fn user_id(d: &DecodedPgn, f: FieldRef, default: i64) -> i64 {
    int_field(d, f).unwrap_or(default)
}

/// Resolve an enum field by its constant. `Lookup` carries the integer
/// code directly; the `names` fallback list handles the rare case where
/// the decoder produced a bare string we still need to map. `-1` when
/// the field is absent from the record.
fn enum_field(d: &DecodedPgn, f: FieldRef, names: &[(&str, i64)]) -> i64 {
    enum_of(d.field(f), names)
}

/// Same as [`enum_field`] but keyed by field *name* — used only for the
/// PGN-independent AIS header field(s).
fn enum_by_name(d: &DecodedPgn, name: &str, names: &[(&str, i64)]) -> i64 {
    enum_of(d.field_by_name(name), names)
}

fn enum_of(field: Option<&canboat_core::DecodedField>, names: &[(&str, i64)]) -> i64 {
    let Some(f) = field else {
        return -1;
    };
    if let Some(n) = f.value.lookup_value() {
        return n as i64;
    }
    if let Some(n) = f.value.as_i64() {
        return n;
    }
    if let Some(s) = f.value.as_str() {
        for (name, val) in names {
            if s == *name {
                return *val;
            }
        }
    }
    -1
}

/// `Rate of Turn` — non-linear AIS encoding. Identical math to
/// [`crate::n2kd::ais::rate_of_turn`].
fn rate_of_turn(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(rot) = num_field_in(d, f, "deg/s") else {
        return -128;
    };
    let v = rot * 60.0;
    let sign = if v >= 0.0 { 1.0 } else { -1.0 };
    let r = 4.733 * v.abs().sqrt();
    let r = (r + 0.5) as i64;
    let r = r.clamp(-127, 127) * sign as i64;
    if (-127..=127).contains(&r) { r } else { -128 }
}

fn sog(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(v) = num_field(d, f) else {
        return 1023;
    };
    let n = (v * 19.438_444_92 + 0.5) as i64;
    if (0..=1022).contains(&n) { n } else { 1023 }
}

fn cog(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(v) = num_field_in(d, f, "deg") else {
        return 3600;
    };
    let n = (v * 10.0 + 0.5) as i64;
    if (0..=3599).contains(&n) { n } else { 3600 }
}

fn heading(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(v) = num_field_in(d, f, "deg") else {
        return 511;
    };
    let n = (v + 0.5) as i64;
    if (0..=359).contains(&n) { n } else { 511 }
}

fn longitude(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(v) = num_field(d, f) else {
        return 0x6791AC0;
    };
    let n = (v * 600_000.0).round() as i64;
    if (-108_000_000..=108_000_000).contains(&n) {
        n
    } else {
        0x6791AC0
    }
}

fn latitude(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(v) = num_field(d, f) else {
        return 0x3412140;
    };
    let n = (v * 600_000.0).round() as i64;
    if (-54_000_000..=54_000_000).contains(&n) {
        n
    } else {
        0x3412140
    }
}

fn altitude(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(v) = num_field(d, f) else {
        return 4095;
    };
    let n = (v + 0.5) as i64;
    if (0..=4094).contains(&n) { n } else { 4095 }
}

fn draft(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(v) = num_field(d, f) else {
        return 0;
    };
    let n = (v * 10.0 + 0.5) as i64;
    n.clamp(0, 255)
}

/// `Communication State` is decoded as a `FieldValue::Binary` (raw
/// bytes) or `FieldValue::Number` depending on the canboat.json
/// field type. We accept either: numeric path returns the value
/// directly; binary path packs the first 3 bytes little-endian into
/// the 19-bit slot, matching canboat C's text-form parser.
fn comm_state(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(f) = d.field(f) else {
        return 393_222;
    };
    if let Some(n) = f.value.as_i64() {
        return n & ((1 << 19) - 1);
    }
    if let FieldValue::Binary(bytes) = &f.value
        && bytes.len() >= 3
    {
        let v = (bytes[0] as i64) | ((bytes[1] as i64) << 8) | ((bytes[2] as i64) << 16);
        return v & ((1 << 19) - 1);
    }
    if let Some(s) = f.value.as_str() {
        let trimmed = s.trim_matches([' ', '"']);
        if let Ok(v) = trimmed.parse::<i64>() {
            return v & ((1 << 19) - 1);
        }
        let bytes: Vec<u8> = trimmed
            .split_whitespace()
            .filter_map(|t| u8::from_str_radix(t, 16).ok())
            .collect();
        if bytes.len() >= 3 {
            let v = (bytes[0] as i64) | ((bytes[1] as i64) << 8) | ((bytes[2] as i64) << 16);
            return v & ((1 << 19) - 1);
        }
    }
    0
}

/// Format a `FieldValue::Date(days)` as `YYYY.MM.DD`, then return a
/// 10-byte buffer in canboat's canonical date string form. Returns
/// `None` if the field is missing or not a date.
fn date_string(d: &DecodedPgn, f: FieldRef) -> Option<String> {
    let f = d.field(f)?;
    let FieldValue::Date(days) = f.value else {
        return None;
    };
    let mut buf = String::with_capacity(10);
    canboat_core::output::format_date(days, &mut buf).ok()?;
    Some(buf)
}

/// A DATE-typed field packed as `(year << 9) | (month << 5) | day`.
fn ais_date(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(s) = date_string(d, f) else {
        return 0;
    };
    let mut parts = s.split('.');
    let y: u32 = parts.next().and_then(|p| p.parse().ok()).unwrap_or(0);
    let m: u32 = parts.next().and_then(|p| p.parse().ok()).unwrap_or(0);
    let d: u32 = parts.next().and_then(|p| p.parse().ok()).unwrap_or(0);
    let y = if y > 9999 { 0 } else { y };
    let m = if m > 12 { 0 } else { m };
    let d = if d > 31 { 0 } else { d };
    ((y as i64) << 9) | ((m as i64) << 5) | d as i64
}

/// A TIME-typed field packed as `(hour << 12) | (minute << 6) | second`.
fn ais_time(d: &DecodedPgn, f: FieldRef) -> i64 {
    let Some(f) = d.field(f) else {
        return 0;
    };
    let FieldValue::Time { seconds, .. } = f.value else {
        return 0;
    };
    if !seconds.is_finite() || seconds < 0.0 {
        return 0;
    }
    let whole = seconds.trunc() as u64;
    let h = (whole / 3600) as u32;
    let m = ((whole / 60) % 60) as u32;
    let sec_int = (whole % 60) as u32;
    let h = if h > 24 { 24 } else { h };
    let m = if m > 60 { 60 } else { m };
    let sec_int = if sec_int > 60 { 60 } else { sec_int };
    ((h as i64) << 12) | ((m as i64) << 6) | sec_int as i64
}

/// `ETA Date` + `ETA Time` packed into the type-5 ETA field
/// (month/day/hour/minute). Canboat C reads bytes 5,6 and 8,9 of the
/// formatted date string, which works for both `YYYY.MM.DD` and
/// `YYYY-MM-DD` separators — we do the same.
fn ais_eta(d: &DecodedPgn, eta_date: FieldRef, eta_time: FieldRef) -> i64 {
    let (mut month, mut day) = (0u32, 0u32);
    if let Some(s) = date_string(d, eta_date) {
        let bytes = s.as_bytes();
        if bytes.len() >= 10 {
            month =
                10 * (bytes[5].wrapping_sub(b'0')) as u32 + (bytes[6].wrapping_sub(b'0')) as u32;
            day = 10 * (bytes[8].wrapping_sub(b'0')) as u32 + (bytes[9].wrapping_sub(b'0')) as u32;
        }
        if month > 12 {
            month = 0;
        }
        if day > 31 {
            day = 0;
        }
    }
    let (mut hour, mut minute) = (24u32, 60u32);
    if let Some(f) = d.field(eta_time)
        && let FieldValue::Time { seconds, .. } = f.value
        && seconds.is_finite()
        && seconds >= 0.0
    {
        let whole = seconds.trunc() as u64;
        hour = (whole / 3600) as u32;
        minute = ((whole / 60) % 60) as u32;
        if hour > 24 {
            hour = 24;
        }
        if minute > 60 {
            minute = 60;
        }
    }
    ((month as i64) << 16) | ((day as i64) << 11) | ((hour as i64) << 6) | minute as i64
}

/// Pack the four `Position reference / Length / Beam` fields into
/// the 30-bit AIS dimension field. Mirrors `aisShipDimensions` in
/// canboat C. The four field constants differ per PGN (ship vs AtoN),
/// so the caller passes the right ones.
fn ship_dimensions(
    d: &DecodedPgn,
    length: FieldRef,
    beam: FieldRef,
    ref_bow: FieldRef,
    ref_starboard: FieldRef,
) -> i64 {
    let length = num_field(d, length).unwrap_or(0.0) * 10.0;
    let beam = num_field(d, beam).unwrap_or(0.0) * 10.0;
    let ref_bow = num_field(d, ref_bow).unwrap_or(0.0) * 10.0;
    let ref_starboard = num_field(d, ref_starboard).unwrap_or(0.0) * 10.0;
    let to_stern = (length - ref_bow).clamp(0.0, 5110.0);
    let to_port = (beam - ref_starboard).clamp(0.0, 630.0);
    (((ref_bow + 5.0) / 10.0) as i64) << 21
        | (((to_stern + 5.0) / 10.0) as i64) << 12
        | (((to_port + 5.0) / 10.0) as i64) << 6
        | ((ref_starboard + 5.0) / 10.0) as i64
}

fn split_aton_name(s: &str) -> (&str, &str) {
    if s.len() <= 20 {
        return (s, "");
    }
    (&s[..20], s[20..].trim_end_matches('\0'))
}

#[cfg(test)]
mod tests {
    use super::*;
    use canboat_core::{PgnDatabase, json_to_decoded};

    /// Build a DecodedPgn from an analyzer name-value line for testing
    /// the struct-path field helpers.
    fn decode(line: &str) -> DecodedPgn {
        json_to_decoded(line, PgnDatabase::embedded(canboat_core::Units::Metric))
            .expect("known PGN")
    }

    #[test]
    fn sog_clamps_above_range() {
        // 999 m/s ≈ 1940 kn; canboat clamps to the 1023 "unknown"
        // sentinel. (Ported from the deleted JSON-path `ais::sog` test.)
        let d = decode(r#"{"pgn":129039,"src":1,"fields":{"SOG":999.0}}"#);
        assert_eq!(sog(&d, field::ais_class_bposition_report::SOG), 1023);
    }

    #[test]
    fn longitude_default_on_missing() {
        // No Longitude field → ITU "not available" sentinel.
        let d = decode(r#"{"pgn":129039,"src":1,"fields":{}}"#);
        assert_eq!(
            longitude(&d, field::ais_class_bposition_report::LONGITUDE),
            0x6791AC0
        );
    }
}
