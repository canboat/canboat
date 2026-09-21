// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! Character sets for 8-bit string fields, and the single table the C
//! analyzer and the Rust runtime are both generated from.
//!
//! # Why a field would declare one
//!
//! NMEA 2000 leaves a byte >= 0x80 in an 8-bit string field undefined, so
//! canboat reads a string as UTF-8 and falls back to Latin-1 when that is not
//! well-formed (#864 / #866). That covers every device seen so far but one: a
//! Fusion head unit forwards FM radio text with its RDS bytes unconverted, and
//! there 0x91 is `ä` where Latin-1 has a C1 control character. Such a field
//! declares `encoding:`, which replaces only the fallback -- UTF-8 is still
//! tried first.
//!
//! That ordering is not merely cautious. **RDS2 carries text as UTF-8**, in
//! new groups, while keeping the old character set for the legacy 0A/2A
//! groups that carry the station name today. So the day Fusion ships a head
//! unit that speaks RDS2, the same field starts arriving as valid UTF-8 and
//! decodes correctly with no database change. Trying UTF-8 first is what makes
//! the declaration safe to add now.
//!
//! # RDS_G0
//!
//! The Basic RDS character set: IEC 62106 table E.1, published identically as
//! the "Complete EBU Latin based repertoire" in ETSI TS 101 756 **V1.6.1**
//! (2014-05) annex C, whose note reads
//!
//! > This character set is identical to the Basic RDS character set,
//! > see [2] table E.1
//!
//! <https://www.etsi.org/deliver/etsi_ts/101700_101799/101756/01.06.01_60/ts_101756v010601p.pdf#page=42>
//!
//! Transcribed from table C.1 and cross-checked against table C.2.
//!
//! **Use this one, not the 2015 rewrite.** TS 101 756 **V1.8.1** (2015-12)
//! annex C reassigns 36 of these 224 positions -- the whole 0xA0..0xBF symbol
//! block becomes Baltic letters, 0x24 becomes `ł`, and `\ { | }` become
//! `Ů « ů »`. That extension is **DAB only**: it never propagated to RDS,
//! whose answer to Unicode was RDS2's UTF-8 rather than a new code table, and
//! broadcasters use G0 essentially exclusively in any case. The two agree on
//! every accented Latin letter that turns up in a station name, 0x91 among
//! them, so a capture cannot tell them apart -- which is exactly why this is
//! pinned to a citation rather than to evidence.
//!
//! If a DAB text field ever needs the V1.8.1 repertoire it should be added
//! beside this as its own encoding, not folded into it.
//!
//! Third-party copies of this table are unreliable: a widely used one has
//! `β` at 0x8D where the spec has `ß`, plus three more errors. It was worth
//! going to the standard.
//!
//! # Undefined codes
//!
//! The table covers 0x20..=0xFE except 0x7F. Everything else -- 0x00..0x1F,
//! 0x7F and 0xFF -- passes through as its own byte value: canboat shortens a
//! string at the first NUL and trims 0xff and whitespace padding off the end,
//! and both need to keep seeing those bytes as themselves.
//!
//! # Where it lives
//!
//! keel owns the table because keel already generates both consumers' tables
//! and may not depend on either (MERGE-CANBOAT-RS.md §9). `keel generate`
//! emits `crates/canboat-core/src/charset_generated.rs` and
//! `analyzer/charset-generated-data.h` from it, so the three decoders cannot
//! drift.

/// Basic RDS (G0) for 0x20..=0xFF as Unicode; `\0` marks a position the
/// standard leaves undefined, which decodes to the byte itself.
#[rustfmt::skip]
pub const RDS_G0: [char; 224] = [
    // 0x20
    ' ', '!', '"', '#', '¤', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/',
    // 0x30
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?',
    // 0x40
    '@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O',
    // 0x50
    'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '[', '\\', ']', '―', '_',
    // 0x60
    '║', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
    // 0x70
    'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '¯', '\0',
    // 0x80
    'á', 'à', 'é', 'è', 'í', 'ì', 'ó', 'ò', 'ú', 'ù', 'Ñ', 'Ç', 'Ş', 'ß', '¡', 'Ĳ',
    // 0x90
    'â', 'ä', 'ê', 'ë', 'î', 'ï', 'ô', 'ö', 'û', 'ü', 'ñ', 'ç', 'ş', 'ğ', 'ı', 'ĳ',
    // 0xA0
    'ª', 'α', '©', '‰', 'Ğ', 'ě', 'ň', 'ő', 'π', '€', '£', '$', '←', '↑', '→', '↓',
    // 0xB0
    'º', '¹', '²', '³', '±', 'İ', 'ń', 'ű', 'µ', '¿', '÷', '°', '¼', '½', '¾', '§',
    // 0xC0
    'Á', 'À', 'É', 'È', 'Í', 'Ì', 'Ó', 'Ò', 'Ú', 'Ù', 'Ř', 'Č', 'Š', 'Ž', 'Ð', 'Ŀ',
    // 0xD0
    'Â', 'Ä', 'Ê', 'Ë', 'Î', 'Ï', 'Ô', 'Ö', 'Û', 'Ü', 'ř', 'č', 'š', 'ž', 'đ', 'ŀ',
    // 0xE0
    'Ã', 'Å', 'Æ', 'Œ', 'ŷ', 'Ý', 'Õ', 'Ø', 'Þ', 'Ŋ', 'Ŕ', 'Ć', 'Ś', 'Ź', 'Ŧ', 'ð',
    // 0xF0
    'ã', 'å', 'æ', 'œ', 'ŵ', 'ý', 'õ', 'ø', 'þ', 'ŋ', 'ŕ', 'ć', 'ś', 'ź', 'ŧ', '\0',
];

/// Character sets a field may name in `encoding:`. Absent means `LATIN1`,
/// which is what every string field decoded as before this existed.
pub const ENCODINGS: [&str; 2] = ["LATIN1", "RDS_G0"];

/// One byte of Basic RDS. Codes the standard leaves undefined keep their own
/// value so the padding trims downstream still recognise them.
pub fn rds_g0_char(b: u8) -> char {
    if b < 0x20 {
        return b as char;
    }
    match RDS_G0[b as usize - 0x20] {
        '\0' => b as char,
        c => c,
    }
}

/// Read `bytes` in the named charset. Only reached once a UTF-8 attempt has
/// failed, so this never has to choose between the two.
pub fn decode(encoding: Option<&str>, bytes: &[u8]) -> String {
    match encoding {
        Some("RDS_G0") => bytes.iter().map(|&b| rds_g0_char(b)).collect(),
        // `b as char` IS Latin-1 in Rust: U+0000..U+00FF map one to one.
        _ => bytes.iter().map(|&b| b as char).collect(),
    }
}

// =====================================================================
// Generators. The C analyzer and the Rust runtime each get a copy of the
// table above so neither has to reach into keel, which they cannot do
// (MERGE-CANBOAT-RS.md §9). Both are emitted by `keel generate` and
// committed, exactly like the PGN and lookup tables.
// =====================================================================

/// The shared preamble both generated files carry.
fn banner(comment: &str) -> String {
    format!(
        "{comment} Generated by keel from keel/src/charset.rs. DO NOT EDIT.\n\
         {comment}\n\
         {comment} Basic RDS (G0) character set: IEC 62106 table E.1, published as the\n\
         {comment} \"Complete EBU Latin based repertoire\" in ETSI TS 101 756 V1.6.1 annex C.\n\
         {comment} NOT the V1.8.1 (2015) repertoire, which is a DAB-only extension that\n\
         {comment} reassigns 36 of these positions. See keel/src/charset.rs.\n\
         {comment}\n\
         {comment} 0 marks a code the standard leaves undefined; the decoder passes that\n\
         {comment} byte through unchanged so the padding trims still recognise it.\n"
    )
}

/// `analyzer/charset-generated-data.h` — the table as Unicode code points,
/// for the C analyzer's string printer to re-encode as UTF-8.
pub fn emit_charset_h() -> String {
    let mut out = banner("//");
    out.push_str("\n#ifndef CHARSET_GENERATED_DATA_H_\n#define CHARSET_GENERATED_DATA_H_\n\n");
    out.push_str("// Indexed by (byte - 0x20); bytes below 0x20 are never looked up.\n");
    out.push_str("static const uint16_t RDS_G0_TO_UNICODE[224] = {\n");
    for hi in 2..16 {
        out.push_str(&format!("    /* 0x{hi:X}0 */ "));
        let row: Vec<String> = (0..16)
            .map(|lo| {
                let c = RDS_G0[(hi - 2) * 16 + lo];
                format!("0x{:04X}", if c == '\0' { 0 } else { c as u32 })
            })
            .collect();
        out.push_str(&row.join(", "));
        out.push_str(",\n");
    }
    out.push_str("};\n\n#endif\n");
    out
}

/// `crates/canboat-core/src/charset_generated.rs` — the same table for the
/// Rust runtime's `decode_text`.
pub fn emit_charset_rs() -> String {
    let mut out = banner("//");
    out.push_str("\n/// Basic RDS (G0) for 0x20..=0xFF; `\\0` is an undefined code.\n");
    out.push_str("#[rustfmt::skip]\n");
    out.push_str("pub static RDS_G0: [char; 224] = [\n");
    for hi in 2..16 {
        out.push_str(&format!("    // 0x{hi:X}0\n    "));
        let row: Vec<String> = (0..16)
            .map(|lo| {
                let c = RDS_G0[(hi - 2) * 16 + lo];
                match c {
                    '\0' => "'\\0'".to_string(),
                    '\'' => "'\\''".to_string(),
                    '\\' => "'\\\\'".to_string(),
                    c => format!("'{c}'"),
                }
            })
            .collect();
        out.push_str(&row.join(", "));
        out.push_str(",\n");
    }
    out.push_str("];\n\n");
    out.push_str(
        "/// One byte of Basic RDS. Undefined codes keep their own value so the\n\
         /// padding trims downstream still recognise them.\n\
         pub fn rds_g0_char(b: u8) -> char {\n\
         \x20   if b < 0x20 {\n\
         \x20       return b as char;\n\
         \x20   }\n\
         \x20   match RDS_G0[b as usize - 0x20] {\n\
         \x20       '\\0' => b as char,\n\
         \x20       c => c,\n\
         \x20   }\n\
         }\n",
    );
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The byte that motivated all of this. `Bohuslän` is a real Swedish
    /// province, so 0x91 can only be `ä` — Latin-1 gives U+0091, a C1 control
    /// character that renders as nothing at all.
    #[test]
    fn the_fusion_station_name_decodes() {
        let wire = [0x42, 0x6f, 0x68, 0x75, 0x73, 0x6c, 0x91, 0x6e];
        assert_eq!(decode(Some("RDS_G0"), &wire), "Bohuslän");
        assert_eq!(decode(None, &wire), "Bohusl\u{91}n");
    }

    /// Spot-checks against TS 101 756 V1.6.1 table C.1.
    #[test]
    fn the_table_matches_the_spec() {
        for (byte, want) in [
            (0x24u8, '¤'),
            (0x5e, '―'),
            (0x60, '║'),
            (0x7e, '¯'),
            (0x90, 'â'),
            (0x91, 'ä'),
            (0x8d, 'ß'),
            (0x8f, 'Ĳ'),
            (0xa1, 'α'),
            (0xa9, '€'),
            (0xab, '$'),
            (0xbd, '½'),
            (0xee, 'Ŧ'),
        ] {
            assert_eq!(rds_g0_char(byte), want, "byte 0x{byte:02x}");
        }
    }

    /// Positions where the 2015 DAB rewrite (TS 101 756 V1.8.1) disagrees.
    /// This is the Basic RDS set, so these must stay as they are; the test
    /// fails loudly if someone "updates" the table to the newer repertoire.
    #[test]
    fn this_is_basic_rds_not_the_2015_dab_repertoire() {
        assert_eq!(rds_g0_char(0x24), '¤'); // V1.8.1: ł
        assert_eq!(rds_g0_char(0x5c), '\\'); // V1.8.1: Ů
        assert_eq!(rds_g0_char(0x7c), '|'); // V1.8.1: ů
        assert_eq!(rds_g0_char(0xa1), 'α'); // V1.8.1: Ņ
        assert_eq!(rds_g0_char(0xbd), '½'); // V1.8.1: ē
        assert_eq!(rds_g0_char(0x8f), 'Ĳ'); // V1.8.1: Ÿ
    }

    /// Four positions where a widely-copied third-party table is wrong.
    /// 0x8D is the one that matters: `ß` appears in German station names.
    #[test]
    fn the_positions_third_party_tables_get_wrong() {
        assert_eq!(rds_g0_char(0x8d), 'ß'); // not β
        assert_eq!(rds_g0_char(0x60), '║'); // not ‖
        assert_eq!(rds_g0_char(0x9d), 'ğ'); // not ǧ
        assert_eq!(rds_g0_char(0xa4), 'Ğ'); // not Ǧ
    }

    /// Letters and digits are themselves, so an all-ASCII name decodes the
    /// same whichever branch it takes.
    #[test]
    fn letters_and_digits_are_unchanged() {
        assert_eq!(decode(Some("RDS_G0"), b"Radio 1"), "Radio 1");
    }

    /// Codes the standard leaves undefined keep their byte value, so the
    /// NUL-shortening and padding trims downstream still see them.
    #[test]
    fn the_undefined_codes_pass_through() {
        for b in [0x00u8, 0x0a, 0x1f, 0x7f, 0xff] {
            assert_eq!(rds_g0_char(b), b as char, "byte 0x{b:02x}");
        }
    }

    /// No declaration, or `LATIN1`, is the pre-existing behaviour.
    #[test]
    fn the_default_is_latin1() {
        assert_eq!(decode(None, &[0x91]), "\u{91}");
        assert_eq!(decode(Some("LATIN1"), &[0xe4]), "ä");
        assert_eq!(decode(Some("LATIN1"), &[0x91]), "\u{91}");
    }

    /// Exactly two positions in 0x20..=0xFF are undefined, and they are the
    /// two the standard omits. Everything else must have come from the table,
    /// so an off-by-one that shifted a row into a gap — or left one behind —
    /// shows up here rather than as a wrong glyph nobody notices.
    #[test]
    fn exactly_two_positions_are_undefined() {
        let passthrough: Vec<u8> = (0x20..=0xffu8)
            .filter(|&b| rds_g0_char(b) == b as char && RDS_G0[b as usize - 0x20] == '\0')
            .collect();
        assert_eq!(passthrough, vec![0x7f, 0xff]);

        // And every other position carries a real character, not the sentinel.
        for b in 0x20..=0xffu8 {
            if matches!(b, 0x7f | 0xff) {
                continue;
            }
            assert_ne!(RDS_G0[b as usize - 0x20], '\0', "byte 0x{b:02x} is a hole");
        }
    }
}
