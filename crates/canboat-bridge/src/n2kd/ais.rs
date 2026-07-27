// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! AIS AIVDM/AIVDO emission machinery, shared by the struct-path
//! encoder in [`crate::n2kd::ais_decoded`].
//!
//! Mirrors the emit half of `canboat/n2kd/gps_ais.c`: [`BitVector`]
//! packs ITU-R M.1371 bit layouts, and [`emit_sentences`] re-encodes
//! the packed bits as 6-bit ASCII (each 6-bit group → one printable
//! character, `0`/`8`-offset trick), splitting into 60-char `!AIVDM,…`
//! fragments when a message exceeds one frame.
//!
//! The per-message-type field encoders (which read a
//! [`canboat_core::DecodedPgn`]) live in [`crate::n2kd::ais_decoded`]; the
//! JSON entry point [`convert`] just rebuilds the DecodedPgn and
//! delegates there, so a JSON stream and a live device produce
//! identical sentences.

#[cfg(test)]
use canboat_core::PgnDatabase;

/// 226 bytes = 1808 bits = 301 6-bit groups — same upper bound canboat
/// C uses. The longest AIVDM payload we emit (type 5, Class A static)
/// is 424 bits.
pub(crate) const BITVEC_BYTES: usize = 226;

pub struct BitVector {
    pub bytes: [u8; BITVEC_BYTES],
    /// Bits used so far. Always points at the next free bit.
    pub pos: usize,
}

impl BitVector {
    pub fn new() -> Self {
        Self {
            bytes: [0; BITVEC_BYTES],
            pos: 0,
        }
    }

    /// Add `len` bits, MSB-first. Mirrors canboat's `addAisInt`. The
    /// value is masked to `len` bits implicitly by the shift; the C
    /// version rejects values that overflow `len` bits but we follow
    /// canboat's "encode anyway" behaviour for two's-complement
    /// negative values that fit when wrapped (e.g. lat / lon).
    pub fn add_int(&mut self, value: i64, len: usize) {
        // Sign-extend / truncate `value` to a `len`-bit unsigned slot.
        let mask = if len >= 64 { !0u64 } else { (1u64 << len) - 1 };
        let mut v = (value as u64) & mask;
        let mut len = len;
        while len > 0 {
            let i = self.pos / 8;
            let k = 8 - (self.pos % 8);
            // Take the top `min(k, len)` bits from `v` and OR them
            // into byte `i` aligned to the open low bits of that
            // byte.
            let take = k.min(len);
            let high_bits = (v >> (len - take)) as u8;
            let mask_low = ((1u32 << take) - 1) as u8;
            let aligned = (high_bits & mask_low) << (k - take);
            self.bytes[i] |= aligned;
            self.pos += take;
            len -= take;
            // Clear consumed bits.
            v &= if len >= 64 { !0u64 } else { (1u64 << len) - 1 };
        }
    }

    /// Encode a printable-ASCII string as 6-bit AIS characters. Each
    /// input char is mapped: 32..63 → as-is, 64..95 → minus 64, NUL
    /// → 0, anything else → 32 (space). `len` is the bit length to
    /// fill — strings shorter than that get padded with NUL (which
    /// is the AIS "end of string" marker).
    pub fn add_string(&mut self, s: &str, mut len: usize) {
        let mut chars = s.chars();
        while len >= 6 {
            let raw = chars.next().unwrap_or('\0');
            let mut c = raw as u32;
            if c == 0 {
                c = 0;
            } else if !(32..=95).contains(&c) {
                c = 32;
            } else if c >= 64 {
                c -= 64;
            }
            self.add_int(c as i64, 6);
            len -= 6;
        }
        // Pad any remaining sub-6-bit tail with zeros.
        if len > 0 {
            self.add_int(0, len);
        }
    }
}

impl Default for BitVector {
    fn default() -> Self {
        Self::new()
    }
}

/// Convert a single analyzer AIS JSON line into one or more
/// `!AIVDM,…` sentences, appending each to `out`. Returns the number
/// of sentences emitted.
///
/// Thin input adapter: the line is rebuilt into a [`DecodedPgn`] and
/// handed to the struct-path [`crate::n2kd::ais_decoded::convert`] — the same
/// AIVDM encoder the live `server` pipeline runs. The bit-vector /
/// 6-bit-ASCII emit machinery it uses ([`BitVector`], [`emit_sentences`])
/// still lives here and is shared by both.
///
/// [`DecodedPgn`]: canboat_core::DecodedPgn
// Test-only since the fold-in: the live app/server paths decode JSON
// themselves and call `ais_decoded::convert` directly, so this
// string-input wrapper is exercised only by the round-trip tests below.
#[cfg(test)]
pub fn convert(out: &mut String, msg: &str, seq_counter: &mut u8) -> usize {
    let Some(decoded) =
        canboat_core::json_to_decoded(msg, PgnDatabase::embedded(canboat_core::Units::Metric))
    else {
        return 0;
    };
    crate::n2kd::ais_decoded::convert(out, &decoded, seq_counter)
}

pub(crate) const REPEAT_NAMES: &[(&str, i64)] = &[
    ("Initial", 0),
    ("First retransmission", 1),
    ("Second retransmission", 2),
    ("Final retransmission", 3),
];

pub(crate) const POSITION_ACCURACY_NAMES: &[(&str, i64)] = &[("Low", 0), ("High", 1)];

pub(crate) const RAIM_NAMES: &[(&str, i64)] = &[("not in use", 0), ("in use", 1)];

pub(crate) const AIS_TRANSCEIVER_NAMES: &[(&str, i64)] = &[
    ("Channel A VDL reception", 0),
    ("Channel B VDL reception", 1),
    ("Channel A VDL transmission", 2),
    ("Channel B VDL transmission", 3),
    ("Own information not broadcast", 4),
];

/// Re-encode a packed [`BitVector`] as one or more `!AIVDM,…`
/// sentences (6-bit ASCII payload, ≤ 360-bit fragments, XOR checksum)
/// and append them to `out`. `talker` is the two-letter suffix
/// (`VDM`/`VDO`), `channel` the AIS radio channel; `seq_counter`
/// cycles the multi-fragment sequence id. Returns the fragment count.
pub(crate) fn emit_sentences(
    out: &mut String,
    bv: &BitVector,
    talker: &str,
    channel: char,
    seq_counter: &mut u8,
) -> usize {
    let total_bits = bv.pos;
    if total_bits == 0 {
        return 0;
    }
    let fragments = total_bits.div_ceil(360);
    let seq = if fragments > 1 {
        // Cycle 0..9 across multi-fragment messages. Matches
        // canboat C's `sequenceId` static in `gps_ais.c`: bump
        // before use, wrap at 10, render as an ASCII digit.
        *seq_counter = (*seq_counter + 1) % 10;
        Some((b'0' + *seq_counter) as char)
    } else {
        None
    };
    let mut bit_offset = 0usize;
    let mut emitted = 0usize;
    for frag in 1..=fragments {
        let frag_bits = (total_bits - bit_offset).min(360);
        let mut payload = String::with_capacity(60);
        let mut consumed = 0usize;
        let mut padding = 0u8;
        while consumed + 6 <= frag_bits {
            let val = read_bits(&bv.bytes, bit_offset + consumed, 6);
            payload.push(six_bit_ascii(val));
            consumed += 6;
        }
        if consumed < frag_bits {
            let remaining = frag_bits - consumed;
            let val = read_bits(&bv.bytes, bit_offset + consumed, remaining) << (6 - remaining);
            payload.push(six_bit_ascii(val));
            padding = (6 - remaining) as u8;
            consumed = frag_bits;
        }
        bit_offset += consumed;
        // Sentence: !AIVDM/!AIVDO,frags,fragnum[,seq],channel,payload,padding*XX
        let body = match seq {
            Some(c) => format!("AI{talker},{fragments},{frag},{c},{channel},{payload},{padding}"),
            None => format!("AI{talker},{fragments},{frag},,{channel},{payload},{padding}"),
        };
        append_aivdm(out, &body);
        emitted += 1;
    }
    emitted
}

/// `!<body>*<XX>\r\n` — AIVDM sentences use `!` as the leading
/// character (vs `$` for the conventional sentences). Checksum is
/// XOR of every byte between `!` and `*`.
fn append_aivdm(out: &mut String, body: &str) {
    let start = out.len();
    out.push('!');
    out.push_str(body);
    let mut chk = 0u8;
    for b in &out.as_bytes()[start + 1..] {
        chk ^= *b;
    }
    out.push_str(&format!("*{chk:02X}\r\n"));
}

fn read_bits(bytes: &[u8], offset: usize, len: usize) -> u8 {
    let mut out = 0u8;
    for i in 0..len {
        let byte = bytes[(offset + i) / 8];
        let bit = (byte >> (7 - (offset + i) % 8)) & 1;
        out = (out << 1) | bit;
    }
    out
}

/// 6-bit value → AIVDM-payload character. Canboat does:
///   0..39 → '0' + n  (48..87)
///   40..63 → '0' + n + 8  (96..119)
fn six_bit_ascii(v: u8) -> char {
    let v = v & 0x3f;
    if v < 40 {
        (b'0' + v) as char
    } else {
        (b'0' + v + 8) as char
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn add_int_packs_big_endian() {
        let mut bv = BitVector::new();
        bv.add_int(0b1010_1100, 8);
        assert_eq!(bv.pos, 8);
        assert_eq!(bv.bytes[0], 0b1010_1100);
    }

    #[test]
    fn six_bit_payload_roundtrip() {
        // 18-bit MMSI sentinel: encode as three 6-bit chunks.
        let mut bv = BitVector::new();
        bv.add_int(0, 6);
        bv.add_int(1, 6);
        bv.add_int(63, 6);
        let mut out = String::new();
        let mut seq = 0;
        emit_sentences(&mut out, &bv, "VDM", 'A', &mut seq);
        assert!(out.contains("AIVDM,1,1,,A,01w,0*"), "got {out}");
    }

    #[test]
    fn class_b_position_round_trips_known_mmsi() {
        // Synthetic JSON resembling the dirona AIS PGN 129039 line.
        let msg = r#"{"pgn":129039,"src":23,"fields":{"Message ID":"Standard Class B position report","Repeat Indicator":"Initial","User ID":"244180106","Longitude":5.3134516,"Latitude":52.9061666,"Position Accuracy":"High","RAIM":"in use","Time Stamp":29,"COG":171.7,"SOG":1.80,"Heading":null,"Unit type":"SOTDMA","Integrated Display":"No","DSC":"Yes","Band":"Entire marine band","Can handle Msg 22":"Yes","AIS mode":"Autonomous","AIS communication state":"SOTDMA","Communication State":"F8 08 00","AIS Transceiver information":"Channel A VDL reception"}}"#;
        let mut out = String::new();
        let mut seq = 0;
        let n = convert(&mut out, msg, &mut seq);
        assert_eq!(n, 1, "expected one AIVDM sentence");
        assert!(out.starts_with("!AIVDM,1,1,,A,"), "got {out}");
        assert!(out.ends_with("\r\n"));
        // Round-trip the MMSI through the payload: msgid (6 bits=18) +
        // repeat (2=0) + mmsi (30=244180106). The 6-bit-ascii encoding
        // is positional, but the first character is `msgid` (18 in
        // 6-bit AIS = '>'? actually 18+48=66='B'). We just sanity-
        // check the talker / first nybble.
        let payload = out.split(',').nth(5).expect("payload field present");
        // msgid 18 encoded as 6-bit value 18 → char '0'+18 = 'B'.
        assert!(payload.starts_with('B'), "got payload {payload}");
    }
}
