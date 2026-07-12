//! Field decoder: assembled payload -> structured values, against keel's
//! own model. Powers the editor's live decode, `keel decode`, and the R40
//! sample-expectation tests. Independent of the C analyzer by design - the
//! two implementations can differential-test each other over samples/.

use crate::bits::extract_bits;
use crate::model::{Database, Field, Pgn};

type Result<T> = std::result::Result<T, String>;

#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    /// value with display precision derived from the field resolution
    /// (mirrors the C: resolution 0.0001 prints 4 decimals - the raw f64
    /// would show binary noise like 0.19190000000000002)
    Number { value: f64, decimals: u8 },
    Lookup { value: u64, name: Option<String> },
    Bits(Vec<String>),
    Str(String),
    Binary(String), // hex bytes
    Unavailable,
    OutOfRange,
    Reserved,
}

impl std::fmt::Display for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::Number { value, decimals } => write!(f, "{:.*}", *decimals as usize, value),
            Value::Lookup { value, name: Some(n) } => write!(f, "{n} ({value})"),
            Value::Lookup { value, name: None } => write!(f, "{value}"),
            Value::Bits(names) => write!(f, "[{}]", names.join(", ")),
            Value::Str(s) => write!(f, "\"{s}\""),
            Value::Binary(h) => write!(f, "0x{h}"),
            Value::Unavailable => write!(f, "Unavailable"),
            Value::OutOfRange => write!(f, "OutOfRange"),
            Value::Reserved => write!(f, "Reserved"),
        }
    }
}

#[derive(Debug, Clone)]
pub struct DecodedField {
    pub id: String,
    pub name: String,
    pub instance: u32, // 1-based, for repeating sets
    pub value: Value,
    /// Resolved SI unit of the value, if any (e.g. "rad", "m/s") - lets
    /// the editor annotate the live decode.
    pub unit: Option<String>,
    /// Root fieldtype when it wants a human rendering the raw number
    /// hides (DATE -> yyyy.mm.dd, TIME/DURATION -> hh:mm:ss).
    pub kind: Option<String>,
    /// Enumeration this field decodes against, when it is a simple pair
    /// or bit lookup that the editor could extend from evidence.
    pub lookup: Option<String>,
    /// "pair" or "bit" for `lookup` above.
    pub lookup_kind: Option<String>,
    /// Values (pair) or set bit positions (bit) seen here that are not
    /// named in the enumeration - i.e. evidence of missing entries.
    pub unnamed: Vec<u64>,
    /// Bit span actually consumed in the payload (variable fields differ
    /// from the declared width) - powers the editor's evidence grid.
    pub bit_offset: usize,
    pub bits: usize,
}

/// Pick the PGN variant whose match fields all match the payload
/// (port of pgn.c getMatchingPgn): first variant with every match field
/// equal wins; a variant without match fields is the catch-all.
pub fn select_variant<'a>(db: &'a Database, pgn: u32, data: &[u8], j1939: bool) -> Option<&'a Pgn> {
    let list = if j1939 { &db.pgns_j1939 } else { &db.pgns };
    for p in list.iter().filter(|p| p.pgn == pgn && !p.fallback) {
        let mut bit = 0usize;
        let mut ok = true;
        let mut has_match = false;
        for f in &p.fields {
            if f.match_.is_some() {
                has_match = true;
                let want: i64 = db.resolve_match(f).unwrap_or(0) as i64;
                match extract_bits(data, bit, f.res_bits as usize, false, 0) {
                    Some(e) if e.value == want => {}
                    _ => {
                        ok = false;
                        break;
                    }
                }
            }
            bit += f.res_bits as usize;
        }
        if !has_match || ok {
            return Some(p);
        }
    }
    list.iter().find(|p| p.pgn == pgn && p.fallback)
}

/// A variant that agrees with the payload on every match field but one -
/// i.e. the same message shape with a different discriminator. Lets the
/// editor offer "this looks like <id>, clone it and change <field>".
pub struct NearMiss<'a> {
    pub pgn: &'a Pgn,
    pub field_id: String,
    pub field_name: String,
    pub expected: i64,
    pub got: i64,
}

pub fn near_misses<'a>(db: &'a Database, pgn: u32, data: &[u8]) -> Vec<NearMiss<'a>> {
    let mut out = Vec::new();
    for p in db.pgns.iter().filter(|p| p.pgn == pgn && !p.fallback) {
        let mut bit = 0usize;
        let mut diffs: Vec<(String, String, i64, i64, bool)> = Vec::new();
        for f in &p.fields {
            if f.match_.is_some() {
                let want: i64 = db.resolve_match(f).unwrap_or(0) as i64;
                let got = extract_bits(data, bit, f.res_bits as usize, false, 0).map(|e| e.value);
                if got != Some(want) {
                    // Is this the proprietary preamble (vendor identity)? A
                    // differing manufacturer/industry means a different
                    // vendor's message, not the same shape - don't suggest.
                    let preamble = matches!(
                        f.lookup.as_deref(),
                        Some("MANUFACTURER_CODE") | Some("INDUSTRY_CODE")
                    );
                    diffs.push((f.id.clone(), f.name.clone(), want, got.unwrap_or(-1), preamble));
                }
            }
            bit += f.res_bits as usize;
        }
        // exactly one differing match field, and it isn't the vendor preamble
        if diffs.len() == 1 && !diffs[0].4 {
            let (field_id, field_name, expected, got, _) = diffs.into_iter().next().unwrap();
            out.push(NearMiss { pgn: p, field_id, field_name, expected, got });
        }
    }
    out
}

/// Decode state threaded through the field walk.
struct Ctx {
    bit: usize,
    /// numeric raw values by field order, for countField and indirect lookups
    by_order: std::collections::HashMap<u32, i64>,
    /// pending byte length for the next DYNAMIC_FIELD_VALUE (already
    /// overhead-corrected), set when a dynamicFieldLength field decodes
    dyn_len: Option<usize>,
    /// (fieldtype name, explicit bits, nested lookup) selected by the last
    /// DYNAMIC_FIELD_KEY, applied to the next DYNAMIC_FIELD_VALUE - the
    /// key-value idiom where the key names the value's type and length.
    pending_value: Option<(String, Option<u32>, Option<String>)>,
}

pub fn decode(db: &Database, pgn: &Pgn, data: &[u8]) -> Result<Vec<DecodedField>> {
    let mut out = Vec::new();
    let mut ctx = Ctx {
        bit: 0,
        by_order: Default::default(),
        dyn_len: None,
        pending_value: None,
    };

    let (rep_start, rep_count) = pgn
        .repeating1
        .as_ref()
        .map(|r| (r.start as usize, r.count as usize))
        .unwrap_or((usize::MAX, 0));
    let fixed_end = if rep_count > 0 { rep_start - 1 } else { pgn.fields.len() };

    for f in &pgn.fields[..fixed_end.min(pgn.fields.len())] {
        decode_one(db, f, data, &mut ctx, 1, &mut out)?;
    }

    if rep_count > 0 {
        let rep = pgn.repeating1.as_ref().unwrap();
        let n: Option<u64> = rep
            .count_field
            .and_then(|cf| ctx.by_order.get(&cf).copied())
            .map(|v| v.max(0) as u64);
        let set = &pgn.fields[rep_start - 1..rep_start - 1 + rep_count];
        let set_bits: usize = set.iter().map(|f| f.res_bits as usize).sum();
        let mut instance = 1u32;
        loop {
            match n {
                Some(n) if instance as u64 > n => break,
                None if set_bits == 0 || ctx.bit + set_bits > data.len() * 8 => break,
                _ => {}
            }
            if ctx.bit >= data.len() * 8 || instance > 1024 {
                break;
            }
            for f in set {
                decode_one(db, f, data, &mut ctx, instance, &mut out)?;
            }
            instance += 1;
        }
        // fields after the set (incl. a possible second set): decode once
        for f in &pgn.fields[rep_start - 1 + rep_count..] {
            if ctx.bit >= data.len() * 8 {
                break;
            }
            decode_one(db, f, data, &mut ctx, 1, &mut out)?;
        }
    }
    Ok(out)
}

fn decode_one(
    db: &Database,
    f: &Field,
    data: &[u8],
    ctx: &mut Ctx,
    instance: u32,
    out: &mut Vec<DecodedField>,
) -> Result<()> {
    let start_bit = ctx.bit;
    let ft = &db.fieldtypes[f.ft];
    let root = ft.root_name.as_str();
    let declared_bits = f.res_bits as usize;
    let remaining = (data.len() * 8).saturating_sub(ctx.bit);

    // Variable-length fields: DYNAMIC_FIELD_VALUE takes the pending
    // dynamicFieldLength (overhead already subtracted); other variable
    // fields consume the remainder of the payload.
    let bits = if declared_bits == 0 && !matches!(root, "STRING_LAU" | "STRING_LZ") {
        match (root, ctx.dyn_len.take()) {
            ("DYNAMIC_FIELD_VALUE", Some(nbytes)) => (nbytes * 8).min(remaining),
            _ => remaining,
        }
    } else {
        declared_bits
    };

    // Filled by the lookup arms so the caller can flag unnamed values
    // (evidence of missing enumeration entries) to the editor.
    let mut lookup_ref: Option<(String, String)> = None; // (name, kind)
    let mut unnamed: Vec<u64> = Vec::new();
    // Unit taken from a key-selected fieldtype (DYNAMIC_FIELD_VALUE).
    let mut unit_override: Option<String> = None;

    let value = match root {
        "DYNAMIC_FIELD_VALUE" => {
            match ctx.pending_value.take() {
                Some((ftname, ebits, nested)) => {
                    let vft = db.fieldtypes.iter().find(|t| t.name == ftname);
                    let vroot = vft.map(|t| t.root_name.as_str()).unwrap_or("BINARY");
                    let vbits = ebits
                        .map(|b| b as usize)
                        .or_else(|| vft.map(|t| t.size as usize).filter(|&s| s > 0))
                        .unwrap_or(remaining)
                        .min(remaining);
                    unit_override = vft.and_then(|t| t.unit.clone());
                    let val = if let Some(nl) = &nested {
                        // the key also names a nested pair enumeration for the value
                        match extract_bits(data, ctx.bit, vbits, false, 0) {
                            Some(e) => {
                                let nm = db.lookups.get(nl).and_then(|lk| {
                                    lk.pairs.iter().find(|(v, _)| *v == e.raw).map(|(_, n)| n.clone())
                                });
                                Value::Lookup { value: e.raw, name: nm }
                            }
                            None => Value::Unavailable,
                        }
                    } else if vroot == "FLOAT" {
                        match extract_bits(data, ctx.bit, vbits, false, 0) {
                            Some(e) => {
                                let fv = f32::from_bits(e.raw as u32);
                                if fv.is_nan() {
                                    Value::Unavailable
                                } else {
                                    Value::Number { value: fv as f64, decimals: 4 }
                                }
                            }
                            None => Value::Unavailable,
                        }
                    } else if vroot == "NUMBER" {
                        let signed = vft.and_then(|t| t.has_sign) == Some(true);
                        let off = vft.map(|t| t.offset as i64).unwrap_or(0);
                        let res = vft.map(|t| t.resolution).unwrap_or(0.0);
                        match extract_bits(data, ctx.bit, vbits, signed, off) {
                            Some(e) if res != 0.0 => Value::Number {
                                value: e.value as f64 * res,
                                decimals: decimals_for(res),
                            },
                            Some(e) => Value::Number { value: e.value as f64, decimals: 0 },
                            None => Value::Unavailable,
                        }
                    } else {
                        Value::Binary(hex(&slice_bits(data, ctx.bit, vbits)))
                    };
                    ctx.bit += vbits;
                    val
                }
                None => {
                    // no key type: fall back to length hint or the remainder
                    let v = slice_bits(data, ctx.bit, bits);
                    ctx.bit += bits;
                    Value::Binary(hex(&v))
                }
            }
        }
        "STRING_FIX" => {
            let v = slice_bytes(data, ctx.bit, bits / 8);
            ctx.bit += bits;
            Value::Str(trim_string_fix(&v))
        }
        "STRING_LZ" => {
            let len = byte_at(data, ctx.bit).unwrap_or(0) as usize;
            let v = slice_bytes(data, ctx.bit + 8, len);
            ctx.bit += 8 + (len + 1) * 8; // length byte + chars + terminating zero
            Value::Str(String::from_utf8_lossy(&v).into_owned())
        }
        "STRING_LAU" => {
            let total = byte_at(data, ctx.bit).unwrap_or(0) as usize;
            let n = total.saturating_sub(2); // count includes length + encoding bytes
            let v = slice_bytes(data, ctx.bit + 16, n);
            ctx.bit += total.max(2) * 8;
            Value::Str(String::from_utf8_lossy(&v).trim_end_matches('\0').to_string())
        }
        "BINARY" | "RESERVED" | "SPARE" | "VARIABLE" | "ISO_NAME" => {
            let v = slice_bits(data, ctx.bit, bits);
            ctx.bit += bits;
            Value::Binary(hex(&v))
        }
        "LOOKUP" | "INDIRECT_LOOKUP" | "DYNAMIC_FIELD_KEY" | "LOOKUP_TYPE_FIELDTYPE" => {
            let e = extract_bits(data, ctx.bit, bits, false, 0);
            ctx.bit += bits;
            match e {
                None => Value::Unavailable,
                Some(e) => {
                    ctx.by_order.insert(f.order, e.value);
                    match sentinel(f, ft.has_sign, e.raw, bits) {
                        Some(s) => s,
                        None => {
                            let name = lookup_name(db, f, e.raw, &ctx.by_order);
                            // Only pair enumerations can be extended in place;
                            // triplet/fieldtype/indirect need more context.
                            if let Some(("pair", n)) = f.lookup_ref() {
                                lookup_ref = Some((n.to_string(), "pair".into()));
                                if name.is_none() {
                                    unnamed.push(e.raw);
                                }
                            }
                            // key-value idiom: the key names the type + length of
                            // the following DYNAMIC_FIELD_VALUE.
                            if matches!(root, "DYNAMIC_FIELD_KEY" | "LOOKUP_TYPE_FIELDTYPE") {
                                ctx.pending_value = f
                                    .lookup_fieldtype
                                    .as_ref()
                                    .and_then(|n| db.lookups.get(n))
                                    .and_then(|lk| lk.fieldtypes.iter().find(|en| en.value == e.raw))
                                    .map(|en| (en.fieldtype.clone(), en.bits, en.lookup.clone()));
                                if let Some(("fieldtype", n)) = f.lookup_ref() {
                                    lookup_ref = Some((n.to_string(), "fieldtype".into()));
                                    if name.is_none() {
                                        unnamed.push(e.raw);
                                    }
                                }
                            }
                            Value::Lookup { value: e.raw, name }
                        }
                    }
                }
            }
        }
        "BITLOOKUP" => {
            let e = extract_bits(data, ctx.bit, bits, false, 0);
            ctx.bit += bits;
            match e {
                None => Value::Unavailable,
                Some(e) => {
                    if let Some(n) = &f.lookup_bits {
                        lookup_ref = Some((n.clone(), "bit".into()));
                    }
                    let named: std::collections::HashMap<u64, &str> = f
                        .lookup_bits
                        .as_ref()
                        .and_then(|n| db.lookups.get(n))
                        .map(|lk| lk.pairs.iter().map(|(b, n)| (*b, n.as_str())).collect())
                        .unwrap_or_default();
                    let mut names = Vec::new();
                    for b in 0..(bits as u64).min(64) {
                        if e.raw & (1u64 << b) != 0 {
                            match named.get(&b) {
                                Some(n) => names.push(n.to_string()),
                                None => {
                                    names.push(format!("bit {b}"));
                                    unnamed.push(b);
                                }
                            }
                        }
                    }
                    Value::Bits(names)
                }
            }
        }
        "DECIMAL" => {
            // Each byte holds a binary 0..99 (NOT BCD); concatenate as
            // two-digit groups, skipping bytes >= 100 (mirrors C
            // fieldPrintDecimal).
            let mut s = String::new();
            for i in 0..(bits / 8) {
                if let Some(e) = extract_bits(data, ctx.bit + i * 8, 8, false, 0) {
                    let b = e.raw as u8;
                    if b < 100 {
                        s.push_str(&format!("{b:02}"));
                    }
                }
            }
            ctx.bit += bits;
            Value::Str(s)
        }
        "FLOAT" => {
            // 32-bit IEEE-754, no resolution scaling; NaN = unavailable
            let e = extract_bits(data, ctx.bit, bits, false, 0);
            ctx.bit += bits;
            match e {
                Some(e) => {
                    let v = f32::from_bits(e.raw as u32);
                    if v.is_nan() {
                        Value::Unavailable
                    } else {
                        Value::Number { value: v as f64, decimals: 4 }
                    }
                }
                None => Value::Unavailable,
            }
        }
        _ => {
            // numeric family (NUMBER, TIME, DURATION, DATE, PGN, MMSI, ...)
            let signed = ft.has_sign == Some(true);
            let e = extract_bits(data, ctx.bit, bits, signed, f.res_offset as i64);
            ctx.bit += bits;
            match e {
                None => Value::Unavailable,
                Some(e) => {
                    ctx.by_order.insert(f.order, e.value);
                    if f.dynamic_field_length {
                        let overhead = f.dynamic_field_length_overhead as usize;
                        ctx.dyn_len = Some((e.value.max(0) as usize).saturating_sub(overhead));
                    }
                    match sentinel(f, ft.has_sign, e.raw, bits) {
                        Some(s) => s,
                        None if f.res_resolution != 0.0 => Value::Number {
                            value: e.value as f64 * f.res_resolution,
                            decimals: decimals_for(f.res_resolution),
                        },
                        None => Value::Number { value: e.value as f64, decimals: 0 },
                    }
                }
            }
        }
    };

    let unit = match &value {
        // a key-selected value carries its type's unit, not the field's
        Value::Number { .. } => unit_override.or_else(|| f.res_unit.clone()),
        _ => None,
    };
    let kind = match (&value, root) {
        (Value::Number { .. }, "DATE" | "TIME" | "DURATION") => Some(root.to_string()),
        _ => None,
    };
    let (lookup, lookup_kind) = match lookup_ref {
        Some((n, k)) => (Some(n), Some(k)),
        None => (None, None),
    };
    out.push(DecodedField {
        id: f.id.clone(),
        name: f.name.clone(),
        instance,
        value,
        unit,
        kind,
        lookup,
        lookup_kind,
        unnamed,
        bit_offset: start_bit,
        bits: ctx.bit - start_bit,
    });
    Ok(())
}

/// Top-of-range sentinel classification, mirroring the emitter/analyzer
/// (fieldtype.c reservedCount + explainPGNXML emission conditions).
/// Display decimals for a resolution, like the C printers: enough digits
/// to show the resolution's granularity, none for integer resolutions.
fn decimals_for(resolution: f64) -> u8 {
    if resolution >= 1.0 || resolution <= 0.0 {
        0
    } else {
        (-resolution.log10()).ceil().clamp(0.0, 9.0) as u8
    }
}

fn sentinel(f: &Field, has_sign: Option<bool>, raw: u64, bits: usize) -> Option<Value> {
    if f.reserved_count == 0 || bits == 0 || bits >= 64 || f.match_.is_some() {
        return None;
    }
    let highbit = if has_sign == Some(true) && f.res_offset == 0 { bits - 1 } else { bits };
    let top = (1u64 << highbit) - 1;
    if raw == top {
        Some(Value::Unavailable)
    } else if f.reserved_count >= 2 && raw == top - 1 {
        Some(Value::OutOfRange)
    } else if f.reserved_count >= 3 && raw == top - 2 {
        Some(Value::Reserved)
    } else {
        None
    }
}

fn lookup_name(
    db: &Database,
    f: &Field,
    raw: u64,
    by_order: &std::collections::HashMap<u32, i64>,
) -> Option<String> {
    let (kind, name) = f.lookup_ref()?;
    let lk = db.lookups.get(name)?;
    match kind {
        "pair" => lk.pairs.iter().find(|(v, _)| *v == raw).map(|(_, n)| n.clone()),
        "fieldtype" => lk
            .fieldtypes
            .iter()
            .find(|e| e.value == raw)
            .map(|e| e.name.clone()),
        "triplet" => {
            let v1 = f
                .lookup_indirect_order
                .and_then(|o| by_order.get(&o))
                .copied()
                .unwrap_or(-1);
            lk.triplets
                .iter()
                .find(|(a, b, _)| *a == v1.max(0) as u64 && *b == raw)
                .map(|(_, _, n)| n.clone())
        }
        _ => None,
    }
}

fn byte_at(data: &[u8], bit: usize) -> Option<u8> {
    if bit % 8 == 0 { data.get(bit / 8).copied() } else { None }
}

/// Bit-exact binary slice: extracts `bits` bits from `bit` (LSB-first per
/// byte, like every N2K field) into bytes. Needed because reserved/binary
/// fields frequently start at non-byte offsets.
fn slice_bits(data: &[u8], bit: usize, bits: usize) -> Vec<u8> {
    let mut out = Vec::with_capacity(bits.div_ceil(8));
    let mut taken = 0;
    while taken < bits {
        let chunk = (bits - taken).min(8);
        match extract_bits(data, bit + taken, chunk, false, 0) {
            Some(e) => out.push(e.raw as u8),
            None => break,
        }
        taken += chunk;
    }
    out
}

fn slice_bytes(data: &[u8], bit: usize, nbytes: usize) -> Vec<u8> {
    let start = bit / 8;
    if start >= data.len() {
        return Vec::new();
    }
    let end = (start + nbytes).min(data.len());
    data[start..end].to_vec()
}

fn trim_string_fix(v: &[u8]) -> String {
    let end = v
        .iter()
        .position(|&b| b == 0xff || b == b'@' || b == 0)
        .unwrap_or(v.len());
    String::from_utf8_lossy(&v[..end]).trim_end().to_string()
}

fn hex(v: &[u8]) -> String {
    v.iter().map(|b| format!("{b:02x}")).collect()
}
