//! Field decoder: assembled payload -> structured values, against keel's
//! own model. Powers the editor's live decode, `keel decode`, and the R40
//! sample-expectation tests. Independent of the C analyzer by design - the
//! two implementations can differential-test each other over samples/.

use crate::bits::extract_bits;
use crate::model::{Database, Field, Pgn};

type Result<T> = std::result::Result<T, String>;

#[derive(Debug, Clone, PartialEq)]
pub enum Value {
    Number(f64),
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
            Value::Number(n) => write!(f, "{n}"),
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
            if let Some(m) = &f.match_ {
                has_match = true;
                let want: i64 = m.parse().unwrap_or(0);
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

/// Decode state threaded through the field walk.
struct Ctx {
    bit: usize,
    /// numeric raw values by field order, for countField and indirect lookups
    by_order: std::collections::HashMap<u32, i64>,
    /// pending byte length for the next DYNAMIC_FIELD_VALUE (already
    /// overhead-corrected), set when a dynamicFieldLength field decodes
    dyn_len: Option<usize>,
}

pub fn decode(db: &Database, pgn: &Pgn, data: &[u8]) -> Result<Vec<DecodedField>> {
    let mut out = Vec::new();
    let mut ctx = Ctx {
        bit: 0,
        by_order: Default::default(),
        dyn_len: None,
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

    let value = match root {
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
        "BINARY" | "RESERVED" | "SPARE" | "VARIABLE" | "DYNAMIC_FIELD_VALUE" | "ISO_NAME" => {
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
                        None => Value::Lookup {
                            value: e.raw,
                            name: lookup_name(db, f, e.raw, &ctx.by_order),
                        },
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
                    let mut names = Vec::new();
                    if let Some(lk) = f.lookup_bits.as_ref().and_then(|n| db.lookups.get(n)) {
                        for (b, name) in &lk.pairs {
                            if *b < 64 && e.raw & (1u64 << b) != 0 {
                                names.push(name.clone());
                            }
                        }
                    }
                    Value::Bits(names)
                }
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
                        None if f.res_resolution != 0.0 => {
                            Value::Number(e.value as f64 * f.res_resolution)
                        }
                        None => Value::Number(e.value as f64),
                    }
                }
            }
        }
    };

    out.push(DecodedField {
        id: f.id.clone(),
        name: f.name.clone(),
        instance,
        value,
        bit_offset: start_bit,
        bits: ctx.bit - start_bit,
    });
    Ok(())
}

/// Top-of-range sentinel classification, mirroring the emitter/analyzer
/// (fieldtype.c reservedCount + explainPGNXML emission conditions).
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
