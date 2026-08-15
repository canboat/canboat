//! database/ tree loader.
//!
//! This side only *reads* the canonical files (a strict subset of YAML: block
//! style, no folded plain scalars). Whole-file canonical writing is not
//! implemented yet (`keel fmt`); the editor writes targeted block surgery
//! instead, matching this style byte-for-byte.

use std::fs;
use std::path::Path;

use yaml_rust2::{Yaml, YamlLoader};

use crate::model::{
    Database, Expected, Field, FieldType, FieldTypeLookupEntry, Interval, Lookup,
    MISSING_ATTRIBUTES, Pgn, PhysicalQuantity, Repeating, SampleSpec,
};

type Result<T> = std::result::Result<T, String>;

fn load_file(path: &Path) -> Result<Yaml> {
    let text = fs::read_to_string(path).map_err(|e| format!("{}: {e}", path.display()))?;
    let mut docs =
        YamlLoader::load_from_str(&text).map_err(|e| format!("{}: {e}", path.display()))?;
    if docs.len() != 1 {
        return Err(format!(
            "{}: expected exactly one YAML document",
            path.display()
        ));
    }
    Ok(docs.remove(0))
}

// ---- typed accessors ------------------------------------------------------

fn get<'a>(hash: &'a Yaml, key: &str) -> Option<&'a Yaml> {
    match hash {
        Yaml::Hash(h) => h.get(&Yaml::String(key.into())),
        _ => None,
    }
}

fn opt_str(hash: &Yaml, key: &str) -> Option<String> {
    get(hash, key)
        .and_then(|y| y.as_str())
        .map(|s| s.to_string())
}

fn req_str(hash: &Yaml, key: &str, ctx: &str) -> Result<String> {
    opt_str(hash, key).ok_or_else(|| format!("{ctx}: missing key '{key}'"))
}

fn opt_bool(hash: &Yaml, key: &str) -> Option<bool> {
    get(hash, key).and_then(|y| y.as_bool())
}

fn opt_i64(hash: &Yaml, key: &str) -> Option<i64> {
    get(hash, key).and_then(|y| y.as_i64())
}

/// Floats may appear as YAML integers, reals, or the PyYAML spellings
/// `.nan` / `.inf` / `-.inf` that Rust's f64 parser does not accept.
fn yaml_f64(y: &Yaml) -> Option<f64> {
    match y {
        Yaml::Integer(i) => Some(*i as f64),
        Yaml::Real(s) => match s.as_str() {
            ".nan" | ".NaN" => Some(f64::NAN),
            ".inf" | "+.inf" => Some(f64::INFINITY),
            "-.inf" => Some(f64::NEG_INFINITY),
            other => other.parse::<f64>().ok(),
        },
        _ => None,
    }
}

fn opt_f64(hash: &Yaml, key: &str) -> Option<f64> {
    get(hash, key).and_then(yaml_f64)
}

// ---- R01: allowed keys and enum membership --------------------------------
//
// Every key an authored object may carry, and every closed value set it may
// draw from. A key that is not listed here is silently dropped by the typed
// accessors above - `explanaton:` would parse fine and lose the text - and an
// out-of-set enum reaches an emitter that has no case for it. Both fail the
// load instead.

/// PGN documents (`database/pgns/`, `database/j1939/pgns/`).
///
/// `notes` is the per-PGN research annotation described in
/// `database/REFERENCES.md`: authored on purpose, read by humans, and
/// deliberately not carried into the model or any emitted artifact. It is
/// listed so the rule accepts it - not an oversight to be "fixed" by dropping
/// it from the files.
const PGN_KEYS: [&str; 16] = [
    "pgn",
    "id",
    "description",
    "type",
    "priority",
    "interval",
    "explanation",
    "url",
    "researchDoc",
    "fallback",
    "missing",
    "minLength",
    "variantOrder",
    "fields",
    "samples",
    "notes",
];

/// One entry of a PGN's `fields:` list. `repeat` is handled a level up, in
/// `fields_and_repeats`, so it is deliberately absent.
const FIELD_KEYS: [&str; 23] = [
    "id",
    "name",
    "type",
    "bits",
    "resolution",
    "unit",
    "offset",
    "description",
    "note",
    "match",
    "lookup",
    "lookupIndirect",
    "lookupBits",
    "lookupFieldtype",
    "primaryKey",
    "proprietary",
    "allowLookupWidthMismatch",
    "specialValues",
    "bitLengthField",
    "dynamicFieldLength",
    "dynamicFieldLengthOverhead",
    "rangeMin",
    "rangeMax",
];

const LOOKUP_INDIRECT_KEYS: [&str; 2] = ["name", "order"];
const SAMPLE_KEYS: [&str; 2] = ["raw", "expects"];
/// Lookup files. `note` and `valueNotes` are the lookup-level annotations from
/// `database/REFERENCES.md` - same status as a PGN's `notes`.
const LOOKUP_KEYS: [&str; 6] = ["name", "kind", "bits", "values", "note", "valueNotes"];

/// One entry of a `kind: fieldtype` lookup's `values:` list.
const FIELDTYPE_VALUE_KEYS: [&str; 6] = ["value", "name", "type", "bits", "lookup", "lookupKind"];

/// `database/fieldtypes.yaml` entries.
const FIELDTYPE_KEYS: [&str; 17] = [
    "name",
    "base",
    "description",
    "encodingDescription",
    "comment",
    "url",
    "bits",
    "variableSize",
    "unit",
    "offset",
    "resolution",
    "signed",
    "sentinels",
    "physical",
    "print",
    "rangeMin",
    "rangeMax",
];

/// `database/physicalquantities.yaml` entries.
const PHYSICAL_QUANTITY_KEYS: [&str; 6] = [
    "name",
    "description",
    "comment",
    "url",
    "unitDescription",
    "unit",
];

/// Packet types a PGN may declare (model::Pgn::type_).
const PACKET_TYPES: [&str; 4] = ["Single", "Fast", "ISO", "Mixed"];
/// Per-entry lookup kinds inside a `kind: fieldtype` lookup. The lookup's own
/// `kind:` needs no list here - the match in `lookup()` is already exhaustive.
const ENTRY_LOOKUP_KINDS: [&str; 2] = ["pair", "bit"];
/// Sentinel models a fieldtype may declare (emit_c::sentinel_symbol).
const SENTINELS: [&str; 5] = ["None", "TopOfRange", "NaN", "EmptyString", "Variable"];

/// Reject any key outside `allowed`, naming the closest legal spelling when
/// the key looks like a typo rather than an invention.
fn check_keys(y: &Yaml, allowed: &[&str], what: &str, ctx: &str) -> Result<()> {
    let Yaml::Hash(h) = y else {
        return Err(format!("{ctx}: {what} must be a map"));
    };
    for k in h.keys() {
        let Some(k) = k.as_str() else {
            return Err(format!("{ctx}: non-string key in {what}"));
        };
        if allowed.contains(&k) {
            continue;
        }
        return Err(match nearest(k, allowed) {
            Some(hint) => format!("{ctx}: unknown key '{k}' in {what} (did you mean '{hint}'?)"),
            None => format!(
                "{ctx}: unknown key '{k}' in {what}; allowed: {}",
                allowed.join(", ")
            ),
        });
    }
    Ok(())
}

/// Reject a value outside a closed set. The sets are small, so the message
/// simply lists them.
fn check_enum(value: &str, allowed: &[&str], what: &str, ctx: &str) -> Result<()> {
    if allowed.contains(&value) {
        return Ok(());
    }
    Err(format!(
        "{ctx}: {what} '{value}' is not one of: {}",
        allowed.join(", ")
    ))
}

/// The candidate within edit distance 3 (and closer than any other), or None
/// when nothing is near enough for a suggestion to help.
fn nearest<'a>(word: &str, candidates: &[&'a str]) -> Option<&'a str> {
    candidates
        .iter()
        .map(|c| (edit_distance(word, c), *c))
        .filter(|(d, _)| *d <= 3)
        .min_by_key(|(d, _)| *d)
        .map(|(_, c)| c)
}

/// Levenshtein distance, case-insensitive so `RangeMin` suggests `rangeMin`.
fn edit_distance(a: &str, b: &str) -> usize {
    let a: Vec<char> = a.to_lowercase().chars().collect();
    let b: Vec<char> = b.to_lowercase().chars().collect();
    let mut prev: Vec<usize> = (0..=b.len()).collect();
    let mut cur = vec![0usize; b.len() + 1];
    for i in 1..=a.len() {
        cur[0] = i;
        for j in 1..=b.len() {
            let sub = prev[j - 1] + usize::from(a[i - 1] != b[j - 1]);
            cur[j] = sub.min(prev[j] + 1).min(cur[j - 1] + 1);
        }
        std::mem::swap(&mut prev, &mut cur);
    }
    prev[b.len()]
}

// ---- per-object loaders ---------------------------------------------------

fn physical_quantity(y: &Yaml, ctx: &str) -> Result<PhysicalQuantity> {
    check_keys(y, &PHYSICAL_QUANTITY_KEYS, "physical quantity", ctx)?;
    Ok(PhysicalQuantity {
        name: req_str(y, "name", ctx)?,
        description: opt_str(y, "description"),
        comment: opt_str(y, "comment"),
        url: opt_str(y, "url"),
        unit_description: opt_str(y, "unitDescription"),
        unit: opt_str(y, "unit"),
    })
}

fn fieldtype(y: &Yaml, ctx: &str) -> Result<FieldType> {
    check_keys(y, &FIELDTYPE_KEYS, "fieldtype", ctx)?;
    let sentinels = opt_str(y, "sentinels").unwrap_or_else(|| "None".into());
    check_enum(&sentinels, &SENTINELS, "sentinels", ctx)?;
    Ok(FieldType {
        name: req_str(y, "name", ctx)?,
        base: opt_str(y, "base"),
        description: opt_str(y, "description"),
        encoding_description: opt_str(y, "encodingDescription"),
        comment: opt_str(y, "comment"),
        url: opt_str(y, "url"),
        size: opt_i64(y, "bits").unwrap_or(0) as u32,
        variable_size: opt_bool(y, "variableSize").unwrap_or(false),
        unit: opt_str(y, "unit"),
        offset: opt_i64(y, "offset").unwrap_or(0) as i32,
        resolution: opt_f64(y, "resolution").unwrap_or(0.0),
        has_sign: opt_bool(y, "signed"),
        sentinels,
        physical: opt_str(y, "physical"),
        print_function: opt_str(y, "print"),
        range_min_authored: opt_f64(y, "rangeMin"),
        range_max_authored: opt_f64(y, "rangeMax"),
        ..Default::default()
    })
}

fn lookup(y: &Yaml, ctx: &str) -> Result<Lookup> {
    check_keys(y, &LOOKUP_KEYS, "lookup", ctx)?;
    let mut lk = Lookup {
        name: req_str(y, "name", ctx)?,
        kind: req_str(y, "kind", ctx)?,
        bits: opt_i64(y, "bits").ok_or_else(|| format!("{ctx}: missing bits"))? as u32,
        ..Default::default()
    };
    let values = get(y, "values").ok_or_else(|| format!("{ctx}: missing values"))?;
    match lk.kind.as_str() {
        "pair" | "bit" => {
            let Yaml::Hash(h) = values else {
                return Err(format!("{ctx}: values must be a map"));
            };
            for (k, v) in h {
                let key = k
                    .as_i64()
                    .ok_or_else(|| format!("{ctx}: non-integer value key"))?
                    as u64;
                let name = v
                    .as_str()
                    .ok_or_else(|| format!("{ctx}: non-string value name"))?;
                lk.pairs.push((key, name.to_string()));
            }
        }
        "triplet" => {
            let Yaml::Array(a) = values else {
                return Err(format!("{ctx}: values must be a list"));
            };
            for row in a {
                let Yaml::Array(t) = row else {
                    return Err(format!("{ctx}: triplet row must be [v1, v2, name]"));
                };
                lk.triplets.push((
                    t[0].as_i64().unwrap_or(0) as u64,
                    t[1].as_i64().unwrap_or(0) as u64,
                    t[2].as_str().unwrap_or("").to_string(),
                ));
            }
        }
        "fieldtype" => {
            let Yaml::Array(a) = values else {
                return Err(format!("{ctx}: values must be a list"));
            };
            for e in a {
                check_keys(e, &FIELDTYPE_VALUE_KEYS, "fieldtype lookup value", ctx)?;
                let lookup_name = opt_str(e, "lookup");
                let lookup_kind = lookup_name
                    .as_ref()
                    .map(|_| opt_str(e, "lookupKind").unwrap_or_else(|| "pair".into()));
                if let Some(k) = &lookup_kind {
                    check_enum(k, &ENTRY_LOOKUP_KINDS, "lookupKind", ctx)?;
                }
                lk.fieldtypes.push(FieldTypeLookupEntry {
                    value: opt_i64(e, "value").unwrap_or(0) as u64,
                    name: req_str(e, "name", ctx)?,
                    fieldtype: req_str(e, "type", ctx)?,
                    bits: opt_i64(e, "bits").map(|b| b as u32),
                    lookup_kind,
                    lookup: lookup_name,
                });
            }
        }
        other => return Err(format!("{ctx}: unknown lookup kind '{other}'")),
    }
    Ok(lk)
}

fn field(y: &Yaml, ctx: &str) -> Result<Field> {
    check_keys(y, &FIELD_KEYS, "field", ctx)?;
    if let Some(li) = get(y, "lookupIndirect") {
        check_keys(li, &LOOKUP_INDIRECT_KEYS, "lookupIndirect", ctx)?;
    }
    // match may be an integer or a string; keep the scalar verbatim
    let match_ = get(y, "match").map(|m| match m {
        Yaml::Integer(i) => i.to_string(),
        Yaml::String(s) => s.clone(),
        other => format!("{other:?}"),
    });
    let li = get(y, "lookupIndirect");
    let overhead = opt_i64(y, "dynamicFieldLengthOverhead").unwrap_or(0) as u32;
    Ok(Field {
        id: req_str(y, "id", ctx)?,
        name: req_str(y, "name", ctx)?,
        type_: req_str(y, "type", ctx)?,
        bits: opt_i64(y, "bits").map(|b| b as u32),
        resolution: opt_f64(y, "resolution"),
        unit: opt_str(y, "unit"),
        offset: opt_i64(y, "offset").map(|o| o as i32),
        description: opt_str(y, "description"),
        note: opt_str(y, "note"),
        match_,
        lookup: opt_str(y, "lookup"),
        lookup_indirect: li.and_then(|l| opt_str(l, "name")),
        lookup_indirect_order: li.and_then(|l| opt_i64(l, "order")).map(|o| o as u32),
        lookup_bits: opt_str(y, "lookupBits"),
        lookup_fieldtype: opt_str(y, "lookupFieldtype"),
        primary_key: opt_bool(y, "primaryKey").unwrap_or(false),
        proprietary: opt_bool(y, "proprietary").unwrap_or(false),
        allow_lookup_width_mismatch: opt_bool(y, "allowLookupWidthMismatch").unwrap_or(false),
        special_values: opt_i64(y, "specialValues").map(|s| s as u32),
        bit_length_field: opt_str(y, "bitLengthField"),
        dynamic_field_length: opt_bool(y, "dynamicFieldLength").unwrap_or(false) || overhead != 0,
        dynamic_field_length_overhead: overhead,
        range_min: opt_f64(y, "rangeMin"),
        range_max: opt_f64(y, "rangeMax"),
        ..Default::default()
    })
}

fn sample(y: &Yaml, ctx: &str) -> Result<SampleSpec> {
    check_keys(y, &SAMPLE_KEYS, "sample", ctx)?;
    let raw = match get(y, "raw") {
        Some(Yaml::String(s)) => vec![s.clone()],
        Some(Yaml::Array(a)) => a
            .iter()
            .filter_map(|l| l.as_str().map(String::from))
            .collect(),
        _ => return Err(format!("{ctx}: sample needs raw (string or list)")),
    };
    let mut expects = Vec::new();
    if let Some(Yaml::Hash(h)) = get(y, "expects") {
        for (k, v) in h {
            let key = match k {
                Yaml::String(s) => s.clone(),
                other => format!("{other:?}"),
            };
            let e = match v {
                Yaml::Null => Expected::Unavailable,
                Yaml::Integer(i) => Expected::Number(*i as f64),
                Yaml::Real(_) => Expected::Number(yaml_f64(v).unwrap_or(f64::NAN)),
                Yaml::String(s) => Expected::Str(s.clone()),
                Yaml::Boolean(b) => Expected::Str(if *b { "Yes" } else { "No" }.into()),
                Yaml::Array(a) => Expected::List(
                    a.iter()
                        .filter_map(|x| x.as_str().map(String::from))
                        .collect(),
                ),
                other => return Err(format!("{ctx}: unsupported expected value {other:?}")),
            };
            expects.push((key, e));
        }
    }
    Ok(SampleSpec { raw, expects })
}

/// A `- repeat:` block as authored: `start` and `count` come from its position
/// and length in the field list, `countField` names a field by id.
struct RepeatBlock {
    start: u32,
    count: u32,
    count_field: Option<String>,
}

const REPEAT_KEYS: [&str; 2] = ["countField", "fields"];

/// Parse the value of a `repeat:` key, appending its fields to `fields`.
fn repeat_block(y: &Yaml, ctx: &str, fields: &mut Vec<Field>) -> Result<RepeatBlock> {
    let Yaml::Hash(h) = y else {
        return Err(format!("{ctx}: repeat must be a map"));
    };
    for k in h.keys() {
        match k.as_str() {
            Some(k) if REPEAT_KEYS.contains(&k) => {}
            Some(k) => return Err(format!("{ctx}: unknown key '{k}' in repeat block")),
            None => return Err(format!("{ctx}: non-string key in repeat block")),
        }
    }
    let start = fields.len() as u32 + 1;
    let Some(Yaml::Array(nested)) = get(y, "fields") else {
        return Err(format!("{ctx}: repeat block needs a fields list"));
    };
    if nested.is_empty() {
        return Err(format!("{ctx}: repeat block has no fields"));
    }
    for fy in nested {
        if get(fy, "repeat").is_some() {
            return Err(format!("{ctx}: repeat blocks cannot nest"));
        }
        fields.push(field(fy, ctx)?);
    }
    Ok(RepeatBlock {
        start,
        count: nested.len() as u32,
        count_field: opt_str(y, "countField"),
    })
}

/// Flatten the authored field list, lifting `- repeat:` blocks out into the
/// (at most two) repeating sets the emitters and the C runtime expect.
fn fields_and_repeats(y: &Yaml, ctx: &str) -> Result<(Vec<Field>, Vec<Repeating>)> {
    let mut fields = Vec::new();
    let mut blocks: Vec<RepeatBlock> = Vec::new();
    if let Some(Yaml::Array(items)) = get(y, "fields") {
        for item in items {
            match get(item, "repeat") {
                Some(rb) => {
                    if blocks.len() == 2 {
                        return Err(format!("{ctx}: at most two repeat blocks per PGN"));
                    }
                    blocks.push(repeat_block(rb, ctx, &mut fields)?);
                }
                None => fields.push(field(item, ctx)?),
            }
        }
    }
    let repeats = blocks
        .into_iter()
        .map(|b| {
            let count_field = match &b.count_field {
                None => None,
                Some(id) => Some(
                    fields
                        .iter()
                        .position(|f| &f.id == id)
                        .map(|i| i as u32 + 1)
                        .ok_or_else(|| format!("{ctx}: repeat countField '{id}' is not a field"))?,
                ),
            };
            Ok(Repeating {
                count: b.count,
                start: b.start,
                count_field,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok((fields, repeats))
}

fn pgn(y: &Yaml, ctx: &str) -> Result<Pgn> {
    check_keys(y, &PGN_KEYS, "pgn", ctx)?;
    let type_ = req_str(y, "type", ctx)?;
    check_enum(&type_, &PACKET_TYPES, "packet type", ctx)?;
    let interval = match get(y, "interval") {
        None => Interval::Unknown,
        Some(Yaml::Integer(ms)) => Interval::Ms(*ms as u32),
        Some(Yaml::String(s)) if s == "irregular" => Interval::Irregular,
        Some(other) => return Err(format!("{ctx}: bad interval {other:?}")),
    };
    let missing = match get(y, "missing") {
        None => Vec::new(),
        Some(Yaml::Array(a)) => a
            .iter()
            .map(|m| {
                let m = m
                    .as_str()
                    .ok_or_else(|| format!("{ctx}: non-string entry in missing"))?;
                check_enum(m, &MISSING_ATTRIBUTES, "missing attribute", ctx)?;
                Ok(m.to_string())
            })
            .collect::<Result<Vec<_>>>()?,
        Some(_) => return Err(format!("{ctx}: missing must be a list")),
    };
    let (fields, repeats) = fields_and_repeats(y, ctx)?;
    let mut repeats = repeats.into_iter();
    let samples = match get(y, "samples") {
        Some(Yaml::Array(a)) => a
            .iter()
            .map(|s| sample(s, ctx))
            .collect::<Result<Vec<_>>>()?,
        _ => Vec::new(),
    };
    Ok(Pgn {
        pgn: opt_i64(y, "pgn").ok_or_else(|| format!("{ctx}: missing pgn"))? as u32,
        id: req_str(y, "id", ctx)?,
        description: req_str(y, "description", ctx)?,
        type_,
        priority: opt_i64(y, "priority").unwrap_or(0) as u32,
        interval,
        explanation: opt_str(y, "explanation"),
        url: opt_str(y, "url"),
        research_doc: opt_str(y, "researchDoc"),
        fallback: opt_bool(y, "fallback").unwrap_or(false),
        missing,
        repeating1: repeats.next(),
        repeating2: repeats.next(),
        variant_order: opt_i64(y, "variantOrder").unwrap_or(0) as u32,
        min_length: opt_i64(y, "minLength").map(|n| n as u32),
        fields,
        samples,
        ..Default::default()
    })
}

/// Parse a single lookup definition from YAML text (editor candidates).
pub fn parse_lookup_str(text: &str, ctx: &str) -> Result<Lookup> {
    let mut docs = YamlLoader::load_from_str(text).map_err(|e| format!("{ctx}: {e}"))?;
    if docs.len() != 1 {
        return Err(format!("{ctx}: expected exactly one YAML document"));
    }
    lookup(&docs.remove(0), ctx)
}

/// Parse a single PGN definition from YAML text (the editor's candidate
/// documents - the same text that gets saved).
pub fn parse_pgn_str(text: &str, ctx: &str) -> Result<Pgn> {
    let mut docs = YamlLoader::load_from_str(text).map_err(|e| format!("{ctx}: {e}"))?;
    if docs.len() != 1 {
        return Err(format!("{ctx}: expected exactly one YAML document"));
    }
    pgn(&docs.remove(0), ctx)
}

// ---- tree loader ----------------------------------------------------------

fn sorted_yaml_files(dir: &Path) -> Result<Vec<std::path::PathBuf>> {
    let mut files: Vec<_> = fs::read_dir(dir)
        .map_err(|e| format!("{}: {e}", dir.display()))?
        .filter_map(|e| e.ok().map(|e| e.path()))
        .filter(|p| p.extension().is_some_and(|x| x == "yaml"))
        .collect();
    files.sort();
    Ok(files)
}

pub fn load_database(db_dir: &Path, version: &str, schema_version: &str) -> Result<Database> {
    let mut db = Database {
        version: version.to_string(),
        schema_version: schema_version.to_string(),
        ..Default::default()
    };

    let pq_doc = load_file(&db_dir.join("physicalquantities.yaml"))?;
    let Yaml::Array(pqs) = &pq_doc else {
        return Err("physicalquantities.yaml: expected a list".into());
    };
    for p in pqs {
        db.physical_quantities
            .push(physical_quantity(p, "physicalquantities.yaml")?);
    }

    let ft_doc = load_file(&db_dir.join("fieldtypes.yaml"))?;
    let Yaml::Array(fts) = &ft_doc else {
        return Err("fieldtypes.yaml: expected a list".into());
    };
    for f in fts {
        db.fieldtypes.push(fieldtype(f, "fieldtypes.yaml")?);
    }

    for path in sorted_yaml_files(&db_dir.join("lookups"))? {
        let doc = load_file(&path)?;
        let lk = lookup(&doc, &path.display().to_string())?;
        db.lookups.insert(lk.name.clone(), lk);
    }

    for path in sorted_yaml_files(&db_dir.join("pgns"))? {
        let doc = load_file(&path)?;
        db.pgns.push(pgn(&doc, &path.display().to_string())?);
    }
    // Emission order: PGN ascending, then authored variant order (which
    // includes fallback entries - their in-group position is semantic).
    db.pgns.sort_by_key(|p| (p.pgn, p.variant_order));

    let j1939_dir = db_dir.join("j1939/pgns");
    if j1939_dir.is_dir() {
        for path in sorted_yaml_files(&j1939_dir)? {
            let doc = load_file(&path)?;
            db.pgns_j1939.push(pgn(&doc, &path.display().to_string())?);
        }
        db.pgns_j1939.sort_by_key(|p| (p.pgn, p.variant_order));
    }

    Ok(db)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A minimal well-formed PGN document; each test perturbs one thing.
    const PGN_YAML: &str = "\
pgn: 127751
id: dcVoltageCurrent
description: DC Voltage/Current
type: Single
missing:
- SampleData
fields:
- id: sid
  name: SID
  type: UINT8
";

    fn parse(text: &str) -> Result<Pgn> {
        parse_pgn_str(text, "test.yaml")
    }

    #[test]
    fn accepts_a_well_formed_pgn() {
        let p = parse(PGN_YAML).unwrap();
        assert_eq!(p.type_, "Single");
        assert_eq!(p.missing, vec!["SampleData"]);
    }

    // The case that motivated the rule: `Plain` is not a packet type, but it
    // used to load fine and only blow up half-way through generation.
    #[test]
    fn rejects_an_unknown_packet_type() {
        let err = parse(&PGN_YAML.replace("type: Single", "type: Plain")).unwrap_err();
        assert!(err.contains("packet type 'Plain'"), "{err}");
        assert!(err.contains("Single, Fast, ISO, Mixed"), "{err}");
    }

    #[test]
    fn accepts_every_packet_type() {
        for t in PACKET_TYPES {
            let text = PGN_YAML.replace("type: Single", &format!("type: {t}"));
            assert!(parse(&text).is_ok(), "{t} should load");
        }
    }

    #[test]
    fn rejects_an_unknown_pgn_key_and_suggests_the_near_miss() {
        let err = parse(&format!("{PGN_YAML}explanaton: oops\n")).unwrap_err();
        assert!(err.contains("unknown key 'explanaton' in pgn"), "{err}");
        assert!(err.contains("did you mean 'explanation'?"), "{err}");
    }

    #[test]
    fn lists_the_allowed_keys_when_nothing_is_close() {
        let err = parse(&format!("{PGN_YAML}wibble: oops\n")).unwrap_err();
        assert!(err.contains("allowed: pgn, id, description"), "{err}");
    }

    #[test]
    fn rejects_an_unknown_field_key() {
        let err = parse(&PGN_YAML.replace("  name: SID", "  nme: SID")).unwrap_err();
        assert!(err.contains("unknown key 'nme' in field"), "{err}");
        assert!(err.contains("did you mean 'name'?"), "{err}");
    }

    // `notes:` (PGN) and `valueNotes:` (lookup) are authored annotations that
    // the loader accepts and drops on purpose - see PGN_KEYS.
    #[test]
    fn accepts_the_documented_annotation_keys() {
        parse(&format!("{PGN_YAML}notes: seen on a Maretron CLMD12\n")).unwrap();
    }

    #[test]
    fn rejects_an_unknown_missing_attribute() {
        let err = parse(&PGN_YAML.replace("- SampleData", "- SampleDate")).unwrap_err();
        assert!(err.contains("missing attribute 'SampleDate'"), "{err}");
    }

    // An emptied-out `missing:` leaves a bare key behind; that is a shape
    // error, not an empty list.
    #[test]
    fn rejects_a_missing_list_that_is_not_a_list() {
        for bad in ["missing:\n", "missing: SampleData\n"] {
            let text = PGN_YAML.replace("missing:\n- SampleData\n", bad);
            let err = parse(&text).unwrap_err();
            assert!(err.contains("missing must be a list"), "{bad}: {err}");
        }
    }

    #[test]
    fn absent_missing_key_is_fine() {
        let text = PGN_YAML.replace("missing:\n- SampleData\n", "");
        assert!(parse(&text).unwrap().missing.is_empty());
    }

    #[test]
    fn suggests_only_for_near_misses() {
        assert_eq!(nearest("explanaton", &PGN_KEYS), Some("explanation"));
        assert_eq!(nearest("Description", &PGN_KEYS), Some("description"));
        assert_eq!(nearest("wibble", &PGN_KEYS), None);
    }

    #[test]
    fn edit_distance_counts_edits() {
        assert_eq!(edit_distance("name", "name"), 0);
        assert_eq!(edit_distance("nme", "name"), 1);
        assert_eq!(edit_distance("kitten", "sitting"), 3);
    }
}
