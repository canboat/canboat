//! database/ tree loader.
//!
//! The YAML writer lives in the Python bootstrap until `keel fmt` lands; this
//! side only needs to *read* the canonical machine-written files (strict
//! subset of YAML, PyYAML-safe styles).

use std::fs;
use std::path::Path;

use yaml_rust2::{Yaml, YamlLoader};

use crate::model::{
    Database, Field, FieldType, FieldTypeLookupEntry, Interval, Lookup, Pgn, PhysicalQuantity,
    Repeating,
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

// ---- per-object loaders ---------------------------------------------------

fn physical_quantity(y: &Yaml, ctx: &str) -> Result<PhysicalQuantity> {
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
        sentinels: opt_str(y, "sentinels").unwrap_or_else(|| "None".into()),
        physical: opt_str(y, "physical"),
        print_function: opt_str(y, "print"),
        range_min_authored: opt_f64(y, "rangeMin"),
        range_max_authored: opt_f64(y, "rangeMax"),
        ..Default::default()
    })
}

fn lookup(y: &Yaml, ctx: &str) -> Result<Lookup> {
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
                let lookup_name = opt_str(e, "lookup");
                lk.fieldtypes.push(FieldTypeLookupEntry {
                    value: opt_i64(e, "value").unwrap_or(0) as u64,
                    name: req_str(e, "name", ctx)?,
                    fieldtype: req_str(e, "type", ctx)?,
                    bits: opt_i64(e, "bits").map(|b| b as u32),
                    lookup_kind: if lookup_name.is_some() {
                        Some(opt_str(e, "lookupKind").unwrap_or_else(|| "pair".into()))
                    } else {
                        None
                    },
                    lookup: lookup_name,
                });
            }
        }
        other => return Err(format!("{ctx}: unknown lookup kind '{other}'")),
    }
    Ok(lk)
}

fn field(y: &Yaml, ctx: &str) -> Result<Field> {
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
        dynamic_field_length: opt_bool(y, "dynamicFieldLength").unwrap_or(false) || overhead != 0,
        dynamic_field_length_overhead: overhead,
        range_min: opt_f64(y, "rangeMin"),
        range_max: opt_f64(y, "rangeMax"),
        ..Default::default()
    })
}

fn repeating(y: &Yaml) -> Repeating {
    Repeating {
        count: opt_i64(y, "count").unwrap_or(0) as u32,
        start: opt_i64(y, "start").unwrap_or(0) as u32,
        count_field: opt_i64(y, "countField").map(|c| c as u32),
    }
}

fn pgn(y: &Yaml, ctx: &str) -> Result<Pgn> {
    let interval = match get(y, "interval") {
        None => Interval::Unknown,
        Some(Yaml::Integer(ms)) => Interval::Ms(*ms as u32),
        Some(Yaml::String(s)) if s == "irregular" => Interval::Irregular,
        Some(other) => return Err(format!("{ctx}: bad interval {other:?}")),
    };
    let missing = match get(y, "missing") {
        Some(Yaml::Array(a)) => a
            .iter()
            .filter_map(|m| m.as_str().map(String::from))
            .collect(),
        _ => Vec::new(),
    };
    let fields = match get(y, "fields") {
        Some(Yaml::Array(a)) => a
            .iter()
            .map(|fy| field(fy, ctx))
            .collect::<Result<Vec<_>>>()?,
        _ => Vec::new(),
    };
    Ok(Pgn {
        pgn: opt_i64(y, "pgn").ok_or_else(|| format!("{ctx}: missing pgn"))? as u32,
        id: req_str(y, "id", ctx)?,
        description: req_str(y, "description", ctx)?,
        type_: req_str(y, "type", ctx)?,
        priority: opt_i64(y, "priority").unwrap_or(0) as u32,
        interval,
        explanation: opt_str(y, "explanation"),
        url: opt_str(y, "url"),
        research_doc: opt_str(y, "researchDoc"),
        fallback: opt_bool(y, "fallback").unwrap_or(false),
        missing,
        repeating1: get(y, "repeating1").map(repeating),
        repeating2: get(y, "repeating2").map(repeating),
        variant_order: opt_i64(y, "variantOrder").unwrap_or(0) as u32,
        fields,
        ..Default::default()
    })
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

    let order_doc = load_file(&db_dir.join("lookups.order.yaml"))?;
    if let Yaml::Hash(h) = &order_doc {
        for (k, v) in h {
            let kind = k.as_str().unwrap_or_default().to_string();
            let names = match v {
                Yaml::Array(a) => a
                    .iter()
                    .filter_map(|n| n.as_str().map(String::from))
                    .collect(),
                _ => Vec::new(),
            };
            db.lookup_order.insert(kind, names);
        }
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
