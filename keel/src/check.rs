//! The rule engine - migration step 2 (DESIGN.md §5).
//!
//! Every invariant is a named rule with an ID, a location and a message, so
//! the editor, `keel check` and CI report identically. Two rule families are
//! enforced earlier as hard errors in derive::fill because nothing can be
//! computed without them: R03 (byte alignment feeds every length) and R10
//! (fieldtype overrule conflicts poison resolution/size/offset).

use crate::model::{ACTISENSE_BEM, Database, Pgn};

#[derive(Debug)]
pub struct Violation {
    pub rule: &'static str,
    pub error: bool, // false = warning
    pub location: String,
    pub message: String,
}

fn pgn_loc(p: &Pgn) -> String {
    format!("pgns/{:06}-{}.yaml", p.pgn, p.id)
}

/// pgn.h pgnRange[]: (start, end, step, who, type)
const PGN_RANGES: [(u32, u32, u32, &str, &str); 8] = [
    (0xe800, 0xee00, 256, "ISO 11783", "Single"),
    (0xef00, 0xef00, 256, "NMEA", "Single"),
    (0xf000, 0xfeff, 1, "NMEA", "Single"),
    (0xff00, 0xffff, 1, "Manufacturer", "Single"),
    (0x1ed00, 0x1ee00, 256, "NMEA", "Fast"),
    (0x1ef00, 0x1ef00, 256, "Manufacturer", "Fast"),
    (0x1f000, 0x1feff, 1, "NMEA", "Mixed"),
    (0x1ff00, 0x1ffff, 1, "Manufacturer", "Fast"),
];

/// pgn.h IS_MANUFACTURER_PGN()
fn is_manufacturer_pgn(pgn: u32) -> bool {
    (0xff00..=0xffff).contains(&pgn) || pgn == 0x1ef00 || (0x1ff00..=0x1ffff).contains(&pgn)
}

pub fn check(db: &Database) -> Vec<Violation> {
    let mut v = Vec::new();

    check_fieldtypes(db, &mut v); // R23
    check_lookup_wiring(db, &mut v); // R08, R22
    for pgn in &db.pgns {
        check_pgn_range(pgn, &mut v); // R02
        check_frame_length(pgn, &mut v); // R04
        check_repeating(pgn, &mut v); // R05
        check_dynamic_length(db, pgn, &mut v); // R06
        check_proprietary(db, pgn, &mut v); // R07
        check_special_values(pgn, &mut v); // R09
        check_field_count(pgn, &mut v); // R11
        check_reserved_ids(pgn, &mut v); // R12
    }
    check_variants(db, &mut v); // R20
    check_unique_ids(db, &mut v); // R21
    v
}

// R02: PGN number in a valid range; PDU1 PGNs end in 0x00; packet type
// agrees with the range (pgn.c checkPgnList).
fn check_pgn_range(p: &Pgn, v: &mut Vec<Violation>) {
    if p.pgn >= ACTISENSE_BEM {
        return; // BEM pseudo-PGNs live outside the wire ranges by design
    }
    let range = PGN_RANGES.iter().find(|(_, end, ..)| p.pgn <= *end);
    let Some((start, _end, step, _who, rtype)) = range else {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(p),
            message: format!("PGN {} is beyond every known PGN range", p.pgn),
        });
        return;
    };
    if p.pgn < *start {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(p),
            message: format!("PGN {} is not part of a valid PGN range", p.pgn),
        });
        return;
    }
    if *step == 256 && p.pgn & 0xff != 0 {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(p),
            message: format!("PGN {} (0x{:x}) is PDU1 and must end in 0x00", p.pgn, p.pgn),
        });
    }
    if !(*rtype == p.type_ || *rtype == "Mixed" || p.type_ == "ISO") {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(p),
            message: format!(
                "PGN {} is in a {rtype} range but has packet type {}",
                p.pgn, p.type_
            ),
        });
    }
}

// R04: fixed single-frame PGNs are exactly 8 bytes (ISO Request 59904
// excepted); a variable single-frame PGN's fixed part must fit the frame.
fn check_frame_length(p: &Pgn, v: &mut Vec<Violation>) {
    if p.type_ != "Single" {
        return;
    }
    if !p.is_variable && p.length != 8 && p.pgn != 59904 {
        v.push(Violation {
            rule: "R04",
            error: true,
            location: pgn_loc(p),
            message: format!("single-frame PGN is {} bytes, must be exactly 8", p.length),
        });
    }
    if p.is_variable && p.length > 8 {
        v.push(Violation {
            rule: "R04",
            error: true,
            location: pgn_loc(p),
            message: format!("{} fixed bytes do not fit a single 8-byte frame", p.length),
        });
    }
}

// R05: repeating sets are self-consistent.
fn check_repeating(p: &Pgn, v: &mut Vec<Violation>) {
    let nfields = p.fields.len() as u32;
    for (n, rep) in [(1, &p.repeating1), (2, &p.repeating2)] {
        let Some(rep) = rep else { continue };
        if rep.count == 0 {
            continue;
        }
        let mut bad = Vec::new();
        if rep.start == 0 {
            bad.push("no way to determine the repeating set (start missing)".to_string());
        } else if rep.start - 1 + rep.count > nfields {
            bad.push(format!(
                "fields {}..{} exceed the {} defined fields",
                rep.start,
                rep.start + rep.count - 1,
                nfields
            ));
        }
        if let Some(cf) = rep.count_field {
            if cf == 0 || cf >= rep.start {
                bad.push(format!("countField {cf} must reference a field before the set"));
            }
        }
        for msg in bad {
            v.push(Violation {
                rule: "R05",
                error: true,
                location: pgn_loc(p),
                message: format!("repeating set {n}: {msg}"),
            });
        }
    }
}

// R06: DYNAMIC_FIELD_VALUE needs an earlier length source; overhead only on
// length fields; dynamicFieldLength only on DYNAMIC_FIELD_LENGTH fields.
fn check_dynamic_length(db: &Database, p: &Pgn, v: &mut Vec<Violation>) {
    let mut have_length_source = false;
    for f in &p.fields {
        let root = &db.fieldtypes[f.ft].root_name;
        if f.dynamic_field_length && root != "DYNAMIC_FIELD_LENGTH" {
            v.push(Violation {
                rule: "R06",
                error: true,
                location: pgn_loc(p),
                message: format!("field '{}': dynamicFieldLength on a {root} field", f.id),
            });
        }
        if f.dynamic_field_length_overhead != 0 && !f.dynamic_field_length {
            v.push(Violation {
                rule: "R06",
                error: true,
                location: pgn_loc(p),
                message: format!("field '{}': overhead without dynamicFieldLength", f.id),
            });
        }
        if f.dynamic_field_length || root == "DYNAMIC_FIELD_KEY" {
            have_length_source = true;
        }
        if root == "DYNAMIC_FIELD_VALUE" && !have_length_source {
            v.push(Violation {
                rule: "R06",
                error: true,
                location: pgn_loc(p),
                message: format!(
                    "field '{}': DYNAMIC_FIELD_VALUE without an earlier DYNAMIC_FIELD_LENGTH or DYNAMIC_FIELD_KEY",
                    f.id
                ),
            });
        }
    }
}

// R07: manufacturer-proprietary PGNs open with Manufacturer Code / reserved /
// Industry Code, or explicitly declare missing: [MissingCompanyFields].
fn check_proprietary(db: &Database, p: &Pgn, v: &mut Vec<Violation>) {
    if !is_manufacturer_pgn(p.pgn) || p.fallback {
        return;
    }
    if p.missing.iter().any(|m| m == "MissingCompanyFields") {
        return;
    }
    let ok = p.fields.len() >= 3
        && p.fields[0].lookup.as_deref() == Some("MANUFACTURER_CODE")
        && p.fields[0].res_bits == 11
        && db.fieldtypes[p.fields[1].ft].root_name == "RESERVED"
        && p.fields[1].res_bits == 2
        && p.fields[2].lookup.as_deref() == Some("INDUSTRY_CODE")
        && p.fields[2].res_bits == 3;
    if !ok {
        v.push(Violation {
            rule: "R07",
            error: true,
            location: pgn_loc(p),
            message: "proprietary PGN must start with Manufacturer Code (11 bits) / \
                      Reserved (2) / Industry Code (3), or declare missing: [MissingCompanyFields]"
                .into(),
        });
    }
}

// R08 + R22: lookup wiring. Every reference resolves with the right kind and
// bit width; every defined lookup is referenced somewhere (warning).
fn check_lookup_wiring(db: &Database, v: &mut Vec<Violation>) {
    let mut referenced: std::collections::HashSet<&str> = std::collections::HashSet::new();

    for p in &db.pgns {
        for f in &p.fields {
            let Some((kind, name)) = f.lookup_ref() else { continue };
            match db.lookups.get(name) {
                None => v.push(Violation {
                    rule: "R08",
                    error: true,
                    location: pgn_loc(p),
                    message: format!("field '{}': unknown lookup '{name}'", f.id),
                }),
                Some(lk) => {
                    referenced.insert(&lk.name);
                    if lk.kind != kind {
                        v.push(Violation {
                            rule: "R08",
                            error: true,
                            location: pgn_loc(p),
                            message: format!(
                                "field '{}': lookup '{name}' is a {} enumeration, referenced as {kind}",
                                f.id, lk.kind
                            ),
                        });
                    }
                    // Width policy: a named value that cannot fit the field
                    // is unreachable - error. A mere width disagreement is
                    // inherited pgn.h laxity (the C never checked; YES_NO is
                    // declared 2 bits and idiomatically used in 1-bit flags)
                    // - warning, candidate for cleanup.
                    let named_max = match lk.kind.as_str() {
                        "pair" => lk.pairs.iter().map(|(v, _)| *v).max(),
                        "bit" => lk.pairs.iter().map(|(v, _)| *v).max(),
                        "triplet" => lk.triplets.iter().map(|(_, v2, _)| *v2).max(),
                        _ => lk.fieldtypes.iter().map(|e| e.value).max(),
                    };
                    if let Some(named_max) = named_max {
                        let reachable = if lk.kind == "bit" {
                            named_max < f.res_bits as u64 // bit index within width
                        } else {
                            f.res_bits >= 64 || named_max < (1u64 << f.res_bits)
                        };
                        if !reachable {
                            // Bit enumerations are legitimately shared across
                            // fields of different widths (DISABLED_SATELLITES:
                            // 40 positions over 32/24-bit constellations); an
                            // out-of-width *bit* simply never occurs. An
                            // out-of-range *value* is a dead table entry.
                            v.push(Violation {
                                rule: "R08",
                                error: lk.kind != "bit",
                                location: pgn_loc(p),
                                message: format!(
                                    "field '{}' is {} bits; lookup '{name}' names unreachable value {named_max}",
                                    f.id, f.res_bits
                                ),
                            });
                        } else if f.res_bits != lk.bits {
                            v.push(Violation {
                                rule: "R08",
                                error: false,
                                location: pgn_loc(p),
                                message: format!(
                                    "field '{}' is {} bits but lookup '{name}' declares {}",
                                    f.id, f.res_bits, lk.bits
                                ),
                            });
                        }
                    }
                }
            }
        }
    }

    // Nested references inside fieldtype enumerations
    for lk in db.lookups.values() {
        for e in &lk.fieldtypes {
            if db.fieldtype(&e.fieldtype).is_err() {
                v.push(Violation {
                    rule: "R08",
                    error: true,
                    location: format!("lookups/{}.yaml", lk.name),
                    message: format!("value {}: unknown fieldtype '{}'", e.value, e.fieldtype),
                });
            }
            if let Some(nested) = &e.lookup {
                match db.lookups.get(nested) {
                    None => v.push(Violation {
                        rule: "R08",
                        error: true,
                        location: format!("lookups/{}.yaml", lk.name),
                        message: format!("value {}: unknown nested lookup '{nested}'", e.value),
                    }),
                    Some(n) => {
                        referenced.insert(&n.name);
                    }
                }
            }
        }
    }

    let mut unreferenced: Vec<&String> = db
        .lookups
        .keys()
        .filter(|n| !referenced.contains(n.as_str()))
        .collect();
    unreferenced.sort();
    for name in unreferenced {
        v.push(Violation {
            rule: "R22",
            error: false,
            location: format!("lookups/{name}.yaml"),
            message: "lookup enumeration is never referenced by any field".into(),
        });
    }
}

// R09: SPECIAL_VALUES overrides stay within the sentinel model.
fn check_special_values(p: &Pgn, v: &mut Vec<Violation>) {
    for f in &p.fields {
        if let Some(sv) = f.special_values {
            let max = if f.res_bits == 0 { 0 } else { (1u64 << f.res_bits.min(63)) - 1 };
            if sv > 3 || (sv as u64) > max {
                v.push(Violation {
                    rule: "R09",
                    error: true,
                    location: pgn_loc(p),
                    message: format!("field '{}': specialValues {sv} out of range (0-3, < 2^bits)", f.id),
                });
            }
        }
    }
}

// R11: the C runtime's fieldList array holds 33 entries.
fn check_field_count(p: &Pgn, v: &mut Vec<Violation>) {
    if p.fields.len() > 33 {
        v.push(Violation {
            rule: "R11",
            error: true,
            location: pgn_loc(p),
            message: format!("{} fields exceed the C runtime limit of 33", p.fields.len()),
        });
    }
}

// R12: Reserved/Spare fields follow the id convention.
fn check_reserved_ids(p: &Pgn, v: &mut Vec<Violation>) {
    for f in &p.fields {
        let want = match f.name.as_str() {
            "Reserved" => "reserved",
            "Spare" => "spare",
            _ => continue,
        };
        let rest = f.id.strip_prefix(want);
        let ok = matches!(rest, Some(r) if r.is_empty() || r.chars().all(|c| c.is_ascii_digit()));
        if !ok {
            v.push(Violation {
                rule: "R12",
                error: true,
                location: pgn_loc(p),
                message: format!("field '{}' named '{}' must have id '{want}[N]'", f.id, f.name),
            });
        }
    }
}

// R20: multiple variants of one PGN must be distinguishable: every variant
// carries match fields except at most one catch-all, and match sets differ.
fn check_variants(db: &Database, v: &mut Vec<Violation>) {
    let mut by_pgn: std::collections::BTreeMap<u32, Vec<&Pgn>> = Default::default();
    for p in &db.pgns {
        if !p.fallback {
            by_pgn.entry(p.pgn).or_default().push(p);
        }
    }
    for (pgn, group) in by_pgn {
        if group.len() < 2 {
            continue;
        }
        let mut matchless = Vec::new();
        let mut seen_sets: std::collections::HashMap<String, &str> = Default::default();
        for p in &group {
            let set: Vec<String> = p
                .fields
                .iter()
                .enumerate()
                .filter_map(|(i, f)| f.match_.as_ref().map(|m| format!("{i}={m}")))
                .collect();
            if set.is_empty() {
                matchless.push(p.id.as_str());
                continue;
            }
            let key = set.join(",");
            if let Some(other) = seen_sets.insert(key.clone(), &p.id) {
                // Warning (not error) for now: two inherited pgn.h cases
                // exist (Maretron 126720, BEP 130820) where the later
                // variant is unreachable at runtime. Re-harden to error
                // once those are resolved. See keel/FINDINGS.md.
                v.push(Violation {
                    rule: "R20",
                    error: false,
                    location: pgn_loc(p),
                    message: format!(
                        "match set [{key}] duplicates variant '{other}' of PGN {pgn}; the later variant is unreachable"
                    ),
                });
            }
        }
        if matchless.len() > 1 {
            v.push(Violation {
                rule: "R20",
                error: true,
                location: format!("pgns/{pgn:06}-*.yaml"),
                message: format!(
                    "PGN {pgn} has {} variants without match fields ({}); at most one catch-all is decodable",
                    matchless.len(),
                    matchless.join(", ")
                ),
            });
        }
    }
}

// R21: ids are unique - PGN ids globally, field ids within a PGN.
fn check_unique_ids(db: &Database, v: &mut Vec<Violation>) {
    let mut pgn_ids: std::collections::HashMap<&str, u32> = Default::default();
    for p in &db.pgns {
        if let Some(other) = pgn_ids.insert(&p.id, p.pgn) {
            v.push(Violation {
                rule: "R21",
                error: true,
                location: pgn_loc(p),
                message: format!("PGN id '{}' already used by PGN {other}", p.id),
            });
        }
        let mut field_ids: std::collections::HashSet<&str> = Default::default();
        for f in &p.fields {
            if !field_ids.insert(&f.id) {
                v.push(Violation {
                    rule: "R21",
                    error: true,
                    location: pgn_loc(p),
                    message: format!("duplicate field id '{}'", f.id),
                });
            }
        }
    }
}

// R23: the fieldtype hierarchy resolves and every root can print.
fn check_fieldtypes(db: &Database, v: &mut Vec<Violation>) {
    for ft in &db.fieldtypes {
        if ft.base.is_none() && ft.print_function.is_none() {
            v.push(Violation {
                rule: "R23",
                error: true,
                location: "fieldtypes.yaml".into(),
                message: format!("root fieldtype '{}' has no print function", ft.name),
            });
        }
    }
}
