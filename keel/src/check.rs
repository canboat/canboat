//! The rule engine - migration step 2 (DESIGN.md §5).
//!
//! Every invariant is a named rule with an ID, a location and a message, so
//! the editor, `keel check` and CI report identically. Two rule families are
//! enforced earlier as hard errors in derive::fill because nothing can be
//! computed without them: R03 (byte alignment feeds every length) and R10
//! (fieldtype overrule conflicts poison resolution/size/offset).

use crate::decode;
use crate::model::{ACTISENSE_BEM, Database, Expected, Pgn};
use crate::samples;

#[derive(Debug)]
pub struct Violation {
    pub rule: &'static str,
    pub error: bool, // false = warning
    pub location: String,
    pub message: String,
}

fn pgn_loc(prefix: &str, p: &Pgn) -> String {
    format!("{prefix}pgns/{:06}-{}.yaml", p.pgn, p.id)
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
    check_lookup_wiring(db, &mut v); // R08, R22 (references from BOTH trees)
    // The marine and J1939 lists are separate namespaces (both contain the
    // ISO PGNs), so variant and id uniqueness are checked per tree.
    for (prefix, list) in [("", &db.pgns), ("j1939/", &db.pgns_j1939)] {
        for pgn in list {
            check_pgn_range(prefix, pgn, &mut v); // R02
            check_frame_length(prefix, pgn, &mut v); // R04
            check_repeating(prefix, pgn, &mut v); // R05
            check_dynamic_length(prefix, db, pgn, &mut v); // R06
            check_proprietary(prefix, db, pgn, &mut v); // R07
            check_special_values(prefix, pgn, &mut v); // R09
            check_field_count(prefix, pgn, &mut v); // R11
            check_reserved_ids(prefix, pgn, &mut v); // R12
            check_match_values(prefix, db, pgn, &mut v); // R13
        }
        check_variants(prefix, list, &mut v); // R20
        check_unique_ids(prefix, list, &mut v); // R21
        for pgn in list.iter() {
            check_samples(prefix, db, pgn, prefix == "j1939/", &mut v); // R40
        }
    }
    v
}

// R40: every stored sample must decode against THIS variant and satisfy its
// (partial) expectations. Captures become regression tests (DESIGN.md §7.2).
fn check_samples(prefix: &str, db: &Database, p: &Pgn, j1939: bool, v: &mut Vec<Violation>) {
    for (si, spec) in p.samples.iter().enumerate() {
        let loc = || format!("{} sample {}", pgn_loc(prefix, p), si + 1);
        let mut fail = |msg: String| {
            v.push(Violation {
                rule: "R40",
                error: true,
                location: loc(),
                message: msg,
            })
        };

        let mut frames = Vec::new();
        let mut bad = false;
        for line in &spec.raw {
            match samples::parse_line(line) {
                Ok(f) => frames.push(f),
                Err(e) => {
                    fail(format!("cannot parse raw line: {e}"));
                    bad = true;
                }
            }
        }
        if bad || frames.is_empty() {
            continue;
        }
        let assembled = match samples::reassemble(&frames, |pgn| {
            (if j1939 { &db.pgns_j1939 } else { &db.pgns })
                .iter()
                .any(|q| q.pgn == pgn && q.type_ == "Fast")
        }) {
            Ok(a) => a,
            Err(e) => {
                fail(e);
                continue;
            }
        };
        if assembled.len() != 1 {
            fail(format!("expected exactly one message, got {}", assembled.len()));
            continue;
        }
        let a = &assembled[0];
        if a.pgn != p.pgn {
            fail(format!("sample is PGN {}, file defines {}", a.pgn, p.pgn));
            continue;
        }
        match decode::select_variant(db, a.pgn, &a.data, j1939) {
            Some(sel) if sel.id == p.id => {}
            Some(sel) => {
                fail(format!("sample selects variant '{}', not '{}'", sel.id, p.id));
                continue;
            }
            None => {
                fail("no variant matches the sample".into());
                continue;
            }
        }
        let decoded = match decode::decode(db, p, &a.data) {
            Ok(d) => d,
            Err(e) => {
                fail(format!("decode failed: {e}"));
                continue;
            }
        };
        for (key, expected) in &spec.expects {
            let (id, instance) = match key.rsplit_once('.') {
                Some((base, n)) if n.chars().all(|c| c.is_ascii_digit()) => {
                    (base, n.parse::<u32>().unwrap_or(1))
                }
                _ => (key.as_str(), 1),
            };
            let Some(d) = decoded.iter().find(|d| d.id == id && d.instance == instance) else {
                fail(format!("expected field '{key}' was not decoded"));
                continue;
            };
            if let Some(diff) = expect_mismatch(expected, &d.value) {
                fail(format!("field '{key}': {diff}"));
            }
        }
    }
}

/// None = match; Some(message) = value-level diff.
fn expect_mismatch(expected: &Expected, got: &decode::Value) -> Option<String> {
    use decode::Value as V;
    let ok = match (expected, got) {
        (Expected::Unavailable, V::Unavailable) => true,
        (Expected::Number(e), V::Number { value: g, decimals }) => {
            // Expectations are authored at the field's display precision
            // (a FLOAT_RAD shown as 2.7307 hides a full-precision f64), so
            // a nanoscale tolerance would reject every rounded value.
            // Allow one displayed unit-in-the-last-place; integer-precision
            // fields (decimals == 0) keep an effectively exact match.
            let slop = (e.abs() * 1e-9).max(1e-12);
            let tol = if *decimals > 0 {
                10f64.powi(-(*decimals as i32)) + slop
            } else {
                slop
            };
            (e - g).abs() <= tol
        }
        (Expected::Number(e), V::Lookup { value, .. }) => *e == *value as f64,
        (Expected::Str(e), V::Str(g)) => e == g,
        (Expected::Str(e), V::Lookup { name: Some(n), .. }) => e == n,
        (Expected::Str(e), V::Binary(h)) => e.trim_start_matches("0x") == h,
        (Expected::Str(e), V::OutOfRange) => e == "OutOfRange",
        (Expected::Str(e), V::Reserved) => e == "Reserved",
        (Expected::List(e), V::Bits(g)) => e == g,
        _ => false,
    };
    if ok {
        None
    } else {
        Some(format!("expected {expected:?}, decoded {got}"))
    }
}

// R02: PGN number in a valid range; PDU1 PGNs end in 0x00; packet type
// agrees with the range (pgn.c checkPgnList).
fn check_pgn_range(prefix: &str, p: &Pgn, v: &mut Vec<Violation>) {
    if p.pgn >= ACTISENSE_BEM {
        return; // BEM pseudo-PGNs live outside the wire ranges by design
    }
    let range = PGN_RANGES.iter().find(|(_, end, ..)| p.pgn <= *end);
    let Some((start, _end, step, _who, rtype)) = range else {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(prefix, p),
            message: format!("PGN {} is beyond every known PGN range", p.pgn),
        });
        return;
    };
    if p.pgn < *start {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(prefix, p),
            message: format!("PGN {} is not part of a valid PGN range", p.pgn),
        });
        return;
    }
    if *step == 256 && p.pgn & 0xff != 0 {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(prefix, p),
            message: format!("PGN {} (0x{:x}) is PDU1 and must end in 0x00", p.pgn, p.pgn),
        });
    }
    if !(*rtype == p.type_ || *rtype == "Mixed" || p.type_ == "ISO") {
        v.push(Violation {
            rule: "R02",
            error: true,
            location: pgn_loc(prefix, p),
            message: format!(
                "PGN {} is in a {rtype} range but has packet type {}",
                p.pgn, p.type_
            ),
        });
    }
}

// R04: fixed single-frame PGNs are exactly 8 bytes (ISO Request 59904
// excepted); a variable single-frame PGN's fixed part must fit the frame.
fn check_frame_length(prefix: &str, p: &Pgn, v: &mut Vec<Violation>) {
    if p.type_ != "Single" {
        return;
    }
    if !p.is_variable && p.length != 8 && p.pgn != 59904 {
        v.push(Violation {
            rule: "R04",
            error: true,
            location: pgn_loc(prefix, p),
            message: format!("single-frame PGN is {} bytes, must be exactly 8", p.length),
        });
    }
    if p.is_variable && p.length > 8 {
        v.push(Violation {
            rule: "R04",
            error: true,
            location: pgn_loc(prefix, p),
            message: format!("{} fixed bytes do not fit a single 8-byte frame", p.length),
        });
    }
}

// R05: repeating sets are self-consistent.
fn check_repeating(prefix: &str, p: &Pgn, v: &mut Vec<Violation>) {
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
                bad.push(format!(
                    "countField {cf} must reference a field before the set"
                ));
            }
        }
        for msg in bad {
            v.push(Violation {
                rule: "R05",
                error: true,
                location: pgn_loc(prefix, p),
                message: format!("repeating set {n}: {msg}"),
            });
        }
    }
}

// R06: DYNAMIC_FIELD_VALUE needs an earlier length source; overhead only on
// length fields; dynamicFieldLength only on DYNAMIC_FIELD_LENGTH fields.
fn check_dynamic_length(prefix: &str, db: &Database, p: &Pgn, v: &mut Vec<Violation>) {
    let mut have_length_source = false;
    for f in &p.fields {
        let root = &db.fieldtypes[f.ft].root_name;
        if f.dynamic_field_length && root != "DYNAMIC_FIELD_LENGTH" {
            v.push(Violation {
                rule: "R06",
                error: true,
                location: pgn_loc(prefix, p),
                message: format!("field '{}': dynamicFieldLength on a {root} field", f.id),
            });
        }
        if f.dynamic_field_length_overhead != 0 && !f.dynamic_field_length {
            v.push(Violation {
                rule: "R06",
                error: true,
                location: pgn_loc(prefix, p),
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
                location: pgn_loc(prefix, p),
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
fn check_proprietary(prefix: &str, db: &Database, p: &Pgn, v: &mut Vec<Violation>) {
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
            location: pgn_loc(prefix, p),
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

    for (prefix, list) in [("", &db.pgns), ("j1939/", &db.pgns_j1939)] {
        for p in list {
            for f in &p.fields {
                let Some((kind, name)) = f.lookup_ref() else {
                    continue;
                };
                match db.lookups.get(name) {
                    None => v.push(Violation {
                        rule: "R08",
                        error: true,
                        location: pgn_loc(prefix, p),
                        message: format!("field '{}': unknown lookup '{name}'", f.id),
                    }),
                    Some(lk) => {
                        referenced.insert(&lk.name);
                        if lk.kind != kind {
                            // name the YAML key the field should use instead
                            let want_key = match lk.kind.as_str() {
                                "bit" => "lookupBits",
                                "fieldtype" => "lookupFieldtype",
                                "triplet" => "lookupIndirect",
                                _ => "lookup",
                            };
                            let have_key = match kind {
                                "bit" => "lookupBits",
                                "fieldtype" => "lookupFieldtype",
                                "triplet" => "lookupIndirect",
                                _ => "lookup",
                            };
                            v.push(Violation {
                            rule: "R08",
                            error: true,
                            location: pgn_loc(prefix, p),
                            message: format!(
                                "field '{}': '{name}' is a {} enumeration but is referenced with '{have_key}:'; use '{want_key}:' instead",
                                f.id, lk.kind
                            ),
                        });
                        }
                        // Width policy: a width that differs from the lookup's
                        // declared width is an ERROR unless the field explicitly
                        // opts in with `allowLookupWidthMismatch: true` (shared
                        // enumerations, flag idioms - FINDINGS.md F2). A named
                        // *value* that cannot fit the field is always an error
                        // (dead table entry), except out-of-width *bit* positions
                        // in an opted-in shared bit enumeration.
                        let named_max = match lk.kind.as_str() {
                            "pair" | "bit" => lk.pairs.iter().map(|(v, _)| *v).max(),
                            "triplet" => lk.triplets.iter().map(|(_, v2, _)| *v2).max(),
                            _ => lk.fieldtypes.iter().map(|e| e.value).max(),
                        };
                        let mismatch = f.res_bits != lk.bits;
                        let unreachable = named_max.is_some_and(|named_max| {
                            if lk.kind == "bit" {
                                named_max >= f.res_bits as u64 // bit index beyond width
                            } else {
                                f.res_bits < 64 && named_max >= (1u64 << f.res_bits)
                            }
                        });
                        if unreachable && !(lk.kind == "bit" && f.allow_lookup_width_mismatch) {
                            v.push(Violation {
                            rule: "R08",
                            error: true,
                            location: pgn_loc(prefix, p),
                            message: format!(
                                "field '{}' is {} bits; lookup '{name}' names unreachable value {}",
                                f.id,
                                f.res_bits,
                                named_max.unwrap()
                            ),
                        });
                        } else if mismatch && !f.allow_lookup_width_mismatch {
                            v.push(Violation {
                                rule: "R08",
                                error: true,
                                location: pgn_loc(prefix, p),
                                message: format!(
                                    "field '{}' is {} bits but lookup '{name}' declares {}; \
                                 add allowLookupWidthMismatch: true if deliberate",
                                    f.id, f.res_bits, lk.bits
                                ),
                            });
                        } else if !mismatch && f.allow_lookup_width_mismatch {
                            v.push(Violation {
                            rule: "R08",
                            error: false,
                            location: pgn_loc(prefix, p),
                            message: format!(
                                "field '{}': unnecessary allowLookupWidthMismatch (widths agree)",
                                f.id
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
fn check_special_values(prefix: &str, p: &Pgn, v: &mut Vec<Violation>) {
    for f in &p.fields {
        if let Some(sv) = f.special_values {
            let max = if f.res_bits == 0 {
                0
            } else {
                (1u64 << f.res_bits.min(63)) - 1
            };
            if sv > 3 || (sv as u64) > max {
                v.push(Violation {
                    rule: "R09",
                    error: true,
                    location: pgn_loc(prefix, p),
                    message: format!(
                        "field '{}': specialValues {sv} out of range (0-3, < 2^bits)",
                        f.id
                    ),
                });
            }
        }
    }
}

// R13: a `match` discriminator is compared numerically against the field's
// unsigned extracted bits (decode::select_variant) and emitted as "=<number>"
// to the C tables and <Match> XML. It must resolve to a non-negative integer
// that fits the field's width; on a lookup field an enum NAME is allowed and
// resolved via model::resolve_match. An unresolvable value (non-numeric with no
// matching lookup name) or an oversized value would be emitted verbatim but
// read as 0 by the decoder, silently stopping the variant from matching its
// real frames.
fn check_match_values(prefix: &str, db: &Database, p: &Pgn, v: &mut Vec<Violation>) {
    for f in &p.fields {
        let Some(m) = &f.match_ else { continue };
        match db.resolve_match(f) {
            None => {
                let hint = if f.lookup_ref().is_some() {
                    "not an integer, and not a value name in its lookup"
                } else {
                    "not a non-negative integer"
                };
                v.push(Violation {
                    rule: "R13",
                    error: true,
                    location: pgn_loc(prefix, p),
                    message: format!("field '{}': match value {m:?} does not resolve ({hint})", f.id),
                });
            }
            Some(val) if f.res_bits < 64 => {
                let max = (1u64 << f.res_bits) - 1;
                if val > max {
                    v.push(Violation {
                        rule: "R13",
                        error: true,
                        location: pgn_loc(prefix, p),
                        message: format!(
                            "field '{}': match value {val} exceeds the field's {}-bit range (max {max})",
                            f.id, f.res_bits
                        ),
                    });
                }
            }
            Some(_) => {}
        }
    }
}

// R11: the C runtime's fieldList array holds 33 entries.
fn check_field_count(prefix: &str, p: &Pgn, v: &mut Vec<Violation>) {
    if p.fields.len() > 33 {
        v.push(Violation {
            rule: "R11",
            error: true,
            location: pgn_loc(prefix, p),
            message: format!("{} fields exceed the C runtime limit of 33", p.fields.len()),
        });
    }
}

// R12: Reserved/Spare fields follow the id convention.
fn check_reserved_ids(prefix: &str, p: &Pgn, v: &mut Vec<Violation>) {
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
                location: pgn_loc(prefix, p),
                message: format!(
                    "field '{}' named '{}' must have id '{want}[N]'",
                    f.id, f.name
                ),
            });
        }
    }
}

// R20: multiple variants of one PGN must be distinguishable: every variant
// carries match fields except at most one catch-all, and match sets differ.
fn check_variants(prefix: &str, pgns: &[Pgn], v: &mut Vec<Violation>) {
    let mut by_pgn: std::collections::BTreeMap<u32, Vec<&Pgn>> = Default::default();
    for p in pgns {
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
                // A later variant with an identical match set is unreachable
                // at runtime (getMatchingPgn returns the first match), so this
                // is an error. The inherited duplicates that once softened it
                // (Maretron 126720, BEP 130820, Navico 130817) were pruned in
                // the resync from master. See keel/FINDINGS.md F1.
                v.push(Violation {
                    rule: "R20",
                    error: true,
                    location: pgn_loc(prefix, p),
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
                location: format!("{prefix}pgns/{pgn:06}-*.yaml"),
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
fn check_unique_ids(prefix: &str, pgns: &[Pgn], v: &mut Vec<Violation>) {
    let mut pgn_ids: std::collections::HashMap<&str, u32> = Default::default();
    for p in pgns {
        if let Some(other) = pgn_ids.insert(&p.id, p.pgn) {
            v.push(Violation {
                rule: "R21",
                error: true,
                location: pgn_loc(prefix, p),
                message: format!("PGN id '{}' already used by PGN {other}", p.id),
            });
        }
        let mut field_ids: std::collections::HashSet<&str> = Default::default();
        for f in &p.fields {
            if !field_ids.insert(&f.id) {
                v.push(Violation {
                    rule: "R21",
                    error: true,
                    location: pgn_loc(prefix, p),
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
