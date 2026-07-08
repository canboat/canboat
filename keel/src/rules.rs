//! The rule inventory — the single source of truth for what `keel check`
//! guarantees. Every invariant enforced anywhere in the loader, the derive
//! pass, `check.rs`, or CI has an entry here, so `keel rules` can print the
//! current set and DESIGN.md need not duplicate (and drift from) it.
//!
//! `check.rs` tags each `Violation` with the same `id`; keep the two in step
//! (the `ids_referenced_by_check` test guards it).

#[derive(Clone, Copy, PartialEq)]
pub enum Scope {
    Shape,
    IntraPgn,
    CrossFile,
    CrossRelease,
    Sample,
}

impl Scope {
    fn title(self) -> &'static str {
        match self {
            Scope::Shape => "Shape (schema over the YAML)",
            Scope::IntraPgn => "Intra-PGN semantics",
            Scope::CrossFile => "Cross-file",
            Scope::CrossRelease => "Cross-release (CI only)",
            Scope::Sample => "Samples",
        }
    }
    /// Emission order.
    const ORDER: [Scope; 5] = [
        Scope::Shape,
        Scope::IntraPgn,
        Scope::CrossFile,
        Scope::Sample,
        Scope::CrossRelease,
    ];
}

pub struct Rule {
    pub id: &'static str,
    pub scope: Scope,
    /// Human severity, incl. the context-dependent cases spelled out.
    pub severity: &'static str,
    /// Where the check actually lives (module::fn, "loader", "derive", CI).
    pub enforced_in: &'static str,
    /// One line.
    pub title: &'static str,
    /// The full explanation.
    pub detail: &'static str,
}

pub const RULES: &[Rule] = &[
    Rule {
        id: "R01",
        scope: Scope::Shape,
        severity: "error",
        enforced_in: "loader (yamlio)",
        title: "Well-formed objects: required and allowed keys, value types, enum membership.",
        detail: "Enforced while parsing YAML into the model: an unknown key, a \
                 wrong value type, or an out-of-set enum value fails the load \
                 before any semantic rule runs.",
    },
    Rule {
        id: "R02",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_pgn_range",
        title: "PGN number lies in a valid range; PDU1 PGNs end in 0x00; type agrees with the range.",
        detail: "The PGN falls inside a known pgnRange. PDU1 (addressable, PF < \
                 240) PGNs have a zero low byte. The declared packet type is \
                 consistent with the range's kind. BEM pseudo-PGNs live outside \
                 the wire ranges by design and are exempt.",
    },
    Rule {
        id: "R03",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "derive",
        title: "Fixed fields up to the first repeating field sum to whole bytes.",
        detail: "Every downstream length depends on a byte-aligned fixed \
                 prefix, so this is a hard error raised in the derive pass \
                 rather than a soft check.",
    },
    Rule {
        id: "R04",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_frame_length",
        title: "Single-frame length sanity (fixed = 8 bytes, variable <= 8).",
        detail: "Fixed single-frame PGNs are exactly 8 bytes (ISO Request 59904 \
                 excepted); variable single-frame PGNs are at most 8 bytes. The \
                 packet type must agree with the range's type, allowing the \
                 ISO-TP / mixed exceptions.",
    },
    Rule {
        id: "R05",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_repeating",
        title: "Repeating sets are self-consistent.",
        detail: "start and count stay within the field list; countField, when \
                 present, references an integer field before the set; its \
                 absence means repeat-until-data-exhausted.",
    },
    Rule {
        id: "R06",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_dynamic_length",
        title: "Dynamic-length fields have an earlier length source.",
        detail: "A DYNAMIC_FIELD_VALUE field is preceded by the length field it \
                 depends on, and any dynamicFieldLength overhead is within a \
                 sane bound.",
    },
    Rule {
        id: "R07",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_proprietary",
        title: "Proprietary PGNs open with the manufacturer preamble.",
        detail: "Manufacturer-proprietary-range PGNs start with Manufacturer \
                 Code / reserved / Industry Code, unless they explicitly carry \
                 missing: [MissingCompanyFields].",
    },
    Rule {
        id: "R08",
        scope: Scope::IntraPgn,
        severity: "error (stale opt-in: warning)",
        enforced_in: "check::check_lookup_wiring",
        title: "Lookup references resolve with the right kind and a compatible width.",
        detail: "Every lookup reference resolves to an enumeration of the \
                 matching kind (pair/bit/triplet/fieldtype). A field width that \
                 differs from the lookup's declared width is an error unless the \
                 field opts in with allowLookupWidthMismatch: true; a named \
                 value that cannot fit the field is an error even then (except \
                 out-of-width bit positions in an opted-in shared bit \
                 enumeration). An opt-in on a field whose widths agree is a \
                 stale-opt-in warning. See FINDINGS.md F2.",
    },
    Rule {
        id: "R09",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_special_values",
        title: "Sentinel / special-value logic stays within the reserved model.",
        detail: "specialValues overrides land inside the field range; a lookup \
                 that names values in the sentinel region reduces the reserved \
                 count accordingly; no sentinels on match fields or 64-bit \
                 fields.",
    },
    Rule {
        id: "R10",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "derive",
        title: "Fieldtype constraints don't contradict the base type.",
        detail: "resolution / offset / unit may not contradict the base type, \
                 and a unit implies a known physical quantity. A hard error in \
                 derive because it poisons every resolved attribute.",
    },
    Rule {
        id: "R11",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_field_count",
        title: "At most 33 fields per PGN.",
        detail: "Matches the C runtime's fixed fieldList array size. Raiseable, \
                 but enforced to keep the generated tables in step with the \
                 analyzer.",
    },
    Rule {
        id: "R12",
        scope: Scope::IntraPgn,
        severity: "error",
        enforced_in: "check::check_reserved_ids",
        title: "Reserved / Spare fields follow the id convention.",
        detail: "Fields of type RESERVED / SPARE use the reserved / spare id \
                 suffix convention so generated code and consumers can identify \
                 them structurally.",
    },
    Rule {
        id: "R20",
        scope: Scope::CrossFile,
        severity: "warning (temporary; error once FINDINGS F1 is resolved)",
        enforced_in: "check::check_variants",
        title: "PGN variants are distinguishable; at most one catch-all per PGN.",
        detail: "Each (pgn, match-set) combination is unique — a later variant \
                 with an identical match set is unreachable at runtime — and a \
                 PGN number has at most one match-less catch-all. Softened to a \
                 warning while two inherited duplicates remain (FINDINGS.md F1); \
                 re-hardened to an error once main's fix flows back.",
    },
    Rule {
        id: "R21",
        scope: Scope::CrossFile,
        severity: "error",
        enforced_in: "check::check_unique_ids",
        title: "Ids are unique per scope.",
        detail: "PGN ids are unique globally; field ids are unique within a PGN.",
    },
    Rule {
        id: "R22",
        scope: Scope::CrossFile,
        severity: "error (unreferenced enumeration: warning)",
        enforced_in: "check::check_lookup_wiring",
        title: "Every lookup reference resolves; every enumeration is referenced.",
        detail: "A reference to a missing enumeration is an error; an \
                 enumeration that no field references is a (dead-data) warning. \
                 See FINDINGS.md F3.",
    },
    Rule {
        id: "R23",
        scope: Scope::CrossFile,
        severity: "error",
        enforced_in: "check::check_fieldtypes",
        title: "The fieldtype hierarchy is a DAG rooted in printable types.",
        detail: "Every fieldtype resolves through its base chain to a root that \
                 has a print function; no cycles.",
    },
    Rule {
        id: "R30",
        scope: Scope::CrossRelease,
        severity: "classification (CI)",
        enforced_in: "tools/contract.py (CI)",
        title: "Contract classification against the previous release.",
        detail: "Compares the database against the previous release and \
                 classifies changes (frozen ids, wire-encoding changes, \
                 additions). Run in CI, not by keel check.",
    },
    Rule {
        id: "R40",
        scope: Scope::Sample,
        severity: "error",
        enforced_in: "check::check_samples",
        title: "Stored samples decode against their variant and meet their expectations.",
        detail: "Each captured sample reassembles to exactly one message of the \
                 file's PGN, selects this variant, decodes, and matches every \
                 asserted field in its (partial) expects block. Captures become \
                 permanent regression tests; a later change to the layout, a \
                 lookup, or a fieldtype that alters any expected decode fails \
                 here with a value-level diff.",
    },
];

/// Human-readable inventory, grouped by scope.
pub fn render_text() -> String {
    let mut out = String::from(
        "keel rule inventory — the invariants keel check enforces.\n\
         Source of truth: keel/src/rules.rs (this list is generated).\n",
    );
    for scope in Scope::ORDER {
        let group: Vec<&Rule> = RULES.iter().filter(|r| r.scope == scope).collect();
        if group.is_empty() {
            continue;
        }
        out.push_str(&format!("\n{}\n", scope.title()));
        for r in group {
            out.push_str(&format!("  {}  [{}]  {}\n", r.id, r.severity, r.enforced_in));
            out.push_str(&format!("      {}\n", r.title));
            out.push_str(&format!("      {}\n", wrap(r.detail, 72, "      ")));
        }
    }
    out
}

/// Markdown, e.g. for docs.
pub fn render_md() -> String {
    let mut out = String::from(
        "# keel rules\n\n\
         The invariants `keel check` enforces. Generated by `keel rules` from \
         `keel/src/rules.rs` — the canonical source.\n",
    );
    for scope in Scope::ORDER {
        let group: Vec<&Rule> = RULES.iter().filter(|r| r.scope == scope).collect();
        if group.is_empty() {
            continue;
        }
        out.push_str(&format!("\n## {}\n\n", scope.title()));
        for r in group {
            out.push_str(&format!(
                "- **{}** ({}, _{}_) — {} {}\n",
                r.id,
                r.severity,
                r.enforced_in,
                r.title,
                r.detail,
            ));
        }
    }
    out
}

/// Re-flow a paragraph to `width`, indenting continuation lines with `indent`.
fn wrap(text: &str, width: usize, indent: &str) -> String {
    let mut out = String::new();
    let mut col = 0usize;
    for (i, word) in text.split_whitespace().enumerate() {
        if i != 0 && col + 1 + word.len() > width {
            out.push('\n');
            out.push_str(indent);
            col = 0;
        } else if i != 0 {
            out.push(' ');
            col += 1;
        }
        out.push_str(word);
        col += word.len();
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_unique_sorted_and_populated() {
        let mut prev = "";
        for r in RULES {
            assert!(r.id.len() == 3 && r.id.starts_with('R'), "bad id {}", r.id);
            assert!(r.id > prev, "ids must be strictly ascending: {} after {prev}", r.id);
            assert!(!r.title.is_empty() && !r.detail.is_empty(), "{} needs text", r.id);
            assert!(!r.enforced_in.is_empty() && !r.severity.is_empty());
            prev = r.id;
        }
    }

    // The ids check.rs actually tags Violations with must all be documented
    // here (R01/R03/R10 live in the loader/derive, R30 in CI, so they are
    // documented but never emitted by check.rs — that direction is fine).
    #[test]
    fn ids_referenced_by_check_are_documented() {
        let src = include_str!("check.rs");
        let documented: std::collections::HashSet<&str> = RULES.iter().map(|r| r.id).collect();
        let mut i = 0;
        while let Some(p) = src[i..].find("rule: \"R") {
            let start = i + p + "rule: \"".len();
            let id = &src[start..start + 3];
            assert!(documented.contains(id), "check.rs emits {id} but rules.rs has no entry");
            i = start;
        }
    }
}
