// (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

//! The database under `database/` is the source of truth, so `cargo test`
//! holds it to the same contract CI does: `keel check` must be clean, and
//! every committed generated artifact must still be what the emitters
//! produce from it.
//!
//! These run the real loader over the real tree — the same call path
//! `keel check` and `keel generate --check` drive — so an edit that breaks
//! an invariant, or a regenerate that was never committed, fails here
//! rather than in CI.

use std::path::{Path, PathBuf};

use keel::model::{Database, FieldType};
use keel::{check, derive, emit_c, emit_rust, emit_text, emit_xml, read_versions, yamlio};

/// The repo root — `keel/`'s parent, resolved from the manifest dir so the
/// test does not depend on the working directory.
fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .expect("keel/ has a parent")
        .to_path_buf()
}

/// Load and derive the database exactly as `main.rs` does, also handing back
/// the authored (pre-percolation) fieldtypes that
/// `fieldtype-generated-data.h` is rendered from.
fn load() -> (PathBuf, Database, Vec<FieldType>) {
    let root = repo_root();
    let (version, schema) = read_versions(&root).expect("common/version.h is readable");
    let mut db =
        yamlio::load_database(&root.join("database"), &version, &schema).expect("database/ loads");
    let authored_fieldtypes = db.fieldtypes.clone();
    derive::fill(&mut db).expect("derive pass succeeds");
    (root, db, authored_fieldtypes)
}

/// The committed database satisfies every rule `keel check` enforces.
/// Warnings are allowed (they are advisory); errors are not.
#[test]
fn check_reports_no_errors() {
    let (_, db, _) = load();
    let violations = check::check(&db);
    let errors: Vec<String> = violations
        .iter()
        .filter(|v| v.error)
        .map(|v| format!("{} {}: {}", v.rule, v.location, v.message))
        .collect();
    assert!(
        errors.is_empty(),
        "keel check found {} error(s):\n{}",
        errors.len(),
        errors.join("\n")
    );
}

/// The loader produces a plausible database rather than an empty one — a
/// guard against `check` passing vacuously if the tree failed to load.
#[test]
fn the_database_is_populated() {
    let (_, db, _) = load();
    assert!(db.pgns.len() > 500, "got {} pgns", db.pgns.len());
    assert!(db.lookups.len() > 200, "got {} lookups", db.lookups.len());
    assert!(
        db.fieldtypes.len() > 100,
        "got {} fieldtypes",
        db.fieldtypes.len()
    );
}

/// Every committed generated artifact is still byte-identical to what the
/// emitters produce. This is `keel generate --check` as a cargo test: the
/// C tables, canboat.xml and the Rust schema all go stale silently
/// otherwise, and the runtime compiles the committed copy.
///
/// Not on Windows: the runner checks the tree out with CRLF, so every
/// committed artifact differs from the emitters' LF output in a way that
/// says nothing about whether it is stale. `rust-ci.yml` skips the golden
/// fixtures for the same reason. The bytes this guards are the ones in
/// the repository, which is what the other platforms see.
#[test]
#[cfg_attr(
    windows,
    ignore = "the Windows runner checks out CRLF; the byte-exact diff is meaningless there"
)]
fn generated_artifacts_are_up_to_date() {
    let (root, db, authored_fieldtypes) = load();
    let artifacts: Vec<(&str, String)> = vec![
        ("docs/canboat.xml", emit_xml::emit_xml(&db, false)),
        (
            "analyzer/lookup-generated-data.h",
            emit_c::emit_lookup_h(&db, false),
        ),
        (
            "analyzer/lookup-j1939-generated-data.h",
            emit_c::emit_lookup_h(&db, true),
        ),
        (
            "analyzer/physicalquantity-generated-data.h",
            emit_c::emit_physicalquantity_data_h(&db),
        ),
        (
            "analyzer/fieldtype-generated-data.h",
            emit_c::emit_fieldtype_data_h(&authored_fieldtypes),
        ),
        (
            "analyzer/pgn-generated-data.h",
            emit_c::emit_pgn_data_h(&db, false),
        ),
        (
            "analyzer/pgn-j1939-generated-data.h",
            emit_c::emit_pgn_data_h(&db, true),
        ),
        (
            "crates/canboat/src/engine/schema_generated.rs",
            emit_rust::emit_schema(&db, &root, false),
        ),
        (
            "crates/canboat/src/engine/schema_generated_j1939.rs",
            emit_rust::emit_schema(&db, &root, true),
        ),
        (
            "crates/canboat/src/engine/fastpacket_generated.rs",
            emit_rust::emit_fastpacket(&db),
        ),
    ];

    let mut stale = Vec::new();
    for (rel, emitted) in &artifacts {
        let committed = std::fs::read_to_string(root.join(rel)).unwrap_or_default();
        if &committed != emitted {
            stale.push(format!(
                "  {rel} ({} vs {} bytes)",
                committed.len(),
                emitted.len()
            ));
        }
    }
    assert!(
        stale.is_empty(),
        "these artifacts are not up to date with database/ — run `keel generate`:\n{}",
        stale.join("\n")
    );
}

/// Emission is a pure function of the database: emitting twice gives the
/// same bytes. A `HashMap` iterated without sorting would show up here.
#[test]
fn emission_is_deterministic() {
    let (root, db, authored) = load();
    assert_eq!(
        emit_xml::emit_xml(&db, false),
        emit_xml::emit_xml(&db, false)
    );
    assert_eq!(
        emit_c::emit_pgn_data_h(&db, false),
        emit_c::emit_pgn_data_h(&db, false)
    );
    assert_eq!(
        emit_c::emit_lookup_h(&db, true),
        emit_c::emit_lookup_h(&db, true)
    );
    assert_eq!(
        emit_c::emit_fieldtype_data_h(&authored),
        emit_c::emit_fieldtype_data_h(&authored)
    );
    assert_eq!(
        emit_rust::emit_schema(&db, &root, false),
        emit_rust::emit_schema(&db, &root, false)
    );
    assert_eq!(
        emit_rust::emit_fastpacket(&db),
        emit_rust::emit_fastpacket(&db)
    );
}

/// The NMEA 2000 and J1939 tables are distinct — a wiring slip that emitted
/// one table for both flavors would otherwise pass every other check.
#[test]
fn the_j1939_tables_differ_from_the_nmea_ones() {
    let (root, db, _) = load();
    assert_ne!(
        emit_c::emit_pgn_data_h(&db, false),
        emit_c::emit_pgn_data_h(&db, true)
    );
    assert_ne!(
        emit_c::emit_lookup_h(&db, false),
        emit_c::emit_lookup_h(&db, true)
    );
    assert_ne!(
        emit_rust::emit_schema(&db, &root, false),
        emit_rust::emit_schema(&db, &root, true)
    );
}

/// `derive::fill` is idempotent: running it again over an already-filled
/// database must not change what the emitters see. The pass percolates
/// fieldtype defaults into fields, so a non-idempotent step would compound.
#[test]
fn the_derive_pass_is_idempotent() {
    let (root, mut db, _) = load();
    let once = emit_rust::emit_schema(&db, &root, false);
    derive::fill(&mut db).expect("second derive pass succeeds");
    assert_eq!(once, emit_rust::emit_schema(&db, &root, false));
}

/// `keel explain` renders the whole database as prose. It is not a
/// committed artifact, so nothing else would notice it panicking on a
/// PGN shape the database grows later.
#[test]
fn the_text_explanation_covers_the_whole_database() {
    let (_, db, _) = load();
    let out = emit_text::emit_text(&db, false);

    assert!(out.starts_with("CANboat version v"), "missing banner");
    assert!(out.contains("_______ Complete PGNs _________"));
    assert!(out.contains("_______ Incomplete PGNs _________"));

    // Every PGN below the ACTISENSE_BEM synthetic range gets a heading,
    // in one section or the other.
    let headings = out.matches("\nPGN: ").count();
    let expected = db.pgns.iter().filter(|p| p.pgn < 0x40000).count();
    assert_eq!(headings, expected, "one heading per non-synthetic PGN");

    // A couple of well-known PGNs and a field name, so the body is real
    // content rather than just section headers.
    assert!(out.contains("Vessel Heading"), "127250 missing");
    assert!(out.contains("GNSS Position Data"), "129029 missing");
}

/// The J1939 flavor explains its own table, not the marine one.
#[test]
fn the_text_explanation_has_a_j1939_flavor() {
    let (_, db, _) = load();
    assert_ne!(
        emit_text::emit_text(&db, false),
        emit_text::emit_text(&db, true)
    );
}

/// DM1's trouble codes carry a 19-bit SPN split around the FMI. `continues:`
/// on the 3 high bits places them 21 bits after the start of the 16 low
/// ones, the joined value is 19 bits wide, and the SPN is named from the
/// lookup keel builds out of the J1939 fields' `spn:` attributes.
#[test]
fn a_dm1_spn_is_joined_and_named() {
    let (_, db, _) = load();
    let dm1 = db
        .pgns_j1939
        .iter()
        .find(|p| p.pgn == 65226)
        .expect("DM1 is defined");
    let spn = dm1.fields.iter().find(|f| f.id == "spn").unwrap();
    assert_eq!(spn.res_continuation, Some((21, 3)));
    assert_eq!(spn.value_bits(), 19);
    assert_eq!(spn.res_range_max, 524287.0);

    let lk = &db.lookups[derive::SPN_LOOKUP];
    assert_eq!(lk.bits, 19);
    assert!(lk.pairs.contains(&(190, "Engine RPM".to_string())));
    assert!(
        db.lookups_used(true).contains(derive::SPN_LOOKUP)
            && !db.lookups_used(false).contains(derive::SPN_LOOKUP),
        "only the J1939 tree carries the SPN lookup"
    );
}

/// R43 refuses a `continues:` that cannot be joined: one naming no earlier
/// field, one with a scaling of its own, and one continuing a signed field.
#[test]
fn r43_refuses_a_continuation_that_cannot_be_joined() {
    let r43 = |db: &Database| {
        check::check(db)
            .into_iter()
            .filter(|v| v.rule == "R43" && v.error)
            .count()
    };
    let (_, mut db, _) = load();
    assert_eq!(r43(&db), 0);
    let high = |db: &mut Database| {
        db.pgns_j1939
            .iter_mut()
            .find(|p| p.pgn == 65226)
            .unwrap()
            .fields
            .iter_mut()
            .find(|f| f.id == "spnHigh")
            .unwrap()
            .clone()
    };
    let original = high(&mut db);
    let set = |db: &mut Database, f: keel::model::Field| {
        let dm1 = db.pgns_j1939.iter_mut().find(|p| p.pgn == 65226).unwrap();
        let slot = dm1.fields.iter_mut().find(|g| g.id == f.id).unwrap();
        *slot = f;
        derive::fill(db).expect("derive pass succeeds");
    };

    let mut f = original.clone();
    f.continues = Some("oc".into()); // a later field
    set(&mut db, f);
    assert_eq!(r43(&db), 1);

    let mut f = original.clone();
    f.resolution = Some(0.5);
    set(&mut db, f);
    assert_eq!(r43(&db), 1);
    set(&mut db, original);

    // The continued field itself must be unsigned: the bits are joined raw.
    let dm1 = db.pgns_j1939.iter().find(|p| p.pgn == 65226).unwrap();
    let mut low = dm1.fields.iter().find(|f| f.id == "spn").unwrap().clone();
    low.type_ = "INTEGER".into();
    low.lookup = None;
    set(&mut db, low);
    assert_eq!(r43(&db), 1);
}
