//! Artifact emission shared by `keel generate`, the editor's post-save
//! "regenerate" action and the up-to-date test: the one list of every file
//! keel writes.

use std::path::{Path, PathBuf};

use crate::model::{Database, FieldType, Protocol};
use crate::{charset, emit_c, emit_rust, emit_xml};

/// Emit every generated artifact for the given (filled) database.
/// `authored_fieldtypes` is the pre-percolation state fieldtype-generated-data.h is
/// rendered from.
pub fn emit_artifacts(
    root: &Path,
    db: &Database,
    authored_fieldtypes: &[FieldType],
) -> Vec<(PathBuf, String)> {
    let mut out = Vec::new();
    for protocol in Protocol::ALL {
        let suffix = match protocol {
            Protocol::Nmea2000 => String::new(),
            other => format!("-{}", other.name()),
        };
        out.push((
            root.join(format!("docs/canboat{suffix}.xml")),
            emit_xml::emit_xml(db, protocol),
        ));
        // The Rust tables. These used to be produced by a build script,
        // which could not be published: a build script cannot read
        // `database/`, which sits above the package root. Generated here and
        // committed instead.
        out.push((
            root.join(format!(
                "crates/canboat/src/engine/schema_generated{}.rs",
                suffix.replace('-', "_")
            )),
            emit_rust::emit_schema(db, root, protocol),
        ));
        // The C analyzer covers NMEA 2000 and J1939 only.
        if protocol != Protocol::Quick {
            out.push((
                root.join(format!("analyzer/lookup{suffix}-generated-data.h")),
                emit_c::emit_lookup_h(db, protocol),
            ));
            out.push((
                root.join(format!("analyzer/pgn{suffix}-generated-data.h")),
                emit_c::emit_pgn_data_h(db, protocol),
            ));
        }
    }
    out.extend([
        (
            root.join("analyzer/physicalquantity-generated-data.h"),
            emit_c::emit_physicalquantity_data_h(db),
        ),
        (
            root.join("analyzer/fieldtype-generated-data.h"),
            emit_c::emit_fieldtype_data_h(authored_fieldtypes),
        ),
        (
            root.join("crates/canboat/src/engine/fastpacket_generated.rs"),
            emit_rust::emit_fastpacket(db),
        ),
        // The Basic RDS character set, for fields that declare
        // `encoding: RDS_G0`. Emitted from keel/src/charset.rs so the C and
        // Rust decoders cannot drift from keel's own.
        (
            root.join("analyzer/charset-generated-data.h"),
            charset::emit_charset_h(),
        ),
        (
            root.join("crates/canboat/src/engine/charset_generated.rs"),
            charset::emit_charset_rs(),
        ),
    ]);
    out
}
