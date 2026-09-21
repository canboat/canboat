//! Artifact emission shared by `keel generate` and the editor's
//! post-save "regenerate" action.

use std::path::{Path, PathBuf};

use crate::model::{Database, FieldType};
use crate::{charset, emit_c, emit_xml};

/// Emit every generated artifact for the given (filled) database.
/// `authored_fieldtypes` is the pre-percolation state fieldtype-generated-data.h is
/// rendered from.
pub fn emit_artifacts(
    root: &Path,
    db: &Database,
    authored_fieldtypes: &[FieldType],
) -> Vec<(PathBuf, String)> {
    vec![
        (
            root.join("docs/canboat.xml"),
            emit_xml::emit_xml(db, "normal"),
        ),
        (
            root.join("analyzer/lookup-generated-data.h"),
            emit_c::emit_lookup_h(db, false),
        ),
        (
            root.join("analyzer/lookup-j1939-generated-data.h"),
            emit_c::emit_lookup_h(db, true),
        ),
        (
            root.join("analyzer/physicalquantity-generated-data.h"),
            emit_c::emit_physicalquantity_data_h(db),
        ),
        (
            root.join("analyzer/fieldtype-generated-data.h"),
            emit_c::emit_fieldtype_data_h(authored_fieldtypes),
        ),
        (
            root.join("analyzer/pgn-generated-data.h"),
            emit_c::emit_pgn_data_h(db, false),
        ),
        (
            root.join("analyzer/pgn-j1939-generated-data.h"),
            emit_c::emit_pgn_data_h(db, true),
        ),
        // The Basic RDS character set, for fields that declare
        // `encoding: RDS_G0`. Emitted from keel/src/charset.rs so the C and
        // Rust decoders cannot drift from keel's own.
        (
            root.join("analyzer/charset-generated-data.h"),
            charset::emit_charset_h(),
        ),
        (
            root.join("crates/canboat-core/src/charset_generated.rs"),
            charset::emit_charset_rs(),
        ),
    ]
}
