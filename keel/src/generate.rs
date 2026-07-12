//! Artifact emission shared by `keel generate` and the editor's
//! post-save "regenerate" action.

use std::path::{Path, PathBuf};

use crate::model::{Database, FieldType};
use crate::{emit_c, emit_xml};

/// Emit every generated artifact for the given (filled) database.
/// `authored_fieldtypes` is the pre-percolation state fieldtype-data.h is
/// rendered from.
pub fn emit_artifacts(
    root: &Path,
    db: &Database,
    authored_fieldtypes: &[FieldType],
    style: emit_xml::FloatStyle,
) -> Vec<(PathBuf, String)> {
    vec![
        (
            root.join("docs/canboat.xml"),
            emit_xml::emit_xml(db, "normal", style),
        ),
        (root.join("analyzer/lookup.h"), emit_c::emit_lookup_h(db)),
        (
            root.join("analyzer/physicalquantity-data.h"),
            emit_c::emit_physicalquantity_data_h(db),
        ),
        (
            root.join("analyzer/fieldtype-data.h"),
            emit_c::emit_fieldtype_data_h(authored_fieldtypes),
        ),
        (root.join("analyzer/pgn-data.h"), emit_c::emit_pgn_data_h(db, false)),
        (
            root.join("analyzer/pgn-j1939-data.h"),
            emit_c::emit_pgn_data_h(db, true),
        ),
    ]
}
