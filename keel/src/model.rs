//! The in-memory PGN database model.
//!
//! Authored attributes mirror what the YAML files store (decisions only);
//! derived attributes (suffix `res_*` and the "filled by derive" fields) are
//! computed exactly like the C analyzer computes them, so the XML emitter can
//! reproduce analyzer-explain.c's output byte-for-byte.
//!
//! Tri-state booleans (C `Bool`: True/False/Null) are `Option<bool>`.

use std::collections::HashMap;

/// `<MissingAttribute>` names in emission order (analyzer-explain.c).
pub const MISSING_ATTRIBUTES: [&str; 7] = [
    "Fields",
    "FieldLengths",
    "Resolution",
    "Lookups",
    "SampleData",
    "Interval",
    "MissingCompanyFields",
];

pub const ACTISENSE_BEM: u32 = 0x40000;
pub const IKONVERT_BEM: u32 = 0x40100;

#[derive(Debug, Clone, Default)]
pub struct PhysicalQuantity {
    pub name: String,
    pub description: Option<String>,
    pub comment: Option<String>,
    pub url: Option<String>,
    pub unit_description: Option<String>,
    pub unit: Option<String>,
}

#[derive(Debug, Clone, Default)]
pub struct FieldType {
    pub name: String,
    pub description: Option<String>,
    pub encoding_description: Option<String>,
    pub comment: Option<String>,
    pub url: Option<String>,
    pub size: u32, // bits; 0 = per-field
    pub variable_size: bool,
    pub base: Option<String>,
    pub unit: Option<String>,
    pub offset: i32,
    pub resolution: f64,
    pub has_sign: Option<bool>,
    pub sentinels: String, // "None" | "TopOfRange" | "NaN" | "EmptyString" | "Variable"
    pub physical: Option<String>,
    pub print_function: Option<String>,
    /// Inert in C (QUIRKS.md Q11); kept to regenerate fieldtype.h faithfully.
    /// range_min becomes readable once the fieldtype.h generator (migration
    /// step 3) lands; range_max already participates in the Q11 guard.
    #[allow(dead_code)]
    pub range_min_authored: Option<f64>,
    pub range_max_authored: Option<f64>,

    // --- filled by derive::fill_fieldtypes ---
    pub range_min: f64,
    pub range_max: f64,
    pub root_name: String,
    pub root_sentinels: String,
}

#[derive(Debug, Clone, Default)]
pub struct FieldTypeLookupEntry {
    pub value: u64,
    pub name: String,
    pub fieldtype: String,
    pub bits: Option<u32>,
    pub lookup: Option<String>,
    pub lookup_kind: Option<String>,
}

#[derive(Debug, Clone, Default)]
pub struct Lookup {
    pub name: String,
    pub kind: String, // "pair" | "triplet" | "bit" | "fieldtype"
    pub bits: u32,
    pub pairs: Vec<(u64, String)>,
    pub triplets: Vec<(u64, u64, String)>,
    pub fieldtypes: Vec<FieldTypeLookupEntry>,
}

#[derive(Debug, Clone, Default)]
pub struct Field {
    // --- authored ---
    pub id: String,
    pub name: String,
    pub type_: String,
    pub bits: Option<u32>,
    pub resolution: Option<f64>,
    pub unit: Option<String>,
    pub offset: Option<i32>,
    pub description: Option<String>,
    pub match_: Option<String>, // printed verbatim; int-parsed for the lookup description
    pub lookup: Option<String>,
    pub lookup_indirect: Option<String>,
    pub lookup_indirect_order: Option<u32>,
    pub lookup_bits: Option<String>,
    pub lookup_fieldtype: Option<String>,
    pub primary_key: bool,
    pub proprietary: bool,
    pub special_values: Option<u32>,
    pub dynamic_field_length: bool,
    pub dynamic_field_length_overhead: u32,
    pub range_min: Option<f64>,
    pub range_max: Option<f64>,

    // --- filled by derive::fill ---
    pub ft: usize, // index into Database::fieldtypes
    pub res_bits: u32,
    pub res_resolution: f64,
    pub res_unit: Option<String>,
    pub res_offset: i32,
    pub res_range_min: f64,
    pub res_range_max: f64,
    pub reserved_count: u32,
    pub order: u32,
}

impl Field {
    /// (kind, name) of whichever lookup this field references.
    pub fn lookup_ref(&self) -> Option<(&'static str, &str)> {
        if let Some(n) = &self.lookup {
            return Some(("pair", n));
        }
        if let Some(n) = &self.lookup_indirect {
            return Some(("triplet", n));
        }
        if let Some(n) = &self.lookup_bits {
            return Some(("bit", n));
        }
        if let Some(n) = &self.lookup_fieldtype {
            return Some(("fieldtype", n));
        }
        None
    }
}

#[derive(Debug, Clone, PartialEq)]
pub enum Interval {
    Unknown,
    Irregular,
    Ms(u32),
}

impl Default for Interval {
    fn default() -> Self {
        Interval::Unknown
    }
}

#[derive(Debug, Clone, Default)]
pub struct Repeating {
    pub count: u32,
    pub start: u32,
    pub count_field: Option<u32>,
}

#[derive(Debug, Clone, Default)]
pub struct Pgn {
    // --- authored ---
    pub pgn: u32,
    pub id: String,
    pub description: String,
    pub type_: String, // "Single" | "Fast" | "ISO" | "Mixed"
    pub priority: u32,
    pub interval: Interval,
    pub explanation: Option<String>,
    pub url: Option<String>,
    pub research_doc: Option<String>,
    pub fallback: bool,
    pub missing: Vec<String>, // sans the automatic Interval entry
    pub repeating1: Option<Repeating>,
    pub repeating2: Option<Repeating>,
    pub variant_order: u32,
    pub fields: Vec<Field>,

    // --- filled by derive::fill ---
    pub field_count: u32,
    pub length: u32,
    pub is_variable: bool,
}

impl Pgn {
    /// The `<Missing>` list as emitted: authored plus the automatic Interval
    /// attribute when the interval is unknown (fieldtype.c:486).
    pub fn missing_effective(&self) -> Vec<&'static str> {
        MISSING_ATTRIBUTES
            .iter()
            .copied()
            .filter(|m| {
                if *m == "Interval" {
                    self.interval == Interval::Unknown
                } else {
                    self.missing.iter().any(|x| x == m)
                }
            })
            .collect()
    }

    pub fn is_complete(&self) -> bool {
        self.missing_effective().is_empty()
    }
}

#[derive(Debug, Default)]
pub struct Database {
    pub physical_quantities: Vec<PhysicalQuantity>,
    pub fieldtypes: Vec<FieldType>,
    pub ft_index: HashMap<String, usize>,
    pub lookups: HashMap<String, Lookup>,
    pub lookup_order: HashMap<String, Vec<String>>, // kind -> names, emission order
    pub pgns: Vec<Pgn>,
    pub version: String,
    pub schema_version: String,
}

impl Database {
    pub fn index(&mut self) {
        self.ft_index = self
            .fieldtypes
            .iter()
            .enumerate()
            .map(|(i, ft)| (ft.name.clone(), i))
            .collect();
    }

    pub fn fieldtype(&self, name: &str) -> Result<usize, String> {
        self.ft_index
            .get(name)
            .copied()
            .ok_or_else(|| format!("fieldType '{name}' not found"))
    }

    pub fn ordered_lookups(&self, kind: &str) -> Vec<&Lookup> {
        self.lookup_order
            .get(kind)
            .map(|names| names.iter().map(|n| &self.lookups[n]).collect())
            .unwrap_or_default()
    }
}
