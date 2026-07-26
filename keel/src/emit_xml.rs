//! canboat.xml emitter - a line-faithful port of analyzer-explain.c's
//! explainXML() v2 output (v1 is dead, DESIGN.md §2).
//!
//! Every deliberate oddity reproduced here is cataloged in QUIRKS.md (Q1-Q19);
//! the golden byte-diff against analyzer-explain output guards them all.

use crate::cformat::{c_15g, c_g, rust_15g, rust_g, xml_escape};
use crate::model::{ACTISENSE_BEM, Database, Field, IKONVERT_BEM, Interval, Pgn};

// NB: no `\`-line-continuations here - they strip the next line's leading
// whitespace, which corrupts the indented license URL line.
const LICENSE: &str = concat!(
    "This file is part of CANboat.\n",
    "\n",
    "Licensed under the Apache License, Version 2.0 (the \"License\");\n",
    "you may not use this file except in compliance with the License.\n",
    "You may obtain a copy of the License at\n",
    "\n",
    "    http://www.apache.org/licenses/LICENSE-2.0\n",
    "\n",
    "Unless required by applicable law or agreed to in writing, software\n",
    "distributed under the License is distributed on an \"AS IS\" BASIS,\n",
    "WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n",
    "See the License for the specific language governing permissions and\n",
    "limitations under the License.\n",
);

fn copyright(version: &str) -> String {
    format!(
        "CANboat version v{version}\n\n(C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.\nFor more information see https://github.com/canboat/canboat\n\n{LICENSE}\n"
    )
}

/// Which float formatter to use: C (%g via snprintf, golden-compatible) or
/// Rust's shortest round-trip formatting (candidate replacement, QUIRKS Q6).
#[derive(Clone, Copy, PartialEq)]
pub enum FloatStyle {
    C,
    Rust,
}

pub struct Emitter<'a> {
    db: &'a Database,
    out: String,
    style: FloatStyle,
}

impl<'a> Emitter<'a> {
    pub fn new(db: &'a Database, style: FloatStyle) -> Self {
        Emitter {
            db,
            out: String::with_capacity(4 << 20),
            style,
        }
    }

    fn g(&self, v: f64) -> String {
        match self.style {
            FloatStyle::C => c_g(v),
            FloatStyle::Rust => rust_g(v),
        }
    }

    fn g15(&self, v: f64) -> String {
        match self.style {
            FloatStyle::C => c_15g(v),
            FloatStyle::Rust => rust_15g(v),
        }
    }

    fn p(&mut self, text: &str) {
        self.out.push_str(text);
    }

    /// printXML(): skipped entirely when text is None.
    fn xml(&mut self, indent: usize, element: &str, text: Option<&str>) {
        if let Some(t) = text {
            let pad = " ".repeat(indent);
            self.out
                .push_str(&format!("{pad}<{element}>{}</{element}>\n", xml_escape(t)));
        }
    }

    fn xml_u(&mut self, indent: usize, element: &str, value: u64) {
        let pad = " ".repeat(indent);
        self.out
            .push_str(&format!("{pad}<{element}>{value}</{element}>\n"));
    }

    // ----- sections ------------------------------------------------------

    fn header(&mut self) {
        let cp = copyright(&self.db.version);
        self.p("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
        self.p(&format!("<!--\n{cp}\n-->\n"));
        self.p("<?xml-stylesheet type=\"text/xsl\" href=\"canboat.xsl\"?>\n");
        self.p(&format!(
            "<PGNDefinitions xmlns:xsi=\"http://www.w3.org/2001/XMLSchema-instance\" \
             xmlns:xsd=\"http://www.w3.org/2001/XMLSchema\">\n  <SchemaVersion>{}</SchemaVersion>\n",
            self.db.schema_version
        ));
        self.p("  <Comment>See https://github.com/canboat/canboat for the full source code</Comment>\n");
        self.p(&format!(
            "  <CreatorCode>Canboat NMEA2000 Analyzer</CreatorCode>\n  \
             <License>Apache License Version 2.0</License>\n  <Version>{}</Version>\n",
            self.db.version
        ));
        self.p(&format!("  <Copyright>{cp}\n</Copyright>\n"));
    }

    fn physical_quantities(&mut self) {
        // explainPhysicalQuantityXML: raw printf, no escaping (QUIRKS Q4)
        self.p("  <PhysicalQuantities>\n");
        for pq in &self.db.physical_quantities {
            self.p(&format!("    <PhysicalQuantity Name=\"{}\">\n", pq.name));
            for (tag, val) in [
                ("Description", &pq.description),
                ("Comment", &pq.comment),
                ("URL", &pq.url),
                ("UnitDescription", &pq.unit_description),
                ("Unit", &pq.unit),
            ] {
                if let Some(v) = val {
                    self.p(&format!("      <{tag}>{v}</{tag}>\n"));
                }
            }
            self.p("    </PhysicalQuantity>\n");
        }
        self.p("  </PhysicalQuantities>\n");
    }

    fn fieldtypes(&mut self) {
        // explainFieldTypesXML: raw printf, roots only (QUIRKS Q4)
        self.p("  <FieldTypes>\n");
        for ft in &self.db.fieldtypes {
            if ft.base.is_some() {
                continue;
            }
            self.p(&format!("    <FieldType Name=\"{}\">\n", ft.name));
            for (tag, val) in [
                ("Description", &ft.description),
                ("EncodingDescription", &ft.encoding_description),
                ("Comment", &ft.comment),
                ("URL", &ft.url),
            ] {
                if let Some(v) = val {
                    self.p(&format!("      <{tag}>{v}</{tag}>\n"));
                }
            }
            if ft.size != 0 {
                self.p(&format!("      <Bits>{}</Bits>\n", ft.size));
            }
            if ft.offset != 0 {
                if ft.resolution == 1.0 || ft.resolution == 0.0 {
                    self.p(&format!("      <Offset>{}</Offset>\n", ft.offset));
                } else {
                    self.p(&format!(
                        "      <Offset>{}</Offset>\n",
                        (ft.offset as f64 * ft.resolution) as i64
                    ));
                }
            }
            if ft.variable_size {
                self.p("      <VariableSize>true</VariableSize>\n");
            }
            if let Some(u) = &ft.unit {
                self.p(&format!("      <Unit>{u}</Unit>\n"));
            }
            if let Some(s) = ft.has_sign {
                self.p(&format!(
                    "      <Signed>{}</Signed>\n",
                    if s { "true" } else { "false" }
                ));
            }
            if ft.resolution != 1.0 && ft.resolution != 0.0 {
                let r = self.g15(ft.resolution);
                self.p(&format!("      <Resolution>{r}</Resolution>\n"));
            }
            if !ft.range_min.is_nan() {
                let r = self.g15(ft.range_min);
                self.p(&format!("      <RangeMin>{r}</RangeMin>\n"));
            }
            if !ft.range_max.is_nan() {
                let r = self.g15(ft.range_max);
                self.p(&format!("      <RangeMax>{r}</RangeMax>\n"));
            }
            self.p(&format!("      <Sentinels>{}</Sentinels>\n", ft.sentinels));
            self.p("    </FieldType>\n");
        }
        self.p("  </FieldTypes>\n");
    }

    fn missing(&mut self) {
        self.p("  <MissingEnumerations>\n");
        for (name, text) in [
            (
                "Fields",
                "The list of fields is incomplete; some fields maybe be missing or their attributes may be incorrect",
            ),
            (
                "FieldLengths",
                "The length of one or more fields is likely incorrect",
            ),
            (
                "Resolution",
                "The resolution of one or more fields is likely incorrect",
            ),
            (
                "Lookups",
                "One or more of the lookup fields contain missing or incorrect values",
            ),
            (
                "SampleData",
                "The PGN has not been seen in any logfiles yet",
            ),
            ("Interval", "The default transmission interval is not known"),
            (
                "MissingCompanyFields",
                "The manufacturer specific PGN seems to not contain the mandatory Company and Industry fields; this is likely a bug or at least incompatibility in the device",
            ),
        ] {
            self.p(&format!(
                "    <MissingAttribute Name=\"{name}\">{text}</MissingAttribute>\n"
            ));
        }
        self.p("  </MissingEnumerations>\n");
    }

    fn lookup_sections(&mut self) {
        self.p("  <LookupEnumerations>\n");
        for lk in self.db.ordered_lookups("pair") {
            let max_value = (1u128 << lk.bits) - 1;
            self.p(&format!(
                "    <LookupEnumeration Name='{}' MaxValue='{max_value}'>\n",
                lk.name
            ));
            for (value, name) in &lk.pairs {
                self.p(&format!(
                    "      <EnumPair Value='{value}' Name='{}' />\n",
                    xml_escape(name)
                ));
            }
            self.p("    </LookupEnumeration>\n");
        }
        self.p("  </LookupEnumerations>\n");

        self.p("  <LookupIndirectEnumerations>\n");
        for lk in self.db.ordered_lookups("triplet") {
            let max_value = (1u128 << lk.bits) - 1;
            self.p(&format!(
                "    <LookupIndirectEnumeration Name='{}' MaxValue='{max_value}'>\n",
                lk.name
            ));
            for (v1, v2, name) in &lk.triplets {
                self.p(&format!(
                    "      <EnumTriplet Value1='{v1}' Value2='{v2}' Name='{}' />\n",
                    xml_escape(name)
                ));
            }
            self.p("    </LookupIndirectEnumeration>\n");
        }
        self.p("  </LookupIndirectEnumerations>\n");

        self.p("  <LookupBitEnumerations>\n");
        for lk in self.db.ordered_lookups("bit") {
            let max_value = lk.bits - 1;
            self.p(&format!(
                "    <LookupBitEnumeration Name='{}' MaxValue='{max_value}'>\n",
                lk.name
            ));
            for (bit, name) in &lk.pairs {
                self.p(&format!(
                    "      <BitPair Bit='{bit}' Name='{}' />\n",
                    xml_escape(name)
                ));
            }
            self.p("    </LookupBitEnumeration>\n");
        }
        self.p("  </LookupBitEnumerations>\n");

        self.p("  <LookupFieldTypeEnumerations>\n");
        for lk in self.db.ordered_lookups("fieldtype") {
            let max_value = (1u128 << lk.bits) - 1;
            self.p(&format!(
                "    <LookupFieldTypeEnumeration Name='{}' MaxValue='{max_value}'>\n",
                lk.name
            ));
            let entries = lk.fieldtypes.clone();
            for entry in &entries {
                self.enum_fieldtype(entry);
            }
            self.p("    </LookupFieldTypeEnumeration>\n");
        }
        self.p("  </LookupFieldTypeEnumerations>\n");
    }

    /// explainFieldtypeXMLv2, including the Signed newline quirk (QUIRKS Q2).
    fn enum_fieldtype(&mut self, entry: &crate::model::FieldTypeLookupEntry) {
        let fti = self
            .db
            .fieldtype(&entry.fieldtype)
            .expect("lookup fieldtype");
        let ft = &self.db.fieldtypes[fti];
        self.p(&format!(
            "      <EnumFieldType Value='{}' Name='{}'",
            entry.value,
            xml_escape(&entry.name)
        ));
        self.p(&format!(" FieldType='{}'", ft.root_name));
        if let Some(s) = ft.has_sign {
            self.p(&format!(" Signed='{}'\n", if s { "true" } else { "false" }));
        }
        if ft.resolution != 0.0 {
            self.p(&format!(" Resolution='{}'", self.g(ft.resolution)));
        }
        if let Some(u) = &ft.unit {
            self.p(&format!(" Unit='{u}'"));
        }
        let bits = entry.bits.unwrap_or(ft.size);
        if bits != 0 {
            self.p(&format!(" Bits='{bits}'"));
        }
        if let Some(nested) = entry.lookup.clone() {
            self.p(">\n");
            let kind = entry.lookup_kind.clone().unwrap_or_else(|| "pair".into());
            self.lookup_reference(&kind, &nested, None);
            self.p("      </EnumFieldType>\n");
        } else {
            self.p("/>\n");
        }
    }

    /// explainLookupFunction() v2: reference elements at indent 10.
    fn lookup_reference(&mut self, kind: &str, name: &str, order: Option<u32>) {
        match kind {
            "pair" => self.xml(10, "LookupEnumeration", Some(name)),
            "bit" => self.xml(10, "LookupBitEnumeration", Some(name)),
            "triplet" => {
                self.xml(10, "LookupIndirectEnumeration", Some(name));
                self.xml_u(
                    10,
                    "LookupIndirectEnumerationFieldOrder",
                    order.unwrap_or(0) as u64,
                );
            }
            "fieldtype" => self.xml(10, "LookupFieldTypeEnumeration", Some(name)),
            _ => {}
        }
    }

    // ----- PGNs -----------------------------------------------------------

    /// Port of explainPGNXML(), v2 path.
    fn pgn(&mut self, pgn: &Pgn) {
        self.p(&format!("    <PGNInfo>\n      <PGN>{}</PGN>\n", pgn.pgn));
        self.xml(6, "Id", Some(&pgn.id));
        self.xml(6, "Description", Some(&pgn.description));
        if pgn.priority != 0 {
            // 4-space indent quirk (QUIRKS Q1)
            self.p(&format!("    <Priority>{}</Priority>\n", pgn.priority));
        }
        self.xml(6, "Explanation", pgn.explanation.as_deref());
        self.xml(6, "URL", pgn.url.as_deref());
        self.xml(6, "ResearchDoc", pgn.research_doc.as_deref());
        self.xml(6, "Type", Some(&pgn.type_));
        self.xml(
            6,
            "Complete",
            Some(if pgn.is_complete() { "true" } else { "false" }),
        );
        if pgn.fallback {
            self.xml(6, "Fallback", Some("true"));
        }

        let missing = pgn.missing_effective();
        if !missing.is_empty() {
            self.p("      <Missing>\n");
            for name in missing {
                self.xml(8, "MissingAttribute", Some(name));
            }
            self.p("      </Missing>\n");
        }

        self.xml_u(6, "FieldCount", pgn.field_count as u64);
        if pgn.is_variable {
            self.xml_u(6, "MinLength", pgn.length as u64);
        } else {
            self.xml_u(6, "Length", pgn.length as u64);
        }

        for (n, rep) in [(1, &pgn.repeating1), (2, &pgn.repeating2)] {
            if let Some(rep) = rep {
                if rep.count > 0 {
                    self.xml_u(6, &format!("RepeatingFieldSet{n}Size"), rep.count as u64);
                    self.xml_u(
                        6,
                        &format!("RepeatingFieldSet{n}StartField"),
                        rep.start as u64,
                    );
                    if let Some(cf) = rep.count_field {
                        self.xml_u(6, &format!("RepeatingFieldSet{n}CountField"), cf as u64);
                    }
                }
            }
        }

        match pgn.interval {
            Interval::Ms(ms) if ms != 0 => self.xml_u(6, "TransmissionInterval", ms as u64),
            Interval::Irregular => self.xml(6, "TransmissionIrregular", Some("true")),
            _ => {}
        }

        if !pgn.fields.is_empty() {
            self.p("      <Fields>\n");
            let mut bit_offset = 0u32;
            let mut show_bit_offset = true;
            for f in &pgn.fields {
                (bit_offset, show_bit_offset) = self.field(f, bit_offset, show_bit_offset);
            }
            self.p("      </Fields>\n");
        }
        self.p("    </PGNInfo>\n");
    }

    fn field(&mut self, f: &Field, mut bit_offset: u32, mut show_bit_offset: bool) -> (u32, bool) {
        let ft = &self.db.fieldtypes[f.ft];
        let lookup_ref = f.lookup_ref().map(|(k, n)| (k, n.to_string()));
        self.p(&format!(
            "        <Field>\n          <Order>{}</Order>\n",
            f.order
        ));
        self.xml(10, "Id", Some(&f.id));
        self.xml(10, "Name", Some(&f.name));

        match &f.description {
            Some(d) if !d.is_empty() && !d.starts_with(',') => {
                self.xml(10, "Description", Some(d));
            }
            _ => {
                if f.match_.is_some() && lookup_ref.is_some() {
                    // filterPair: a match field's description is the lookup name
                    let (kind, name) = lookup_ref.as_ref().unwrap();
                    let desc = self.match_description(f, kind, name);
                    self.p(&format!("          <Description>{desc}</Description>\n"));
                }
            }
        }

        if f.res_bits == 0 {
            self.p("          <BitLengthVariable>true</BitLengthVariable>\n");
            if f.type_ == "BINARY" {
                self.xml_u(10, "BitLengthField", (f.order - 1) as u64);
            }
        } else {
            self.xml_u(10, "BitLength", f.res_bits as u64);
        }
        if show_bit_offset {
            self.xml_u(10, "BitOffset", bit_offset as u64);
            self.xml_u(10, "BitStart", (bit_offset % 8) as u64);
        }
        bit_offset += f.res_bits;

        if f.proprietary {
            self.xml(10, "Condition", Some("PGNIsProprietary"));
        }
        if f.match_.is_some() {
            // Emitted numerically even when authored as a lookup name
            // (model::resolve_match); the C analyzer matches on the number.
            let n = self
                .db
                .resolve_match(f)
                .expect("match resolves (guaranteed by check R13)");
            self.xml(10, "Match", Some(&n.to_string()));
        } else {
            self.xml(10, "Unit", f.res_unit.as_deref());
        }

        if f.res_resolution != 0.0 {
            let r = self.g(f.res_resolution);
            self.p(&format!("          <Resolution>{r}</Resolution>\n"));
        }
        if let Some(s) = ft.has_sign {
            self.p(&format!(
                "          <Signed>{}</Signed>\n",
                if s { "true" } else { "false" }
            ));
        }
        if f.res_offset != 0 {
            if f.res_resolution == 1.0 || f.res_resolution == 0.0 {
                self.p(&format!("          <Offset>{}</Offset>\n", f.res_offset));
            } else {
                self.p(&format!(
                    "          <Offset>{}</Offset>\n",
                    (f.res_offset as f64 * f.res_resolution) as i64
                ));
            }
        }

        if f.dynamic_field_length && f.dynamic_field_length_overhead != 0 {
            self.p(&format!(
                "          <DynamicFieldLengthOverhead>{}</DynamicFieldLengthOverhead>\n",
                f.dynamic_field_length_overhead
            ));
        }

        let is_match = f.match_.is_some();
        if !f.res_range_min.is_nan() {
            let r = self.g15(f.res_range_min);
            self.p(&format!("          <RangeMin>{r}</RangeMin>\n"));
        } else if lookup_ref.is_some() && !is_match {
            let r = self.g15(0.0);
            self.p(&format!("          <RangeMin>{r}</RangeMin>\n"));
        }

        if !f.res_range_max.is_nan() {
            if f.res_resolution == 1.0
                && f.res_bits == 64
                && ft.has_sign == Some(false)
                && f.res_offset == 0
            {
                self.p(&format!("          <RangeMax>{}</RangeMax>\n", u64::MAX));
            } else {
                let r = self.g15(f.res_range_max);
                self.p(&format!("          <RangeMax>{r}</RangeMax>\n"));
            }
        } else if lookup_ref.is_some() && !is_match {
            let r = self.g15(((1u128 << f.res_bits) - 1) as f64);
            self.p(&format!("          <RangeMax>{r}</RangeMax>\n"));
        }

        if f.reserved_count > 0
            && f.res_bits < 64
            && !f.res_range_min.is_nan()
            && !is_match
            && ft.root_sentinels == "TopOfRange"
        {
            let highbit = if ft.has_sign == Some(true) && f.res_offset == 0 {
                f.res_bits - 1
            } else {
                f.res_bits
            };
            let raw = (1u64 << highbit) - 1;
            self.p(&format!("          <UnknownValue>{raw}</UnknownValue>\n"));
            if f.reserved_count >= 2 {
                self.p(&format!(
                    "          <OutOfRangeValue>{}</OutOfRangeValue>\n",
                    raw - 1
                ));
            }
            if f.reserved_count >= 3 {
                self.p(&format!(
                    "          <ReservedValue>{}</ReservedValue>\n",
                    raw - 2
                ));
            }
        }

        self.xml(10, "FieldType", Some(&ft.root_name.clone()));
        if let Some(phys) = ft.physical.clone() {
            self.xml(10, "PhysicalQuantity", Some(&phys));
        }

        if let Some((kind, name)) = &lookup_ref {
            self.lookup_reference(kind, name, f.lookup_indirect_order);
        }

        if f.primary_key {
            self.xml(10, "PartOfPrimaryKey", Some("true"));
        }

        if self.db.fieldtypes[f.ft].variable_size || f.proprietary || f.res_bits == 0 {
            show_bit_offset = false;
        }
        self.p("        </Field>\n");
        (bit_offset, show_bit_offset)
    }

    fn match_description(&self, f: &Field, kind: &str, name: &str) -> String {
        let Some(lk) = self.db.lookups.get(name) else {
            return String::new();
        };
        let Some(want) = self.db.resolve_match(f) else {
            return String::new();
        };
        if kind == "fieldtype" {
            lk.fieldtypes
                .iter()
                .filter(|e| e.value == want)
                .map(|e| e.name.as_str())
                .collect()
        } else {
            lk.pairs
                .iter()
                .filter(|(v, _)| *v == want)
                .map(|(_, n)| n.as_str())
                .collect()
        }
    }

    // ----- top level ------------------------------------------------------

    pub fn emit(mut self, which: &str) -> String {
        self.header();
        if which == "normal" || which == "j1939" {
            // The J1939 document is the "normal" layout of the J1939 build
            // (analyzer-explain-j1939): full sections, its own pgnList.
            self.physical_quantities();
            self.fieldtypes();
            self.missing();
            self.lookup_sections();
        }
        self.p("  <PGNs>\n");
        let pgns: Vec<Pgn> = if which == "j1939" {
            self.db.pgns_j1939.clone()
        } else {
            self.db.pgns.clone()
        };
        for pgn in &pgns {
            let include = match which {
                "normal" | "j1939" => pgn.pgn < ACTISENSE_BEM,
                "actisense" => (ACTISENSE_BEM..IKONVERT_BEM).contains(&pgn.pgn),
                "ikonvert" => pgn.pgn >= IKONVERT_BEM,
                _ => false,
            };
            if include {
                self.pgn(pgn);
            }
        }
        self.p("  </PGNs>\n</PGNDefinitions>\n");
        self.out
    }
}

pub fn emit_xml(db: &Database, which: &str, style: FloatStyle) -> String {
    Emitter::new(db, style).emit(which)
}
