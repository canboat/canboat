"""canboat.xml emitter - a line-faithful port of analyzer-explain.c's
explainXML() v2 output (v1 is dead, DESIGN.md §2).

Faithfulness notes ("quirks", all deliberate, all verified by the golden
byte-diff against analyzer-explain output):
  - <Priority> is indented 4 spaces where its siblings use 6 (C printf bug).
  - EnumFieldType's Signed='...' attribute is followed by a newline INSIDE
    the tag (C prints " Signed='%s'\\n").
  - printXML() escapes & < > " but not the apostrophe; the FieldTypes and
    PhysicalQuantities sections don't escape at all (raw printf).
  - Floats use C %g / %.15g; RangeMax of an unsigned 64-bit resolution-1
    field prints as the integer UINT64_MAX.
"""

from __future__ import annotations

import math

from .cformat import c_15g, c_g, xml_escape
from .derive import UINT64_MAX, sentinels_for_fieldtype
from .model import ACTISENSE_BEM, IKONVERT_BEM, Database, Field_, Lookup, Pgn

LICENSE = (
    "This file is part of CANboat.\n"
    "\n"
    'Licensed under the Apache License, Version 2.0 (the "License");\n'
    "you may not use this file except in compliance with the License.\n"
    "You may obtain a copy of the License at\n"
    "\n"
    "    http://www.apache.org/licenses/LICENSE-2.0\n"
    "\n"
    "Unless required by applicable law or agreed to in writing, software\n"
    'distributed under the License is distributed on an "AS IS" BASIS,\n'
    "WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.\n"
    "See the License for the specific language governing permissions and\n"
    "limitations under the License.\n"
)


def copyright_text(version: str) -> str:
    return (
        f"CANboat version v{version}\n\n"
        "(C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.\n"
        "For more information see https://github.com/canboat/canboat\n"
        "\n" + LICENSE + "\n"
    )


class Emitter:
    def __init__(self, db: Database):
        self.db = db
        self.out: list = []

    def p(self, text: str) -> None:
        self.out.append(text)

    def xml(self, indent: int, element: str, text) -> None:
        """printXML(): skipped entirely when text is None."""
        if text is None:
            return
        self.p(f"{' ' * indent}<{element}>{xml_escape(text)}</{element}>\n")

    def xml_u(self, indent: int, element: str, value: int) -> None:
        self.p(f"{' ' * indent}<{element}>{value}</{element}>\n")

    # ----- sections ------------------------------------------------------

    def header(self) -> None:
        cp = copyright_text(self.db.version)
        self.p('<?xml version="1.0" encoding="utf-8"?>\n')
        self.p(f"<!--\n{cp}\n-->\n")
        self.p('<?xml-stylesheet type="text/xsl" href="canboat.xsl"?>\n')
        self.p(
            '<PGNDefinitions xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"'
            ' xmlns:xsd="http://www.w3.org/2001/XMLSchema">\n'
            f"  <SchemaVersion>{self.db.schema_version}</SchemaVersion>\n"
        )
        self.p("  <Comment>See https://github.com/canboat/canboat for the full source code</Comment>\n")
        self.p(
            "  <CreatorCode>Canboat NMEA2000 Analyzer</CreatorCode>\n"
            "  <License>Apache License Version 2.0</License>\n"
            f"  <Version>{self.db.version}</Version>\n"
        )
        self.p(f"  <Copyright>{cp}\n</Copyright>\n")

    def physical_quantities(self) -> None:
        # explainPhysicalQuantityXML: raw printf, no escaping
        self.p("  <PhysicalQuantities>\n")
        for pq in self.db.physical_quantities:
            self.p(f'    <PhysicalQuantity Name="{pq.name}">\n')
            if pq.description is not None:
                self.p(f"      <Description>{pq.description}</Description>\n")
            if pq.comment is not None:
                self.p(f"      <Comment>{pq.comment}</Comment>\n")
            if pq.url is not None:
                self.p(f"      <URL>{pq.url}</URL>\n")
            if pq.unit_description is not None:
                self.p(f"      <UnitDescription>{pq.unit_description}</UnitDescription>\n")
            if pq.unit is not None:
                self.p(f"      <Unit>{pq.unit}</Unit>\n")
            self.p("    </PhysicalQuantity>\n")
        self.p("  </PhysicalQuantities>\n")

    def fieldtypes(self) -> None:
        # explainFieldTypesXML: raw printf, roots only
        self.p("  <FieldTypes>\n")
        for ft in self.db.fieldtypes:
            if ft.base is not None:
                continue
            self.p(f'    <FieldType Name="{ft.name}">\n')
            if ft.description is not None:
                self.p(f"      <Description>{ft.description}</Description>\n")
            if ft.encoding_description is not None:
                self.p(f"      <EncodingDescription>{ft.encoding_description}</EncodingDescription>\n")
            if ft.comment is not None:
                self.p(f"      <Comment>{ft.comment}</Comment>\n")
            if ft.url is not None:
                self.p(f"      <URL>{ft.url}</URL>\n")
            if ft.size != 0:
                self.p(f"      <Bits>{ft.size}</Bits>\n")
            if ft.offset != 0:
                if ft.resolution in (1.0, 0.0):
                    self.p(f"      <Offset>{ft.offset}</Offset>\n")
                else:
                    self.p(f"      <Offset>{int(ft.offset * ft.resolution)}</Offset>\n")
            if ft.variable_size:
                self.p("      <VariableSize>true</VariableSize>\n")
            if ft.unit is not None:
                self.p(f"      <Unit>{ft.unit}</Unit>\n")
            if ft.has_sign is not None:
                self.p(f"      <Signed>{'true' if ft.has_sign else 'false'}</Signed>\n")
            if ft.resolution not in (1.0, 0.0):
                self.p(f"      <Resolution>{c_15g(ft.resolution)}</Resolution>\n")
            if not math.isnan(ft.range_min):
                self.p(f"      <RangeMin>{c_15g(ft.range_min)}</RangeMin>\n")
            if not math.isnan(ft.range_max):
                self.p(f"      <RangeMax>{c_15g(ft.range_max)}</RangeMax>\n")
            self.p(f"      <Sentinels>{ft.sentinels}</Sentinels>\n")
            self.p("    </FieldType>\n")
        self.p("  </FieldTypes>\n")

    def missing(self) -> None:
        self.p("  <MissingEnumerations>\n")
        for name, text in (
            ("Fields", "The list of fields is incomplete; some fields maybe be missing or their attributes may be incorrect"),
            ("FieldLengths", "The length of one or more fields is likely incorrect"),
            ("Resolution", "The resolution of one or more fields is likely incorrect"),
            ("Lookups", "One or more of the lookup fields contain missing or incorrect values"),
            ("SampleData", "The PGN has not been seen in any logfiles yet"),
            ("Interval", "The default transmission interval is not known"),
            (
                "MissingCompanyFields",
                "The manufacturer specific PGN seems to not contain the mandatory Company and Industry fields; "
                "this is likely a bug or at least incompatibility in the device",
            ),
        ):
            self.p(f'    <MissingAttribute Name="{name}">{text}</MissingAttribute>\n')
        self.p("  </MissingEnumerations>\n")

    def lookup_sections(self) -> None:
        self.p("  <LookupEnumerations>\n")
        for lk in self.db.ordered_lookups("pair"):
            max_value = (1 << lk.bits) - 1
            self.p(f"    <LookupEnumeration Name='{lk.name}' MaxValue='{max_value}'>\n")
            for value, name in lk.pairs:
                self.p(f"      <EnumPair Value='{value}' Name='{xml_escape(name)}' />\n")
            self.p("    </LookupEnumeration>\n")
        self.p("  </LookupEnumerations>\n")

        self.p("  <LookupIndirectEnumerations>\n")
        for lk in self.db.ordered_lookups("triplet"):
            max_value = (1 << lk.bits) - 1
            self.p(f"    <LookupIndirectEnumeration Name='{lk.name}' MaxValue='{max_value}'>\n")
            for v1, v2, name in lk.triplets:
                self.p(f"      <EnumTriplet Value1='{v1}' Value2='{v2}' Name='{xml_escape(name)}' />\n")
            self.p("    </LookupIndirectEnumeration>\n")
        self.p("  </LookupIndirectEnumerations>\n")

        self.p("  <LookupBitEnumerations>\n")
        for lk in self.db.ordered_lookups("bit"):
            max_value = lk.bits - 1
            self.p(f"    <LookupBitEnumeration Name='{lk.name}' MaxValue='{max_value}'>\n")
            for bit, name in lk.pairs:
                self.p(f"      <BitPair Bit='{bit}' Name='{xml_escape(name)}' />\n")
            self.p("    </LookupBitEnumeration>\n")
        self.p("  </LookupBitEnumerations>\n")

        self.p("  <LookupFieldTypeEnumerations>\n")
        for lk in self.db.ordered_lookups("fieldtype"):
            max_value = (1 << lk.bits) - 1
            self.p(f"    <LookupFieldTypeEnumeration Name='{lk.name}' MaxValue='{max_value}'>\n")
            for entry in lk.fieldtypes:
                self.enum_fieldtype(entry)
            self.p("    </LookupFieldTypeEnumeration>\n")
        self.p("  </LookupFieldTypeEnumerations>\n")

    def enum_fieldtype(self, entry) -> None:
        """explainFieldtypeXMLv2, including the Signed newline quirk."""
        ft = self.db.fieldtype(entry.fieldtype)
        self.p(f"      <EnumFieldType Value='{entry.value}' Name='{xml_escape(entry.name)}'")
        self.p(f" FieldType='{ft.root().name}'")
        if ft.has_sign is not None:
            self.p(f" Signed='{'true' if ft.has_sign else 'false'}'\n")
        if ft.resolution != 0.0:
            self.p(f" Resolution='{c_g(ft.resolution)}'")
        if ft.unit is not None:
            self.p(f" Unit='{ft.unit}'")
        bits = entry.bits if entry.bits else ft.size
        if bits != 0:
            self.p(f" Bits='{bits}'")
        if entry.lookup is not None:
            self.p(">\n")
            self.lookup_reference(entry.lookup_kind or "pair", entry.lookup, None)
            self.p("      </EnumFieldType>\n")
        else:
            self.p("/>\n")

    def lookup_reference(self, kind: str, name: str, order) -> None:
        """explainLookupFunction() v2: reference elements at indent 10."""
        if kind == "pair":
            self.xml(10, "LookupEnumeration", name)
        elif kind == "bit":
            self.xml(10, "LookupBitEnumeration", name)
        elif kind == "triplet":
            self.xml(10, "LookupIndirectEnumeration", name)
            self.xml_u(10, "LookupIndirectEnumerationFieldOrder", order)
        elif kind == "fieldtype":
            self.xml(10, "LookupFieldTypeEnumeration", name)

    # ----- PGNs -----------------------------------------------------------

    def pgn(self, pgn: Pgn) -> None:
        """Port of explainPGNXML(), v2 path."""
        self.p(f"    <PGNInfo>\n      <PGN>{pgn.pgn}</PGN>\n")
        self.xml(6, "Id", pgn.id)
        self.xml(6, "Description", pgn.description)
        if pgn.priority != 0:
            self.p(f"    <Priority>{pgn.priority}</Priority>\n")  # 4-space quirk
        self.xml(6, "Explanation", pgn.explanation)
        self.xml(6, "URL", pgn.url)
        self.xml(6, "ResearchDoc", pgn.research_doc)
        self.xml(6, "Type", pgn.type)
        self.xml(6, "Complete", "true" if pgn.is_complete() else "false")
        if pgn.fallback:
            self.xml(6, "Fallback", "true")

        missing = pgn.missing_effective()
        if missing:
            self.p("      <Missing>\n")
            for name in missing:
                self.xml(8, "MissingAttribute", name)
            self.p("      </Missing>\n")

        self.xml_u(6, "FieldCount", pgn.field_count)
        if pgn.is_variable:
            self.xml_u(6, "MinLength", pgn.length)
        else:
            self.xml_u(6, "Length", pgn.length)

        for n, rep in ((1, pgn.repeating1), (2, pgn.repeating2)):
            if rep is not None and rep.count > 0:
                self.xml_u(6, f"RepeatingFieldSet{n}Size", rep.count)
                self.xml_u(6, f"RepeatingFieldSet{n}StartField", rep.start)
                if rep.count_field is not None:
                    self.xml_u(6, f"RepeatingFieldSet{n}CountField", rep.count_field)

        if isinstance(pgn.interval, int) and pgn.interval != 0:
            self.xml_u(6, "TransmissionInterval", pgn.interval)
        if pgn.interval == "irregular":
            self.xml(6, "TransmissionIrregular", "true")

        if pgn.fields:
            self.p("      <Fields>\n")
            bit_offset = 0
            show_bit_offset = True
            for f in pgn.fields:
                bit_offset, show_bit_offset = self.field(f, bit_offset, show_bit_offset)
            self.p("      </Fields>\n")
        self.p("    </PGNInfo>\n")

    def field(self, f: Field_, bit_offset: int, show_bit_offset: bool):
        ft = f.ft
        lookup_ref = f.lookup_ref()
        self.p(f"        <Field>\n          <Order>{f.order}</Order>\n")
        self.xml(10, "Id", f.id)
        self.xml(10, "Name", f.name)

        if f.description is not None and f.description != "" and not f.description.startswith(","):
            self.xml(10, "Description", f.description)
        elif f.match is not None and lookup_ref is not None:
            # filterPair: the description of a match field is the lookup name
            name = self.match_description(f, lookup_ref)
            self.p(f"          <Description>{name}</Description>\n")

        if f.res_bits == 0:
            self.p("          <BitLengthVariable>true</BitLengthVariable>\n")
            if f.type == "BINARY":
                self.xml_u(10, "BitLengthField", f.order - 1)
        else:
            self.xml_u(10, "BitLength", f.res_bits)
        if show_bit_offset:
            self.xml_u(10, "BitOffset", bit_offset)
            self.xml_u(10, "BitStart", bit_offset % 8)
        bit_offset += f.res_bits

        if f.proprietary:
            self.xml(10, "Condition", "PGNIsProprietary")
        if f.match is not None:
            self.xml(10, "Match", str(f.match))
        else:
            self.xml(10, "Unit", f.res_unit)

        if f.res_resolution != 0.0:
            self.p(f"          <Resolution>{c_g(f.res_resolution)}</Resolution>\n")
        if ft.has_sign is not None:
            self.p(f"          <Signed>{'true' if ft.has_sign else 'false'}</Signed>\n")
        if f.res_offset != 0:
            if f.res_resolution in (1.0, 0.0):
                self.p(f"          <Offset>{f.res_offset}</Offset>\n")
            else:
                self.p(f"          <Offset>{int(f.res_offset * f.res_resolution)}</Offset>\n")

        if f.dynamic_field_length and f.dynamic_field_length_overhead != 0:
            self.p(
                f"          <DynamicFieldLengthOverhead>{f.dynamic_field_length_overhead}"
                "</DynamicFieldLengthOverhead>\n"
            )

        is_match = f.match is not None
        if not math.isnan(f.res_range_min):
            self.p(f"          <RangeMin>{c_15g(f.res_range_min)}</RangeMin>\n")
        elif lookup_ref is not None and not is_match:
            self.p(f"          <RangeMin>{c_15g(0.0)}</RangeMin>\n")

        if not math.isnan(f.res_range_max):
            if f.res_resolution == 1.0 and f.res_bits == 64 and ft.has_sign is False and f.res_offset == 0:
                self.p(f"          <RangeMax>{UINT64_MAX}</RangeMax>\n")
            else:
                self.p(f"          <RangeMax>{c_15g(f.res_range_max)}</RangeMax>\n")
        elif lookup_ref is not None and not is_match:
            self.p(f"          <RangeMax>{c_15g(float((1 << f.res_bits) - 1))}</RangeMax>\n")

        if (
            f.reserved_count > 0
            and f.res_bits < 64
            and not math.isnan(f.res_range_min)
            and not is_match
            and sentinels_for_fieldtype(ft) == "TopOfRange"
        ):
            highbit = f.res_bits - 1 if (ft.has_sign is True and f.res_offset == 0) else f.res_bits
            raw = (1 << highbit) - 1
            self.p(f"          <UnknownValue>{raw}</UnknownValue>\n")
            if f.reserved_count >= 2:
                self.p(f"          <OutOfRangeValue>{raw - 1}</OutOfRangeValue>\n")
            if f.reserved_count >= 3:
                self.p(f"          <ReservedValue>{raw - 2}</ReservedValue>\n")

        self.xml(10, "FieldType", ft.root().name)
        if ft.physical is not None:
            self.xml(10, "PhysicalQuantity", ft.physical)

        if lookup_ref is not None:
            kind, name = lookup_ref
            self.lookup_reference(kind, name, f.lookup_indirect_order)

        if f.primary_key:
            self.xml(10, "PartOfPrimaryKey", "true")

        if ft.variable_size or f.proprietary or f.res_bits == 0:
            show_bit_offset = False
        self.p("        </Field>\n")
        return bit_offset, show_bit_offset

    def match_description(self, f: Field_, lookup_ref) -> str:
        kind, name = lookup_ref
        lk = self.db.lookups.get(name)
        if lk is None:
            return ""
        match = f.match if isinstance(f.match, int) else int(str(f.match), 0)
        if kind == "fieldtype":
            return "".join(e.name for e in lk.fieldtypes if e.value == match)
        return "".join(n for v, n in lk.pairs if v == match)

    # ----- top level ------------------------------------------------------

    def emit(self, which: str = "normal") -> str:
        self.header()
        if which == "normal":
            self.physical_quantities()
            self.fieldtypes()
            self.missing()
            self.lookup_sections()
        self.p("  <PGNs>\n")
        for pgn in self.db.pgns:
            if which == "normal" and pgn.pgn < ACTISENSE_BEM:
                self.pgn(pgn)
            elif which == "actisense" and ACTISENSE_BEM <= pgn.pgn < IKONVERT_BEM:
                self.pgn(pgn)
            elif which == "ikonvert" and pgn.pgn >= IKONVERT_BEM:
                self.pgn(pgn)
        self.p("  </PGNs>\n</PGNDefinitions>\n")
        return "".join(self.out)


def emit_xml(db: Database, which: str = "normal") -> str:
    return Emitter(db).emit(which)
