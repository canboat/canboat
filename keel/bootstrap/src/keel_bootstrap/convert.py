"""Bootstrap converter: docs/canboat.xml + analyzer/fieldtype.h -> database/ YAML.

Migration step 1 (DESIGN.md §9). The converter builds an *authored-data-only*
model (DESIGN.md §3.1): everything derive.py can recompute is stripped, and a
reconciliation pass adds explicit authored overrides wherever the recomputed
value does not reproduce the XML byte-for-byte. The final gate is the golden
test: emit_xml(model) must equal the input document exactly.

Retired after migration step 5.
"""

from __future__ import annotations

import math
import sys
import xml.etree.ElementTree as ET

from . import cheader, derive, emit_xml, xmlread
from .cformat import c_15g, c_g
from .model import Database, Field_, FieldTypeLookupEntry, Pgn, Repeating

LOOKUP_KEYS = {
    "LookupEnumeration": "lookup",
    "LookupBitEnumeration": "lookup_bits",
    "LookupIndirectEnumeration": "lookup_indirect",
    "LookupFieldTypeEnumeration": "lookup_fieldtype",
}


class ConvertError(Exception):
    pass


def _tri(signed_text):
    return None if signed_text is None else signed_text == "true"


class FieldTypePicker:
    """Reverse-maps the flattened XML view (root type + attributes) back to
    the most specific derived fieldtype from fieldtype.h."""

    def __init__(self, fieldtypes: list):
        self.fieldtypes = fieldtypes
        self.fieldtypes_by_name = {ft.name: ft for ft in fieldtypes}

    def pick(
        self,
        root: str,
        signed,
        physical,
        bits,  # None = variable/absent
        resolution_text,  # raw XML text: %g of the resolution, or None
        unit,
        offset: int,
        is_match: bool,
        constrain_physical: bool = True,
    ):
        best = None
        best_score = -1
        for ft in self.fieldtypes:
            if ft.root().name != root:
                continue
            if ft.has_sign != signed:
                continue
            if constrain_physical and (ft.physical or None) != physical:
                continue
            if bits is None:
                if ft.size != 0:
                    continue
            elif ft.size not in (0, bits):
                continue
            # Resolutions compare as %g text: the XML is %g-lossy (e.g.
            # 1/11 prints as 0.0909091), and matching the exact double from
            # fieldtype.h is what makes the derived ranges bit-exact.
            if ft.resolution != 0.0 and c_g(ft.resolution) != resolution_text:
                continue
            if not is_match:
                if ft.unit is not None and ft.unit != unit:
                    continue
                if ft.unit is None and unit is not None and ft.root().name != root:
                    continue
            if ft.offset != 0:
                # xml Offset prints (int64)(offset * res) when res not in
                # (0, 1); compare in printed form (the raw offset is only
                # recoverable after the type is known)
                res = ft.resolution if ft.resolution != 0.0 else resolution
                printed = ft.offset if res in (0.0, 1.0) else int(ft.offset * res)
                if printed != offset:
                    continue
            score = (
                (ft.size != 0)
                + (ft.resolution != 0.0)
                + (ft.unit is not None)
                + (ft.offset != 0)
                + (ft.physical is not None)
            )
            if score > best_score:
                best, best_score = ft, score
        return best


def build_field(picker, db: Database, fe, pgn_id: str) -> Field_:
    raw = xmlread.parse_field_element(fe)
    is_match = raw["Match"] is not None
    variable = raw["BitLengthVariable"] == "true"
    bits = None if variable else int(raw["BitLength"])
    resolution_text = raw["Resolution"]
    resolution = float(resolution_text) if resolution_text is not None else 0.0
    signed = _tri(raw["Signed"])
    unit = raw["Unit"]
    xml_offset = int(raw["Offset"]) if raw["Offset"] is not None else 0

    root = raw["FieldType"]
    has_lookup = any(raw[tag] is not None for tag in LOOKUP_KEYS)
    if has_lookup:
        # Lookup fields are authored as their root type + explicit bits:
        # that is what the pgn.h macros do (LOOKUP_FIELD(name, BITS(n), ENUM)),
        # and a derived type pick (e.g. INDUSTRY) would be a guess.
        ft = picker.fieldtypes_by_name.get(root)
    else:
        ft = picker.pick(root, signed, raw["PhysicalQuantity"], bits, resolution_text, unit, xml_offset, is_match)
    if ft is None:
        raise ConvertError(
            f"{pgn_id} field '{raw['Name']}': no fieldtype matches root={root} signed={signed} "
            f"bits={bits} res={resolution} unit={unit} physical={raw['PhysicalQuantity']}"
        )

    # Offsets live only on fieldtypes: the C aborts on any field-level
    # offset differing from its type (fieldtype.c:384), so the picker must
    # have found an offset-bearing type.
    if xml_offset != 0 and ft.offset == 0:
        raise ConvertError(
            f"{pgn_id} field '{raw['Name']}': offset {xml_offset} but no fieldtype carries it"
        )

    f = Field_(
        id=raw["Id"],
        name=raw["Name"],
        type=ft.name,
        description=raw["Description"],
        primary_key=raw["PartOfPrimaryKey"] == "true",
        proprietary=raw["Condition"] == "PGNIsProprietary",
    )
    if is_match:
        text = raw["Match"]
        f.match = int(text) if text.lstrip("-").isdigit() else text
    if bits is not None and ft.size == 0:
        f.bits = bits
    if ft.resolution == 0.0 and resolution != derive.default_field_resolution_for(ft):
        f.resolution = resolution
    if not is_match and unit is not None and ft.unit is None:
        f.unit = unit
    if raw["DynamicFieldLengthOverhead"] is not None:
        f.dynamic_field_length = True
        f.dynamic_field_length_overhead = int(raw["DynamicFieldLengthOverhead"])
    elif ft.root().name == "DYNAMIC_FIELD_LENGTH":
        f.dynamic_field_length = True

    for tag, attr in LOOKUP_KEYS.items():
        if raw[tag] is not None:
            setattr(f, attr, raw[tag])
    if raw["LookupIndirectEnumerationFieldOrder"] is not None:
        f.lookup_indirect_order = int(raw["LookupIndirectEnumerationFieldOrder"])
    return f


def build_pgn(picker, db: Database, elem) -> Pgn:
    pgn = Pgn(
        pgn=int(elem.findtext("PGN")),
        id=elem.findtext("Id"),
        description=elem.findtext("Description"),
        type=elem.findtext("Type"),
        priority=int(elem.findtext("Priority") or 0),
        explanation=elem.findtext("Explanation"),
        url=elem.findtext("URL"),
        research_doc=elem.findtext("ResearchDoc"),
        fallback=elem.findtext("Fallback") == "true",
    )
    missing_elem = elem.find("Missing")
    if missing_elem is not None:
        pgn.missing = [m.text for m in missing_elem]

    ti = elem.findtext("TransmissionInterval")
    if ti is not None:
        pgn.interval = int(ti)
    elif elem.findtext("TransmissionIrregular") == "true":
        pgn.interval = "irregular"
    # else None: unknown; <Missing> then contains the automatic Interval entry.
    # Drop that entry from the authored list (derived, model.missing_effective()).
    if pgn.interval is None:
        pgn.missing = [m for m in pgn.missing if m != "Interval"]

    for n in (1, 2):
        size = elem.findtext(f"RepeatingFieldSet{n}Size")
        if size is not None:
            rep = Repeating(
                count=int(size),
                start=int(elem.findtext(f"RepeatingFieldSet{n}StartField")),
                count_field=(
                    int(elem.findtext(f"RepeatingFieldSet{n}CountField"))
                    if elem.findtext(f"RepeatingFieldSet{n}CountField") is not None
                    else None
                ),
            )
            setattr(pgn, f"repeating{n}", rep)

    fields_elem = elem.find("Fields")
    if fields_elem is not None:
        for fe in fields_elem:
            pgn.fields.append(build_field(picker, db, fe, pgn.id))
    return pgn


def build_fieldtype_lookup_entries(picker, raw_entries, lookup_name: str) -> list:
    entries = []
    for r in raw_entries:
        ft = picker.pick(
            r["root"], _tri(r["signed"]), None, r["bits"], r["resolution"], r["unit"], 0, False,
            constrain_physical=False,
        )
        if ft is None:
            raise ConvertError(
                f"lookup {lookup_name} value {r['value']}: no fieldtype matches root={r['root']} "
                f"signed={r['signed']} res={r['resolution']} unit={r['unit']} bits={r['bits']}"
            )
        entry = FieldTypeLookupEntry(value=r["value"], name=r["name"], fieldtype=ft.name)
        if r["bits"] is not None and ft.size != r["bits"]:
            entry.bits = r["bits"]
        if r["lookup"] is not None:
            entry.lookup = r["lookup"]
            entry.lookup_kind = r["lookup_kind"]
        entries.append(entry)
    return entries


# ---------------------------------------------------------------------------
# Reconciliation: author explicit overrides where derivation cannot reproduce
# the XML (explicit C initializers, %g-lossy resolutions, SPECIAL_VALUES ...).
# ---------------------------------------------------------------------------

SENTINEL_TAGS = ("UnknownValue", "OutOfRangeValue", "ReservedValue")


def reconcile_pgn(db: Database, pgn: Pgn, original_block: str) -> bool:
    """Compare the emitted block against the original; author overrides for
    the differences derive.py legitimately cannot know. Returns True when
    the block now reproduces exactly."""
    for _ in range(4):
        emitted = _emit_block(db, pgn)
        if emitted == original_block:
            return True
        _author_overrides(db, pgn, original_block)
        derive.fill(db)
    return _emit_block(db, pgn) == original_block


def _emit_block(db: Database, pgn: Pgn) -> str:
    em = emit_xml.Emitter(db)
    em.pgn(pgn)
    return "".join(em.out)


def _author_overrides(db: Database, pgn: Pgn, original_block: str) -> None:
    orig = ET.fromstring(original_block)
    orig_fields = orig.find("Fields")
    orig_fields = list(orig_fields) if orig_fields is not None else []
    if len(orig_fields) != len(pgn.fields):
        raise ConvertError(f"{pgn.id}: field count mismatch during reconcile")

    for f, fe in zip(pgn.fields, orig_fields):
        raw = xmlread.parse_field_element(fe)

        # Ranges: explicit C initializers / %g-lossy derivations
        want_min = float(raw["RangeMin"]) if raw["RangeMin"] is not None else math.nan
        want_max = float(raw["RangeMax"]) if raw["RangeMax"] is not None else math.nan
        have_min_txt = None if math.isnan(f.res_range_min) else c_15g(f.res_range_min)
        have_max_txt = None if math.isnan(f.res_range_max) else c_15g(f.res_range_max)
        if (raw["RangeMin"] or None) != have_min_txt or (raw["RangeMax"] or None) != have_max_txt:
            lookup_else = f.lookup_ref() is not None and f.match is None
            if not (lookup_else and raw["RangeMin"] is not None and math.isnan(f.res_range_min)):
                f.range_min = want_min
                f.range_max = want_max

        # Sentinels: SPECIAL_VALUES overrides
        want_count = sum(1 for tag in SENTINEL_TAGS if raw[tag] is not None)
        have_count = (
            f.reserved_count
            if (
                f.reserved_count > 0
                and f.res_bits < 64
                and not math.isnan(f.res_range_min)
                and f.match is None
                and derive.sentinels_for_fieldtype(f.ft) == "TopOfRange"
            )
            else 0
        )
        if want_count != have_count and f.special_values is None:
            f.special_values = want_count


def convert(xml_path: str, fieldtype_header: str, verbose: bool = False, bem_paths: dict = None):
    """Returns (db, failures): the authored Database and a list of PGN ids
    whose blocks could not be reproduced (for iterative refinement).

    bem_paths: optional {"actisense": path, "ikonvert": path} pointing at
    -explain-ngt-xml / -explain-ik-xml output; the BEM pseudo-PGNs live in
    pgnList (the analyzer decodes them at runtime) but not in canboat.xml.
    """
    import copy

    raw = xmlread.parse(xml_path)
    fieldtypes = cheader.parse_fieldtypes(fieldtype_header)
    authored_fieldtypes = copy.deepcopy(fieldtypes)  # derive.fill() percolates in place

    db = Database(
        physical_quantities=raw.physical_quantities,
        fieldtypes=fieldtypes,
        lookups=raw.lookups,
        lookup_order=raw.lookup_order,
        version=raw.version,
        schema_version=raw.schema_version,
    )
    db.authored_fieldtypes = authored_fieldtypes  # what yamlio writes to fieldtypes.yaml
    db.index()
    # The picker matches against *resolved* fieldtype attributes (the XML
    # flattens each field to its root type + resolved attrs), so percolate
    # base types and physical quantities first.
    derive.fill_fieldtypes(db)
    picker = FieldTypePicker(fieldtypes)

    for name, entries_raw in raw.lookup_fieldtype_raw.items():
        db.lookups[name].fieldtypes = build_fieldtype_lookup_entries(picker, entries_raw, name)

    for elem, _block in raw.pgn_elements:
        db.pgns.append(build_pgn(picker, db, elem))

    # Actisense/iKonvert BEM pseudo-PGNs, from their own XML documents
    bem_blocks = []  # [(pgn, block)] joins the reconcile pass below
    for which, path in (bem_paths or {}).items():
        bem_raw = xmlread.parse(path)
        for elem, block in bem_raw.pgn_elements:
            pgn = build_pgn(picker, db, elem)
            db.pgns.append(pgn)
            bem_blocks.append((pgn, block))

    # Variant precedence: when one PGN number has multiple entries, their
    # order is semantic - runtime matching precedence (pgn.c getMatchingPgn)
    # - so it is authored. Fallback entries take part: their position in the
    # group is pgnList authored order (0xE800 sits first in 59392, but the
    # 0x1EF00 fallback sits last in 126720).
    by_pgn = {}
    for p in db.pgns:
        by_pgn.setdefault(p.pgn, []).append(p)
    for group in by_pgn.values():
        if len(group) > 1:
            for n, p in enumerate(group, start=1):
                p.variant_order = n

    derive.fill(db)

    # R08 opt-in: the inherited pgn.h data uses lookups at differing widths
    # (YES_NO in 1-bit flags, DISABLED_SATELLITES shared over 32/24 bits).
    # Mark those fields explicitly so `keel check` treats any NEW mismatch
    # as an error (FINDINGS.md F2).
    for pgn in db.pgns:
        for f in pgn.fields:
            ref = f.lookup_ref()
            if ref is not None:
                lk = db.lookups.get(ref[1])
                if lk is not None and f.res_bits != lk.bits:
                    f.allow_lookup_width_mismatch = True

    failures = []
    normal_pairs = list(zip(db.pgns, raw.pgn_elements))
    all_blocks = [(pgn, block) for pgn, (_elem, block) in normal_pairs] + bem_blocks
    for pgn, block in all_blocks:
        if not reconcile_pgn(db, pgn, block):
            failures.append(pgn.id)
            if verbose:
                _report_block_diff(db, pgn, block)

    return db, failures


def _report_block_diff(db: Database, pgn: Pgn, original_block: str) -> None:
    import difflib

    emitted = _emit_block(db, pgn)
    diff = difflib.unified_diff(
        original_block.splitlines(keepends=True),
        emitted.splitlines(keepends=True),
        fromfile=f"canboat.xml:{pgn.id}",
        tofile="emitted",
    )
    sys.stderr.writelines(diff)
