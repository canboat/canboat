"""Reader for the current docs/canboat.xml (bootstrap only).

Produces raw structures for convert.py: model objects for the sections that
map 1:1 (physical quantities, lookups) and ElementTree elements plus exact
source slices for the PGNs, so the converter can verify its authored model
reproduces each <PGNInfo> block byte-for-byte.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass

from .model import FieldTypeLookupEntry, Lookup, PhysicalQuantity


@dataclass
class RawXml:
    version: str
    schema_version: str
    physical_quantities: list
    lookups: dict  # name -> Lookup
    lookup_order: dict  # kind -> [names]
    pgn_elements: list  # [(ET element, source slice text)]
    fieldtypes_slice: str  # exact text of the <FieldTypes> section
    lookup_fieldtype_raw: dict  # name -> [dict of EnumFieldType attrs + children]


def _text(elem, tag):
    child = elem.find(tag)
    return child.text if child is not None else None


def parse(path: str) -> RawXml:
    with open(path, encoding="utf-8") as f:
        source = f.read()
    root = ET.fromstring(source)

    version = root.findtext("Version")
    schema_version = root.findtext("SchemaVersion")

    pqs = []
    for pq in root.find("PhysicalQuantities"):
        pqs.append(
            PhysicalQuantity(
                name=pq.get("Name"),
                description=_text(pq, "Description"),
                comment=_text(pq, "Comment"),
                url=_text(pq, "URL"),
                unit_description=_text(pq, "UnitDescription"),
                unit=_text(pq, "Unit"),
            )
        )

    lookups = {}
    lookup_order = {"pair": [], "triplet": [], "bit": [], "fieldtype": []}
    lookup_fieldtype_raw = {}

    for lk in root.find("LookupEnumerations"):
        name = lk.get("Name")
        max_value = int(lk.get("MaxValue"))
        lookups[name] = Lookup(
            name=name,
            kind="pair",
            bits=(max_value + 1).bit_length() - 1,
            pairs=[(int(e.get("Value")), e.get("Name")) for e in lk],
        )
        lookup_order["pair"].append(name)

    for lk in root.find("LookupIndirectEnumerations"):
        name = lk.get("Name")
        max_value = int(lk.get("MaxValue"))
        lookups[name] = Lookup(
            name=name,
            kind="triplet",
            bits=(max_value + 1).bit_length() - 1,
            triplets=[(int(e.get("Value1")), int(e.get("Value2")), e.get("Name")) for e in lk],
        )
        lookup_order["triplet"].append(name)

    for lk in root.find("LookupBitEnumerations"):
        name = lk.get("Name")
        max_value = int(lk.get("MaxValue"))
        lookups[name] = Lookup(
            name=name,
            kind="bit",
            bits=max_value + 1,
            pairs=[(int(e.get("Bit")), e.get("Name")) for e in lk],
        )
        lookup_order["bit"].append(name)

    for lk in root.find("LookupFieldTypeEnumerations"):
        name = lk.get("Name")
        max_value = int(lk.get("MaxValue"))
        entries_raw = []
        for e in lk:
            nested = None
            nested_kind = None
            for child_tag, kind in (
                ("LookupEnumeration", "pair"),
                ("LookupBitEnumeration", "bit"),
                ("LookupIndirectEnumeration", "triplet"),
                ("LookupFieldTypeEnumeration", "fieldtype"),
            ):
                sub = e.find(child_tag)
                if sub is not None:
                    nested, nested_kind = sub.text, kind
            entries_raw.append(
                {
                    "value": int(e.get("Value")),
                    "name": e.get("Name"),
                    "root": e.get("FieldType"),
                    "signed": e.get("Signed"),
                    "resolution": e.get("Resolution"),
                    "unit": e.get("Unit"),
                    "bits": int(e.get("Bits")) if e.get("Bits") else None,
                    "lookup": nested,
                    "lookup_kind": nested_kind,
                }
            )
        # convert.py resolves the specific fieldtype per entry
        lookups[name] = Lookup(name=name, kind="fieldtype", bits=(max_value + 1).bit_length() - 1)
        lookup_order["fieldtype"].append(name)
        lookup_fieldtype_raw[name] = entries_raw

    # PGNs: pair each element with its exact source block for verification
    slices = source.split("    <PGNInfo>\n")[1:]
    blocks = ["    <PGNInfo>\n" + s.split("    </PGNInfo>\n")[0] + "    </PGNInfo>\n" for s in slices]
    pgn_elems = list(root.find("PGNs"))
    if len(pgn_elems) != len(blocks):
        raise ValueError(f"PGNInfo count mismatch: {len(pgn_elems)} elements vs {len(blocks)} source blocks")

    ft_start = source.index("  <FieldTypes>\n")
    ft_end = source.index("  </FieldTypes>\n") + len("  </FieldTypes>\n")

    return RawXml(
        version=version,
        schema_version=schema_version,
        physical_quantities=pqs,
        lookups=lookups,
        lookup_order=lookup_order,
        pgn_elements=list(zip(pgn_elems, blocks)),
        fieldtypes_slice=source[ft_start:ft_end],
        lookup_fieldtype_raw=lookup_fieldtype_raw,
    )


def parse_field_element(fe) -> dict:
    """Flatten one <Field> element into a dict of raw values."""
    d = {tag: fe.findtext(tag) for tag in (
        "Order",
        "Id",
        "Name",
        "Description",
        "BitLength",
        "BitLengthVariable",
        "BitLengthField",
        "BitOffset",
        "BitStart",
        "Condition",
        "Match",
        "Unit",
        "Resolution",
        "Signed",
        "Offset",
        "DynamicFieldLengthOverhead",
        "RangeMin",
        "RangeMax",
        "UnknownValue",
        "OutOfRangeValue",
        "ReservedValue",
        "FieldType",
        "PhysicalQuantity",
        "LookupEnumeration",
        "LookupBitEnumeration",
        "LookupIndirectEnumeration",
        "LookupIndirectEnumerationFieldOrder",
        "LookupFieldTypeEnumeration",
        "PartOfPrimaryKey",
    )}
    return d
