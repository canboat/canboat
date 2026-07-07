"""The in-memory PGN database model.

Authored attributes mirror what the YAML files store (decisions only);
derived attributes (marked "filled by derive.py") are computed exactly like
the C analyzer computes them, so the XML emitter can reproduce
analyzer-explain.c's output.

Tri-state booleans (C type `Bool`: True/False/Null) are represented as
True / False / None.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Union

NAN = math.nan

# PacketType strings, in enum order (pgn.h PACKET_TYPE_STR)
PACKET_TYPES = ("Single", "Fast", "ISO", "Mixed")

# <MissingAttribute> names in emission order (analyzer-explain.c explainPGNXML)
MISSING_ATTRIBUTES = (
    "Fields",
    "FieldLengths",
    "Resolution",
    "Lookups",
    "SampleData",
    "Interval",
    "MissingCompanyFields",
)

# Sentinel conventions (fieldtype.h Sentinels enum, names per sentinelsName())
SENTINELS = ("None", "TopOfRange", "NaN", "EmptyString", "Variable")

ACTISENSE_BEM = 0x40000
IKONVERT_BEM = 0x40100

UINT16_MAX = 0xFFFF


@dataclass
class PhysicalQuantity:
    name: str
    description: Optional[str] = None
    comment: Optional[str] = None
    url: Optional[str] = None
    unit_description: Optional[str] = None  # <UnitDescription>, C .unit
    unit: Optional[str] = None  # <Unit>, C .abbreviation


@dataclass
class FieldType:
    name: str
    description: Optional[str] = None
    encoding_description: Optional[str] = None
    comment: Optional[str] = None
    url: Optional[str] = None
    size: int = 0  # bits; 0 = per-field
    variable_size: bool = False
    base: Optional[str] = None  # baseFieldType name
    unit: Optional[str] = None
    offset: int = 0
    resolution: float = 0.0
    has_sign: Optional[bool] = None  # tri-state
    sentinels: str = "None"
    physical: Optional[str] = None  # PhysicalQuantity name
    print_function: Optional[str] = None  # C fieldPrint* symbol (for fieldtype.h generation)
    # Explicit .rangeMin/.rangeMax initializers exist in fieldtype.h but are
    # functionally inert: fillFieldType()'s `rangeMax == 0.0` guard sends any
    # explicit value to the NaN branch. Kept only to regenerate fieldtype.h
    # faithfully; derive.py ignores them, exactly like the C does.
    range_min_authored: Optional[float] = None
    range_max_authored: Optional[float] = None

    # Filled by derive.fill_fieldtypes()
    base_ptr: Optional["FieldType"] = None
    range_min: float = NAN
    range_max: float = NAN

    def root(self) -> "FieldType":
        ft = self
        while ft.base_ptr is not None:
            ft = ft.base_ptr
        return ft


@dataclass
class FieldTypeLookupEntry:
    """One EnumFieldType row of a LookupFieldTypeEnumeration."""

    value: int
    name: str
    fieldtype: str  # specific FieldType name
    bits: Optional[int] = None  # authored override of the fieldtype's size
    lookup: Optional[str] = None  # nested lookup enumeration name (pair)
    lookup_kind: Optional[str] = None  # 'pair' | 'bit' | ... when lookup set


@dataclass
class Lookup:
    name: str
    kind: str  # 'pair' | 'triplet' | 'bit' | 'fieldtype'
    bits: int  # field width (pair/triplet/fieldtype: MaxValue = 2^bits-1;
    #            bit: MaxValue = bits-1, matching lookup.h LOOKUP_TYPE_BITFIELD)
    pairs: list = field(default_factory=list)  # [(value, name)] / [(bit, name)]
    triplets: list = field(default_factory=list)  # [(v1, v2, name)]
    fieldtypes: list = field(default_factory=list)  # [FieldTypeLookupEntry]


@dataclass
class Field_:
    """A PGN field. Suffix avoids clashing with dataclasses.field."""

    # --- authored ---
    id: str
    name: str
    type: str  # specific FieldType name
    bits: Optional[int] = None  # required iff fieldtype has no size and not variable
    resolution: Optional[float] = None
    unit: Optional[str] = None
    offset: Optional[int] = None
    description: Optional[str] = None
    match: Optional[int] = None
    lookup: Optional[str] = None  # pair enumeration
    lookup_indirect: Optional[str] = None  # triplet enumeration
    lookup_indirect_order: Optional[int] = None  # val1Order: 1-based order of the first key field
    lookup_bits: Optional[str] = None  # bit enumeration
    lookup_fieldtype: Optional[str] = None  # fieldtype enumeration
    primary_key: bool = False
    proprietary: bool = False  # field only present when PGN is in proprietary range
    allow_lookup_width_mismatch: bool = False  # R08 opt-in (FINDINGS.md F2)
    special_values: Optional[int] = None  # SPECIAL_VALUES override (reservedOverride - 1)
    dynamic_field_length: bool = False
    dynamic_field_length_overhead: int = 0
    range_min: Optional[float] = None  # authored override ("all values valid" idiom etc.)
    range_max: Optional[float] = None

    # --- filled by derive.fill_pgn() ---
    ft: Optional[FieldType] = None
    res_bits: int = 0  # resolved size in bits; 0 = variable
    res_resolution: float = 0.0
    res_unit: Optional[str] = None
    res_offset: int = 0
    res_range_min: float = NAN
    res_range_max: float = NAN
    reserved_count: int = 0
    order: int = 0

    def lookup_ref(self):
        """(kind, name) of whichever lookup this field references, or None."""
        if self.lookup is not None:
            return ("pair", self.lookup)
        if self.lookup_indirect is not None:
            return ("triplet", self.lookup_indirect)
        if self.lookup_bits is not None:
            return ("bit", self.lookup_bits)
        if self.lookup_fieldtype is not None:
            return ("fieldtype", self.lookup_fieldtype)
        return None


@dataclass
class Repeating:
    count: int  # how many fields repeat
    start: int  # 1-based order of the first repeating field
    count_field: Optional[int] = None  # 1-based order of the count field; None = until exhausted


@dataclass
class Pgn:
    # --- authored ---
    pgn: int
    id: str
    description: str
    type: str  # 'Single' | 'Fast' | 'ISO' | 'Mixed'
    fields: list = field(default_factory=list)  # [Field_]
    priority: int = 0
    interval: Union[int, str, None] = None  # ms | 'irregular' | None (unknown)
    explanation: Optional[str] = None
    url: Optional[str] = None
    research_doc: Optional[str] = None
    fallback: bool = False
    missing: list = field(default_factory=list)  # MISSING_ATTRIBUTES names, sans auto-Interval
    repeating1: Optional[Repeating] = None
    repeating2: Optional[Repeating] = None
    variant_order: int = 0  # precedence among same-PGN variants (authored when > 1 variant)

    # --- filled by derive.fill_pgn() ---
    field_count: int = 0
    length: int = 0  # bytes up to first repeating field
    is_variable: bool = False

    def missing_effective(self) -> list:
        """The <Missing> list as emitted: authored plus the automatic
        Interval attribute when the interval is unknown (fieldtype.c:486)."""
        result = [m for m in self.missing if m != "Interval"]
        if self.interval is None:
            result.append("Interval")
        return [m for m in MISSING_ATTRIBUTES if m in result]

    def is_complete(self) -> bool:
        return not self.missing_effective()


@dataclass
class Database:
    physical_quantities: list = field(default_factory=list)  # ordered
    fieldtypes: list = field(default_factory=list)  # ordered, roots and derived
    lookups: dict = field(default_factory=dict)  # name -> Lookup
    lookup_order: dict = field(default_factory=dict)  # kind -> [names] (emission order)
    pgns: list = field(default_factory=list)  # ordered as emitted
    version: str = ""  # CANboat VERSION
    schema_version: str = ""  # SCHEMA_VERSION

    def fieldtype(self, name: str) -> FieldType:
        ft = self._ft_index.get(name)
        if ft is None:
            raise KeyError(f"fieldType '{name}' not found")
        return ft

    def index(self) -> None:
        self._ft_index = {ft.name: ft for ft in self.fieldtypes}

    def ordered_lookups(self, kind: str) -> list:
        return [self.lookups[n] for n in self.lookup_order.get(kind, [])]
