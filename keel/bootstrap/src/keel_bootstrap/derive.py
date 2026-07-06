"""Derivation of computed attributes - the Python port of the C analyzer's
fill logic. Source of truth for semantics:

  - fieldtype.c fillFieldType()            (inheritance, ranges, reservedCount)
  - fieldtype.c getMinRange()/getMaxRange()/fixupUnit()/reservedCountForSize()
  - analyzer-explain.c getMinimalPgnLength()

The port is deliberately line-faithful, including C quirks:
  - min()/max() macros: `x <= y ? x : y` - a NaN comparison is false, so
    clamping a NaN range yields the clamp value. Python's NaN comparisons
    behave identically, so the direct translation matches bit-for-bit.
  - uint64 arithmetic converted to double exactly like C does.

The migration golden test (emitted XML == analyzer-explain output) is what
proves this port correct across the whole database.
"""

from __future__ import annotations

import math

from .model import NAN, Database, Field_, Lookup, Pgn

UINT64_MAX = (1 << 64) - 1


def c_min(x: float, y: float) -> float:
    return x if x <= y else y  # analyzer.h: ((x) <= (y) ? (x) : (y))


def c_max(x: float, y: float) -> float:
    return x if x >= y else y


def reserved_count_for_size(size: int) -> int:
    """fieldtype.c reservedCountForSize(): top-of-range sentinels by width."""
    return 3 if size >= 8 else 2 if size >= 4 else 1 if size >= 2 else 0


def get_min_range(size: int, resolution: float, sign: bool, offset: int) -> float:
    highbit = size - 1 if (sign and offset == 0) else size
    if not sign or offset != 0:
        return (0 + offset) * resolution
    return ((1 << highbit) - 1) * resolution * -1.0


def get_max_range(
    size: int,
    resolution: float,
    sign: bool,
    offset: int,
    lookup,  # Optional[Lookup]; pair kind only (fieldtype.c EXPLAIN block)
    specialvalues: int,
) -> float:
    highbit = size - 1 if (sign and offset == 0) else size
    max_value = (UINT64_MAX if highbit >= 64 else (1 << highbit) - 1) - specialvalues
    if offset != 0:
        max_value += offset
    if lookup is not None and lookup.kind == "pair" and lookup.pairs:
        # A lookup naming values in the sentinel region raises rangeMax
        max_value = max(max_value, max(v for v, _ in lookup.pairs))
    return max_value * resolution


# Field types (by ROOT name) whose field macros set no resolution at all;
# every other field macro in pgn.h sets an explicit resolution (usually 1).
# Empirically verified by the converter: any field that deviates gets an
# explicit authored `resolution:` in its YAML.
NO_DEFAULT_RESOLUTION_ROOTS = {
    "STRING_FIX",
    "STRING_LZ",
    "STRING_LAU",
    "VARIABLE",
    "DYNAMIC_FIELD_VALUE",
}


def default_field_resolution_for(ft) -> float:
    """The resolution a pgn.h field macro sets when the YAML doesn't author
    one: 1 everywhere except the string/variable types whose macros set none."""
    return 0.0 if ft.root().name in NO_DEFAULT_RESOLUTION_ROOTS else 1.0


def default_field_resolution(db: Database, type_name: str) -> float:
    return default_field_resolution_for(db.fieldtype(type_name))


def fill_fieldtypes(db: Database) -> None:
    """Port of fieldtype.c fillFieldType() part 1: percolate physical
    quantities and base types, compute type-level ranges."""
    pq_by_name = {pq.name: pq for pq in db.physical_quantities}

    for ft in db.fieldtypes:
        if ft.physical is not None:
            pq = pq_by_name.get(ft.physical)
            if pq is None:
                raise ValueError(f"FieldType '{ft.name}' has unlisted physical quantity '{ft.physical}'")
            if ft.unit is None:
                ft.unit = pq.unit
            if ft.url is None:
                ft.url = pq.url
        elif ft.unit is not None:
            raise ValueError(f"FieldType '{ft.name}' has unit '{ft.unit}' but no physical quantity")

    seen = {}
    for ft in db.fieldtypes:
        if ft.base is not None:
            base = seen.get(ft.base)
            if base is None:
                raise ValueError(f"baseFieldType '{ft.base}' must be ordered before FieldType '{ft.name}'")
            ft.base_ptr = base
            if ft.physical is None:
                ft.physical = base.physical
            if ft.has_sign is None and base.has_sign is not None:
                ft.has_sign = base.has_sign
            if ft.unit is None and base.unit is not None:
                ft.unit = base.unit
            if ft.size == 0 and base.size != 0:
                ft.size = base.size
            if ft.resolution == 0.0 and base.resolution != 0.0:
                ft.resolution = base.resolution
            elif ft.resolution != 0.0 and base.resolution != 0.0 and ft.resolution != base.resolution:
                raise ValueError(f"Cannot overrule resolution of '{base.name}' in '{ft.name}'")
            if ft.print_function is None:
                ft.print_function = base.print_function

        # NB: an explicit .rangeMax initializer trips the `rangeMax == 0.0`
        # guard and lands in the NaN branch - fieldtype-level authored ranges
        # are inert in C, and stay inert here (MMSI, FIELD_INDEX).
        if (
            ft.size != 0
            and ft.resolution != 0.0
            and ft.has_sign is not None
            and (ft.range_max_authored in (None, 0.0))
        ):
            ft.range_min = get_min_range(ft.size, ft.resolution, ft.has_sign, ft.offset)
            ft.range_max = get_max_range(
                ft.size, ft.resolution, ft.has_sign, ft.offset, None, reserved_count_for_size(ft.size)
            )
        else:
            ft.range_min = NAN
            ft.range_max = NAN
        seen[ft.name] = ft


def fixup_unit(f: Field_, f_has_sign: bool) -> None:
    """Port of fieldtype.c fixupUnit(), SI branch only (analyzer-explain runs
    with showSI = true)."""
    if f.res_unit == "rad":
        if f_has_sign:
            f.res_range_min = c_max(f.res_range_min, -3.1415926)
            f.res_range_max = c_min(f.res_range_max, 3.1415926)
        else:
            f.res_range_max = c_min(f.res_range_max, 2 * 3.1415926)


def fill_field(db: Database, pgn: Pgn, f: Field_, order: int) -> None:
    """Port of fieldtype.c fillFieldType() per-field body (lines 334-465)."""
    ft = db.fieldtype(f.type)
    f.ft = ft
    f_has_sign = ft.has_sign is True

    # resolution: authored -> fieldtype -> macro default. The "default 1"
    # only applies when the type carries no resolution of its own: pgn.h
    # macros for resolution-bearing types (UFIX16 with a typed macro etc.)
    # leave .resolution at 0 so the C inherits it from the fieldtype.
    if f.resolution is not None:
        res = f.resolution
    elif ft.resolution != 0.0:
        res = ft.resolution
    else:
        res = default_field_resolution_for(ft)
    if ft.resolution != 0.0 and ft.resolution != res:
        raise ValueError(f"PGN {pgn.pgn} field '{f.name}': cannot overrule resolution of '{ft.name}'")
    f.res_resolution = res

    # size
    bits = f.bits if f.bits is not None else 0
    if ft.size != 0 and bits == 0:
        bits = ft.size
    if ft.size != 0 and ft.size != bits:
        raise ValueError(f"PGN {pgn.pgn} field '{f.name}': cannot overrule size of '{ft.name}'")
    f.res_bits = bits

    # offset
    offset = f.offset if f.offset is not None else 0
    if ft.offset != 0 and offset == 0:
        offset = ft.offset
    if ft.offset != 0 and ft.offset != offset:
        raise ValueError(f"PGN {pgn.pgn} field '{f.name}': cannot overrule offset of '{ft.name}'")
    f.res_offset = offset

    # unit
    unit = f.unit
    if ft.unit is not None and unit is None:
        unit = ft.unit
    f.res_unit = unit

    # ranges: authored values behave like explicit C initializers
    f.res_range_min = f.range_min if f.range_min is not None else 0.0
    f.res_range_max = f.range_max if f.range_max is not None else 0.0
    if math.isnan(f.res_range_max) or f.res_range_max == 0.0:
        f.res_range_min = ft.range_min
        f.res_range_max = ft.range_max
    if f.res_unit is not None and f.res_resolution != 0.0:
        fixup_unit(f, f_has_sign)

    by_size = reserved_count_for_size(f.res_bits)
    count = f.special_values if f.special_values is not None else by_size

    pair_lookup = db.lookups.get(f.lookup) if f.lookup else None
    if f.res_bits != 0 and f.res_resolution != 0.0 and ft.has_sign is not None and math.isnan(f.res_range_max):
        f.res_range_min = get_min_range(f.res_bits, f.res_resolution, f_has_sign, f.res_offset)
        f.res_range_max = get_max_range(f.res_bits, f.res_resolution, f_has_sign, f.res_offset, pair_lookup, count)

    # reservedCount (fieldtype.c lines 448-462)
    if f.special_values is not None:
        f.reserved_count = count
    elif f.res_bits != 0 and f.res_bits < 64 and f.res_resolution > 0.0 and not math.isnan(f.res_range_max):
        raw_max = (1 << f.res_bits) - 1
        raw_range_max = int(f.res_range_max / f.res_resolution + 0.5)
        f.reserved_count = 0 if raw_range_max >= raw_max else min(raw_max - raw_range_max, by_size)
    else:
        f.reserved_count = by_size

    f.order = order


def fill_pgn_length(pgn: Pgn) -> None:
    """Port of analyzer-explain.c getMinimalPgnLength()."""
    field_count = len(pgn.fields)
    counted = field_count
    is_variable = False
    if pgn.repeating1 is not None and pgn.repeating1.count > 0:
        counted -= pgn.repeating1.count + (pgn.repeating2.count if pgn.repeating2 else 0)
        is_variable = True

    length = 0
    for f in pgn.fields[:counted]:
        if f.res_bits == 0:
            is_variable = True
        else:
            length += f.res_bits

    if length % 8 != 0:
        raise ValueError(
            f"PGN {pgn.pgn} '{pgn.description}' has a length of {length} bits that does not fill bytes exactly"
        )
    pgn.field_count = field_count
    pgn.length = length // 8
    pgn.is_variable = is_variable


def fill(db: Database) -> None:
    db.index()
    fill_fieldtypes(db)
    for pgn in db.pgns:
        for j, f in enumerate(pgn.fields):
            fill_field(db, pgn, f, j + 1)
        fill_pgn_length(pgn)


def sentinels_for_fieldtype(ft) -> str:
    """analyzer-explain.c sentinelsForFieldType(): the ROOT type's convention."""
    return ft.root().sentinels
