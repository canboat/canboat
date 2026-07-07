"""database/ tree reader and writer.

Canonical YAML style (DESIGN.md §3.5): machine-written, fixed key order,
authored data only. The writer and loader are exact inverses; the golden
test exercises the full round trip (XML -> model -> YAML -> model -> XML).
"""

from __future__ import annotations

import math
import os

import yaml

from .model import (
    Database,
    Field_,
    FieldType,
    FieldTypeLookupEntry,
    Lookup,
    Pgn,
    PhysicalQuantity,
    Repeating,
)


class _Dumper(yaml.SafeDumper):
    pass


def _str_presenter(dumper, data):
    if "\n" in data:
        return dumper.represent_scalar("tag:yaml.org,2002:str", data, style="|")
    return dumper.represent_scalar("tag:yaml.org,2002:str", data)


_Dumper.add_representer(str, _str_presenter)


def _dump(data, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, Dumper=_Dumper, sort_keys=False, allow_unicode=True, width=120)


def _load(path: str):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)


def _clean(d: dict) -> dict:
    return {k: v for k, v in d.items() if v is not None and v is not False and v != []}


# --------------------------------------------------------------------------
# dump
# --------------------------------------------------------------------------


def field_to_dict(f: Field_) -> dict:
    d = {
        "id": f.id,
        "name": f.name,
        "type": f.type,
        "bits": f.bits,
        "resolution": f.resolution,
        "unit": f.unit,
        "offset": f.offset,
        "description": f.description,
        "note": getattr(f, "note", None),
        "match": f.match,
        "lookup": f.lookup,
        "lookupIndirect": None,
        "lookupBits": f.lookup_bits,
        "lookupFieldtype": f.lookup_fieldtype,
        "primaryKey": f.primary_key,
        "proprietary": f.proprietary,
        "allowLookupWidthMismatch": f.allow_lookup_width_mismatch,
        "specialValues": f.special_values,
        "dynamicFieldLength": f.dynamic_field_length or None,
        "dynamicFieldLengthOverhead": f.dynamic_field_length_overhead or None,
        "rangeMin": f.range_min,
        "rangeMax": f.range_max,
    }
    if f.lookup_indirect is not None:
        d["lookupIndirect"] = {"name": f.lookup_indirect, "order": f.lookup_indirect_order}
    if f.special_values is not None:
        d["specialValues"] = f.special_values  # keep explicit 0
        return {k: v for k, v in d.items() if not (v is None and k != "specialValues") and v is not False and v != []}
    return _clean(d)


def pgn_to_dict(p: Pgn) -> dict:
    d = {
        "pgn": p.pgn,
        "id": p.id,
        "description": p.description,
        "type": p.type,
        "fallback": p.fallback,
        "variantOrder": p.variant_order or None,
        "priority": p.priority or None,
        "interval": p.interval,
        "missing": list(p.missing) or None,
        "explanation": p.explanation,
        "url": p.url,
        "researchDoc": p.research_doc,
        "notes": getattr(p, "notes", None),
        "fields": [field_to_dict(f) for f in p.fields],
    }
    for n in (1, 2):
        rep = getattr(p, f"repeating{n}")
        if rep is not None:
            d[f"repeating{n}"] = _clean({"count": rep.count, "start": rep.start, "countField": rep.count_field})
    return _clean(d)


def lookup_to_dict(lk: Lookup) -> dict:
    d = {"name": lk.name, "kind": lk.kind, "bits": lk.bits}
    if lk.kind in ("pair", "bit"):
        d["values"] = {v: n for v, n in lk.pairs}
    elif lk.kind == "triplet":
        d["values"] = [[v1, v2, n] for v1, v2, n in lk.triplets]
    else:
        d["values"] = [
            _clean(
                {
                    "value": e.value,
                    "name": e.name,
                    "type": e.fieldtype,
                    "bits": e.bits,
                    "lookup": e.lookup,
                    "lookupKind": e.lookup_kind if e.lookup_kind != "pair" else None,
                }
            )
            for e in lk.fieldtypes
        ]
    return d


def fieldtype_to_dict(ft: FieldType) -> dict:
    d = _clean(
        {
            "name": ft.name,
            "base": ft.base,
            "description": ft.description,
            "encodingDescription": ft.encoding_description,
            "comment": ft.comment,
            "url": ft.url,
            "bits": ft.size or None,
            "variableSize": ft.variable_size,
            "unit": ft.unit,
            "offset": ft.offset or None,
            "resolution": ft.resolution if ft.resolution != 0.0 else None,
            "signed": ft.has_sign,
            "sentinels": ft.sentinels if ft.sentinels != "None" else None,
            "physical": ft.physical,
            "print": ft.print_function,
            "rangeMin": ft.range_min_authored,
            "rangeMax": ft.range_max_authored,
        }
    )
    # signed is tri-state: False (unsigned) is meaningful, only Null is absent.
    if ft.has_sign is False:
        d["signed"] = False
    return d


def pq_to_dict(pq: PhysicalQuantity) -> dict:
    return _clean(
        {
            "name": pq.name,
            "description": pq.description,
            "comment": pq.comment,
            "url": pq.url,
            "unitDescription": pq.unit_description,
            "unit": pq.unit,
        }
    )


def _preserve_notes(d: dict, path: str) -> dict:
    """Research notes are authored in the YAML (migration step 4), not in the
    XML the converter reads - carry them over when regenerating a file."""
    if not os.path.exists(path):
        return d
    old = _load(path)
    if not isinstance(old, dict):
        return d
    if "notes" in old and "notes" not in d:
        d["notes"] = old["notes"]
    if "note" in old and "note" not in d:
        d["note"] = old["note"]
    if "valueNotes" in old and "valueNotes" not in d:
        d["valueNotes"] = old["valueNotes"]
    old_fields = {f.get("id"): f for f in old.get("fields", [])} if "fields" in old else {}
    for f in d.get("fields", []):
        of = old_fields.get(f.get("id"))
        if of and "note" in of and "note" not in f:
            f["note"] = of["note"]
    return d


def write_database(db: Database, out_dir: str) -> None:
    _dump([pq_to_dict(pq) for pq in db.physical_quantities], os.path.join(out_dir, "physicalquantities.yaml"))
    # derive.fill() percolates base/physical attributes into the FieldType
    # objects in place; fieldtypes.yaml must store the authored state.
    fieldtypes = getattr(db, "authored_fieldtypes", db.fieldtypes)
    _dump([fieldtype_to_dict(ft) for ft in fieldtypes], os.path.join(out_dir, "fieldtypes.yaml"))

    for lk in db.lookups.values():
        path = os.path.join(out_dir, "lookups", f"{lk.name}.yaml")
        _dump(_preserve_notes(lookup_to_dict(lk), path), path)
    # Temporary manifest: preserves lookup.h emission order for the golden
    # byte-diff. Dies post-switchover in favor of sorted order (DESIGN.md).
    _dump(dict(db.lookup_order), os.path.join(out_dir, "lookups.order.yaml"))

    for p in db.pgns:
        path = os.path.join(out_dir, "pgns", f"{p.pgn:06d}-{p.id}.yaml")
        _dump(_preserve_notes(pgn_to_dict(p), path), path)
    for p in getattr(db, "pgns_j1939", []):
        path = os.path.join(out_dir, "j1939", "pgns", f"{p.pgn:06d}-{p.id}.yaml")
        _dump(_preserve_notes(pgn_to_dict(p), path), path)

    # Prune orphans: a resync must also delete files whose PGN entry (or
    # lookup) disappeared upstream, else the Rust loader keeps emitting them.
    def prune(directory, expected):
        if not os.path.isdir(directory):
            return
        for name in os.listdir(directory):
            if name.endswith(".yaml") and name not in expected:
                os.remove(os.path.join(directory, name))
                print(f"keel convert: pruned stale {os.path.join(directory, name)}")

    prune(os.path.join(out_dir, "pgns"), {f"{p.pgn:06d}-{p.id}.yaml" for p in db.pgns})
    prune(
        os.path.join(out_dir, "j1939", "pgns"),
        {f"{p.pgn:06d}-{p.id}.yaml" for p in getattr(db, "pgns_j1939", [])},
    )
    prune(os.path.join(out_dir, "lookups"), {f"{lk.name}.yaml" for lk in db.lookups.values()})


# --------------------------------------------------------------------------
# load
# --------------------------------------------------------------------------


def field_from_dict(d: dict) -> Field_:
    f = Field_(
        id=d["id"],
        name=d["name"],
        type=d["type"],
        bits=d.get("bits"),
        resolution=d.get("resolution"),
        unit=d.get("unit"),
        offset=d.get("offset"),
        description=d.get("description"),
        match=d.get("match"),
        lookup=d.get("lookup"),
        lookup_bits=d.get("lookupBits"),
        lookup_fieldtype=d.get("lookupFieldtype"),
        primary_key=d.get("primaryKey", False),
        proprietary=d.get("proprietary", False),
        allow_lookup_width_mismatch=d.get("allowLookupWidthMismatch", False),
        special_values=d.get("specialValues"),
        dynamic_field_length=d.get("dynamicFieldLength", False) or d.get("dynamicFieldLengthOverhead") is not None,
        dynamic_field_length_overhead=d.get("dynamicFieldLengthOverhead", 0),
        range_min=d.get("rangeMin"),
        range_max=d.get("rangeMax"),
    )
    li = d.get("lookupIndirect")
    if li is not None:
        f.lookup_indirect = li["name"]
        f.lookup_indirect_order = li.get("order")
    return f


def pgn_from_dict(d: dict) -> Pgn:
    p = Pgn(
        pgn=d["pgn"],
        id=d["id"],
        description=d["description"],
        type=d["type"],
        priority=d.get("priority", 0),
        interval=d.get("interval"),
        explanation=d.get("explanation"),
        url=d.get("url"),
        research_doc=d.get("researchDoc"),
        fallback=d.get("fallback", False),
        missing=d.get("missing", []),
        variant_order=d.get("variantOrder", 0),
        fields=[field_from_dict(fd) for fd in d.get("fields", [])],
    )
    for n in (1, 2):
        rep = d.get(f"repeating{n}")
        if rep is not None:
            setattr(p, f"repeating{n}", Repeating(rep["count"], rep["start"], rep.get("countField")))
    return p


def lookup_from_dict(d: dict) -> Lookup:
    lk = Lookup(name=d["name"], kind=d["kind"], bits=d["bits"])
    if lk.kind in ("pair", "bit"):
        lk.pairs = [(v, n) for v, n in d["values"].items()]
    elif lk.kind == "triplet":
        lk.triplets = [(v1, v2, n) for v1, v2, n in d["values"]]
    else:
        lk.fieldtypes = [
            FieldTypeLookupEntry(
                value=e["value"],
                name=e["name"],
                fieldtype=e["type"],
                bits=e.get("bits"),
                lookup=e.get("lookup"),
                lookup_kind=("pair" if e.get("lookup") and not e.get("lookupKind") else e.get("lookupKind")),
            )
            for e in d["values"]
        ]
    return lk


def fieldtype_from_dict(d: dict) -> FieldType:
    return FieldType(
        name=d["name"],
        base=d.get("base"),
        description=d.get("description"),
        encoding_description=d.get("encodingDescription"),
        comment=d.get("comment"),
        url=d.get("url"),
        size=d.get("bits", 0),
        variable_size=d.get("variableSize", False),
        unit=d.get("unit"),
        offset=d.get("offset", 0),
        resolution=d.get("resolution", 0.0),
        has_sign=d.get("signed"),
        sentinels=d.get("sentinels", "None"),
        physical=d.get("physical"),
        print_function=d.get("print"),
        range_min_authored=d.get("rangeMin"),
        range_max_authored=d.get("rangeMax"),
    )


def pq_from_dict(d: dict) -> PhysicalQuantity:
    return PhysicalQuantity(
        name=d["name"],
        description=d.get("description"),
        comment=d.get("comment"),
        url=d.get("url"),
        unit_description=d.get("unitDescription"),
        unit=d.get("unit"),
    )


def load_database(db_dir: str, version: str, schema_version: str) -> Database:
    db = Database(version=version, schema_version=schema_version)
    db.physical_quantities = [pq_from_dict(d) for d in _load(os.path.join(db_dir, "physicalquantities.yaml"))]
    db.fieldtypes = [fieldtype_from_dict(d) for d in _load(os.path.join(db_dir, "fieldtypes.yaml"))]

    lookups_dir = os.path.join(db_dir, "lookups")
    for name in sorted(os.listdir(lookups_dir)):
        if name.endswith(".yaml"):
            lk = lookup_from_dict(_load(os.path.join(lookups_dir, name)))
            db.lookups[lk.name] = lk
    db.lookup_order = _load(os.path.join(db_dir, "lookups.order.yaml"))

    pgns_dir = os.path.join(db_dir, "pgns")
    pgns = [pgn_from_dict(_load(os.path.join(pgns_dir, name))) for name in sorted(os.listdir(pgns_dir)) if name.endswith(".yaml")]
    # Emission order: PGN ascending, then authored variant order (which
    # includes fallback entries - their in-group position is semantic).
    pgns.sort(key=lambda p: (p.pgn, p.variant_order))
    db.pgns = pgns
    return db
