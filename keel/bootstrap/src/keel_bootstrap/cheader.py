"""Bootstrap parser for analyzer/fieldtype.h.

The canboat.xml FieldTypes section only contains the *root* field types;
the derived vocabulary (UINT8, UFIX16, ...) that fields are authored in
exists only in the C header. This module parses the fieldTypeList[]
designated initializers once, to seed database/fieldtypes.yaml.

It is a bootstrap tool: after migration step 5 fieldtype.h is generated
FROM the YAML and this parser is retired.
"""

from __future__ import annotations

import re

from .model import FieldType

# Constants that may appear as initializer values (from pgn.h / fieldtype.h).
CONSTANTS = {
    "RES_LAT_LONG": 1.0e-7,
    "RES_LAT_LONG_64": 1.0e-16,
    "RES_PERCENTAGE": 100.0 / 25000.0,
    "RES_RADIANS": 1e-4,
    "RES_ROTATION": 1e-3 / 32.0,
    "RES_HIRES_ROTATION": 1e-6 / 32.0,
}

SENTINEL_NAMES = {
    "SENTINEL_NONE": "None",
    "SENTINEL_TOP_OF_RANGE": "TopOfRange",
    "SENTINEL_NAN": "NaN",
    "SENTINEL_EMPTY_STRING": "EmptyString",
    "SENTINEL_VARIABLE": "Variable",
}

KEY_MAP = {
    "name": "name",
    "description": "description",
    "encodingDescription": "encoding_description",
    "comment": "comment",
    "url": "url",
    "size": "size",
    "variableSize": "variable_size",
    "baseFieldType": "base",
    "unit": "unit",
    "offset": "offset",
    "resolution": "resolution",
    "hasSign": "has_sign",
    "sentinels": "sentinels",
    "pf": "print_function",
    "physical": "physical",
    "rangeMin": "range_min_authored",
    "rangeMax": "range_max_authored",
    # dropped: v1Type (v1 output dies), external
    "v1Type": None,
    "external": None,
}


def _strip_comments(text: str) -> str:
    """Remove /* */ and // comments - string-aware, so that URLs inside
    string literals ("https://...") survive."""
    out = []
    i = 0
    n = len(text)
    in_str = False
    while i < n:
        c = text[i]
        if in_str:
            out.append(c)
            if c == "\\" and i + 1 < n:
                out.append(text[i + 1])
                i += 2
                continue
            if c == '"':
                in_str = False
            i += 1
        elif c == '"':
            in_str = True
            out.append(c)
            i += 1
        elif c == "/" and i + 1 < n and text[i + 1] == "*":
            end = text.index("*/", i + 2)
            i = end + 2
            out.append(" ")
        elif c == "/" and i + 1 < n and text[i + 1] == "/":
            end = text.find("\n", i)
            i = n if end == -1 else end
        else:
            out.append(c)
            i += 1
    return "".join(out)


def _extract_list(text: str, anchor: str) -> str:
    """Return the initializer body between `anchor ... = {` and its `};`."""
    start = text.index(anchor)
    open_brace = text.index("{", start)
    depth = 0
    i = open_brace
    in_str = False
    while i < len(text):
        c = text[i]
        if in_str:
            if c == "\\":
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return text[open_brace + 1 : i]
        i += 1
    raise ValueError(f"unbalanced braces after {anchor!r}")


def _split_entries(body: str):
    """Yield the top-level { ... } groups of an array initializer body."""
    depth = 0
    in_str = False
    start = None
    i = 0
    while i < len(body):
        c = body[i]
        if in_str:
            if c == "\\":
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
        elif c == "{":
            if depth == 0:
                start = i
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0 and start is not None:
                yield body[start + 1 : i]
                start = None
        i += 1


def _split_assignments(entry: str):
    """Split one entry into `.key = value` strings at depth-0 commas."""
    parts = []
    depth = 0
    in_str = False
    cur = []
    i = 0
    while i < len(entry):
        c = entry[i]
        if in_str:
            cur.append(c)
            if c == "\\" and i + 1 < len(entry):
                cur.append(entry[i + 1])
                i += 2
                continue
            if c == '"':
                in_str = False
        elif c == '"':
            in_str = True
            cur.append(c)
        elif c in "{(":
            depth += 1
            cur.append(c)
        elif c in "})":
            depth -= 1
            cur.append(c)
        elif c == "," and depth == 0:
            parts.append("".join(cur))
            cur = []
        else:
            cur.append(c)
        i += 1
    if "".join(cur).strip():
        parts.append("".join(cur))
    return parts


_STRING_RE = re.compile(r'"((?:[^"\\]|\\.)*)"')


def _parse_value(raw: str):
    raw = raw.strip()
    # Concatenated C string literals
    if raw.startswith('"'):
        pieces = _STRING_RE.findall(raw)
        joined = "".join(pieces)
        return joined.replace('\\"', '"').replace("\\n", "\n").replace("\\\\", "\\")
    # Tri-state Bool enum: {Null=0, False=1, True=2}. C's lowercase stdbool
    # values alias into it: `false` == 0 == Null, `true` == 1 == False(!).
    # fieldtype.h's ISO_NAME actually contains `.hasSign = false` (a latent
    # typo meaning Null - no Signed attribute emitted); reproduce faithfully.
    if raw == "True":
        return True
    if raw == "False" or raw == "true":
        return False
    if raw == "Null" or raw == "false":
        return None
    if raw in SENTINEL_NAMES:
        return SENTINEL_NAMES[raw]
    if raw.startswith("&"):
        return raw[1:]  # &ANGLE -> physical quantity name
    if raw.startswith("fieldPrint"):
        return raw
    if raw in CONSTANTS:
        return CONSTANTS[raw]
    # common/pow.h macros: POW2(n)=2.0^n, POW2NEG(n)=2.0^-n, POW10I(n)=10^n
    # (all bit-exact as Python powers: powers of two, and integer 10^n)
    raw = re.sub(r"\bPOW2\((\d+)\)", lambda m: repr(2.0 ** int(m.group(1))), raw)
    raw = re.sub(r"\bPOW2NEG\((\d+)\)", lambda m: repr(2.0 ** -int(m.group(1))), raw)
    raw = re.sub(r"\bPOW10I?\((\d+)\)", lambda m: str(10 ** int(m.group(1))), raw)
    # Numeric expression: digits, ., e/E, + - * / ( ) and hex
    if re.fullmatch(r"[0-9a-fxA-FX.eE+\-*/() ]+", raw):
        return eval(raw, {"__builtins__": {}})  # noqa: S307 - sanitized above
    raise ValueError(f"cannot parse initializer value: {raw!r}")


_PQ_DEF_RE = re.compile(
    r"static const PhysicalQuantity (\w+)\s*=?\s*\{.*?\.name\s*=\s*\"([^\"]+)\"", re.S
)


def parse_fieldtypes(path: str) -> list:
    """Parse fieldTypeList[] from analyzer/fieldtype.h into FieldType objects."""
    with open(path, encoding="utf-8") as f:
        text = _strip_comments(f.read())

    # .physical = &GEO_LATITUDE references the C variable; the XML and the
    # model use the .name string ("GEOGRAPHICAL_LATITUDE"). Map var -> name.
    pq_names = {var: name for var, name in _PQ_DEF_RE.findall(text)}

    body = _extract_list(text, "FieldType fieldTypeList[] =")
    result = []
    for entry in _split_entries(body):
        ft = FieldType(name="")
        for assignment in _split_assignments(entry):
            assignment = assignment.strip()
            if not assignment:
                continue
            m = re.match(r"\.(\w+)\s*=\s*(.*)", assignment, flags=re.S)
            if not m:
                raise ValueError(f"cannot parse assignment: {assignment!r}")
            ckey, raw = m.group(1), m.group(2)
            if ckey not in KEY_MAP:
                raise ValueError(f"unknown FieldType member .{ckey}")
            key = KEY_MAP[ckey]
            if key is None:
                continue
            value = _parse_value(raw)
            if key == "size":
                value = int(value)
            if key == "variable_size":
                value = bool(value)
            if key == "physical":
                value = pq_names.get(value, value)
            setattr(ft, key, value)
        if not ft.name:
            raise ValueError(f"FieldType entry without name: {entry[:80]!r}")
        result.append(ft)
    return result
