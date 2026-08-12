#!/usr/bin/env python3
#
# Copyright (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# Canboat "public contract" tooling.
#
# The PGN database in docs/canboat.json is the source from which downstream
# projects (ts-pgns, canboatjs, the Signal K plugins, ...) generate their
# TypeScript types, their decoders and their test fixtures. A handful of the
# values in canboat.json are therefore *load bearing*: changing them changes a
# generated identifier or a decoded value, and so breaks downstream consumers.
#
# This script distils canboat.json down to just that load-bearing surface (its
# "signature"), and diffs two signatures to classify every change as:
#
#   breaking  - retained tier, not currently emitted. canboat regenerates the
#               downstream database, so no docs/canboat.json change is treated
#               as source-incompatible: even a removed/renamed/retyped
#               identifier is picked up when downstream regenerates its types,
#               which is a minor concern, not a major one. The tier (and its
#               "BREAKING CHANGE:" footer / exit code 4) is kept so the
#               machinery stays stable if a genuine major trigger is ever added.
#   minor     - any change downstream must react to: a generated identifier
#               removed / renamed / retyped, an enum / value / label change, a
#               PGN/lookup/value disappearing, OR a decoded value / wire
#               encoding change (resolution/offset/sign/bit-width, transport
#               type, match rebinding, primaryKey, a field made optional).
#               Downstream regenerates and updates fixtures. Minor bump.
#   additive  - new PGN / field / lookup / lookup value. Backwards compatible,
#               but still warrants at least a minor bump (a new field is an
#               extra key in a strict-equality fixture downstream).
#   cosmetic  - description / unit / comment text. Not a compatibility concern;
#               downstream updates its own fixtures. Patch at most.
#
# The diff is computed from the generated output, not from commit messages, so
# it works retroactively across a whole release window regardless of whether
# anyone remembered to flag a break.
#
# Pure Python 3 standard library (matches tools/md2html.py).

import argparse
import json
import sys


# --------------------------------------------------------------------------- #
# Signature extraction
# --------------------------------------------------------------------------- #

# Per-field attributes that are part of the public contract. The value next to
# each name is the severity that a *change* to that attribute carries.
# Note: a field's "order" is kept in the signature (rename detection ignores
# it) but is deliberately NOT classified here. Its sequence index shifts as a
# side effect of inserting/removing earlier fields, which would double-report a
# break already captured by the add/remove. Genuine layout breaks surface
# through bits/type/lookup changes instead.
FIELD_ATTRS = {
    "type": "minor",  # FieldType -> generated TS type changes (downstream regenerates)
    "lookup": "minor",  # which enum the TS type references + label source
    "bits": "minor",  # BitLength -> value range / encoded bytes; type unchanged
    "signed": "minor",  # value sign; TS type stays number
    "resolution": "minor",  # scaled value
    "offset": "minor",  # value
    "unit": "cosmetic",  # SI unit, downstream conversion only
}
# "primaryKey" is handled separately in _diff_fields. A flip either direction
# only changes key semantics (the decoded key set), not whether downstream
# source compiles, so it is classified "minor" rather than "breaking".


def _field_lookup(field):
    """Return the single enumeration a field references, normalised."""
    for key in (
        "LookupEnumeration",
        "LookupBitEnumeration",
        "LookupIndirectEnumeration",
        "LookupFieldTypeEnumeration",
    ):
        if key in field:
            return field[key]
    return None


def _field_signature(field):
    return {
        "order": field.get("Order"),
        "type": field.get("FieldType"),
        "bits": field.get("BitLength"),
        "signed": bool(field.get("Signed", False)),
        "resolution": field.get("Resolution"),
        "offset": field.get("Offset"),
        "lookup": _field_lookup(field),
        "primaryKey": bool(field.get("PartOfPrimaryKey", False)),
        "unit": field.get("Unit"),
    }


def _pgn_key(pgn):
    # A PGN number can carry several definitions, distinguished by Match fields.
    # The Id is the stable, unique identity of each definition and is what
    # ts-pgns turns into a class name.
    return "%s:%s" % (pgn.get("PGN"), pgn.get("Id"))


def _pgn_signature(pgn):
    match = {
        f["Id"]: f["Match"] for f in pgn.get("Fields", []) if "Match" in f
    }
    fields = {}
    for f in pgn.get("Fields", []):
        fid = f.get("Id")
        if fid is None:
            continue
        fields[fid] = _field_signature(f)
    return {
        "pgn": pgn.get("PGN"),
        "id": pgn.get("Id"),
        "description": pgn.get("Description"),
        "type": pgn.get("Type"),
        "fallback": bool(pgn.get("Fallback", False)),
        "match": match,
        "fields": fields,
    }


def _lookup_values(enum, value_key, name_key="Name"):
    out = {}
    for v in enum.get(value_key, []):
        # keyed by numeric value (stringified for stable JSON), -> label
        out[str(v.get("Value"))] = v.get(name_key)
    return out


def extract_signature(db):
    """Reduce a parsed canboat.json to its load-bearing contract signature."""
    pgns = {}
    for pgn in db.get("PGNs", []):
        pgns[_pgn_key(pgn)] = _pgn_signature(pgn)

    lookups = {}
    for enum in db.get("LookupEnumerations", []):
        lookups[enum["Name"]] = {
            "kind": "enum",
            "values": _lookup_values(enum, "EnumValues"),
        }
    for enum in db.get("LookupIndirectEnumerations", []):
        lookups[enum["Name"]] = {
            "kind": "indirect",
            "values": _lookup_values(enum, "EnumValues"),
        }
    for enum in db.get("LookupBitEnumerations", []):
        lookups[enum["Name"]] = {
            "kind": "bit",
            "values": _lookup_values(enum, "EnumBitValues"),
        }
    for enum in db.get("LookupFieldTypeEnumerations", []):
        lookups[enum["Name"]] = {
            "kind": "fieldtype",
            "values": _lookup_values(enum, "EnumFieldTypeValues"),
        }

    return {
        "version": db.get("Version"),
        "pgns": pgns,
        "lookups": lookups,
    }


# --------------------------------------------------------------------------- #
# Diff + classification
# --------------------------------------------------------------------------- #

ORDER = {"cosmetic": 0, "additive": 1, "minor": 2, "breaking": 3}


class Change:
    __slots__ = ("severity", "category", "subject", "detail")

    def __init__(self, severity, category, subject, detail):
        self.severity = severity
        self.category = category  # short machine tag, e.g. "field-removed"
        self.subject = subject  # human anchor, e.g. "PGN 130845 SimnetKeyValue"
        self.detail = detail

    def __repr__(self):
        return "%s: %s - %s" % (self.severity, self.subject, self.detail)


def _pgns_by_number(sig):
    by_num = {}
    for key, p in sig["pgns"].items():
        by_num.setdefault(p["pgn"], {})[p["id"]] = p
    return by_num


def _diff_fields(changes, anchor, old_fields, new_fields):
    old_ids = set(old_fields)
    new_ids = set(new_fields)

    removed = old_ids - new_ids
    added = new_ids - old_ids

    # Light 1:1 rename detection: a single removed + single added field whose
    # signatures (minus order) are identical is almost certainly a rename.
    renamed = {}
    if len(removed) == 1 and len(added) == 1:
        (r,) = tuple(removed)
        (a,) = tuple(added)
        ro = {k: v for k, v in old_fields[r].items() if k != "order"}
        ao = {k: v for k, v in new_fields[a].items() if k != "order"}
        if ro == ao:
            renamed[r] = a
            removed = set()
            added = set()

    for r, a in renamed.items():
        changes.append(
            Change(
                "minor",
                "field-renamed",
                anchor,
                "field '%s' renamed to '%s'" % (r, a),
            )
        )
    for fid in sorted(removed):
        changes.append(
            Change("minor", "field-removed", anchor, "field '%s' removed" % fid)
        )
    for fid in sorted(added):
        changes.append(
            Change("additive", "field-added", anchor, "field '%s' added" % fid)
        )

    for fid in sorted(old_ids & new_ids):
        o = old_fields[fid]
        n = new_fields[fid]
        for attr, severity in FIELD_ATTRS.items():
            if o.get(attr) != n.get(attr):
                changes.append(
                    Change(
                        severity,
                        "field-%s" % attr,
                        anchor,
                        "field '%s' %s: %r -> %r"
                        % (fid, attr, o.get(attr), n.get(attr)),
                    )
                )
        if o.get("primaryKey") != n.get("primaryKey"):
            # A primaryKey flip either direction is only a key-semantics change:
            # downstream still compiles, but the decoded key set differs, so it
            # warrants a minor bump rather than a major.
            changes.append(
                Change(
                    "minor",
                    "field-primaryKey",
                    anchor,
                    "field '%s' primaryKey: %r -> %r"
                    % (fid, o.get("primaryKey"), n.get("primaryKey")),
                )
            )


def _diff_pgn_pair(changes, old_p, new_p):
    anchor = "PGN %s %s" % (new_p["pgn"], new_p["id"])
    if old_p["description"] != new_p["description"]:
        changes.append(
            Change(
                "cosmetic",
                "pgn-description",
                anchor,
                "description: %r -> %r"
                % (old_p["description"], new_p["description"]),
            )
        )
    if old_p["type"] != new_p["type"]:
        changes.append(
            Change(
                "minor",
                "pgn-type",
                anchor,
                "transport type: %r -> %r" % (old_p["type"], new_p["type"]),
            )
        )
    if old_p["match"] != new_p["match"]:
        changes.append(
            Change(
                "minor",
                "pgn-match",
                anchor,
                "match fields: %r -> %r" % (old_p["match"], new_p["match"]),
            )
        )
    _diff_fields(changes, anchor, old_p["fields"], new_p["fields"])


def _diff_pgns(changes, old_sig, new_sig):
    old_by_num = _pgns_by_number(old_sig)
    new_by_num = _pgns_by_number(new_sig)

    for num in sorted(set(old_by_num) | set(new_by_num)):
        old_defs = old_by_num.get(num, {})
        new_defs = new_by_num.get(num, {})
        old_ids = set(old_defs)
        new_ids = set(new_defs)

        removed = old_ids - new_ids
        added = new_ids - old_ids

        # 1:1 Id rename within the same PGN number.
        if len(removed) == 1 and len(added) == 1:
            (r,) = tuple(removed)
            (a,) = tuple(added)
            anchor = "PGN %s" % num
            changes.append(
                Change(
                    "minor",
                    "pgn-renamed",
                    anchor,
                    "definition Id renamed '%s' -> '%s'" % (r, a),
                )
            )
            _diff_pgn_pair(changes, old_defs[r], new_defs[a])
            removed = set()
            added = set()

        for rid in sorted(removed):
            changes.append(
                Change(
                    "minor",
                    "pgn-removed",
                    "PGN %s %s" % (num, rid),
                    "PGN definition removed",
                )
            )
        for aid in sorted(added):
            changes.append(
                Change(
                    "additive",
                    "pgn-added",
                    "PGN %s %s" % (num, aid),
                    "new PGN definition",
                )
            )
        for shared in sorted(old_ids & new_ids):
            _diff_pgn_pair(changes, old_defs[shared], new_defs[shared])


def _diff_lookups(changes, old_sig, new_sig):
    old = old_sig["lookups"]
    new = new_sig["lookups"]
    for name in sorted(set(old) | set(new)):
        if name not in new:
            changes.append(
                Change("minor", "lookup-removed", name, "enumeration removed")
            )
            continue
        if name not in old:
            changes.append(
                Change("additive", "lookup-added", name, "new enumeration")
            )
            continue
        ov = old[name]["values"]
        nv = new[name]["values"]
        def _vkey(x):
            try:
                return (0, int(x))
            except (TypeError, ValueError):
                return (1, str(x))

        for value in sorted(set(ov) | set(nv), key=_vkey):
            if value not in nv:
                changes.append(
                    Change(
                        "minor",
                        "lookup-value-removed",
                        name,
                        "value %s (%r) removed" % (value, ov[value]),
                    )
                )
            elif value not in ov:
                changes.append(
                    Change(
                        "additive",
                        "lookup-value-added",
                        name,
                        "value %s (%r) added" % (value, nv[value]),
                    )
                )
            elif ov[value] != nv[value]:
                changes.append(
                    Change(
                        "minor",
                        "lookup-label",
                        name,
                        "value %s label: %r -> %r"
                        % (value, ov[value], nv[value]),
                    )
                )


def diff_signatures(old_sig, new_sig):
    changes = []
    _diff_pgns(changes, old_sig, new_sig)
    _diff_lookups(changes, old_sig, new_sig)
    # Sort: severity desc, then category, then subject.
    changes.sort(key=lambda c: (-ORDER[c.severity], c.category, c.subject))
    return changes


def max_severity(changes):
    if not changes:
        return None
    return max(changes, key=lambda c: ORDER[c.severity]).severity


# severity -> required conventional-commit level / release-please bump
REQUIRED_BUMP = {
    "breaking": "major",
    "minor": "minor",
    "additive": "minor",
    "cosmetic": "patch",
}

# human-facing section labels, highest severity first
SEVERITY_LABELS = [
    ("breaking", "BREAKING CHANGES (source-incompatible)"),
    ("minor", "Minor breaking (recompiles; decode/encode changed)"),
    ("additive", "Added"),
    ("cosmetic", "Cosmetic"),
]


# --------------------------------------------------------------------------- #
# Reporting
# --------------------------------------------------------------------------- #

def render_text(changes):
    if not changes:
        return "No contract changes.\n"
    out = []
    for sev, label in SEVERITY_LABELS:
        group = [c for c in changes if c.severity == sev]
        if not group:
            continue
        out.append("%s (%d):" % (label, len(group)))
        for c in group:
            out.append("  - %s: %s" % (c.subject, c.detail))
        out.append("")
    return "\n".join(out)


def render_footer(changes):
    """A conventional-commit 'BREAKING CHANGE:' footer for the commit/PR.

    release-please reads this footer to bump the major version and to write
    the breaking-changes section of the changelog. Empty when nothing is
    source-breaking (a minor-breaking-only change needs a feat:, not a footer).
    """
    breaking = [c for c in changes if c.severity == "breaking"]
    if not breaking:
        return ""
    parts = ["%s (%s)" % (c.subject, c.detail) for c in breaking]
    return "BREAKING CHANGE: " + "; ".join(parts) + "\n"


def render_markdown(changes):
    """A block suitable for a release-notes / PR-description paste."""
    breaking = [c for c in changes if c.severity == "breaking"]
    minor = [c for c in changes if c.severity == "minor"]
    additive = [c for c in changes if c.severity == "additive"]
    out = []
    if breaking:
        out.append("### ⚠ BREAKING CHANGES")
        out.append("")
        for c in breaking:
            out.append("- **%s**: %s" % (c.subject, c.detail))
        out.append("")
    if minor:
        out.append("### Changed (minor breaking — re-check decoders/fixtures)")
        out.append("")
        for c in minor:
            out.append("- %s: %s" % (c.subject, c.detail))
        out.append("")
    if additive:
        out.append("### Added")
        out.append("")
        for c in additive:
            out.append("- %s: %s" % (c.subject, c.detail))
        out.append("")
    return "\n".join(out)


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #

def _load(path):
    if path == "-":
        return json.load(sys.stdin)
    with open(path, "r", encoding="utf-8") as fh:
        return json.load(fh)


def _as_signature(path):
    """Accept either a raw canboat.json or an already-extracted signature."""
    data = _load(path)
    if "PGNs" in data:
        return extract_signature(data)
    return data


def cmd_signature(args):
    sig = extract_signature(_load(args.canboat_json))
    json.dump(sig, sys.stdout, indent=2, sort_keys=True)
    sys.stdout.write("\n")
    return 0


def cmd_diff(args):
    old = _as_signature(args.old)
    new = _as_signature(args.new)
    changes = diff_signatures(old, new)

    if args.format == "json":
        json.dump(
            [
                {
                    "severity": c.severity,
                    "category": c.category,
                    "subject": c.subject,
                    "detail": c.detail,
                }
                for c in changes
            ],
            sys.stdout,
            indent=2,
        )
        sys.stdout.write("\n")
    elif args.format == "markdown":
        sys.stdout.write(render_markdown(changes))
    elif args.format == "footer":
        sys.stdout.write(render_footer(changes))
    else:
        sys.stdout.write(render_text(changes))

    sev = max_severity(changes)
    if args.summary:
        bump = REQUIRED_BUMP.get(sev, "none")
        sys.stderr.write(
            "\nHighest change: %s -> requires a %s release.\n"
            % (sev or "none", bump)
        )
    # Exit code encodes severity so callers can gate: 0 none, 1 cosmetic,
    # 2 additive, 3 breaking.
    return 0 if sev is None else ORDER[sev] + 1


def main(argv=None):
    p = argparse.ArgumentParser(description="Canboat public-contract tooling.")
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("signature", help="extract the contract signature")
    sp.add_argument("canboat_json", nargs="?", default="docs/canboat.json")
    sp.set_defaults(func=cmd_signature)

    dp = sub.add_parser("diff", help="diff two canboat.json files or signatures")
    dp.add_argument("old", help="baseline canboat.json or signature ('-' = stdin)")
    dp.add_argument("new", help="candidate canboat.json or signature")
    dp.add_argument(
        "--format",
        choices=("text", "markdown", "json", "footer"),
        default="text",
    )
    dp.add_argument(
        "--summary",
        action="store_true",
        help="print the required release bump to stderr",
    )
    dp.set_defaults(func=cmd_diff)

    args = p.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
