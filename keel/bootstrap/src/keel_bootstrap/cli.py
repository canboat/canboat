"""keel command line interface. See DESIGN.md §4 for the command set;
during migration step 1 only `convert` and `generate [--check]` exist."""

from __future__ import annotations

import argparse
import os
import re
import sys


def find_repo_root(start: str = ".") -> str:
    path = os.path.abspath(start)
    while True:
        if os.path.isdir(os.path.join(path, "analyzer")) and os.path.isdir(os.path.join(path, "docs")):
            return path
        parent = os.path.dirname(path)
        if parent == path:
            raise SystemExit("keel: cannot find canboat repository root (looked for analyzer/ and docs/)")
        path = parent


def read_versions(root: str):
    with open(os.path.join(root, "common", "version.h"), encoding="utf-8") as f:
        text = f.read()
    version = re.search(r'#define VERSION "([^"]+)"', text).group(1)
    schema = re.search(r'#define SCHEMA_VERSION "([^"]+)"', text).group(1)
    return version, schema


def _bem_sources(root: str, args) -> dict:
    """Locate or generate the Actisense/iKonvert BEM XML documents. These are
    not committed artifacts; the bootstrap needs the C analyzer-explain."""
    import glob
    import subprocess
    import tempfile

    paths = {}
    explicit = {"actisense": args.actisense_xml, "ikonvert": args.ikonvert_xml}
    binaries = glob.glob(os.path.join(root, "rel", "*", "analyzer-explain"))
    flags = {"actisense": "-explain-ngt-xml", "ikonvert": "-explain-ik-xml"}
    for which, path in explicit.items():
        if path is not None:
            paths[which] = path
        elif binaries:
            out = subprocess.run(
                [binaries[0], flags[which], "-camel"], capture_output=True, text=True, check=True
            ).stdout
            tmp = tempfile.NamedTemporaryFile(
                mode="w", suffix=f"-{which}.xml", delete=False, encoding="utf-8"
            )
            tmp.write(out)
            tmp.close()
            paths[which] = tmp.name
        else:
            print(
                f"keel convert: no --{which}-xml and no rel/*/analyzer-explain binary; "
                f"{which} BEM pseudo-PGNs are NOT converted",
                file=sys.stderr,
            )
    return paths


def cmd_convert(args) -> int:
    from . import convert, derive, emit_xml, yamlio

    root = find_repo_root(args.root)
    xml_path = os.path.join(root, "docs", "canboat.xml")
    header = os.path.join(root, "analyzer", "fieldtype.h")
    out_dir = os.path.join(root, args.out)

    bem_paths = _bem_sources(root, args)
    j1939_xml = args.j1939_xml
    if j1939_xml is None:
        import glob as _glob
        import subprocess as _sp
        import tempfile as _tf

        binaries = _glob.glob(os.path.join(root, "rel", "*", "analyzer-explain-j1939"))
        if binaries:
            out = _sp.run([binaries[0], "-explain-xml", "-camel"], capture_output=True, text=True, check=True).stdout
            tmp = _tf.NamedTemporaryFile(mode="w", suffix="-j1939.xml", delete=False, encoding="utf-8")
            tmp.write(out)
            tmp.close()
            j1939_xml = tmp.name
        else:
            print("keel convert: no --j1939-xml and no analyzer-explain-j1939 binary; J1939 NOT converted",
                  file=sys.stderr)
    db, failures = convert.convert(xml_path, header, verbose=args.verbose, bem_paths=bem_paths,
                                   j1939_xml=j1939_xml)

    yamlio.write_database(db, out_dir)
    print(f"keel convert: wrote {len(db.pgns)} pgns, {len(db.lookups)} lookups to {out_dir}")

    if failures:
        print(f"keel convert: {len(failures)} PGN blocks NOT byte-exact: {', '.join(failures[:10])}"
              + (" ..." if len(failures) > 10 else ""), file=sys.stderr)

    ok = True
    references = {"normal": xml_path, **bem_paths}
    if j1939_xml is not None:
        references["j1939"] = j1939_xml
    for which, path in references.items():
        source_db = getattr(db, "j1939_db", None) if which == "j1939" else db
        emitted = emit_xml.emit_xml(source_db, "normal" if which == "j1939" else which)
        with open(path, encoding="utf-8") as f:
            original = f.read()
        if emitted == original:
            print(f"keel convert: golden check OK ({which}) - emitted XML is byte-identical")
        else:
            ok = False
            print(f"keel convert: {which} document does not reproduce byte-for-byte yet", file=sys.stderr)
            if args.diff:
                _write_diff(original, emitted, f"{args.diff}.{which}")
                print(f"keel convert: diff written to {args.diff}.{which}", file=sys.stderr)
    return 0 if ok else 1


def cmd_generate(args) -> int:
    from . import derive, emit_xml, yamlio

    root = find_repo_root(args.root)
    version, schema = read_versions(root)
    db_dir = os.path.join(root, "database")
    if not os.path.isdir(db_dir):
        raise SystemExit(f"keel: no database/ tree at {db_dir} (run `keel convert` first)")

    db = yamlio.load_database(db_dir, version, schema)
    derive.fill(db)
    emitted = emit_xml.emit_xml(db)

    xml_path = os.path.join(root, "docs", "canboat.xml")
    if args.check:
        with open(xml_path, encoding="utf-8") as f:
            original = f.read()
        if emitted != original:
            print("keel generate --check: docs/canboat.xml is NOT up to date with database/", file=sys.stderr)
            if args.diff:
                _write_diff(original, emitted, args.diff)
            return 1
        print("keel generate --check: docs/canboat.xml is up to date")
        return 0

    with open(xml_path, "w", encoding="utf-8") as f:
        f.write(emitted)
    print(f"keel generate: wrote {xml_path}")
    return 0


def _write_diff(original: str, emitted: str, path: str) -> None:
    import difflib

    diff = difflib.unified_diff(
        original.splitlines(keepends=True), emitted.splitlines(keepends=True),
        fromfile="docs/canboat.xml", tofile="emitted",
    )
    with open(path, "w", encoding="utf-8") as f:
        f.writelines(diff)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(prog="keel", description="CANboat PGN database tool")
    parser.add_argument("--root", default=".", help="repository root (default: auto-detect upward)")
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("convert", help="bootstrap: convert docs/canboat.xml to database/ YAML")
    p.add_argument("--out", default="database", help="output directory relative to repo root")
    p.add_argument("--diff", help="write unified diff of non-reproducing output to this file")
    p.add_argument("--actisense-xml", help="analyzer-explain -explain-ngt-xml output (default: run the binary)")
    p.add_argument("--ikonvert-xml", help="analyzer-explain -explain-ik-xml output (default: run the binary)")
    p.add_argument("--j1939-xml", help="analyzer-explain-j1939 -explain-xml output (default: run the binary)")
    p.add_argument("--verbose", action="store_true")
    p.set_defaults(func=cmd_convert)

    p = sub.add_parser("generate", help="generate artifacts from database/")
    p.add_argument("--check", action="store_true", help="verify committed artifacts are current; write nothing")
    p.add_argument("--diff", help="write unified diff on --check failure to this file")
    p.set_defaults(func=cmd_generate)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
