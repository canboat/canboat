# Using the definitions in your own project

CANboat's PGN database is YAML in [`database/`](../database), from which `keel`
generates everything else. For use in your own software it is published in
`docs/`, which is also the [documentation site](https://canboat.github.io/canboat):

| | JSON | XML | Documentation |
| --- | --- | --- | --- |
| NMEA 2000 | `canboat.json` | `canboat.xml` | `canboat.html` |
| SAE J1939 | `canboat-j1939.json` | `canboat-j1939.xml` | `canboat-j1939.html` |
| Quick PCS | `canboat-quick.json` | `canboat-quick.xml` | `canboat-quick.html` |
| canboat's own pseudo-PGNs | `canboat-pseudo.json` | `canboat-pseudo.xml` | |

and NMEA 2000 as a CAN database, `canboat.dbc`. Pin your copy to a release:
semantic versioning tells you of major (x.0.0) and minor (0.x.0) changes to the
definitions.

You may not need the files at all: the `canboat` Rust crate has the database
compiled in ([crates/canboat/README.md](../crates/canboat/README.md)), and
[canboatjs](https://github.com/canboat/canboatjs) is CANboat for JavaScript. The
C tables in `analyzer/` are generated for the analyzer, and their form may
change; do not build on them. The old v1 files, `pgns.xml` and `pgns.json`,
were removed; pin an older release if you still need them.

## XML

`canboat.xsd` is the schema of `canboat.xml`, and documents almost every element
in it: what each attribute means, which are required and which you can skip.
Start with the PGNs and their fields, and add the other definitions as you need
them: if you do not show the names of lookup values, you need no
`LookupEnumerations`.

## JSON

The JSON is generated from the XML by an XSLT transformation, so it has the same
elements and attributes. There is no JSON schema, but `canboat.xsd` documents the
JSON just as well: read it.

## Writing your own decoder

All the data is in the definitions, but each `FieldType` needs its own
conversion code; the C `analyzer` (`analyzer/`) and the Rust crate
(`crates/canboat/src/engine/`) show how.

Plan for **repeating field sets** from the start. Some PGNs carry zero or more
repetitions of a group of fields, a *field set*, and a PGN can have two. They
are given by `RepeatingFieldSet1Size`, `RepeatingFieldSet1StartField` and
`RepeatingFieldSet1CountField`, and the same for set 2; in the
[documentation](https://canboat.github.io/canboat/canboat.html), search for
"set 1" and "set 2". Any field after a field set has no fixed position, so for
those PGNs a decoder has to track where in the data it is. If it does that for
these, it may as well do it for every PGN, instead of relying on each field's
`BitStart` and `BitOffset`. If you skip the PGNs with field sets, every field you
decode has a `BitStart` and `BitOffset`.

## The definitions are a public contract

Other projects generate their types and decoders from `canboat.json`, and key on
its names, units and resolutions, so a change to a definition can break them.
Every change to it is classified, and a release's version follows:

| Change to `canboat.json` | Release |
| --- | --- |
| A new PGN, field, lookup or lookup value | minor (0.x.0) |
| A field, PGN, lookup or lookup value removed or renamed; a field's type, lookup, bits, sign, resolution or offset changed; a PGN's type or match changed | minor (0.x.0) |
| A description or unit text | patch (0.0.x) |

So read the release notes of a minor release before you take it.
[CONTRIBUTING.md](../CONTRIBUTING.md) says how a change is checked.
