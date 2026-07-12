# Contributing to CANboat

Thanks for helping improve CANboat! This guide covers how to get a change
merged. For build instructions and the architecture of the PGN database, see
**[AGENTS.md](AGENTS.md)**.

## The cardinal rule: source and generated move together

The NMEA 2000 PGN database is **YAML** under `database/` (one file per PGN
variant in `pgns/`, one per enumeration in `lookups/`, plus
`fieldtypes.yaml`; `database/j1939/` for the J1939 build). Validate your edit
with `keel/keel check`. The files under `docs/` (`canboat.xml`,
`canboat.html`, `canboat.json`), `docs/canboat.dbc`, AND the analyzer's C
data tables (`analyzer/lookup.h`, `analyzer/*-data.h`) are **generated**
from the database and committed to git. If your change touches the
database, regenerate the artifacts in the **same** commit:

```sh
make generated
```

`make generated` runs the full test suite first, so it also tells you if a
golden output changed. Source and generated files must move together or CI
will fail.

## Pull request titles

We **squash-merge** every pull request, so the **PR title becomes the commit
message** on `master`. Our release tooling
([release-please](https://github.com/googleapis/release-please)) parses that
title to decide the next version number and to write `CHANGELOG.md`. A good
title is therefore *required* — it is release metadata, not decoration.

Use the [Conventional Commits](https://www.conventionalcommits.org) format:

```
type(scope): short summary in the imperative
```

- **type** — required, lowercase, from the table below.
- **scope** — optional, in parentheses. Use a manufacturer (`navico`,
  `mercury`), a PGN number (`126993`), an issue number (`693`), or a subsystem
  (`json`, `socketcan-serial`).
- **summary** — concise, lower-case start, no trailing period, imperative mood
  ("add", not "added" or "adds").

### Types and what they do

| Type | Version bump | CHANGELOG section |
|------|--------------|-------------------|
| `feat` | **minor** (x.**Y**.z) | Added |
| `fix` | **patch** (x.y.**Z**) | Fixed |
| `perf`, `refactor` | none | Changed |
| `revert` | none | Fixed |
| `docs`, `ci`, `build`, `test`, `chore` | none | hidden |

A PR whose title is only a hidden type (e.g. `chore:`) does **not** trigger a
release on its own.

### Breaking changes

Add `!` after the type/scope for an incompatible change. This forces a
**major** bump (X.y.z):

```
feat(navico)!: rework Simnet 130845/130846 Key/Parameter PGNs from firmware
```

### Examples (from this repo)

```
fix(126993): heartbeat "Data transmit offset" is in milliseconds
feat(navico): decode 65313 Depth Quality and 130825 NDP2k Alert
feat(693): NMEA 2000 gateway network status PGN — rename + emission
feat(navico)!: rework Simnet 130845/130846 Key/Parameter PGNs from firmware
chore: add real-world Fusion and Victron sample captures
```

### Common mistakes

- **No type prefix** (`Improve B&G keys`) — release-please ignores it: no
  version bump, no changelog entry.
- **A type it does not recognise** (`navico(130824): …`) — also ignored. The
  title must *start* with a known type; put the manufacturer in the **scope**:
  `feat(navico): …`.
- **Past tense, capitalised, or a trailing period** — works, but reads badly in
  the changelog.
- **Marking database work as `chore`** — new PGN/field support is `feat`; a
  wrong decode is `fix`.

## Adding or fixing a PGN

The recommended path is the evidence-first editor:

```sh
keel/keel edit          # local web editor, no network, no npm
```

Paste real captures (canboat PLAIN, candump, or raw hex) and the editor
reassembles them, stacks the bytes so structure stands out, and decodes them
live as you build the field list. If your frame nearly matches an existing
definition it offers to **clone** that variant so you don't redefine shared
fields. Enumerations (pair/bit/triplet/fieldtype) are editable in place, and
unnamed values a sample reveals are flagged. Saving is refused while any rule
fails, and you can adopt the decodes as regression expectations that
`keel check` re-verifies forever.

You can also edit the YAML under `database/` by hand and run `keel/keel check`.
Either way, author only the observed data — order, bit offsets, lengths, ranges
and sentinels are derived. AGENTS.md §9 has the full workflow.

## Database changes should be backed by evidence

New or corrected decodes should be justified by a real capture (add it under
`samples/` and cite it in the PGN's `notes:` key) or by public documentation. The
goal is a database that is reverse-engineered from observation, not guesswork.

## Need help?

Open an issue, or a draft PR to discuss an approach before polishing it.
