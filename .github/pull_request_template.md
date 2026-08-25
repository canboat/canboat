<!--
  The PR TITLE becomes the squash-merge commit message and drives the version
  bump + CHANGELOG. Use Conventional Commits:  type(scope): summary
  e.g.  fix(126993): correct transmit offset units
  See CONTRIBUTING.md for the type table.
-->

## What does this PR do?



<!--
  CHANGING THE PGN DATABASE? The database is a YAML tree - edit it there, never
  in the generated files:

    1. database/pgns/<pgn:06>-<id>.yaml   (or database/lookups/, database/j1939/)
       or run  keel/keel edit  for a local web editor
    2. keel/keel check                    validates the tree
    3. make generated                     regenerates docs/*, docs/canboat.dbc,
                                          analyzer/*-generated-data.h and the
                                          Rust schema_generated.rs
    4. commit the regenerated files IN THE SAME COMMIT

  docs/canboat.{xml,json,html,dbc}, analyzer/*-generated-data.h and
  *_generated*.rs are OUTPUT. Editing them by hand will be undone by the next
  regeneration, and the build-ubuntu gate (which regenerates and diffs) fails.

  Those files are marked linguist-generated, so GitHub collapses them in the
  "Files changed" view - your actual database edit is the YAML file.

  Branches in this repo get regenerated automatically by the "Autofix generated
  files" workflow if you forget. FORK PRs ARE NOT AUTO-FIXED - run
  `make generated` yourself or build-ubuntu will fail.

  See AGENTS.md for the full workflow.
-->

## Checklist

- [ ] PR **title** follows Conventional Commits (`feat` / `fix` / … — see [CONTRIBUTING.md](../CONTRIBUTING.md))
- [ ] If I changed the PGN database, I edited the **YAML under `database/`** (not the generated files) and `keel/keel check` passes
- [ ] I ran `make generated` and committed the regenerated files in the same commit
- [ ] Tests pass (`make tests`)
- [ ] I ran `make pr` and acted on its contract report (mark the title `type!:` if it reports a breaking change)
- [ ] New or changed decodes are backed by a real capture (added to `samples/`) or public documentation
