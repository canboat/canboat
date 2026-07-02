<!--
  The PR TITLE becomes the squash-merge commit message and drives the version
  bump + CHANGELOG. Use Conventional Commits:  type(scope): summary
  e.g.  fix(126993): correct transmit offset units
  See CONTRIBUTING.md for the type table.
-->

## What does this PR do?



## Checklist

- [ ] PR **title** follows Conventional Commits (`feat` / `fix` / … — see [CONTRIBUTING.md](../CONTRIBUTING.md))
- [ ] If I changed the PGN database (`analyzer/*.h`), I ran `make generated` and committed the regenerated `docs/` and `dbc-exporter/` files
- [ ] Tests pass (`make tests`)
- [ ] New or changed decodes are backed by a real capture (added to `samples/`) or public documentation
