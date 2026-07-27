#!/bin/sh
#
# Copyright (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# Guard the keel dependency closure (MERGE-CANBOAT-RS.md §9).
#
# keel is built by a cargo shim inside analyzer/Makefile, so it is on the path
# of anyone regenerating the database. Its dependency closure therefore has to
# stay tiny: dragging in the runtime's tree (tokio, socketcan, ratatui, ...)
# would wreck first-build time for contributors who only touch the C or the
# YAML. That invariant is load-bearing for goals #1 and #4, so it is enforced
# here rather than trusted.
#
# keel and the runtime crates share ONE workspace at the repo root, so a
# dependency edge between them is a one-line edit away -- which is exactly why
# this is checked. It used to be structurally impossible (separate
# workspaces); it no longer is.
#
# NB: `cargo metadata` is useless for this now. In a workspace it reports every
# member and every member's dependencies, so keel would appear to "depend on"
# the whole runtime. `cargo tree -p keel` resolves keel's own subtree, which is
# the thing we actually care about.

set -eu

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"

echo "== keel dependency-closure check =="

closure="$(cd "$root" && cargo tree -p keel -e normal,build --prefix none 2>/dev/null \
  | awk 'NF {print $1}' | sort -u)"

if [ -z "$closure" ]; then
  echo "ERROR: could not resolve keel's dependency tree." >&2
  exit 2
fi

echo "keel's closure is $(echo "$closure" | wc -l | tr -d ' ') packages:"
echo "$closure" | sed 's/^/  /'

status=0

# 1. No runtime crate may appear anywhere in the subtree.
for forbidden in canboat-core canboat-io canboat-tokio canboat-bridge canboat-wire \
                 canboat-schema canboat-cli canboat tokio socketcan ratatui serde serde_json; do
  if echo "$closure" | grep -qx "$forbidden"; then
    echo "ERROR: keel's dependency closure contains '$forbidden'." >&2
    status=1
  fi
done

# 2. Belt and braces: keel must not even name a workspace sibling. Rule 1 would
#    catch it too, but this points at the offending line rather than the fallout.
if grep -qE '^[[:space:]]*canboat[a-z-]*[[:space:]]*=' "$root/keel/Cargo.toml"; then
  echo "ERROR: keel/Cargo.toml declares a dependency on a canboat-* crate." >&2
  status=1
fi

if [ "$status" -eq 0 ]; then
  echo "OK: keel's closure is free of the runtime dependency tree."
fi
exit "$status"
