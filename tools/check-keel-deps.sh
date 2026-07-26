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
# keel/ and rust/ are separate Cargo workspaces, which already makes a
# dependency edge between them impossible to express. This script exists so
# that if the two are ever merged into one workspace, the invariant fails
# loudly instead of silently eroding.

set -eu

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"

echo "== keel dependency-closure check =="

# Every package cargo would build for keel, name only.
closure="$(cd "$root/keel" && cargo metadata --format-version 1 --quiet \
  | python3 -c 'import json,sys; print("\n".join(sorted(p["name"] for p in json.load(sys.stdin)["packages"])))')"

echo "keel's closure is $(echo "$closure" | wc -l | tr -d ' ') packages:"
echo "$closure" | sed 's/^/  /'

status=0

# 1. No runtime crate may appear.
for forbidden in canboat-core canboat-io canboat-tokio canboat-bridge canboat-wire \
                 canboat-schema canboat-cli canboat tokio socketcan ratatui serde serde_json; do
  if echo "$closure" | grep -qx "$forbidden"; then
    echo "ERROR: keel's dependency closure contains '$forbidden'." >&2
    status=1
  fi
done

# 2. Nothing may reach into rust/ by path.
if grep -qE '^\s*[a-zA-Z0-9_-]+\s*=.*path\s*=\s*"[^"]*rust/' "$root/keel/Cargo.toml"; then
  echo "ERROR: keel/Cargo.toml declares a path dependency into rust/." >&2
  status=1
fi

if [ "$status" -eq 0 ]; then
  echo "OK: keel's closure is free of the runtime dependency tree."
fi
exit "$status"
