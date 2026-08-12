#!/usr/bin/env bash
#
# Copyright (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# Upward-compatibility check for the canboat public contract.
#
# Diffs the committed docs/canboat.json against the merge-base with the target
# branch, classifies the changes (breaking / minor / additive / cosmetic via
# tools/contract.py) and checks that the change is *declared* at a high enough
# conventional-commit level:
#
#   breaking  -> major  (a 'type!:' header or a 'BREAKING CHANGE:' footer)
#   additive  -> minor  (a 'feat:' commit)
#   minor     -> minor
#   cosmetic  -> patch  (anything)
#
# release-please then turns that declaration into the version bump and the
# changelog. This script does not modify anything; it reports, and -- with
# --gate -- exits non-zero when the change is under-declared.
#
# Usage:
#   tools/contract-pr.sh [--gate] [--base <ref>]
#
# Environment (used by CI; optional):
#   BASE_REF   target branch ref to diff against     (default: origin/master)
#   PR_TITLE   pull-request title  (counts towards the declared level)
#   PR_BODY    pull-request body   (counts towards the declared level)

set -euo pipefail

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/.." && pwd)"
canboat_json="$root/docs/canboat.json"

gate=0
base="${BASE_REF:-origin/master}"
while [ $# -gt 0 ]; do
  case "$1" in
    --gate) gate=1 ;;
    --base) shift; base="$1" ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
  shift
done

if ! mergebase="$(git merge-base "$base" HEAD 2>/dev/null)"; then
  echo "contract-pr: cannot find merge-base with '$base' (fetch it first)." >&2
  exit 2
fi

# Baseline contract = canboat.json as it was at the merge-base.
basejson="$(mktemp)"
trap 'rm -f "$basejson"' EXIT
if ! git show "$mergebase:docs/canboat.json" > "$basejson" 2>/dev/null; then
  echo "contract-pr: docs/canboat.json not present at $mergebase." >&2
  exit 2
fi

echo "== canboat upward-compatibility check =="
echo "base: $base ($mergebase)"
echo

# Classify. contract.py exits 0..4 (none/cosmetic/additive/minor/breaking).
set +e
python3 "$here/contract.py" diff "$basejson" "$canboat_json"
sev=$?
set -e

case "$sev" in
  0) required=none  ;;
  1) required=patch ;;   # cosmetic
  2) required=minor ;;   # additive
  3) required=minor ;;   # minor breaking
  4) required=major ;;   # breaking
  *) echo "contract.py failed (exit $sev)"; exit 2 ;;
esac

# What did the author declare? Conventional-commit markers in the commit range,
# plus the PR title/body when CI supplies them.
range_text="$(git log --format='%B' "$mergebase"..HEAD 2>/dev/null || true)"
combined="${PR_TITLE:-}
${PR_BODY:-}
${range_text}"

declared=patch
if printf '%s\n' "$combined" | grep -qE '^[a-z]+(\([^)]*\))?!:' \
   || printf '%s\n' "$combined" | grep -q 'BREAKING CHANGE'; then
  declared=major
elif printf '%s\n' "$combined" | grep -qE '^feat(\([^)]*\))?:'; then
  declared=minor
fi

rank() { case "$1" in none) echo 0;; patch) echo 1;; minor) echo 2;; major) echo 3;; esac; }

echo
echo "required by contract change: $required"
echo "declared by commits/PR:      $declared"

if [ "$(rank "$required")" -le "$(rank "$declared")" ]; then
  echo "OK: the change is declared at a sufficient level."
  exit 0
fi

echo
echo "*** UNDER-DECLARED: this change needs a '$required' release. ***"
if [ "$required" = "major" ]; then
  echo "Mark the change breaking: use a 'type!:' header and add this footer to"
  echo "the commit / PR description so release-please bumps the major version:"
  echo
  # contract.py exits with the severity code; don't let that abort the script.
  python3 "$here/contract.py" diff "$basejson" "$canboat_json" --format footer || true
else
  echo "Use a 'feat:' commit so release-please bumps the minor version."
fi

if [ "$gate" -eq 1 ]; then
  exit 1
fi
echo
echo "(advisory run — not failing. CI will enforce this.)"
exit 0
