#!/usr/bin/env bash
# Gate the `canboat` facade's public API against a checked-in snapshot.
#
# The facade is the ONLY supported dependency for external consumers, so its
# public surface is a contract (see docs/library-api-plan.md §3). This script
# regenerates the surface for each library feature set and diffs it against
# docs/public-api/<featureset>.txt. A diff means the public API changed — either
# update the snapshot on purpose (--bless) or reconsider the change.
#
# Requires: cargo-public-api + a nightly toolchain for rustdoc JSON.
#   rustup toolchain install nightly
#   cargo install cargo-public-api
#
# Usage:
#   scripts/check-public-api.sh          # diff against the snapshots (CI mode)
#   scripts/check-public-api.sh --bless  # overwrite the snapshots
set -euo pipefail

cd "$(dirname "$0")/.."
OUT_DIR="docs/public-api"
mkdir -p "$OUT_DIR"

# The library feature sets whose surfaces we lock. `decode` is the lean
# baseline; each larger set is a superset. The `cli` feature is NOT listed —
# the binary is not part of the public library contract.
FEATURESETS=(
    "decode"
    "decode,io,nmea0183,ais"
    "decode,io,node,bridge,nmea0183,ais"
)

if ! cargo public-api --version >/dev/null 2>&1; then
    echo "error: cargo-public-api not installed." >&2
    echo "  rustup toolchain install nightly && cargo install cargo-public-api" >&2
    exit 127
fi

bless=0
[[ "${1:-}" == "--bless" ]] && bless=1
status=0

for set in "${FEATURESETS[@]}"; do
    name="${set//,/+}"
    snapshot="$OUT_DIR/$name.txt"
    current="$(cargo public-api -p canboat --no-default-features --features "$set" 2>/dev/null)"
    if [[ $bless -eq 1 ]]; then
        printf '%s\n' "$current" > "$snapshot"
        echo "blessed $snapshot"
    elif [[ ! -f "$snapshot" ]]; then
        echo "missing snapshot $snapshot (run with --bless to create)" >&2
        status=1
    elif ! diff -u "$snapshot" <(printf '%s\n' "$current"); then
        echo "PUBLIC API CHANGED for [$set] — bless intentionally or revert" >&2
        status=1
    fi
done

exit $status
