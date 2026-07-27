#!/bin/bash
# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# Manual parity check: drive canboat C n2kd and canboat-rs n2kd from
# the same analyzer-JSON input, then capture each of the five TCP
# ports on both and report per-port line counts. Use it after any
# change that touches `crates/canboat/src/n2kd/` or `canboat_core::snapshot` to
# confirm the shapes still line up with canboat C.
#
# Not run by `cargo test` — this needs a built canboat C n2kd binary,
# free local TCP ports, and a few seconds of wall-clock; keep it as a
# manual smoke test invoked by hand.
#
# Inputs (override via env):
#   C_N2KD    path to canboat C n2kd binary
#             default: ../../../canboat/rel/darwin-arm64/n2kd
#   RS_N2KD   path to canboat-rs n2kd — now the `n2kd` argv[0] shim
#             into the unified `canboat` binary. Create it with
#             `./target/release/canboat install-shims` (which drops a
#             `target/release/n2kd -> canboat` symlink), or point this
#             at `canboat` renamed/symlinked to `n2kd`.
#             default: ../../../target/release/n2kd
#   INPUT     path to analyzer-JSON file (canboat C needs `-json -nv`
#             input — the first line must contain both `"version":`
#             and `"showLookupValues":true`).
#             default: /tmp/live-analyzed-nv.json
#
# To capture a fresh INPUT from a live canboat-pipeline (e.g. on the
# Pi), tap its analyzer-JSON stream port directly (already -nv) and
# prepend the version banner canboat C wants:
#
#   { echo '{"version":"7.1.0","units":"std","showLookupValues":true}'; \
#     timeout 60 nc <pipeline-host> 2598; } > /tmp/live-analyzed-nv.json
#
# Then `canboat install-shims` (once) and `./run-parity.sh`.

set -u

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
C_N2KD="${C_N2KD:-$REPO_ROOT/../canboat/rel/darwin-arm64/n2kd}"
RS_N2KD="${RS_N2KD:-$REPO_ROOT/target/release/n2kd}"
INPUT="${INPUT:-/tmp/live-analyzed-nv.json}"

# Non-overlapping base ports so both versions can run side-by-side.
C_BASE=13597
RS_BASE=23597

# Names per offset 0..5 in canboat C's order.
NAMES=(snapshot stream nmea0183 input ais status)

OUT="${OUT:-/tmp/n2kd-parity}"

for path in "$C_N2KD" "$RS_N2KD" "$INPUT"; do
    [[ -f $path ]] || { echo "missing: $path" >&2; exit 1; }
done

# Kill orphans from a previous run.
pkill -9 n2kd 2>/dev/null || true
sleep 1
rm -rf "$OUT"
mkdir -p "$OUT"

#######################
# PASS 1: streaming-port subscribers attach FIRST, then we feed.
#######################
echo "=== streaming pass (subscribe-first) ==="
( sleep 4; cat "$INPUT"; sleep 90 ) | "$C_N2KD"  -p "$C_BASE"  -q > "$OUT/c-stdout.txt"  2> "$OUT/c-stderr.txt"  &
C_PID=$!
( sleep 4; cat "$INPUT"; sleep 90 ) | "$RS_N2KD" -p "$RS_BASE"    > "$OUT/rs-stdout.txt" 2> "$OUT/rs-stderr.txt" &
RS_PID=$!

# Wait for sockets to bind.
for _ in $(seq 1 30); do
    nc -z localhost "$C_BASE"  2>/dev/null && \
    nc -z localhost "$RS_BASE" 2>/dev/null && break
    sleep 0.2
done

hold() { sleep 18; }

# Stream-style ports: stream / nmea0183. NB: AIS and status are
# *snapshot* ports in canboat C too — captured in pass 2.
hold | nc localhost $((C_BASE + 1))  > "$OUT/c-stream.txt"   2>/dev/null &
hold | nc localhost $((RS_BASE + 1)) > "$OUT/rs-stream.txt"  2>/dev/null &
hold | nc localhost $((C_BASE + 2))  > "$OUT/c-nmea0183.txt"  2>/dev/null &
hold | nc localhost $((RS_BASE + 2)) > "$OUT/rs-nmea0183.txt" 2>/dev/null &
sleep 19  # let the hold expire and flush

#######################
# PASS 2: snapshot-style ports (snap / ais / status) — connect after
# the cache is populated; each port dumps once and closes.
#######################
echo "=== snapshot pass (post-cache dumps) ==="
for OFF in 0 4 5; do
    NAME=${NAMES[$OFF]}
    ( sleep 3 ) | nc localhost $((C_BASE  + OFF)) > "$OUT/c-$NAME.txt"  2>/dev/null
    ( sleep 3 ) | nc localhost $((RS_BASE + OFF)) > "$OUT/rs-$NAME.txt" 2>/dev/null
done

# Tear down.
kill "$C_PID" "$RS_PID" 2>/dev/null || true
sleep 1
pkill -9 n2kd 2>/dev/null || true
wait 2>/dev/null || true

#######################
# REPORT
#######################
echo
echo "Per-port summary (lines C / Rust):"
for NAME in snapshot stream nmea0183 ais status; do
    CL=$(wc -l < "$OUT/c-$NAME.txt"  2>/dev/null || echo 0)
    RL=$(wc -l < "$OUT/rs-$NAME.txt" 2>/dev/null || echo 0)
    printf "  %-10s  C:%6s  Rust:%6s\n" "$NAME" "$CL" "$RL"
done

echo
echo "Outputs in $OUT/  (c-*.txt / rs-*.txt)."
echo "Stderr diagnostics in $OUT/c-stderr.txt / rs-stderr.txt."
