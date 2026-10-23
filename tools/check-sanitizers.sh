#!/bin/sh
#
# Copyright (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# Build the C tools with AddressSanitizer and UndefinedBehaviorSanitizer, run
# the C regression suite with them, and decode every capture in samples/.
#
# Undefined behaviour usually prints the right answer, until a compiler
# decides otherwise; #1082 shifted into bit 63 of a signed int64_t for every
# 64-bit BITLOOKUP field and nothing noticed. The suite alone does not reach
# every field type, which is why the samples are decoded as well. A candump
# capture goes through candump2analyzer first; a sample that is not input at
# all is skipped by name, and any other that exits nonzero fails the check,
# so a sample cannot quietly go undecoded.
#
# The build goes to rel/sanitize, so it does not touch the normal one.
#
#   tools/check-sanitizers.sh          # needs clang (or CC=gcc)

set -eu

cd "$(dirname "$0")/.."

CC=${CC:-clang}
SAN="-fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer"
# Leak checking is off: memory a tool holds until it exits is not what this
# is looking for.
ASAN_OPTIONS=${ASAN_OPTIONS:-detect_leaks=0}
export ASAN_OPTIONS

make -j"${JOBS:-4}" PLATFORM=sanitize CC="$CC" CFLAGS="-Wall -O1 -g $SAN" LDFLAGS="$SAN" tests

# Not analyzer input: a JSON payload dump, an NGT-1 serial trace, and the
# analyzer's own text output.
skip='samples/NKpayloadsW2K-1.txt samples/actisense-ngt1-fw2690.txt samples/merrimac-ydwg-2020.txt'

err=$(mktemp)
frames=$(mktemp)
trap 'rm -f "$err" "$frames"' EXIT
fail=0

# run NAME CMD...: run CMD (stdin already redirected by the caller); a
# nonzero exit or a sanitizer report fails the check.
run() {
  name=$1
  shift
  status=0
  "$@" 2> "$err" || status=$?
  if [ "$status" -ne 0 ] || grep -q -e 'Sanitizer' -e 'runtime error' "$err"; then
    echo "::error::$name exited with status $status" >&2
    grep -m 5 -e 'Sanitizer' -e 'runtime error' -e 'ERROR' "$err" >&2 || true
    fail=1
    return 1
  fi
}

for f in $(git ls-files samples | grep -E '\.(raw|log|txt|candump|asc|n2k|all)$'); do
  case " $skip " in *" $f "*) continue ;; esac
  input=$f
  # A candump capture goes through candump2analyzer first, as it would live.
  if awk '!/^#/ && NF { print; exit }' "$f" \
    | grep -qE '^\(|^interface = |^ *<0x|^ +[[:alnum:]]+ +[[:xdigit:]]+ +\[[0-9]+\]'; then
    run "candump2analyzer $f" rel/sanitize/candump2analyzer < "$f" > "$frames" || continue
    input=$frames
  fi
  for analyzer in analyzer analyzer-j1939; do
    run "$analyzer $f" rel/sanitize/$analyzer -json -nv < "$input" > /dev/null || true
  done
done
exit $fail
