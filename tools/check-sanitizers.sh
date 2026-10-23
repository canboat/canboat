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
# every field type, which is why the samples are decoded as well.
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

err=$(mktemp)
trap 'rm -f "$err"' EXIT
fail=0
for f in $(git ls-files samples | grep -E '\.(raw|log|txt|candump|asc|n2k|all)$'); do
  for analyzer in analyzer analyzer-j1939; do
    rel/sanitize/$analyzer -json -nv < "$f" > /dev/null 2> "$err" || true
    if grep -q -e 'Sanitizer' -e 'runtime error' "$err"; then
      echo "::error::$analyzer $f"
      grep -m 5 -e 'Sanitizer' -e 'runtime error' "$err"
      fail=1
    fi
  done
done
exit $fail
