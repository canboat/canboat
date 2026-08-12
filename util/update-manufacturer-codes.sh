#!/bin/sh
#
# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Compare a manufacturer list against one of canboat's lookup tables and
# report what differs. Read-only: it prints, it never edits the YAML.
#
#   util/update-manufacturer-codes.sh <list.tsv> [lookup]
#
#     <list.tsv>   code<TAB>name[<TAB>anything]  (extra columns ignored)
#     [lookup]     MANUFACTURER_CODE (default) or J1939_MANUFACTURER_CODE
#
# For the ISO 11783 / SAE J1939 registry, generate the list with
#
#     tools/isobus-registry.py --tsv > /tmp/isobus.tsv
#     util/update-manufacturer-codes.sh /tmp/isobus.tsv J1939_MANUFACTURER_CODE
#
# The two tables are different allocations of the same 11-bit space, so never
# reconcile the marine table against the ISO registry - see github issue #837.
#
# The previous version of this script had three faults worth remembering,
# because each one hid real drift for years:
#
#   * It read analyzer/lookup.h, which no longer exists; the database moved to
#     database/lookups/*.yaml.
#   * It was append-only. A code already present was skipped unconditionally,
#     so a name that was wrong when first added could never be corrected. It
#     reported missing codes and nothing else.
#   * Its `grep -q "$code"` was unanchored, so a genuinely new code was
#     silently dropped whenever its digits appeared inside an existing one:
#     with 116 present, code 16 read as "already there". Hence `grep -qx`
#     below, and why this rewrite compares with awk on exact fields instead.

set -eu

LIST=${1:-}
LOOKUP=${2:-MANUFACTURER_CODE}

if [ -z "$LIST" ] || [ ! -r "$LIST" ]; then
    echo "usage: $0 <list.tsv> [MANUFACTURER_CODE|J1939_MANUFACTURER_CODE]" >&2
    exit 1
fi

ROOT=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
YAML="$ROOT/database/lookups/$LOOKUP.yaml"

if [ ! -r "$YAML" ]; then
    echo "$0: no such lookup: $YAML" >&2
    exit 1
fi

# code<TAB>name from the YAML's `values:` block.
sed -n 's/^  \([0-9][0-9]*\): \(.*\)$/\1	\2/p' "$YAML" > /tmp/canboat-codes.$$
# code<TAB>name from the supplied list, first two columns only.
awk -F'\t' 'NF>=2 && $1 ~ /^[0-9]+$/ {print $1 "\t" $2}' "$LIST" > /tmp/list-codes.$$
trap 'rm -f /tmp/canboat-codes.$$ /tmp/list-codes.$$ /tmp/report.$$' EXIT

echo "$LOOKUP: $(wc -l < /tmp/canboat-codes.$$) codes; list: $(wc -l < /tmp/list-codes.$$) codes"
echo

# Classify each code, then sort with sort(1): awk's asorti() is a gawk
# extension and the stock macOS awk does not have it.
awk -F'\t' '
    NR==FNR { have[$1] = $2; next }
    {
        seen[$1] = 1
        if (!($1 in have))       printf "1\t%s\t%s\n", $1, $2
        else if (have[$1] != $2) printf "2\t%s\t%s\t%s\n", $1, have[$1], $2
    }
    END { for (c in have) if (!(c in seen)) printf "3\t%s\t%s\n", c, have[c] }
' /tmp/canboat-codes.$$ /tmp/list-codes.$$ | sort -t"$(printf '\t')" -k1,1 -k2,2n > /tmp/report.$$

section() {  # $1 = tag, $2 = heading, $3 = format
    n=$(awk -F'\t' -v t="$1" '$1==t' /tmp/report.$$ | wc -l | tr -d ' ')
    [ "$n" -eq 0 ] && return 0
    printf -- "--- %s (%s) ---\n" "$2" "$n"
    awk -F'\t' -v t="$1" -v fmt="$3" '$1==t { printf fmt, $2, $3, $4 }' /tmp/report.$$
    echo
}

section 1 "in the list, not in $LOOKUP"    '  %5d  %s\n'
section 2 "present but named differently"  '  %5d  canboat: %-45s list: %s\n'
section 3 "in $LOOKUP, not in the list"    '  %5d  %s\n'

[ -s /tmp/report.$$ ] || echo "identical."
rm -f /tmp/report.$$
