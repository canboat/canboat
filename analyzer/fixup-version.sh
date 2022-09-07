#!/bin/bash

set -euo pipefail

ANALYZER="${1}"
FILE="${2}"

NEW_VERSION="$("${ANALYZER}" -version)"

sed -i .bak 's/"version": "[0-9\.]*"/"version": "'"${NEW_VERSION}"'"/' "${FILE}" && rm "${FILE}.bak"


