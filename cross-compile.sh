#!/bin/bash

set -euo pipefail

echo "Crosscompiling for ${1}"

export LD="${1}-gcc"
export CC="${1}-gcc"
export PLATFORM="${1}"

make -e
