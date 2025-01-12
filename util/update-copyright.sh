#!/bin/bash

set -euo pipefail

if [ "$(uname)" != "Darwin" ]
then
  echo "$0: this only works with macOS sed"
  exit 2
fi

year="$(date '+%Y')"
for f in $(grep -lR ' 2009-20[0-9][0-9], ' .)
do
  if [ "${f}" != "./build/update-copyright.sh" ]
  then
    sed -I .bak "s/ 2009-20[0-9][0-9], / 2009-${year}, /" $f && rm -- "${f}".bak
  fi
done

