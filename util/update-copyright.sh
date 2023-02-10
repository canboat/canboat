#!/bin/bash

year=$(date +%Y)
for file in $(git diff --name-only f69073317cfefe45a75433c187539d76f91f0a6e)
do
  if grep -- '-20[0-9][0-9], Kees Verruijt' "${file}" 
  then
    echo "--- <$file>"
    sed -i "" 's/-20[0-9][0-9], Kees Verruijt/-'${year}', Kees Verruijt/' "${file}"
  fi
done
