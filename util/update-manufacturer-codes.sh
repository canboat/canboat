
set -euo pipefail

sed -n '/MANUFACTURER_CODE, [0-9]/s/[^0-9]*\([0-9]*\), .*/\1/p' analyzer/lookup.h > /tmp/exist.txt
echo "$(wc -l /tmp/exist.txt) existing manufacturer codes"
sort -t : -n -k 2 /tmp/comp.txt | 
  while IFS=: read manu code
  do
    if grep -q "$code" /tmp/exist.txt
    then
      :
    else
      echo "LOOKUP(MANUFACTURER_CODE, $code,\"$manu\")"
    fi
  done
