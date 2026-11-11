# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# -src and -dst take a whole decimal address from 0 to 255. Each message in
# address-filters.in has a different source and destination, so a filter
# applied to the wrong field, or not applied at all, lets the wrong line through.
#
.PHONY: test-address-filters
tests: test-address-filters
test-address-filters:
	@set -e; for option in -src -dst; do \
	  if [ $$option = -src ]; then column=3; else column=4; fi; \
	  for address in 0 12 255; do \
	    $(ANALYZER) -q $$option $$address < address-filters.in > $(TEMPDIR)/address-filters.out; \
	    if ! awk -v c=$$column -v a=$$address '$$c != a { bad = 1 } END { exit bad || NR != 1 }' $(TEMPDIR)/address-filters.out; then \
	      echo "$$option $$address did not select exactly the one matching message:"; cat $(TEMPDIR)/address-filters.out; exit 1; \
	    fi; \
	  done; \
	  for address in '' abc 12oops ' 12' '+12' -1 256 9999999999999999999999; do \
	    if $(ANALYZER) -q $$option "$$address" </dev/null >/dev/null 2>$(TEMPDIR)/address-filters.err; then \
	      echo "Unexpectedly accepted $$option '$$address'"; exit 1; \
	    fi; \
	    grep -F -- "Invalid address for $$option:" $(TEMPDIR)/address-filters.err >/dev/null; \
	  done; \
	done
