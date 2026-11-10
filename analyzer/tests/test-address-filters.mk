# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-address-filters
tests: test-address-filters
test-address-filters:
	@set -e; for option in -src -dst; do \
	  for address in 0 12 255; do \
	    $(ANALYZER) -q $$option $$address </dev/null >/dev/null 2>&1; \
	  done; \
	  for address in '' abc 12oops -1 256 9999999999999999999999; do \
	    if $(ANALYZER) -q $$option "$$address" </dev/null >/dev/null 2>$(TEMPDIR)/address-filters.err; then \
	      echo "Unexpectedly accepted $$option '$$address'"; exit 1; \
	    fi; \
	    grep -F -- "Invalid address for $$option:" $(TEMPDIR)/address-filters.err >/dev/null; \
	  done; \
	done
