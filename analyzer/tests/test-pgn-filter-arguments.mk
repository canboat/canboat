# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-pgn-filter-arguments
tests: test-pgn-filter-arguments
test-pgn-filter-arguments:
	@$(ANALYZER) -q 128267 </dev/null >/dev/null 2>&1
	@set -e; for pgn in 128267oops 4295095563 9999999999999999999999; do \
	  if $(ANALYZER) -q "$$pgn" </dev/null >/dev/null 2>&1; then \
	    echo "Unexpectedly accepted PGN '$$pgn'"; exit 1; \
	  fi; \
	done
