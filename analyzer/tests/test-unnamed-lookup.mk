# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A lookup value the lookup does not name is a value unless it is one of
# the field's own sentinels (#1156): 127501's 2-bit Indicator11 of 2
# ("error"), 129540's Range Residual Mode 2, 129038's Nav Status 13 and
# a catch-all's Industry Code 6 are printed as numbers.
#
.PHONY: test-unnamed-lookup
tests: test-unnamed-lookup
test-unnamed-lookup:
	$(ANALYZER) -json < pgn-unnamed-lookup.in > $(TEMPDIR)/pgn-unnamed-lookup.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-unnamed-lookup.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-unnamed-lookup.out
	diff $(TEMPDIR)/pgn-unnamed-lookup.out pgn-unnamed-lookup.out
	diff $(TEMPDIR)/pgn-unnamed-lookup.err pgn-unnamed-lookup.err
