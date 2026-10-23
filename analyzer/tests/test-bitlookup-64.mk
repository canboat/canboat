# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A 64-bit BITLOOKUP (#1082): PGN 130820 Fusion System Capabilities from an
# MS-WB670 and an MS-RA70N, and one with only bits 0 and 63 set. Walking the
# bits up to 63 must not overflow, and bit 63 prints as an unsigned value.
#
.PHONY: test-bitlookup-64
tests: test-bitlookup-64
test-bitlookup-64:
	$(ANALYZER) -json -nv < pgn-bitlookup-64.in > $(TEMPDIR)/pgn-bitlookup-64.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-bitlookup-64.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-bitlookup-64.out
	diff $(TEMPDIR)/pgn-bitlookup-64.out pgn-bitlookup-64.out
	diff $(TEMPDIR)/pgn-bitlookup-64.err pgn-bitlookup-64.err
