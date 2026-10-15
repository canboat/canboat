# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that analyzer correctly interprets Raymarine PGN 65379 structure.
# It should not return an error when parsing 126208 Group Command/Acknowledge messages to PGN 65379
#
.PHONY: test-pgn-65379
tests: test-pgn-65379
test-pgn-65379:	
	$(ANALYZER) < pgn-65379-test.in > $(TEMPDIR)/pgn-65379-test.out -debug -q -fixtime pgn-test 2> $(TEMPDIR)/pgn-65379-test.err
	diff $(TEMPDIR)/pgn-65379-test.out pgn-65379-test.out
	diff $(TEMPDIR)/pgn-65379-test.err pgn-65379-test.err
