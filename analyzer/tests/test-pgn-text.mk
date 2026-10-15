# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests TEXT stability
#
.PHONY: test-pgn-text
tests: test-pgn-text
test-pgn-text:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test.out -debug -q -fixtime pgn-test 2> $(TEMPDIR)/pgn-test.err
	diff $(TEMPDIR)/pgn-test.out pgn-test.out
	diff $(TEMPDIR)/pgn-test.err pgn-test.err
