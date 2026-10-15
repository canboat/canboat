# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests JSON stability
#
.PHONY: test-pgn-json-debug
tests: test-pgn-json-debug
test-pgn-json-debug:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json-debug.out -debug -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json-debug.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json-debug.out
	diff $(TEMPDIR)/pgn-test-json-debug.out pgn-test-json-debug.out
	diff $(TEMPDIR)/pgn-test-json-debug.err pgn-test-json-debug.err
