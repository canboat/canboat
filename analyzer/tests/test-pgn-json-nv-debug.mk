# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests JSON stability
#
.PHONY: test-pgn-json-nv-debug
tests: test-pgn-json-nv-debug
test-pgn-json-nv-debug:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json-nv-debug.out -nv -debug -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json-nv-debug.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json-nv-debug.out
	diff $(TEMPDIR)/pgn-test-json-nv-debug.out pgn-test-json-nv-debug.out
	diff $(TEMPDIR)/pgn-test-json-nv-debug.err pgn-test-json-nv-debug.err
