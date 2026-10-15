# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests JSON stability
#
.PHONY: test-pgn-json-nv
tests: test-pgn-json-nv
test-pgn-json-nv:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json-nv.out -nv -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json-nv.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json-nv.out
	diff $(TEMPDIR)/pgn-test-json-nv.out pgn-test-json-nv.out
	diff $(TEMPDIR)/pgn-test-json-nv.err pgn-test-json-nv.err
