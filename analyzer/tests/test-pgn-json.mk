# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests JSON stability
#
.PHONY: test-pgn-json
tests: test-pgn-json
test-pgn-json:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json.out -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json.out
	diff $(TEMPDIR)/pgn-test-json.out pgn-test-json.out
	diff $(TEMPDIR)/pgn-test-json.err pgn-test-json.err
