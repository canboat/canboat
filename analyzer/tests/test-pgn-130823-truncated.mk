# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-pgn-130823-truncated
tests: test-pgn-130823-truncated
test-pgn-130823-truncated:
	$(ANALYZER) -json -nv < pgn-130823-truncated.in > $(TEMPDIR)/pgn-130823-truncated.out -fixtime pgn-130823-truncated 2> $(TEMPDIR)/pgn-130823-truncated.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-130823-truncated.out
	diff $(TEMPDIR)/pgn-130823-truncated.out pgn-130823-truncated.out
	diff $(TEMPDIR)/pgn-130823-truncated.err pgn-130823-truncated.err
