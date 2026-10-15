# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

.PHONY: test-pgn-130823-directory
tests: test-pgn-130823-directory
test-pgn-130823-directory:
	$(ANALYZER) -json -nv < pgn-130823-directory.in > $(TEMPDIR)/pgn-130823-directory.out -fixtime pgn-130823-directory 2> $(TEMPDIR)/pgn-130823-directory.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-130823-directory.out
	diff $(TEMPDIR)/pgn-130823-directory.out pgn-130823-directory.out
	diff $(TEMPDIR)/pgn-130823-directory.err pgn-130823-directory.err
