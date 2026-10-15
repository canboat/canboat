# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that analyzer correctly formats seconds in Actisense N2K format
#
.PHONY: test-actisense-format
tests: test-actisense-format
test-actisense-format:	
	$(ANALYZER) -q < actisense-format.in > $(TEMPDIR)/actisense-format.out -fixtime pgn-test 2>> $(TEMPDIR)/actisense-format.err
	diff $(TEMPDIR)/actisense-format.out actisense-format.out
	diff $(TEMPDIR)/actisense-format.err actisense-format.err
