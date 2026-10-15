# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that analyzer correctly parses files with nested ISO_NAME fields
#
.PHONY: test-pgn-126983-nv
tests: test-pgn-126983-nv
test-pgn-126983-nv:	
	$(ANALYZER) -json -nv < pgn-126983.in > $(TEMPDIR)/pgn-126983-nv.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-126983-nv.err
	diff $(TEMPDIR)/pgn-126983-nv.out pgn-126983-nv.out
	diff $(TEMPDIR)/pgn-126983-nv.err pgn-126983-nv.err
