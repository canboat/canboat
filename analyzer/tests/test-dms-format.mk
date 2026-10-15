# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that analyzer correctly rounds seconds in long/lat in DMS format
#
.PHONY: test-dms-format
tests: test-dms-format
test-dms-format:	
	$(ANALYZER) -q -geo dd  < dms-format.in >  $(TEMPDIR)/dms-format.out -fixtime pgn-test 2>  $(TEMPDIR)/dms-format.err
	$(ANALYZER) -q -geo dm  < dms-format.in >> $(TEMPDIR)/dms-format.out -fixtime pgn-test 2>> $(TEMPDIR)/dms-format.err
	$(ANALYZER) -q -geo dms < dms-format.in >> $(TEMPDIR)/dms-format.out -fixtime pgn-test 2>> $(TEMPDIR)/dms-format.err
	diff $(TEMPDIR)/dms-format.out dms-format.out
	diff $(TEMPDIR)/dms-format.err dms-format.err
