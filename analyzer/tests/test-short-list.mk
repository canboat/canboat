# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that a repeating item whose first field the payload ends inside is left out (issue #1105)
#
.PHONY: test-short-list
tests: test-short-list
test-short-list:
	$(ANALYZER) < short-list.in > $(TEMPDIR)/short-list.out -json -q -fixtime short-list
	diff $(TEMPDIR)/short-list.out short-list.out
	$(ANALYZER_J1939) < short-list-j1939.in > $(TEMPDIR)/short-list-j1939.out -json -q -fixtime short-list
	diff $(TEMPDIR)/short-list-j1939.out short-list-j1939.out
