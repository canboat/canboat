# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that a repeating item whose first field the payload ends inside is left out (issue #1105)
#
.PHONY: test-short-list
tests: test-short-list
test-short-list:
	$(ANALYZER) < short-list.in > $(TEMPDIR)/short-list.out -json -q -fixtime short-list
	diff $(TEMPDIR)/short-list.out short-list.out
