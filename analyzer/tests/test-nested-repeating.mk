# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A repeating set nested in another: the B&G H5000 table dumps of PGN 130822
# Sections 11 to 15 and 18, each a list of tables that each hold a list of
# values (#1198).
#
.PHONY: test-nested-repeating
tests: test-nested-repeating
test-nested-repeating:
	$(ANALYZER) -json < nested-repeating.in > $(TEMPDIR)/nested-repeating.out -fixtime nested-repeating 2> $(TEMPDIR)/nested-repeating.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/nested-repeating.out
	diff $(TEMPDIR)/nested-repeating.out nested-repeating.out
	diff $(TEMPDIR)/nested-repeating.err nested-repeating.err
	$(ANALYZER) < nested-repeating.in > $(TEMPDIR)/nested-repeating-text.out -fixtime nested-repeating 2> /dev/null
	diff $(TEMPDIR)/nested-repeating-text.out nested-repeating-text.out
