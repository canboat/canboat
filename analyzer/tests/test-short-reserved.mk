# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that a message that is short by just a trailing reserved/spare
# field still decodes instead of raising a PGN analysis error (issue #663).
#
.PHONY: test-short-reserved
tests: test-short-reserved
test-short-reserved:
	$(ANALYZER) -json -nv < short-reserved.in > $(TEMPDIR)/short-reserved.out -fixtime short-reserved 2> $(TEMPDIR)/short-reserved.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/short-reserved.out
	diff $(TEMPDIR)/short-reserved.out short-reserved.out
	diff $(TEMPDIR)/short-reserved.err short-reserved.err
