# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# Under -empty, a trailing Reserved or Spare field is written at its default
# value, and the bytes no field covers are written as "trailing" (#1169).
#
.PHONY: test-empty-trailing
tests: test-empty-trailing
test-empty-trailing:
	$(ANALYZER) -json -empty < empty-trailing.in > $(TEMPDIR)/empty-trailing.out -fixtime empty-trailing 2> $(TEMPDIR)/empty-trailing.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/empty-trailing.out
	diff $(TEMPDIR)/empty-trailing.out empty-trailing.out
	diff $(TEMPDIR)/empty-trailing.err empty-trailing.err
