# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A B&G key/value record whose Length byte announces a numeric value wider than
# 64 bits must be skipped cleanly (no shift past the word width).
#
.PHONY: test-keyvalue-wide-field
tests: test-keyvalue-wide-field
test-keyvalue-wide-field:
	$(ANALYZER) -json < keyvalue-wide-field.in > $(TEMPDIR)/keyvalue-wide-field.out -fixtime pgn-test 2> $(TEMPDIR)/keyvalue-wide-field.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/keyvalue-wide-field.out
	diff $(TEMPDIR)/keyvalue-wide-field.out keyvalue-wide-field.out
	diff $(TEMPDIR)/keyvalue-wide-field.err keyvalue-wide-field.err
