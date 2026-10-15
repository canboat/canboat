# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# Sentinels in a field with an offset (#984): PGN 127513 Peukert Exponent is
# raw 0..250 plus 500, so 1.0..1.5. The lines carry 1.25, the top value 1.5,
# and then 0xfd, 0xfe and 0xff, which are reserved, out of range and not
# available, and so have no value.
#
.PHONY: test-offset-sentinels
tests: test-offset-sentinels
test-offset-sentinels:
	$(ANALYZER) -json < pgn-offset-sentinels.in > $(TEMPDIR)/pgn-offset-sentinels.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-offset-sentinels.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-offset-sentinels.out
	diff $(TEMPDIR)/pgn-offset-sentinels.out pgn-offset-sentinels.out
	diff $(TEMPDIR)/pgn-offset-sentinels.err pgn-offset-sentinels.err
