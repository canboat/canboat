# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A BINARY field that starts mid-byte reads as its value, from bit 0, as a
# number field does (#1143): PGN 130829's Field A (bits 20-23) prints 05,
# not 50. Its Sub Helm and Helm are numbers (#1158). The first frame is
# samples/mercury-proprietary.raw, the second sets Field A = 5, Sub Helm = 3
# and Helm = 0xA.
#
.PHONY: test-binary-mid-byte
tests: test-binary-mid-byte
test-binary-mid-byte:
	$(ANALYZER) -json < pgn-binary-mid-byte.in > $(TEMPDIR)/pgn-binary-mid-byte.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-binary-mid-byte.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-binary-mid-byte.out
	diff $(TEMPDIR)/pgn-binary-mid-byte.out pgn-binary-mid-byte.out
	diff $(TEMPDIR)/pgn-binary-mid-byte.err pgn-binary-mid-byte.err
