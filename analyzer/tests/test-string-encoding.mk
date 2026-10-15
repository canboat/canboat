# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# 8-bit string fields carry whichever encoding the sending device felt like, so
# the four lines cover both branches and the unset case:
#   129285 from a B&G      Latin-1 "Fæke routé" (e6, e9), which is not valid
#                          UTF-8 -- and "Mats?ra Se", where the plotter itself
#                          substituted '?' for a character it could not encode
#   126998 from a Fusion   UTF-8 "Matsūra Fæké" (c5 ab = U+016B), in the same
#                          STRING_LAU field type under the same control byte
#   130820 from a Fusion   UTF-8 in a STRING_LZ
#   126998 all 0xff        an unset field, which drops out entirely
#
# validate-json.py is the real assertion here: it reads the output as UTF-8, so
# it fails if we emit a raw high byte, which is what we used to do. See #864.
#
.PHONY: test-string-encoding
tests: test-string-encoding
test-string-encoding:
	$(ANALYZER) -json -nv < pgn-string-encoding.in > $(TEMPDIR)/pgn-string-encoding.out -fixtime pgn-string-encoding 2> $(TEMPDIR)/pgn-string-encoding.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-string-encoding.out
	diff $(TEMPDIR)/pgn-string-encoding.out pgn-string-encoding.out
	diff $(TEMPDIR)/pgn-string-encoding.err pgn-string-encoding.err
