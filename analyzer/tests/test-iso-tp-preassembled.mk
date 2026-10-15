# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

# This tests that a message that arrives already reassembled - one FAST format
# line holding the whole payload, as produced by anything that has done the ISO
# Transport Protocol reassembly for us - is accepted when it is longer than a
# fast packet's 223-byte ceiling. The input is the 1779-byte PGN 129540 payload
# of test-iso-tp-large-payload collapsed onto a single line, and the expected output is byte for
# byte what reassembling that test's 255 TP.DT frames produces.
#
.PHONY: test-iso-tp-preassembled
tests: test-iso-tp-preassembled
test-iso-tp-preassembled:
	$(ANALYZER) -json -nv < iso-tp-preassembled-test.in > $(TEMPDIR)/iso-tp-preassembled-test.out -fixtime iso-tp-preassembled-test 2> $(TEMPDIR)/iso-tp-preassembled-test.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/iso-tp-preassembled-test.out
	diff $(TEMPDIR)/iso-tp-preassembled-test.out iso-tp-preassembled-test.out
	diff $(TEMPDIR)/iso-tp-preassembled-test.err iso-tp-preassembled-test.err
