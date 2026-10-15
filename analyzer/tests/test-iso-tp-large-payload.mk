# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# This tests that the message output buffer (print.c's mbuf, backed by the
# shared growable StringBuffer) does not silently truncate a large decoded
# message. A 148-satellite PGN 129540 payload - the largest a 148-satellite
# GNSS Sats in View list reassembled via ISO Transport Protocol can be
# (255 packets * 7 bytes, ISO TP's own ceiling) - renders to ~50KB in
# -nv -debug -json mode, well past the old fixed 8192-byte buffer.
#
.PHONY: test-iso-tp-large-payload
tests: test-iso-tp-large-payload
test-iso-tp-large-payload:
	$(ANALYZER) -nv -debug -q -json < iso-tp-large-payload-test.in > $(TEMPDIR)/iso-tp-large-payload-test.out -fixtime iso-tp-large-payload-test 2> $(TEMPDIR)/iso-tp-large-payload-test.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/iso-tp-large-payload-test.out
	diff $(TEMPDIR)/iso-tp-large-payload-test.out iso-tp-large-payload-test.out
	diff $(TEMPDIR)/iso-tp-large-payload-test.err iso-tp-large-payload-test.err
