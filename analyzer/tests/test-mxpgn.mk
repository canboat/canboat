# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

# ShipModul MiniPlex $MXPGN (#997): the ShipModul examples, received and to send,
# behind a TAG block, a fast-packet in its frames, and other NMEA 0183 sentences
# before and among them, which hold no frame. The last line's checksum is wrong.
.PHONY: test-mxpgn
tests: test-mxpgn
test-mxpgn:
	$(ANALYZER) -json -nv < mxpgn.in > $(TEMPDIR)/mxpgn.out -fixtime mxpgn 2> $(TEMPDIR)/mxpgn.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/mxpgn.out
	diff $(TEMPDIR)/mxpgn.out mxpgn.out
	diff $(TEMPDIR)/mxpgn.err mxpgn.err
