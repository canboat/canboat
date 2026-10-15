# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

# Actisense $MXPGN: the data in bus order, not ShipModul's. The Actisense SDK's
# example (127508 at 12.00 V), a W2K-2's Position, COG & SOG and a whole
# fast-packet (129029), and an ISO Request in the send form. As MXPGN it is no
# position at all.
.PHONY: test-actisense-mxpgn
tests: test-actisense-mxpgn
test-actisense-mxpgn:
	$(ANALYZER) -json -nv -format ACTISENSE_MXPGN < actisense-mxpgn.in > $(TEMPDIR)/actisense-mxpgn.out -fixtime actisense-mxpgn 2> $(TEMPDIR)/actisense-mxpgn.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/actisense-mxpgn.out
	diff $(TEMPDIR)/actisense-mxpgn.out actisense-mxpgn.out
	diff $(TEMPDIR)/actisense-mxpgn.err actisense-mxpgn.err
