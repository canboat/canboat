# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

# A real ShipModul MiniPlex-3Wi-N2K (firmware 3.41.0), from ShipModul (the
# same as samples/miniplex-3wi-n2k-*.txt): its NMEA 0183 output, with the
# Address Claims and Product Information of the bus's eight devices as
# $PSMDNC and $PSMDNP, among $GPRMC and others; then its $MXPGN output, with
# 129029 and 129540 in their fast-packet frames.
.PHONY: test-miniplex
tests: test-miniplex
test-miniplex:
	$(ANALYZER) -json -nv < miniplex.in > $(TEMPDIR)/miniplex.out -fixtime miniplex 2> $(TEMPDIR)/miniplex.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/miniplex.out
	diff $(TEMPDIR)/miniplex.out miniplex.out
	diff $(TEMPDIR)/miniplex.err miniplex.err
