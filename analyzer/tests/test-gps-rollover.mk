# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# -quirk gps-rollover. The input is one boat's worth of rolled-over dates: a
# DSC VHF (source 4, Raymarine 1851:491603 -- its PGN 60928 claim comes first)
# stamping PGN 129808 Date of Receipt 2007.01.11 from a GPS that never learned
# about the 2019 week rollover, and the converter at source 115 relaying that
# clock as PGN 126992, once as a GPS source and once as a local crystal clock.
# The 129808 frame is real (samples/pgn129808.raw, address changed); the
# rest is hand-built. All three dates are 1024 weeks short of 2026.08.27.
#
#   gnss     -quirk gps-rollover              only the GPS-sourced 126992 moves
#   product  -quirk gps-rollover=1851:491603  the VHF's 129808 moves too, matched
#                                             by the NAME learned from its claim
#   all      -quirk gps-rollover=all          every date moves, including the
#                                             crystal-clock 126992
#
# The AIS 129794 from the same VHF carries another vessel's ETA and must never
# move. The reference day is the wall clock floored at 2026-01-01, so these
# outputs hold until ~2032, when a 2007 date is closer to the 2038 epoch than
# to 2019.
#
.PHONY: test-gps-rollover
tests: test-gps-rollover
test-gps-rollover:
	$(ANALYZER) -json -nv -quirk gps-rollover < pgn-gps-rollover.in > $(TEMPDIR)/pgn-gps-rollover-gnss.out -fixtime pgn-gps-rollover 2> $(TEMPDIR)/pgn-gps-rollover-gnss.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-gps-rollover-gnss.out
	diff $(TEMPDIR)/pgn-gps-rollover-gnss.out pgn-gps-rollover-gnss.out
	diff $(TEMPDIR)/pgn-gps-rollover-gnss.err pgn-gps-rollover-gnss.err
	$(ANALYZER) -json -nv -quirk gps-rollover=1851:491603 < pgn-gps-rollover.in > $(TEMPDIR)/pgn-gps-rollover-product.out -fixtime pgn-gps-rollover 2> $(TEMPDIR)/pgn-gps-rollover-product.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-gps-rollover-product.out
	diff $(TEMPDIR)/pgn-gps-rollover-product.out pgn-gps-rollover-product.out
	diff $(TEMPDIR)/pgn-gps-rollover-product.err pgn-gps-rollover-product.err
	$(ANALYZER) -json -nv -quirk gps-rollover=all < pgn-gps-rollover.in > $(TEMPDIR)/pgn-gps-rollover-all.out -fixtime pgn-gps-rollover 2> $(TEMPDIR)/pgn-gps-rollover-all.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-gps-rollover-all.out
	diff $(TEMPDIR)/pgn-gps-rollover-all.out pgn-gps-rollover-all.out
	diff $(TEMPDIR)/pgn-gps-rollover-all.err pgn-gps-rollover-all.err
