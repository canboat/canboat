# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A PGN filter argument is a whole positive decimal number. pgn-filter-arguments.in
# holds three different PGNs, one of them the Actisense pseudo PGN 262386, so a
# filter that is accepted but not applied lets the other messages through.
# Anything else is rejected by usage(), which exits 1 with "Unknown or invalid
# argument"; checking both tells a rejection apart from a crash.
#
.PHONY: test-pgn-filter-arguments
tests: test-pgn-filter-arguments
test-pgn-filter-arguments:
	@set -e; for pgn in 59904 127250 262386; do \
	  $(ANALYZER) -q $$pgn < pgn-filter-arguments.in > $(TEMPDIR)/pgn-filter-arguments.out; \
	  if ! awk -v p=$$pgn '$$5 != p { bad = 1 } END { exit bad || NR != 1 }' $(TEMPDIR)/pgn-filter-arguments.out; then \
	    echo "PGN $$pgn did not select exactly the one matching message:"; cat $(TEMPDIR)/pgn-filter-arguments.out; exit 1; \
	  fi; \
	done
	@set -e; for pgn in 128267oops ' 128267' '+128267' 0 -1 4295095563 9999999999999999999999; do \
	  status=0; $(ANALYZER) -q "$$pgn" </dev/null >$(TEMPDIR)/pgn-filter-arguments.out 2>&1 || status=$$?; \
	  if [ $$status -ne 1 ] || ! grep -F "Unknown or invalid argument" $(TEMPDIR)/pgn-filter-arguments.out >/dev/null; then \
	    echo "PGN '$$pgn' was not rejected as an invalid argument (exit $$status):"; cat $(TEMPDIR)/pgn-filter-arguments.out; exit 1; \
	  fi; \
	done
