# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.

#
# A Maretron SIM100 reports every input whose wiring it finds open or
# shorted as 2, the error of a 2-bit OFF_ON field: in 127501's indicators
# and in the Switch Status of 130836 and 130837. Frames captured from two
# SIM100s whose inputs were all in error, next to one that was not.
#
.PHONY: test-switch-error
tests: test-switch-error
test-switch-error:
	$(ANALYZER) -json < pgn-switch-error.in > $(TEMPDIR)/pgn-switch-error.out -fixtime pgn-test 2> $(TEMPDIR)/pgn-switch-error.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-switch-error.out
	diff $(TEMPDIR)/pgn-switch-error.out pgn-switch-error.out
	diff $(TEMPDIR)/pgn-switch-error.err pgn-switch-error.err
