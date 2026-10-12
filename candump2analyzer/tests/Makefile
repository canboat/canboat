#
# (C) 2009-2026, Kees Verruijt, Harlingen, The Netherlands.
#
# This file is part of CANboat.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#

PLATFORM?=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
TARGETDIR=../../rel/$(PLATFORM)
CANDUMP2ANALYZER=$(TARGETDIR)/candump2analyzer
TEMPDIR=/tmp

.PHONY: test1 test2 test3 tests all

all:	tests

#
# This tests that candump2analyzer converts the PCAN-View v1.1 trace format
# (FMT_6) to the analyzer's PLAIN input format, including absolute timestamps
# derived from the ;$STARTTIME= header. See issue #250.
#
test1:
	$(CANDUMP2ANALYZER) pcan-view.in > $(TEMPDIR)/pcan-view.out 2> $(TEMPDIR)/pcan-view.err
	diff $(TEMPDIR)/pcan-view.out pcan-view.out

tests:	test1 test2 test3

#
# The Angstrom candump style ("<0x18eeff01> [8] 05 a0 ..."), FMT_1. Its lines
# carry no time, so candump2analyzer stamps them with the wall clock; compare
# everything after the timestamp column. The Rust candump reader is checked
# against the same pair (crates/canboat/tests/convert_formats.rs, #998).
#
test2:
	$(CANDUMP2ANALYZER) angstrom.in | cut -d, -f2- > $(TEMPDIR)/angstrom.out 2> $(TEMPDIR)/angstrom.err
	diff $(TEMPDIR)/angstrom.out angstrom.out

#
# Angstrom and Debian lines whose bytes do not match their [len] (cut short,
# or more than 8) are skipped with a message, not padded or passed on (#998).
# The Rust candump reader is checked against the same files.
#
test3:
	for f in angstrom-bad-length debian-bad-length; do \
	  $(CANDUMP2ANALYZER) $$f.in 2> $(TEMPDIR)/$$f.err | cut -d, -f2- > $(TEMPDIR)/$$f.out || exit 1; \
	  diff $(TEMPDIR)/$$f.out $$f.out || exit 1; \
	  diff $(TEMPDIR)/$$f.err $$f.err || exit 1; \
	done
