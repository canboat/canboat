#
# (C) 2009-2025, Kees Verruijt, Harlingen, The Netherlands.
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
ACTISENSE=$(TARGETDIR)/actisense-serial
ANALYZER=$(TARGETDIR)/analyzer
SAMPLES=../../samples
TEMPDIR=/tmp

.PHONY: test1 tests all

all:	tests

#
# This tests that actisense-serial reads the W2K-1 / OpenCPN JSON capture
# format (one {"pgn":..,"payload":[..]} per line) and converts it to canboat
# FAST format that analyzer can decode. See discussion #635.
# The W2K payload carries no usable timestamp, so actisense-serial stamps each
# message with wall-clock time; -fixtime fixes only the log lines (stderr), so
# the per-message timestamps in stdout are normalised away before diffing.
#
test1:
	$(ACTISENSE) -r $(SAMPLES)/NKpayloadsW2K-1.txt 2>/dev/null | $(ANALYZER) -json -nv -q -fixtime w2k 2>$(TEMPDIR)/w2k.err | sed 's/"timestamp":"[^"]*"/"timestamp":"-"/' > $(TEMPDIR)/w2k.out
	python3 ../../analyzer/validate-json.py --line-by-line $(TEMPDIR)/w2k.out
	diff $(TEMPDIR)/w2k.out w2k.out
	diff $(TEMPDIR)/w2k.err w2k.err

tests:	test1
