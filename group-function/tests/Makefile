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
COMMAND=$(TARGETDIR)/command-group-function
REQUEST=$(TARGETDIR)/request-group-function
TEMPDIR=/tmp

.PHONY: test-arguments tests all

all:	tests

tests: test-arguments

#
# Every number on the command line is a whole decimal number in range, and every
# <value> an even number of hex digits, at most eight. strtol() used to take a prefix,
# so "12x" commanded address 12 and "1=a" encoded no bytes at all.
#
test-arguments:
	@set -e; \
	out=$$($(COMMAND) 12 2 130306 1=0a03 3=ff | cut -d, -f2-); \
	test "$$out" = "2,126208,0,12,11,01,02,fd,01,f2,02,01,03,0a,03,ff" || { echo "command encoded: $$out"; exit 1; }; \
	out=$$($(REQUEST) 12 2 130306 4294967295 1=0a | cut -d, -f2-); \
	test "$$out" = "2,126208,0,12,13,00,02,fd,01,ff,ff,ff,ff,00,00,01,01,0a" || { echo "request encoded: $$out"; exit 1; }; \
	for args in '12x 2 130306 1=0a' '256 2 130306 1=0a' '12 9 130306 1=0a' '12 2 +130306 1=0a' '12 2 131072 1=0a' \
	            '12 2 130306 0=0a' '12 2 130306 1x=0a'; do \
	  if $(COMMAND) $$args >/dev/null 2>$(TEMPDIR)/group-function.err; then echo "command accepted $$args"; exit 1; fi; \
	  grep -F "Invalid value for" $(TEMPDIR)/group-function.err >/dev/null || { echo "command $$args:"; cat $(TEMPDIR)/group-function.err; exit 1; }; \
	done; \
	for args in '12 2 130306 1=a' '12 2 130306 1=0a0b0c0d0e' '12 2 130306 1=0g' '12 2 130306 =0a' '12 2 130306 1'; do \
	  if $(COMMAND) $$args >/dev/null 2>$(TEMPDIR)/group-function.err; then echo "command accepted $$args"; exit 1; fi; \
	  grep -F "Unknown or invalid argument" $(TEMPDIR)/group-function.err >/dev/null || { echo "command $$args:"; cat $(TEMPDIR)/group-function.err; exit 1; }; \
	done; \
	for args in '12 2 130306 100ms 1=0a' '12 2 130306 4294967296 1=0a' '12 2 130306 -1 1=0a'; do \
	  if $(REQUEST) $$args >/dev/null 2>$(TEMPDIR)/group-function.err; then echo "request accepted $$args"; exit 1; fi; \
	  grep -F "Invalid value for <interval>" $(TEMPDIR)/group-function.err >/dev/null || { echo "request $$args:"; cat $(TEMPDIR)/group-function.err; exit 1; }; \
	done
