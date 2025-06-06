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
ANALYZER=$(TARGETDIR)/analyzer
N2KD=$(TARGETDIR)/n2kd
TEMPDIR=/tmp

all:	tests

test1:
	$(ANALYZER) -format fast < ais-lookup.in -json -nv -empty -fixtime test-n2kd | $(N2KD) --nmea0183 -fixtime test-n2kd > $(TEMPDIR)/ais-lookup.out 2> $(TEMPDIR)/ais-lookup.err
	diff $(TEMPDIR)/ais-lookup.out ais-lookup.out
	diff $(TEMPDIR)/ais-lookup.err ais-lookup.err

test2:
	$(ANALYZER) -format fast < nmea0183.in -json -nv -empty -fixtime test-n2kd | $(N2KD) --nmea0183 -fixtime test-n2kd > $(TEMPDIR)/nmea0183.out 2> $(TEMPDIR)/nmea0183.err
	diff $(TEMPDIR)/nmea0183.out nmea0183.out
	diff $(TEMPDIR)/nmea0183.err nmea0183.err

tests:	test1 test2

.PHONY: test1 test2 tests
