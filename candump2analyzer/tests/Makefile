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
CANDUMP2ANALYZER=$(TARGETDIR)/candump2analyzer
TEMPDIR=/tmp

.PHONY: test1 tests all

all:	tests

#
# This tests that candump2analyzer converts the PCAN-View v1.1 trace format
# (FMT_6) to the analyzer's PLAIN input format, including absolute timestamps
# derived from the ;$STARTTIME= header. See issue #250.
#
test1:
	$(CANDUMP2ANALYZER) pcan-view.in > $(TEMPDIR)/pcan-view.out 2> $(TEMPDIR)/pcan-view.err
	diff $(TEMPDIR)/pcan-view.out pcan-view.out

tests:	test1
