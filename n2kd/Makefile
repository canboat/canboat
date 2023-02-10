#
# (C) 2009-2023, Kees Verruijt, Harlingen, The Netherlands.
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

PLATFORM=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
BUILDDIR?=rel/$(PLATFORM)
TARGETDIR=../$(BUILDDIR)
COMMONDIR=../common
COMMON=$(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(COMMONDIR)/common.h $(COMMONDIR)/license.h $(COMMONDIR)/utf.h $(COMMONDIR)/version.h
ANALYZER=$(TARGETDIR)/analyzer
N2KD=$(TARGETDIR)/n2kd
N2KD_MONITOR=$(TARGETDIR)/n2kd_monitor
TARGETS=$(N2KD) $(N2KD_MONITOR)
LDLIBS+=-lm

CFLAGS= -Wall -O2

all: $(TARGETS)

$(N2KD): main.c nmea0183.c gps_ais.c gps_ais.h nmea0183.h $(COMMON) Makefile
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(N2KD) -I../common main.c gps_ais.c nmea0183.c ../common/common.c $(LDLIBS$(LDLIBS-$(@)))
ifeq ($(notdir $(HELP2MAN)),help2man)
	-$(HELP2MAN) --no-discard-stderr --version-string=Unknown --output=../man/man1/n2kd.1 --name='n2kd' $@
endif

$(N2KD_MONITOR): n2kd_monitor
	cp n2kd_monitor $(N2KD_MONITOR)

tests:	$(ANALYZER) $(N2KD)
	(cd tests; make tests)

clean:
	-rm -f $(N2KD) *.elf *.gdb

.PHONY: clean all tests
