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
BUILDDIR?=rel/$(PLATFORM)
TARGETDIR=../$(BUILDDIR)
COMMONDIR=../common
COMMON=$(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(COMMONDIR)/common.h $(COMMONDIR)/license.h $(COMMONDIR)/utf.h $(COMMONDIR)/version.h
SOCKETCAN_SERIAL=$(TARGETDIR)/socketcan-serial
CANBOAT_JSON=../docs/canboat.json
TABLE=fastpacket-table.h

ifneq (,$(findstring linux,$(PLATFORM)))
TARGETS=$(SOCKETCAN_SERIAL)
else
TARGETS=
endif

CFLAGS= -Wall -O2

all: $(TARGETS)

# Generated at build time from the PGN database, so the single/fast
# decision for the mixed 0x1F000..0x1FFFF range always tracks the spec.
$(TABLE): gen-fastpacket-table.py $(CANBOAT_JSON)
	python3 gen-fastpacket-table.py $(CANBOAT_JSON) $(TABLE)

$(SOCKETCAN_SERIAL): socketcan-serial.c $(TABLE) $(COMMON)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(SOCKETCAN_SERIAL) -I. -I../analyzer -I../common \
		socketcan-serial.c ../common/common.c ../common/parse.c ../common/utf.c $(LDLIBS$(LDLIBS-$(@)))

clean:
	-rm -f $(TARGETS) $(TABLE) *.elf *.gdb
