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
MARETRON=$(TARGETDIR)/maretron-ipg
TARGETS=$(MARETRON)

CFLAGS= -Wall -O2

all: $(TARGETS)

$(MARETRON): maretron-ipg.c maretron-ipg.h $(COMMON)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(MARETRON) -I../common maretron-ipg.c ../common/common.c ../common/parse.c $(LDLIBS$(LDLIBS-$(@)))

clean:
	-rm -f $(TARGETS) *.elf *.gdb
