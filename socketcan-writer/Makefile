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
BUILDDIR?=rel/$(PLATFORM)
TARGETDIR=../$(BUILDDIR)
# Test programs stay out of $(TARGETDIR), which `make install` copies wholesale;
# a trailing / on BUILDDIR must not put them inside it.
TESTDIR=$(patsubst %/,%,$(TARGETDIR))-tests
COMMONDIR=../common
COMMON=$(COMMONDIR)/common.c $(COMMONDIR)/parse.c $(COMMONDIR)/utf.c $(COMMONDIR)/common.h $(COMMONDIR)/license.h $(COMMONDIR)/utf.h $(COMMONDIR)/version.h
SOCKETCAN_WRITER=$(TARGETDIR)/socketcan-writer

ifneq (,$(findstring linux,$(PLATFORM)))
TARGETS=$(SOCKETCAN_WRITER)
else
TARGETS=
endif

CFLAGS= -Wall -O2

all: $(TARGETS)

$(SOCKETCAN_WRITER): socketcan-writer.c $(COMMON)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(SOCKETCAN_WRITER) -I../analyzer -I../common socketcan-writer.c ../common/common.c ../common/parse.c $(LDLIBS$(LDLIBS-$(@)))

clean:
	-rm -f $(TARGETS) $(TESTDIR)/test-socketcan-write *.elf *.gdb


.PHONY: tests
ifneq (,$(findstring linux,$(PLATFORM)))
tests: $(TESTDIR)/test-socketcan-write
	python3 tests/test_write_failure.py $(TESTDIR)/test-socketcan-write

$(TESTDIR):
	mkdir -p $(TESTDIR)

$(TESTDIR)/test-socketcan-write: socketcan-writer.c tests/fail_write.c $(COMMON) | $(TESTDIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -I../analyzer -I../common -o $@ socketcan-writer.c ../common/common.c ../common/parse.c tests/fail_write.c -Wl,--wrap=write $(LDLIBS)
else
tests:
endif
