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
TARGET1=$(TARGETDIR)/iptee
TARGETLIST=$(TARGET1)

CFLAGS= -Wall -O2

all:	$(TARGETDIR) $(TARGETLIST)

$(TARGETDIR):	
	mkdir -p $(TARGETDIR)

clean:
	-rm -f $(TARGETLIST) *.elf *.gdb

$(TARGET1):	iptee.c $(COMMON)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(LDFLAGS) -o $(TARGET1) $(CFLAGS) -I../common iptee.c ../common/common.c
ifeq ($(notdir $(HELP2MAN)),help2man)
	-$(HELP2MAN) --no-discard-stderr --version-string=Unknown --output=../man/man1/iptee.1 --name='iptee' $@
endif
