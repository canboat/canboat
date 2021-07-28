#
# Makefile for all UNIX style platforms including Cygwin
#
# (C) 2009-2017, Kees Verruijt, Harlingen, The Netherlands
#
# $Id:$
#

# s. https://www.gnu.org/prep/standards/html_node/Directory-Variables.html#Directory-Variables
DESTDIR ?= ""
PREFIX ?= /usr/local
EXEC_PREFIX ?= $(PREFIX)
BINDIR=$(EXEC_PREFIX)/bin
SYSCONFDIR= /etc
DATAROOTDIR ?= $(PREFIX)/share
MANDIR= $(DATAROOTDIR)/man

PLATFORM=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
OS=$(shell uname -o 2>&1)
SUBDIRS= actisense-serial analyzer n2kd nmea0183 ip group-function candump2analyzer socketcan-writer ikonvert-serial

MKDIR = mkdir -p
export HELP2MAN=$(shell command -v help2man 2> /dev/null)

CONFDIR=$(SYSCONFDIR)/default

ROOT_UID=0
ROOT_GID=0
ROOT_MOD=0644

all:	bin man1 compile

compile:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir; done
	$(MAKE) -C analyzer json

bin:	rel/$(PLATFORM)

rel/$(PLATFORM):
	$(MKDIR) rel/$(PLATFORM)

man1: man/man1

man/man1:
	$(MKDIR) man/man1

clean:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir clean; done
	-rm -R -f man

install: rel/$(PLATFORM)/analyzer $(DESTDIR)$(BINDIR) $(DESTDIR)$(CONFDIR) $(DESTDIR)$(MANDIR)/man1
	for i in rel/$(PLATFORM)/* util/* */*_monitor; do f=`basename $$i`; echo $$f; rm -f $(DESTDIR)$(BINDIR)/$$f; cp $$i $(DESTDIR)$(BINDIR); done
	for i in config/*; do install -m $(ROOT_MOD) $$i $(DESTDIR)$(CONFDIR); done
ifeq ($(notdir $(HELP2MAN)),help2man)
	for i in man/man1/*; do echo $$i; install -m $(ROOT_MOD) $$i $(DESTDIR)$(MANDIR)/man1; done
endif

zip:
	(cd rel; zip -r ../packetlogger_`date +%Y%m%d`.zip *)
	./rel/$(PLATFORM)/analyzer -explain > packetlogger_`date +%Y%m%d`_explain.txt
	./rel/$(PLATFORM)/analyzer -explain-xml > packetlogger_`date +%Y%m%d`_explain.xml

format:
	for file in */*.c */*.h; do clang-format -i $$file; done

.PHONY : $(SUBDIRS) clean install zip bin format man1

$(DESTDIR)$(BINDIR):
	$(MKDIR) $(DESTDIR)$(BINDIR)

$(DESTDIR)$(CONFDIR):
	$(MKDIR) $(DESTDIR)$(CONFDIR)

$(DESTDIR)$(MANDIR)/man1:
	$(MKDIR) $(DESTDIR)$(MANDIR)/man1
