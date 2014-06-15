#
# Makefile for all UNIX style platforms including Cygwin
#
# (C) 2009-2014, Kees Verruijt, Harlingen, The Netherlands
#
# $Id:$
#

PLATFORM=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
OS=$(shell uname -o 2>&1)
SUBDIRS= actisense-serial analyzer n2kd nmea0183 ip group-function candump2analyzer
# The closed source code includes more directories


all:	bin
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir; done

bin:
	mkdir -p rel/$(PLATFORM)

clean:
	for dir in $(SUBDIRS); do $(MAKE) -C $$dir clean; done
	
install:
	for i in rel/$(PLATFORM)/* util/* */*_monitor; do f=`basename $$i`; rm /usr/local/bin/$$f; cp $$i /usr/local/bin; done
	killall -9 actisense-serial n2kd

zip:
	(cd rel; zip -r ../packetlogger_`date +%Y%m%d`.zip *)
	./rel/$(PLATFORM)/analyzer -explain > packetlogger_`date +%Y%m%d`_explain.txt
	./rel/$(PLATFORM)/analyzer -explain-xml > packetlogger_`date +%Y%m%d`_explain.xml

.PHONY : $(SUBDIRS) clean install zip bin
