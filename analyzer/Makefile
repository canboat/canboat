
PLATFORM=$(shell uname | tr '[A-Z]' '[a-z]')-$(shell uname -m)
TARGETDIR=../rel/$(PLATFORM)
ANALYZER=$(TARGETDIR)/analyzer
TARGETS=$(ANALYZER)

all: $(TARGETS)

$(ANALYZER): analyzer.c analyzer.h pgn.h ../common/common.c ../common/common.h
	$(CC) -o $(ANALYZER) -I../common analyzer.c ../common/common.c $(LDLIBS$(LDLIBS-$(@)))

clean:
	-rm -f $(N2KD) *.elf *.gdb

