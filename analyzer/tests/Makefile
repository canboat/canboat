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
TARGETDIR=../../rel/$(PLATFORM)
ANALYZER=$(TARGETDIR)/analyzer
TEMPDIR=/tmp

.PHONY: test1 test2 test3 test4 test5 test6 test7 tests

all:	tests

test1:
	$(ANALYZER) < recombine-frames.in > $(TEMPDIR)/recombine-frames.out -debug -q -fixtime recombine 2> $(TEMPDIR)/recombine-frames.err
	diff $(TEMPDIR)/recombine-frames.out recombine-frames.out
	diff $(TEMPDIR)/recombine-frames.err recombine-frames.err

test2:	
	$(ANALYZER) < switch-multi-to-one-line.in > $(TEMPDIR)/switch-multi-to-one-line.out -debug -q -fixtime switch-multi-to-one-line 2> $(TEMPDIR)/switch-multi-to-one-line.err
	diff $(TEMPDIR)/switch-multi-to-one-line.out switch-multi-to-one-line.out
	diff $(TEMPDIR)/switch-multi-to-one-line.err switch-multi-to-one-line.err

#
# This tests TEXT stability
#
test3:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test.out -debug -q -fixtime pgn-test 2> $(TEMPDIR)/pgn-test.err
	diff $(TEMPDIR)/pgn-test.out pgn-test.out
	diff $(TEMPDIR)/pgn-test.err pgn-test.err

#
# This tests JSON stability
#
test4:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json.out -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json.out
	diff $(TEMPDIR)/pgn-test-json.out pgn-test-json.out
	diff $(TEMPDIR)/pgn-test-json.err pgn-test-json.err

#
# This tests JSON stability
#
test5:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json-nv.out -nv -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json-nv.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json-nv.out
	diff $(TEMPDIR)/pgn-test-json-nv.out pgn-test-json-nv.out
	diff $(TEMPDIR)/pgn-test-json-nv.err pgn-test-json-nv.err

#
# This tests JSON stability
#
test6:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json-debug.out -debug -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json-debug.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json-debug.out
	diff $(TEMPDIR)/pgn-test-json-debug.out pgn-test-json-debug.out
	diff $(TEMPDIR)/pgn-test-json-debug.err pgn-test-json-debug.err

#
# This tests JSON stability
#
test7:	
	$(ANALYZER) < pgn-test.in > $(TEMPDIR)/pgn-test-json-nv-debug.out -nv -debug -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-json-nv-debug.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-json-nv-debug.out
	diff $(TEMPDIR)/pgn-test-json-nv-debug.out pgn-test-json-nv-debug.out
	diff $(TEMPDIR)/pgn-test-json-nv-debug.err pgn-test-json-nv-debug.err

#
# This tests capability to read Actisense N2K Ascii format
#
test8:	
	TZ=UTC-1 $(ANALYZER) < pgn-test-actisense.in > $(TEMPDIR)/pgn-test-actisense.out -nv -debug -q -json -fixtime pgn-test 2> $(TEMPDIR)/pgn-test-actisense.err
	python3 ../validate-json.py --line-by-line $(TEMPDIR)/pgn-test-actisense.out
	diff $(TEMPDIR)/pgn-test-actisense.out pgn-test-actisense.out
	diff $(TEMPDIR)/pgn-test-actisense.err pgn-test-actisense.err

tests:	test1 test2 test3 test4 test5 test6 test7 test8
