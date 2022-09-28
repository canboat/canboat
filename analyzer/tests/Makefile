#
# (C) 2009-2021, Kees Verruijt, Harlingen, The Netherlands.
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

tests:	$(ANALYZER) recombine-frames.in recombine-frames.out
	$(ANALYZER) < recombine-frames.in > $(TEMPDIR)/recombine-frames.out -debug -q -fixtime recombine 2> $(TEMPDIR)/recombine-frames.err
	diff $(TEMPDIR)/recombine-frames.out recombine-frames.out
	diff $(TEMPDIR)/recombine-frames.err recombine-frames.err
