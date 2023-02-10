

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

import sys
import re
import subprocess

newline = '\n'
do_schema_version = False
n=1

if (sys.argv[n] == '-w'):
    newline = '\r\n'
    n = n+1
if (sys.argv[n] == '-s'):
    do_schema_version = True
    n = n+1

analyzer = sys.argv[n]
filename = sys.argv[n+1]

if not do_schema_version:
    version = subprocess.check_output([analyzer, "-version"], encoding='utf8')
    version = str(version).split('\n')[0]
    print("Replacing version in",filename,"with",version)
else:
    version = subprocess.check_output([analyzer, "-schema-version"], encoding='utf8')
    version = str(version).split('\n')[0]
    print("Replacing schema version in",filename,"with",version)


with open(filename,'r') as file:
    filedata = file.read()

if not do_schema_version:
    filedata = re.sub('"version": "[^"]*"', '"version": "' + version + '"', filedata)
    filedata = re.sub('CANboat version v[.0-9]*', 'CANboat version v' + version, filedata)
    filedata = re.sub('^VERSION "[^"]*"', 'VERSION "' + version + '"', filedata)
else:
    filedata = re.sub('version="[.0-9]*"', 'version="' + version + '"', filedata)

with open(filename,'w', newline=newline) as file:
    file.write(filedata)

