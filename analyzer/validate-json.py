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

import sys;
import json;
import traceback;

if (sys.argv[1] == '--line-by-line'):
    file = open(sys.argv[2])
    for line in file.readlines():
        try:
            data = json.loads(line)
        except:
            print(line)
            traceback.print_exc(limit=1)
            exit(1)
    print("JSON in", sys.argv[2], "seems valid when parsed line-by-line.")
    exit(0)

res = 0
allowedDuplicates = { } # 'Reserved', 'Spare' }
checkRange = False
allowedNoRange = { 'RESERVED', 'SPARE', 'BINARY', 'STRING_LZ', 'STRING_FIX' }

if (sys.argv[1] == '--range'):
    checkRange = True
    file = open(sys.argv[2])
else:
    file = open(sys.argv[1])
data = json.loads(file.read())
file.close()

pgns = data["PGNs"]
pMap = {}
for pgn in pgns:
    prn = pgn['PGN']
    desc = pgn['Description']
    pid = pgn['Id']

    if (pid in pMap):
        print("ERROR: PGN", prn, "'" +  desc + "' has duplicate Id '" + pid + "'; first used in PGN ", pMap[pid])
        res = 1
    pMap[pid] = prn

    if ('Fields' in pgn):
        fields = pgn['Fields']
        nMap = {}
        for field in fields:
            order = field['Order']
            fid  = field['Id']
            if (not fid in allowedDuplicates):
                if (fid in nMap):
                    print("ERROR: PGN", prn, "'" +  desc + "' has duplicate field id '" + fid + "'; first used as field", nMap[fid], "and repeated as field", order)
                    res = 1
                else:
                    nMap[fid] = order
            if (checkRange):
                if (not 'FieldType' in field):
                    print("ERROR: PGN", prn, "'" +  desc + "' field " , order , fid , "has no FieldType")
                    print(field)
                    res = 1
                else:
                    ft = field['FieldType']
                    if (not 'RangeMax' in field and not ft in allowedNoRange and not 'BitLengthVariable' in field):
                        print("ERROR: PGN", prn, "'" +  desc + "' field " , order , fid , "has no rangeMax")
                        print(field)
                        res = 1


if (res == 0):
    s1 = json.dumps(data, indent = 2)
    s2 = json.dumps(data, indent = 2, ensure_ascii = False)
    if (s1 != s2):
        s1 = s1.splitlines()
        s2 = s2.splitlines()
        print("ERROR: JSON contains non-ASCII characters")
        i = 0
        while (i < len(s1)):
            if (s1[i] != s2[i]):
                print(str(i) + " < " + s1[i])
                print(str(i) + " > " + s2[i])
            i = i + 1
        res = 1
    
if (res == 0):
    print("JSON in", sys.argv[1], "seems valid.")

exit(res)
