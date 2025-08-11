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
import traceback;
import csv;

type_ok = {
        'DATE': 'NUMBER',
        'DATE': 'NUMBER',
        'TIME': 'NUMBER',
        'DURATION': 'NUMBER',
        'BITLOOKUP': 'LOOKUP',
        'SPARE': 'RESERVED',
        }

ignore_prn = [ '129541', '129556', '127505', '128520' ]

ignore_fields = [ 'Communication State', 'Special Maneuver Indicator' ]

ignore_keys = [ '129029_17', '129038_17', '129285_5', '129798_10', '129798_14', '129810_13', '129807_17', '129805_9', '129801_6', '129038_15', '129792_2', '129795_6', '129796_9' ]

fake_lookup = [ 'Reference Station ID', 'Satellite Service ID No.', 'Binary Data' ]

canboat = {}
nmea = {}

with open(sys.argv[1], newline='') as csvfile:
    reader = csv.DictReader(csvfile, ['prn', 'frame', 'length', 'priority', 'fields', 'field', 'name', 'type', 'signed', 'bits', 'unit', 'resolution', 'offset', 'rangeMin', 'rangeMax'])
    for row in reader:
        canboat[row['prn'] + "_" + row['field']] = row

with open(sys.argv[2], newline='') as csvfile:
    reader = csv.DictReader(csvfile, ['prn', 'frame', 'length', 'priority', 'fields', 'field', 'name', 'type', 'signed', 'bits', 'unit', 'resolution', 'offset', 'rangeMin', 'rangeMax'])
    for row in reader:
        nmea[row['prn'] + "_" + row['field']] = row


res = 0
for key in canboat:
    if key in nmea:
        c = canboat[key]
        n = nmea[key]

        # Precisions etc of PGN 129541 are wrong in NMEA db
        if c['prn'] in ignore_prn:
            continue
        if c['name'] in ignore_fields:
            continue
        if key in ignore_keys:
            continue

        if c['type'] == 'DECIMAL':
            continue

        if c['rangeMin'] != '' and n['rangeMin'] != '':
            c['rangeMin'] = float(c['rangeMin'])
            n['rangeMin'] = float(n['rangeMin'])
            c['rangeMax'] = float(c['rangeMax'])
            n['rangeMax'] = float(n['rangeMax'])
            if c['rangeMin'] < 0. and n['rangeMax'] == c['rangeMax'] and n['rangeMin'] == -n['rangeMax']:
                n['rangeMin'] = c['rangeMin']
            if c['type'] == 'LOOKUP' and c['bits'] == '2' and n['rangeMax'] == 3:
                n['rangeMax'] = c['rangeMax']
            if c['type'] == 'NUMBER' and c['bits'] == '8' and n['type'] == 'LOOKUP':
                c['type'] = 'LOOKUP'
            if n['type'] == 'NUMBER' and n['bits'] == '8' and c['type'] == 'LOOKUP':
                n['type'] = 'LOOKUP'
            if c['type'] == 'LOOKUP' and c['bits'] == '8':
                c['rangeMax'] = 253.
            if n['type'] == 'LOOKUP' and n['bits'] == '8':
                n['rangeMax'] = 253.
            if c['type'] == 'BITLOOKUP' and c['rangeMax'] == 65535:
                n['rangeMax'] = 65535.
            if n['rangeMin'] == -0.02306:
                n['rangeMin'] = c['rangeMin']
                n['rangeMax'] = c['rangeMax']
            if n['rangeMax'] != 0 and c['rangeMax'] / n['rangeMax'] > 0.999 and c['rangeMax'] / n['rangeMax'] < 1.001:
                c['rangeMax'] = n['rangeMax']
            if n['rangeMin'] != 0 and c['rangeMin'] / n['rangeMin'] > 0.999 and c['rangeMin'] / n['rangeMin'] < 1.001:
                c['rangeMin'] = n['rangeMin']
            if (c['resolution'] == '1e-09' and n['rangeMin'] == -2.14) \
            or (c['unit'] == 'Pa' and n['rangeMin'] == -210000000.0) \
            or c['unit'] == 'rad/s' \
            or n['rangeMin'] == -3.4028234664e+38 \
            or (n['name'] == 'Time Stamp' and n['type'] == 'LOOKUP') \
            or c['type'] == 'BITLOOKUP':
                n['rangeMin'] = c['rangeMin']
                n['rangeMax'] = c['rangeMax']
                

        if c['type'] == 'RESERVED' and n['type'] == 'NUMBER':
            continue
        if c['type'] == 'NUMBER' and n['type'] == 'RESERVED':
            continue


        # Remove uninteresting differences.
        n['name'] = c['name']
        n['length'] = c['length']
        n['fields'] = c['fields']
        if c['type'] in type_ok:
            c['type'] = type_ok[c['type']]
        if c['name'] in fake_lookup and n['type'] == 'LOOKUP':
            c['type'] = n['type']
            c['rangeMin'] = n['rangeMin']
            c['rangeMax'] = n['rangeMax']
        if c['signed'] == '' and n['signed'] != '':
            c['signed'] = n['signed']
        if c['resolution'] == '1' and n['resolution'] == '':
            n['resolution'] = '1'
        if c['resolution'] != '':
            c['resolution'] = float(c['resolution'])
        if n['resolution'] != '':
            n['resolution'] = float(n['resolution'])
        if n['resolution'] != '' and n['resolution'] != 0 and c['resolution'] / n['resolution'] > 0.999 and c['resolution'] / n['resolution'] < 1.001:
            n['resolution'] = c['resolution']
        if n['unit'] == '' and c['unit'] != '':
            n['unit'] = c['unit']

        # Plain errors in NMEA
        if n['unit'] == 'second':
            n['unit'] = 's'
        if n['resolution'] == 3.12e-05:
            n['resolution'] = 3.125e-05
        if n['resolution'] == 1000.0 and n['unit'] == 'mv': 
            n['resolution'] = c['resolution']
            n['unit'] = c['unit']
        if n['offset'] == '' and c['offset'] == '1': # Forgotten offset 
            n['offset'] = c['offset']
        if n['unit'] == 'Ah' and c['unit'] == 'C': # Not expressed in SI
            n['unit'] = 'C'
            n['resolution'] = n['resolution'] * 3600.
            n['rangeMin'] = n['rangeMin'] * 3600.
            n['rangeMax'] = n['rangeMax'] * 3600.
        if n['signed'] != 'true' and n['rangeMin'] == -655.32:
            n['rangeMin'] = 0.
        if n['unit'] == 'dB re: uV/m':
            n['unit'] = c['unit']
            n['rangeMin'] = c['rangeMin']
            n['rangeMax'] = c['rangeMax']
        

        # ignore NMEA range on MMSI fields
        if c['type'] == 'MMSI' and n['type'] == 'NUMBER':
            n['type'] = 'MMSI'
            n['rangeMin'] = c['rangeMin']
            n['rangeMax'] = c['rangeMax']

        if c != n:
            print('+', end='')
            print(c)
            print('-', end='')
            print(n)
            res = 1

exit(res)
