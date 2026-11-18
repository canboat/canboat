

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

import io
import json
import pathlib
import unittest

import cantools
from dbc_exporter.input.canboat import CanboatReader


class OffsetTests(unittest.TestCase):
    def test_generated_reactive_power_offset_survives_export(self):
        path = pathlib.Path(__file__).resolve().parents[2] / 'docs/canboat.json'
        source = json.loads(path.read_text())
        pgn = next(p for p in source['PGNs'] if p['PGN'] == 65006)
        subset = {'PGNs': [pgn], 'LookupEnumerations': source['LookupEnumerations']}
        database = CanboatReader(io.StringIO(json.dumps(subset))).get_database()
        for db in (database, cantools.database.load_string(database.as_dbc_string(), database_format='dbc')):
            message = db.messages[0]
            values = message.decode(bytes(8), decode_choices=False)
            for field in pgn['Fields']:
                self.assertEqual(values[field['Id']], field.get('Offset', 0))
            high_bit = message.decode(bytes.fromhex('0000008000000000'), decode_choices=False)
            self.assertEqual(high_bit['reactivePower'], 147483648)
            self.assertEqual(values['powerFactor'], -1)
            self.assertFalse(message.get_signal_by_name('reactivePower').is_signed)


if __name__ == '__main__':
    unittest.main()
