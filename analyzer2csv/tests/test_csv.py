

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

import csv
import io
import json
import pathlib
import subprocess
import sys
import unittest

PHP = sys.argv.pop(1) if len(sys.argv) > 1 else 'php'
SCRIPT = pathlib.Path(__file__).resolve().parents[1] / 'analyzer2csv'


class CsvTests(unittest.TestCase):
    def convert(self, record, keys):
        line = json.dumps(record, ensure_ascii=False)
        result = subprocess.run([PHP, '-n', str(SCRIPT), *keys], input=line+'\n', text=True, capture_output=True, check=True)
        rows = list(csv.reader(io.StringIO(result.stdout), strict=True))
        self.assertEqual(rows[0], keys)
        return line, rows[1]

    def test_embedded_json_message_roundtrip(self):
        line, row = self.convert({'timestamp': '2026-11-15T12:00:00', 'pgn': 126996, 'fields': {'model': 'A, "B"'}}, ['message'])
        self.assertEqual(row, [line])
        self.assertEqual(json.loads(row[0])['pgn'], 126996)

    def test_selected_values_with_csv_delimiters(self):
        for text in ['Port, starboard', 'Model "A"', 'first\nsecond', 'backslash\\"quote']:
            with self.subTest(text=text):
                _, row = self.convert({'timestamp': '2026-11-15T12:00:00', 'pgn': 126996, 'description': text}, ['timestamp', 'description', 'pgn'])
                self.assertEqual(row, ['2026-11-15T12:00:00', text, '126996'])


if __name__ == '__main__':
    unittest.main()
