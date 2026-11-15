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


if __name__ == '__main__':
    unittest.main()
