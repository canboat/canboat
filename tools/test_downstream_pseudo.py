

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

import importlib.util
import json
import pathlib
import unittest
from unittest.mock import patch

HERE = pathlib.Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('downstream_notify', HERE / 'downstream-notify.py')
notify = importlib.util.module_from_spec(spec)
spec.loader.exec_module(notify)


class PseudoNotificationTests(unittest.TestCase):
    def test_existing_pseudo_contract_changes_are_reported(self):
        database = json.loads((HERE.parent / 'docs/canboat-pseudo.json').read_text())
        added = database['PGNs'][0]
        def contract_at(ref, path):
            if path == 'docs/canboat-pseudo.json' and ref == 'new':
                return {'PGNs': [added]}
            return {'PGNs': []}
        with patch.object(notify, 'contract_at', side_effect=contract_at):
            summary = notify.database_changes('old', 'new')
        self.assertIn('CANboat pseudo-PGNs', summary)
        self.assertIn(added['Id'], summary)

    def test_first_pseudo_contract_is_reported(self):
        def contract_at(ref, path):
            return None if path == 'docs/canboat-pseudo.json' and ref == 'old' else {'PGNs': []}
        with patch.object(notify, 'contract_at', side_effect=contract_at):
            self.assertIn('is new in this release', notify.database_changes('old', 'new'))

    def test_unchanged_contracts_still_report_no_changes(self):
        with patch.object(notify, 'contract_at', return_value={'PGNs': []}):
            self.assertIn('No changes', notify.database_changes('old', 'new'))


if __name__ == '__main__':
    unittest.main()
