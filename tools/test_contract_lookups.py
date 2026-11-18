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

import copy
import unittest
import contract


class LookupContractTests(unittest.TestCase):
    def changes(self, collection, entries, replacement):
        old = {collection: [{"Name": "TEST", **entries}]}
        new = copy.deepcopy(old)
        replacement(new[collection][0])
        return contract.diff_signatures(contract.extract_signature(old), contract.extract_signature(new))

    def assertChange(self, changes, key, kind):
        """One minor change of the entry with this key, reported as a label or a (field type) definition change."""
        self.assertEqual([c.severity for c in changes], ["minor"])
        self.assertTrue(changes[0].detail.startswith("value %s %s: " % (key, kind)), changes[0].detail)

    def test_bit_label_before_last_entry(self):
        changes = self.changes("LookupBitEnumerations", {"EnumBitValues": [{"Bit": 3, "Name": "First"}, {"Bit": 5, "Name": "Last"}]}, lambda e: e["EnumBitValues"][0].update(Name="Changed"))
        self.assertChange(changes, "3", "label")

    def test_indirect_keys_with_shared_second_value(self):
        changes = self.changes("LookupIndirectEnumerations", {"EnumValues": [{"Value1": 10, "Value2": 130, "Name": "First"}, {"Value1": 20, "Value2": 130, "Name": "Last"}]}, lambda e: e["EnumValues"][0].update(Name="Changed"))
        self.assertChange(changes, "10:130", "label")

    def test_indirect_keys_with_shared_first_value(self):
        changes = self.changes("LookupIndirectEnumerations", {"EnumValues": [{"Value1": 10, "Value2": 130, "Name": "First"}, {"Value1": 10, "Value2": 140, "Name": "Last"}]}, lambda e: e["EnumValues"][1].update(Name="Changed"))
        self.assertChange(changes, "10:140", "label")

    def test_fieldtype_label(self):
        changes = self.changes("LookupFieldTypeEnumerations", {"EnumFieldTypeValues": [{"value": 0, "name": "First", "FieldType": "NUMBER", "Bits": "16"}]}, lambda e: e["EnumFieldTypeValues"][0].update(name="Changed"))
        self.assertChange(changes, "0", "definition")

    def test_fieldtype_wire_encoding(self):
        for key, value in [("Bits", "32"), ("Resolution", 0.1), ("Signed", True), ("FieldType", "STRING_LAU")]:
            with self.subTest(key=key):
                changes = self.changes("LookupFieldTypeEnumerations", {"EnumFieldTypeValues": [{"value": 0, "name": "First", "FieldType": "NUMBER", "Bits": "16"}]}, lambda e: e["EnumFieldTypeValues"][0].update({key: value}))
                self.assertChange(changes, "0", "definition")

    def test_ordinary_enum_unchanged(self):
        self.assertEqual(self.changes("LookupEnumerations", {"EnumValues": [{"Value": 0, "Name": "First"}]}, lambda e: None), [])


if __name__ == "__main__":
    unittest.main()
