import copy
import unittest
import contract


class LookupContractTests(unittest.TestCase):
    def changes(self, collection, entries, replacement):
        old = {collection: [{"Name": "TEST", **entries}]}
        new = copy.deepcopy(old)
        replacement(new[collection][0])
        return contract.diff_signatures(contract.extract_signature(old), contract.extract_signature(new))

    def test_bit_label_before_last_entry(self):
        changes = self.changes("LookupBitEnumerations", {"EnumBitValues": [{"Bit": 0, "Name": "First"}, {"Bit": 1, "Name": "Last"}]}, lambda e: e["EnumBitValues"][0].update(Name="Changed"))
        self.assertEqual([c.severity for c in changes], ["minor"])

    def test_indirect_keys_with_shared_second_value(self):
        changes = self.changes("LookupIndirectEnumerations", {"EnumValues": [{"Value1": 10, "Value2": 130, "Name": "First"}, {"Value1": 20, "Value2": 130, "Name": "Last"}]}, lambda e: e["EnumValues"][0].update(Name="Changed"))
        self.assertEqual([c.severity for c in changes], ["minor"])

    def test_fieldtype_label(self):
        changes = self.changes("LookupFieldTypeEnumerations", {"EnumFieldTypeValues": [{"value": 0, "name": "First", "FieldType": "NUMBER", "Bits": "16"}]}, lambda e: e["EnumFieldTypeValues"][0].update(name="Changed"))
        self.assertEqual([c.severity for c in changes], ["minor"])

    def test_fieldtype_wire_encoding(self):
        for key, value in [("Bits", "32"), ("Resolution", 0.1), ("Signed", True), ("FieldType", "STRING_LAU")]:
            with self.subTest(key=key):
                changes = self.changes("LookupFieldTypeEnumerations", {"EnumFieldTypeValues": [{"value": 0, "name": "First", "FieldType": "NUMBER", "Bits": "16"}]}, lambda e: e["EnumFieldTypeValues"][0].update({key: value}))
                self.assertEqual([c.severity for c in changes], ["minor"])

    def test_ordinary_enum_unchanged(self):
        self.assertEqual(self.changes("LookupEnumerations", {"EnumValues": [{"Value": 0, "Name": "First"}]}, lambda e: None), [])


if __name__ == "__main__":
    unittest.main()
