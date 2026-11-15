# (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0.
import copy
import unittest
import contract


class FieldOrderTests(unittest.TestCase):
    def setUp(self):
        self.old = {'PGNs': [{'PGN': 127250, 'Id': 'test', 'Type': 'Single', 'Fields': [
            {'Order': i + 1, 'Id': name, 'BitLength': 8, 'BitOffset': i * 8, 'FieldType': 'NUMBER'}
            for i, name in enumerate(['a', 'b'])]}]}

    def changes(self, ids):
        new = copy.deepcopy(self.old)
        new['PGNs'][0]['Fields'] = [dict(Order=i + 1, Id=name, BitLength=8, BitOffset=i * 8, FieldType='NUMBER') for i, name in enumerate(ids)]
        return contract.diff_signatures(contract.extract_signature(self.old), contract.extract_signature(new))

    def test_same_size_field_reordering_is_not_invisible(self):
        self.assertEqual([c.severity for c in self.changes(['b', 'a'])], ['minor'])

    def test_insertion_is_not_also_reported_as_reordering(self):
        self.assertEqual([c.category for c in self.changes(['c', 'a', 'b'])], ['field-added'])

    def test_unchanged_order_is_unchanged(self):
        self.assertEqual(self.changes(['a', 'b']), [])


if __name__ == '__main__':
    unittest.main()
