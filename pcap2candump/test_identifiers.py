# (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0.
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest


class IdentifierTests(unittest.TestCase):
    def test_socketcan_identifiers_use_candump_width_and_no_eff_flag(self):
        cases = [(0x89F119CC, '09F119CC'), (0x80000123, '00000123'),
                 (0x123, '123'), (0x1, '001'), (0x7FF, '7FF'),
                 (0x09F119CC, '09F119CC'), (0x20000004, '20000004'),
                 (0xA0000004, '20000004')]
        for order in ['<', '>']:
            for canid, expected in cases:
                with self.subTest(order=order, canid=hex(canid)), tempfile.NamedTemporaryFile() as f:
                    f.write(struct.pack(order + 'IHHIIII', 0xA1B2C3D4, 2, 4, 0, 0, 65535, 227))
                    f.write(struct.pack(order + 'IIII', 1700000000, 123456, 16, 16))
                    f.write(struct.pack('>IBBBB8s', canid, 2, 0, 0, 0, b'\xaa\xbb'))
                    f.flush()
                    result = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name('pcap2candump.py')), f.name], capture_output=True, text=True, check=True)
                    self.assertEqual(result.stdout, f'(1700000000.123456) slcan0 {expected}#aabb\n')


if __name__ == '__main__':
    unittest.main()
