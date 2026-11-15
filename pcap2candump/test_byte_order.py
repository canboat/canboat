# (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0.
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest


class ByteOrderTests(unittest.TestCase):
    def test_big_and_little_endian_captures_match(self):
        outputs = []
        for order in ['<', '>']:
            with tempfile.NamedTemporaryFile() as capture:
                header = struct.pack(order + 'IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 65535, 227)
                record = struct.pack(order + 'IIII', 1700000000, 123456, 16, 16)
                frame = struct.pack('>IBBBB8s', 0x09f119cc, 8, 0, 0, 0, bytes(range(8)))
                capture.write(header + record + frame)
                capture.flush()
                result = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name('pcap2candump.py')), capture.name], capture_output=True, text=True, check=True)
                outputs.append(result.stdout)
        self.assertEqual(outputs, ['(1700000000.123456) slcan0 9f119cc#0001020304050607\n'] * 2)


if __name__ == '__main__':
    unittest.main()
