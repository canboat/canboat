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
                records = b''.join(struct.pack(order + 'IIII', 1700000000 + i, 123456, 16, 16) + struct.pack('>IBBBB8s', 0x09f119cc, 8, 0, 0, 0, bytes(range(8))) for i in range(2))
                capture.write(header + records)
                capture.flush()
                result = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name('pcap2candump.py')), capture.name], capture_output=True, text=True, check=True)
                outputs.append(result.stdout)
        self.assertEqual(outputs, [''.join(f'({1700000000 + i}.123456) slcan0 9f119cc#0001020304050607\n' for i in range(2))] * 2)

    def test_short_record_payload_stops_at_captured_length(self):
        # A snaplen of 8 keeps the SocketCAN header but no payload; the next
        # record must not be read as that frame's data.
        for order in ['<', '>']:
            with self.subTest(order=order), tempfile.NamedTemporaryFile() as capture:
                header = struct.pack(order + 'IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 8, 227)
                short = struct.pack(order + 'IIII', 1700000000, 1, 8, 16) + struct.pack('>IBBBB', 0x09f119cc, 8, 0, 0, 0)
                full = struct.pack(order + 'IIII', 1700000001, 2, 16, 16) + struct.pack('>IBBBB8s', 0x09f119cc, 8, 0, 0, 0, bytes(range(8)))
                capture.write(header + short + full)
                capture.flush()
                result = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name('pcap2candump.py')), capture.name], capture_output=True, text=True, check=True)
                self.assertEqual(result.stdout, '(1700000000.000001) slcan0 9f119cc#\n(1700000001.000002) slcan0 9f119cc#0001020304050607\n')


if __name__ == '__main__':
    unittest.main()
