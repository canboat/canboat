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
        # A snaplen of 9 keeps the SocketCAN header and one payload byte of
        # each 16-byte packet; a DLC above that must not read the next record.
        # The last record is a DLC 0 frame of exactly 24 bytes.
        for order in ['<', '>']:
            with self.subTest(order=order), tempfile.NamedTemporaryFile() as capture:
                header = struct.pack(order + 'IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 9, 227)
                records = [(1, 8, b'\xaa'), (2, 1, b'\xbb'), (3, 0, b'')]
                body = b''.join(struct.pack(order + 'IIII', 1700000000, usec, 8 + len(data), 16) + struct.pack('>IBBBB', 0x09f119cc, dlc, 0, 0, 0) + data for usec, dlc, data in records)
                capture.write(header + body)
                capture.flush()
                result = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name('pcap2candump.py')), capture.name], capture_output=True, text=True, check=True)
                self.assertEqual(result.stdout, '(1700000000.000001) slcan0 9f119cc#aa\n(1700000000.000002) slcan0 9f119cc#bb\n(1700000000.000003) slcan0 9f119cc#\n')


if __name__ == '__main__':
    unittest.main()
