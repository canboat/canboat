# (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0.
import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest


class LinkTypeTests(unittest.TestCase):
    def convert(self, order, link_type):
        header = struct.pack(order + "IHHIIII", 0xa1b2c3d4, 2, 4, 0, 0, 65535, link_type)
        packet = struct.pack(">IBBBB8s", 0x89f119cc, 8, 0, 0, 0, bytes(range(8)))
        record = struct.pack(order + "IIII", 1700000000, 123456, len(packet), len(packet))
        with tempfile.NamedTemporaryFile() as capture:
            capture.write(header + record + packet)
            capture.flush()
            return subprocess.run(
                [sys.executable, str(pathlib.Path(__file__).with_name("pcap2candump.py")), capture.name],
                capture_output=True, text=True)

    def test_rejects_non_socketcan_captures(self):
        for order in ["<", ">"]:
            for link_type in [0, 1, 113, 228]:
                with self.subTest(order=order, link_type=link_type):
                    result = self.convert(order, link_type)
                    self.assertNotEqual(result.returncode, 0)
                    self.assertEqual(result.stdout, "")
                    self.assertIn("unsupported link type", result.stderr)

    def test_accepts_socketcan_in_both_byte_orders(self):
        for order in ["<", ">"]:
            result = self.convert(order, 227 | (1 << 26))
            self.assertEqual(result.returncode, 0, result.stderr)
            self.assertIn("#0001020304050607", result.stdout)


if __name__ == "__main__":
    unittest.main()
