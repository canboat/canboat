

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

import pathlib
import struct
import subprocess
import sys
import tempfile
import unittest


class CaptureRecordsTests(unittest.TestCase):
    def test_short_can_frames_keep_pcap_record_alignment(self):
        header = struct.pack('<IHHIIII', 0xa1b2c3d4, 2, 4, 0, 0, 65535, 227)
        for lengths in ([1, 8], [0, 8], [8, 2, 8]):
            with self.subTest(lengths=lengths), tempfile.NamedTemporaryFile() as capture:
                records = []
                expected = []
                for i, length in enumerate(lengths):
                    data = bytes(range(length))
                    payload = struct.pack('>IBBBB8s', 0x09f119cc, length, 0, 0, 0, data.ljust(8, b'\0'))
                    records.append(struct.pack('<IIII', 1700000000 + i, 123456, len(payload), len(payload)) + payload)
                    expected.append(f'({1700000000 + i}.123456) slcan0 9f119cc#{data.hex()}')
                capture.write(header + b''.join(records))
                capture.flush()
                result = subprocess.run([sys.executable, str(pathlib.Path(__file__).with_name('pcap2candump.py')), capture.name], capture_output=True, text=True, check=True)
                self.assertEqual(result.stdout.splitlines(), expected)


if __name__ == '__main__':
    unittest.main()
