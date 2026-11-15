# (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0.
import os
import pathlib
import subprocess
import sys
import unittest

BINARY = pathlib.Path(sys.argv.pop(1)).resolve()


class WriteFailureTests(unittest.TestCase):
    def test_write_failure_stops_single_and_fast_packet_output(self):
        for size, failure in [(8, 1), (9, 1), (9, 2)]:
            with self.subTest(size=size, failure=failure):
                frame = '2026-11-15T00:00:00.000,3,126996,0,255,' + str(size) + ',' + ','.join(f'{n:02x}' for n in range(size)) + '\n'
                result = subprocess.run([str(BINARY), 'stdout'], input=(frame*2).encode(), capture_output=True, timeout=3, env=dict(os.environ, CANBOAT_TEST_FAIL_WRITE=str(failure)))
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertEqual(result.stderr.count(b'frame-attempt:'), failure, result.stderr)

    def test_successful_fast_packet_output_still_finishes(self):
        frame = '2026-11-15T00:00:00.000,3,126996,0,255,9,' + ','.join(f'{n:02x}' for n in range(9)) + '\n'
        result = subprocess.run([str(BINARY), 'stdout'], input=(frame*2).encode(), capture_output=True, timeout=3, env=dict(os.environ, CANBOAT_TEST_FAIL_WRITE='999'))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr.count(b'frame-attempt:'), 4)
        self.assertEqual(len(result.stdout), 64)


if __name__ == '__main__':
    unittest.main()
