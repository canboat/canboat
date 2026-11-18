# (C) 2009-2026, Kees Verruijt. Licensed under Apache-2.0.
import errno
import os
import pathlib
import subprocess
import sys
import unittest

BINARY = pathlib.Path(sys.argv.pop(1)).resolve()
RETRIES = 50  # SEND_RETRIES in socketcan-writer.c
FRAME = 16  # sizeof(struct can_frame)


def message(size):
    return '2026-11-15T00:00:00.000,3,126996,0,255,' + str(size) + ',' + ','.join(f'{n:02x}' for n in range(size)) + '\n'


def run(messages, fail=0, count=1, error=errno.ENOSPC):
    env = dict(os.environ, CANBOAT_TEST_FAIL_WRITE=str(fail), CANBOAT_TEST_FAIL_COUNT=str(count), CANBOAT_TEST_FAIL_ERRNO=str(error))
    return subprocess.run([str(BINARY), 'stdout'], input=''.join(messages).encode(), capture_output=True, timeout=10, env=env)


class WriteFailureTests(unittest.TestCase):
    def test_write_failure_stops_single_and_fast_packet_output(self):
        for size, failure in [(8, 1), (9, 1), (9, 2)]:
            with self.subTest(size=size, failure=failure):
                result = run([message(size)] * 2, fail=failure)
                self.assertEqual(result.returncode, 1, result.stderr)
                self.assertEqual(result.stderr.count(b'frame-attempt:'), failure, result.stderr)

    def test_successful_fast_packet_output_still_finishes(self):
        result = run([message(9)] * 2)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stderr.count(b'frame-attempt:'), 4)
        self.assertEqual(len(result.stdout), 4 * FRAME)

    def test_interrupted_and_busy_writes_are_retried(self):
        for error in [errno.EINTR, errno.ENOBUFS, errno.EAGAIN]:
            with self.subTest(error=errno.errorcode[error]):
                # Two failed attempts on the first frame, then every frame goes out.
                result = run([message(9), message(8)], fail=1, count=2, error=error)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stderr.count(b'frame-attempt:'), 3 + 2, result.stderr)
                self.assertEqual(len(result.stdout), 3 * FRAME)

    def test_full_queue_drops_the_message_and_continues(self):
        # The first frame of a fast packet never fits: its second frame is not
        # sent either, and the next message still goes out.
        result = run([message(9), message(8)], fail=1, count=RETRIES + 1, error=errno.ENOBUFS)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn(b'dropping CAN id', result.stderr)
        self.assertEqual(result.stderr.count(b'frame-attempt:'), RETRIES + 1 + 1, result.stderr)
        self.assertEqual(len(result.stdout), FRAME)


if __name__ == '__main__':
    unittest.main()
