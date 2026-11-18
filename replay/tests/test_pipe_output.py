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

import os
import pathlib
import select
import subprocess
import sys
import time

binary = pathlib.Path(sys.argv[1]).resolve()
line = b'2026-11-15T12:00:00.000,3,126993,0,255,1,00\n'

# Each frame must reach a pipe while replay waits for more input.
process = subprocess.Popen([str(binary)], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
try:
    process.stdin.write(line)
    process.stdin.flush()
    output = b''
    deadline = time.monotonic() + 2
    while len(output) < len(line):
        ready, _, _ = select.select([process.stdout], [], [], max(0, deadline - time.monotonic()))
        if not ready:
            sys.exit('replay buffered the frame while waiting for more input: %r' % output)
        chunk = os.read(process.stdout.fileno(), len(line) - len(output))
        if not chunk:
            break
        output += chunk
    if output != line:
        sys.exit('unexpected output: %r' % output)
    process.stdin.close()
    try:
        status = process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        sys.exit('replay did not exit at end of input')
    if status != 0:
        sys.exit('replay exited %d at end of input' % status)
finally:
    if process.poll() is None:
        process.kill()
        process.wait()
    for stream in (process.stdin, process.stdout, process.stderr):
        stream.close()

# A frame that cannot be written is an error, not a silent success.
if os.path.exists('/dev/full'):
    with open('/dev/full', 'wb') as full:
        result = subprocess.run([str(binary)], input=line, stdout=full, stderr=subprocess.PIPE, timeout=3, check=False)
    if result.returncode != 1:
        sys.exit('replay exited %d although stdout rejected the frame, expected 1' % result.returncode)
