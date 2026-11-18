

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
import subprocess
import sys

binary = pathlib.Path(sys.argv[1]).resolve()
frame = '0021200 0e 1d ff 9d 08 00 00 00 80 df 3f 9f 34 12 ff 0d\n'
result = subprocess.run([str(binary)], input=frame * 2, text=True, capture_output=True, check=True)
frames = [line.split(',')[1:] for line in result.stdout.splitlines()]
assert len(frames) == 2, result.stdout
assert frames[0] == frames[1], result.stdout
assert frames[0][4] == '8', result.stdout
assert len(frames[0][5:]) == 8, result.stdout
