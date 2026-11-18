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
import tempfile

binary = pathlib.Path(sys.argv[1]).resolve()
sentence = b'$GPGLL,4916.45,N,12311.12,W,225444,A,*1D\r\n'
with tempfile.NamedTemporaryFile() as capture:
    capture.write(sentence)
    capture.flush()
    result = subprocess.run([str(binary), '-r', capture.name], capture_output=True, timeout=3, check=True)
    assert result.stdout == sentence, result.stdout
