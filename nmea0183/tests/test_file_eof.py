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
