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
