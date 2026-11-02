#!/usr/bin/env python3
"""Stage candidate CZone variants alongside untouched BEP definitions for Keel."""
from pathlib import Path
import re
import shutil
import sys

root = Path(sys.argv[1]).resolve()
candidates = Path(__file__).resolve().parent / "candidates"
pgn_dir = root / "database/pgns"
for pgn, order in [(130817, 6), (130822, 11), (130825, 5)]:
    source = candidates / f"{pgn}.yaml"
    target = pgn_dir / f"{pgn}-bepMarineCzoneOutputStatus.yaml"
    assert source.is_file() and pgn_dir.is_dir()
    content = source.read_text()
    content = re.sub(r"(?m)^variantOrder: [0-9]+$", f"variantOrder: {order}", content)
    assert f"variantOrder: {order}" in content
    assert not target.exists()
    target.write_text(content)
    print(f"Staged {target.name}; retained existing BEP PGN {pgn} definition")
