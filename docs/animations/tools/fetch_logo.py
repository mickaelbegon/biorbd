"""Download the biorbd logo into assets/ (the PNG is never committed)."""

import sys
import urllib.request
from pathlib import Path

URL = "https://raw.githubusercontent.com/pyomeca/biorbd_design/main/logo_png/biorbd_full.png"
DEST = Path(__file__).resolve().parents[1] / "assets" / "biorbd_logo.png"


def main(force: bool = False) -> Path:
    if DEST.exists() and not force:
        return DEST
    DEST.parent.mkdir(parents=True, exist_ok=True)
    with urllib.request.urlopen(URL, timeout=30) as response:
        data = response.read()
    if not data.startswith(b"\x89PNG"):
        raise RuntimeError(f"{URL} did not return a PNG file")
    DEST.write_bytes(data)
    return DEST


if __name__ == "__main__":
    print(main(force="--force" in sys.argv))
