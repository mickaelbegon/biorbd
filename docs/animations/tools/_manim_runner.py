"""Run Manim with the biorbd layer installed. Called by render_series.py, not by hand.

Usage: python _manim_runner.py render <file.py> <SceneName> [manim options]
"""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from common import layer  # noqa: E402

layer.install()

from manim.__main__ import main  # noqa: E402

if __name__ == "__main__":
    sys.argv = ["manim"] + sys.argv[1:]
    main()
