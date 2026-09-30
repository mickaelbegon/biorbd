"""Extract check frames from a rendered video with PyAV: ~1 s, 35 %, 65 %, just before the final card, middle of the card.

Usage: python tools/extract_frames.py <video.mp4> [out_dir]
The final card lasts 4.5 s (common/layer.py CARD_DURATION).
"""

import sys
from pathlib import Path

import av

CARD_SECONDS = 4.5


def main(video: Path, out_dir: Path) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    with av.open(str(video)) as container:
        stream = container.streams.video[0]
        duration = float(stream.duration * stream.time_base)
        targets = {
            "t01s": 1.0,
            "p35": 0.35 * duration,
            "p65": 0.65 * duration,
            "before_card": duration - CARD_SECONDS - 0.3,
            "card_mid": duration - CARD_SECONDS / 2,
        }
        frames = []
        for frame in container.decode(stream):
            frames.append((float(frame.pts * stream.time_base), frame))
    saved = []
    for name, target in targets.items():
        pts, frame = min(frames, key=lambda item: abs(item[0] - target))
        path = out_dir / f"{video.stem}_{name}.png"
        frame.to_image().save(path)
        saved.append(path)
    return saved


if __name__ == "__main__":
    src = Path(sys.argv[1])
    dst = Path(sys.argv[2]) if len(sys.argv) > 2 else src.parent / "frames"
    for saved_path in main(src, dst):
        print(saved_path)
