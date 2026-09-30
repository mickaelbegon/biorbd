"""Render the biorbd Manim series.

Examples (run with the Python 3.11 environment that has manim 0.21):

    python tools/render_series.py --list
    python tools/render_series.py AnimForwardKinematics --lang both
    python tools/render_series.py --all --lang both --jobs 3 --strict --collect
    python tools/render_series.py --all --dry

One media folder per (scene, language) job avoids collisions between parallel renders.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUNNER = Path(__file__).resolve().parent / "_manim_runner.py"
SCENE_RE = re.compile(r"^class\s+(\w+)\(\s*(?:Scene|ThreeDScene|MovingCameraScene)\s*\)", re.MULTILINE)
DEFAULT_OUT = Path.home() / "Documents" / "biorbd_animations"
DEFAULT_FFMPEG = Path.home() / "miniconda3" / "envs" / "ffmpeg_anim" / "Library" / "bin"


def discover(include_selftest: bool = False) -> dict[str, Path]:
    """Map scene class name -> file, for every ``anim_*.py`` (``anim__*.py`` are self-tests, opt-in)."""
    scenes: dict[str, Path] = {}
    for path in sorted(ROOT.glob("anim_*.py")):
        if path.name.startswith("anim__") and not include_selftest:
            continue
        for name in SCENE_RE.findall(path.read_text(encoding="utf-8")):
            scenes[name] = path
    return scenes


def parse_quality(text: str) -> tuple[int, int, int]:
    match = re.fullmatch(r"(\d+)p(\d+)", text)
    if not match:
        raise SystemExit(f"--quality must look like 1080p30, got {text!r}")
    height, fps = int(match.group(1)), int(match.group(2))
    return 2 * round(height * 16 / 9 / 2), height, fps


def build_job(scene: str, path: Path, lang: str, args) -> dict:
    width, height, fps = parse_quality(args.quality)
    media = ROOT / "media" / f"{scene}_{lang}"
    command = [
        sys.executable,
        str(RUNNER),
        "render",
        str(path),
        scene,
        "-r",
        f"{width},{height}",
        "--fps",
        str(fps),
        "--disable_caching",
        "--media_dir",
        str(media),
        "-o",
        scene,
    ]
    env = dict(os.environ)
    env.update({"BIORBD_ANIM_LANG": lang, "BIORBD_ANIM_REPORT_DIR": str(media)})
    ffmpeg = Path(os.environ.get("BIORBD_ANIM_FFMPEG_DIR", DEFAULT_FFMPEG))
    if ffmpeg.exists():
        env["PATH"] = str(ffmpeg) + os.pathsep + env.get("PATH", "")
    video = media / "videos" / path.stem / f"{height}p{fps}" / f"{scene}.mp4"
    return {"scene": scene, "lang": lang, "command": command, "env": env, "media": media, "video": video}


def run_job(job: dict, args) -> dict:
    if args.dry:
        print("DRY", " ".join(job["command"]))
        return {**job, "ok": True, "problems": []}
    job["media"].mkdir(parents=True, exist_ok=True)
    # Manim caches text SVGs by content only: a stale cache would hide layout fixes
    shutil.rmtree(job["media"] / "texts", ignore_errors=True)
    log = job["media"] / "render.log"
    with open(log, "w", encoding="utf-8") as handle:
        result = subprocess.run(job["command"], env=job["env"], cwd=ROOT, stdout=handle, stderr=subprocess.STDOUT)
    problems: list[str] = []
    if result.returncode != 0 or not job["video"].exists():
        problems.append(f"render failed (exit {result.returncode}), see {log}")
    report_path = job["media"] / f"report_{job['scene']}_{job['lang']}.json"
    if report_path.exists():
        report = json.loads(report_path.read_text(encoding="utf-8"))
        for item in report["audit"]:
            problems.append(f"audit {item['kind']} at {item['time']}s: {item['text']!r} {item['other']!r}")
        if job["lang"] == "fr":
            problems += [f"missing FR key: {key!r}" for key in report["missing_keys"]]
        if report.get("no_catalog_entry") and not job["scene"].startswith("SelfTest"):
            problems.append("no catalog.json entry for this scene (final card is empty)")
    elif not problems:
        problems.append("no report written")
    ok = not problems if args.strict else job["video"].exists()
    return {**job, "ok": ok, "problems": problems}


def collect(results: list[dict], out: Path) -> None:
    for result in results:
        if not result["video"].exists():
            continue
        folder = out / result["lang"].upper()
        folder.mkdir(parents=True, exist_ok=True)
        shutil.copy2(result["video"], folder / f"{result['scene']}.mp4")
        print(f"collected {folder / (result['scene'] + '.mp4')}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("scene", nargs="?", help="scene class name, e.g. AnimForwardKinematics")
    parser.add_argument("--all", action="store_true", help="render every scene of anim_*.py")
    parser.add_argument("--selftest", action="store_true", help="include the layer self-test scenes (anim__*.py)")
    parser.add_argument("--list", action="store_true", help="list the scenes and exit")
    parser.add_argument("--lang", choices=["en", "fr", "both"], default="both")
    parser.add_argument("--quality", default="1080p30", help="e.g. 1080p30 (default), 480p15 for quick checks")
    parser.add_argument("--jobs", type=int, default=1, help="parallel renders")
    parser.add_argument("--dry", action="store_true", help="print the commands only")
    parser.add_argument(
        "--strict", action="store_true", help="fail on audit findings, missing FR keys or missing catalog entry"
    )
    parser.add_argument("--collect", action="store_true", help="copy the videos to <out>/EN and <out>/FR")
    parser.add_argument("--out", type=Path, default=Path(os.environ.get("BIORBD_ANIM_OUT", DEFAULT_OUT)))
    args = parser.parse_args()

    scenes = discover(args.selftest or bool(args.scene and args.scene.startswith("SelfTest")))
    if args.list:
        for name, path in scenes.items():
            print(f"{name}\t{path.name}")
        return 0
    if args.all:
        names = list(scenes)
    elif args.scene:
        if args.scene not in scenes:
            raise SystemExit(f"unknown scene {args.scene!r}; known: {', '.join(scenes) or '(none)'}")
        names = [args.scene]
    else:
        parser.error("give a scene name or --all")
    langs = ["en", "fr"] if args.lang == "both" else [args.lang]
    jobs = [build_job(name, scenes[name], lang, args) for name in names for lang in langs]

    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        results = list(pool.map(lambda job: run_job(job, args), jobs))
    failed = 0
    for result in results:
        status = "ok " if result["ok"] else "FAIL"
        print(f"[{status}] {result['scene']} ({result['lang']})")
        for problem in result["problems"]:
            print(f"        - {problem}")
        failed += 0 if result["ok"] else 1
    if args.collect and not args.dry:
        collect(results, args.out)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
