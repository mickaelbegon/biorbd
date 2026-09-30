"""Validate catalog.json: required fields, existing files, line ranges inside the files, scenes present.

Usage: python tools/validate_catalog.py [--repo PATH_TO_BIORBD]   (default: the repository containing this folder)
"""

import argparse
import json
import re
import sys
from pathlib import Path

ANIM = Path(__file__).resolve().parents[1]
REQUIRED = ("id", "slug", "scene", "level", "section", "title", "description", "notes", "links")
BILINGUAL = ("section", "title", "description", "notes")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, default=ANIM.parents[1])
    parser.add_argument("--catalog", type=Path, default=ANIM / "catalog.json")
    args = parser.parse_args()
    data = json.loads(args.catalog.read_text(encoding="utf-8"))
    scene_names = set()
    for path in ANIM.glob("anim_*.py"):
        if not path.name.startswith("anim__"):
            scene_names |= set(re.findall(r"^class\s+(\w+)\(", path.read_text(encoding="utf-8"), re.MULTILINE))
    errors: list[str] = []
    seen_ids: set[str] = set()
    for entry in data.get("scenes", []):
        tag = entry.get("id", "?")
        for key in REQUIRED:
            if key not in entry:
                errors.append(f"{tag}: missing field {key}")
        for key in BILINGUAL:
            if key in entry and not (entry[key].get("en") and entry[key].get("fr")):
                errors.append(f"{tag}: {key} needs non-empty en and fr")
        if entry.get("id") in seen_ids:
            errors.append(f"{tag}: duplicate id")
        seen_ids.add(entry.get("id"))
        if entry.get("level") not in (1, 2, 3):
            errors.append(f"{tag}: level must be 1, 2 or 3")
        if entry.get("scene") not in scene_names:
            errors.append(f"{tag}: scene class {entry.get('scene')} not found in anim_*.py")
        links = entry.get("links", [])
        if not 2 <= len(links) <= 5:
            errors.append(f"{tag}: needs 2 to 5 links, has {len(links)}")
        for link in links:
            target = args.repo / link.get("path", "")
            if not target.is_file():
                errors.append(f"{tag}: file {link.get('path')} does not exist")
                continue
            lines = link.get("lines")
            if lines:
                count = len(target.read_text(encoding="utf-8", errors="replace").splitlines())
                if not (1 <= lines[0] <= lines[1] <= count):
                    errors.append(f"{tag}: lines {lines} outside {link['path']} ({count} lines)")
            if not (link.get("label", {}).get("en") and link.get("label", {}).get("fr")):
                errors.append(f"{tag}: link {link.get('path')} needs an en and fr label")
    for message in errors:
        print("ERROR", message)
    print(f"{len(data.get('scenes', []))} scene(s), {len(errors)} error(s)")
    return 1 if errors else 0


if __name__ == "__main__":
    sys.exit(main())
