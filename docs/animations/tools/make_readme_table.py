"""Generate the video table of README.md / README.fr.md from catalog.json (idempotent).

The table sits between the markers ``<!-- TABLE:BEGIN -->`` and ``<!-- TABLE:END -->``.

Usage:
    python tools/make_readme_table.py           # rewrite the tables
    python tools/make_readme_table.py --check   # fail if a table is out of date or a link is broken
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ANIM = Path(__file__).resolve().parents[1]
REPO = ANIM.parents[1]
BEGIN, END = "<!-- TABLE:BEGIN -->", "<!-- TABLE:END -->"
LEVEL = {"en": {1: "1 (basics)", 2: "2", 3: "3 (advanced)"}, "fr": {1: "1 (bases)", 2: "2", 3: "3 (avancé)"}}
HEADERS = {
    "en": ("#", "Video", "What it shows", "Level", "Scene file", "Code examples"),
    "fr": ("#", "Vidéo", "Ce qu'elle montre", "Niveau", "Fichier de scène", "Exemples de code"),
}


def scene_files() -> dict[str, str]:
    files = {}
    for path in sorted(ANIM.glob("anim_*.py")):
        if path.name.startswith("anim__"):
            continue
        for name in re.findall(r"^class\s+(\w+)\(", path.read_text(encoding="utf-8"), re.MULTILINE):
            files[name] = path.name
    return files


def build_table(lang: str, catalog: dict, files: dict[str, str]) -> tuple[str, list[str]]:
    problems: list[str] = []
    head = HEADERS[lang]
    rows = ["| " + " | ".join(head) + " |", "|" + "---|" * len(head)]
    for entry in sorted(catalog["scenes"], key=lambda e: e["id"]):
        scene_file = files.get(entry["scene"])
        if scene_file is None:
            problems.append(f"{entry['id']}: scene {entry['scene']} not found")
            scene_file = "?"
        links = []
        for link in entry["links"]:
            target = REPO / link["path"]
            if not target.is_file():
                problems.append(f"{entry['id']}: {link['path']} does not exist")
            rel = "../../" + link["path"]
            links.append(f"[`{Path(link['path']).name}`]({rel})")
        rows.append(
            "| {id} | **{title}** | {desc} | {level} | [`{file}`]({file}) | {links} |".format(
                id=entry["id"],
                title=entry["title"][lang],
                desc=entry["description"][lang].replace("|", "/"),
                level=LEVEL[lang][entry["level"]],
                file=scene_file,
                links=", ".join(links),
            )
        )
    return "\n".join(rows), problems


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    catalog = json.loads((ANIM / "catalog.json").read_text(encoding="utf-8"))
    files = scene_files()
    status = 0
    for lang, name in (("en", "README.md"), ("fr", "README.fr.md")):
        path = ANIM / name
        text = path.read_text(encoding="utf-8").replace("\r\n", "\n")
        table, problems = build_table(lang, catalog, files)
        pattern = re.compile(re.escape(BEGIN) + r".*?" + re.escape(END), re.DOTALL)
        if not pattern.search(text):
            print(f"ERROR {name}: markers {BEGIN} / {END} not found")
            status = 1
            continue
        new_text = pattern.sub(lambda _: f"{BEGIN}\n{table}\n{END}", text)
        for problem in problems:
            print(f"ERROR {name}: {problem}")
        if problems:
            status = 1
        if args.check:
            if new_text != text:
                print(f"ERROR {name}: table is out of date, run tools/make_readme_table.py")
                status = 1
        elif new_text != text:
            path.write_text(new_text, encoding="utf-8", newline="\n")
            print(f"updated {name}")
        else:
            print(f"{name} is up to date")
    return status


if __name__ == "__main__":
    sys.exit(main())
