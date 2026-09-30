"""Merge catalog_part_<topic>.json files into catalog.json (sorted by id).

Usage: python tools/merge_catalog.py
Each part is one scene entry (a JSON object). An existing entry with the same id is replaced.
"""

import json
from pathlib import Path

ANIM = Path(__file__).resolve().parents[1]


def main() -> None:
    catalog_path = ANIM / "catalog.json"
    catalog = (
        json.loads(catalog_path.read_text(encoding="utf-8")) if catalog_path.exists() else {"schema": 1, "scenes": []}
    )
    by_id = {entry["id"]: entry for entry in catalog["scenes"]}
    for part in sorted(ANIM.glob("catalog_part_*.json")):
        entry = json.loads(part.read_text(encoding="utf-8"))
        by_id[entry["id"]] = entry
        print(f"merged {part.name}")
    catalog["scenes"] = [by_id[key] for key in sorted(by_id)]
    catalog_path.write_text(json.dumps(catalog, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
