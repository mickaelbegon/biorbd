"""EN/FR translation of on-screen text.

The English string is the key. Every number of the key is replaced by ``{0}``, ``{1}``, ...
Text set in a code font is never translated and its numbers are left untouched. In French
every number of a non-code text gets a decimal comma, translated or not.
"""

from __future__ import annotations

import json
import re
from pathlib import Path

I18N_DIR = Path(__file__).resolve().parents[1] / "i18n"
CODE_FONTS = {"Consolas", "Courier New", "Cascadia Mono", "Monospace", "DejaVu Sans Mono"}

_NUMBER = re.compile(r"(?<![\w.])-?\d+(?:\.\d+)?(?:e-?\d+)?(?!\w)")
_PLACEHOLDER = re.compile(r"\{(\d+)\}")

_table: dict[str, str] | None = None
bypass = False  # True while the layer builds text that is already localised
seen_keys: set[str] = set()
missing_keys: set[str] = set()


def is_code_font(font: str | None) -> bool:
    return bool(font) and font in CODE_FONTS


def make_key(text: str) -> tuple[str, list[str]]:
    """Return the translation key of ``text`` and the numbers it contained."""
    numbers: list[str] = []

    def _sub(match: re.Match) -> str:
        numbers.append(match.group(0))
        return "{%d}" % (len(numbers) - 1)

    return _NUMBER.sub(_sub, text), numbers


def load_table(force: bool = False) -> dict[str, str]:
    """Merge every ``i18n/fr_*.json`` (later files, alphabetically, win)."""
    global _table
    if _table is None or force:
        _table = {}
        for path in sorted(I18N_DIR.glob("fr_*.json")):
            with open(path, encoding="utf-8") as handle:
                _table.update(json.load(handle))
    return _table


def _localize_number(number: str) -> str:
    return number.replace(".", ",")


def _fill(template: str, numbers: list[str]) -> str:
    return _PLACEHOLDER.sub(
        lambda m: numbers[int(m.group(1))] if int(m.group(1)) < len(numbers) else m.group(0), template
    )


def translate(text: str, lang: str) -> str:
    """Translate one non-code string. English is returned unchanged (keys are only recorded)."""
    if bypass or not text.strip():
        return text
    key, numbers = make_key(text)
    if not re.search(r"[A-Za-z]{2,}", key):  # pure numbers / symbols: nothing to translate
        return text if lang != "fr" else _fill(key, [_localize_number(n) for n in numbers])
    seen_keys.add(key)
    if lang != "fr":
        return text
    localized = [_localize_number(n) for n in numbers]
    table = load_table()
    if key in table:
        return _fill(table[key], localized)
    # multi-line strings: try line by line
    if "\n" in text:
        lines = text.split("\n")
        out, all_found = [], True
        for line in lines:
            line_key, line_numbers = make_key(line)
            if not re.search(r"[A-Za-z]{2,}", line_key):
                out.append(_fill(line_key, [_localize_number(n) for n in line_numbers]))
            elif line_key in table:
                seen_keys.add(line_key)
                out.append(_fill(table[line_key], [_localize_number(n) for n in line_numbers]))
            else:
                all_found = False
                seen_keys.add(line_key)
                missing_keys.add(line_key)
                out.append(_fill(line_key, [_localize_number(n) for n in line_numbers]))
        seen_keys.discard(key)
        if all_found:
            return "\n".join(out)
        return "\n".join(out)
    missing_keys.add(key)
    return _fill(key, localized)
