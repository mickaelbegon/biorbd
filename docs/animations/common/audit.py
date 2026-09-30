"""Frame audit: text outside the frame, overlapping texts, collisions with the logo."""

from __future__ import annotations

import numpy as np
from manim import MarkupText, Paragraph, Text, config

MARGIN = 0.08  # units allowed outside the safe frame before a text is reported
OVERLAP_RATIO = 0.10  # share of the smaller box that may overlap before it is reported

findings: list[dict] = []
_seen: set[tuple] = set()


def bbox(mobject):
    """Return (x0, x1, y0, y1) of a mobject, or None when it has no extent."""
    if mobject.width < 1e-6 and mobject.height < 1e-6:
        return None
    return (
        mobject.get_left()[0],
        mobject.get_right()[0],
        mobject.get_bottom()[1],
        mobject.get_top()[1],
    )


def intersect_area(a, b) -> float:
    dx = min(a[1], b[1]) - max(a[0], b[0])
    dy = min(a[3], b[3]) - max(a[2], b[2])
    return dx * dy if dx > 0 and dy > 0 else 0.0


def area(a) -> float:
    return max((a[1] - a[0]) * (a[3] - a[2]), 1e-9)


def max_opacity(mobject) -> float:
    best = 0.0
    for member in mobject.family_members_with_points():
        try:
            fill = float(member.get_fill_opacity())
            stroke = float(member.get_stroke_opacity()) if member.get_stroke_width() > 0 else 0.0
        except Exception:  # image mobjects and others
            fill, stroke = float(getattr(member, "fill_opacity", 1.0)), 0.0
        best = max(best, fill, stroke)
        if best > 0.5:
            break
    return best


def is_text(mobject) -> bool:
    return isinstance(mobject, (Text, MarkupText)) and not isinstance(mobject, Paragraph)


def _record(scene, kind: str, first: str, second: str = "") -> None:
    key = (kind, first[:40], second[:40])
    if key in _seen:
        return
    _seen.add(key)
    findings.append({"kind": kind, "text": first[:80], "other": second[:80], "time": round(float(scene.time), 2)})


def check(scene, logo=None) -> None:
    """Audit the scene as it is right now (called after every play)."""
    half_w, half_h = config.frame_width / 2 - MARGIN, config.frame_height / 2 - MARGIN
    texts = []
    for mobject in scene.get_mobject_family_members():
        if is_text(mobject) and (mobject.submobjects or mobject.has_points()):
            texts.append(mobject)
    visible = []
    for text in texts:
        if max_opacity(text) < 0.3:
            continue
        box = bbox(text)
        if box is None:
            continue
        visible.append((text, box))
        label = getattr(text, "text", "") or ""
        if box[0] < -half_w or box[1] > half_w or box[2] < -half_h or box[3] > half_h:
            _record(scene, "out_of_frame", label)
        if logo is not None:
            logo_box = bbox(logo)
            if logo_box is not None and intersect_area(box, logo_box) > 0:
                _record(scene, "logo_collision", label)
    for i in range(len(visible)):
        for j in range(i + 1, len(visible)):
            (t1, b1), (t2, b2) = visible[i], visible[j]
            overlap = intersect_area(b1, b2)
            if overlap > OVERLAP_RATIO * min(area(b1), area(b2)):
                _record(scene, "overlap", getattr(t1, "text", ""), getattr(t2, "text", ""))
