"""Render-time layer applied to every scene without editing it.

``install()`` patches Manim before the scene module is imported:

* the biorbd logo, small, in a corner that is free of content (moves only when it collides);
* a global slow-down (default x1.6) and a minimal reading pause after new text;
* sober text transitions (Write / letter by letter / text morphs become sequential fades);
* EN/FR translation of every non-code text (see ``i18n.py``);
* small Pango text (< 40 pt) is built 4x larger then scaled down (irregular spacing bug);
* a final "Go further" card of 4.5 s built from ``catalog.json``;
* the frame audit (``audit.py``), whose findings go to the report next to the video.

Environment: BIORBD_ANIM_LANG (en|fr), BIORBD_ANIM_SPEED, BIORBD_ANIM_REPORT_DIR.
"""

from __future__ import annotations

import json
import os
from pathlib import Path

import manim
from manim import (
    DOWN,
    LEFT,
    UP,
    Animation,
    FadeIn,
    FadeOut,
    ImageMobject,
    MarkupText,
    Paragraph,
    ReplacementTransform,
    Scene,
    Text,
    Transform,
    VGroup,
    config,
)
from manim.animation.animation import prepare_animation

from . import audit, bio_style as S, i18n

ROOT = Path(__file__).resolve().parents[1]
LANG = os.environ.get("BIORBD_ANIM_LANG", "en")
SPEED = float(os.environ.get("BIORBD_ANIM_SPEED", "1.6"))
REPORT_DIR = os.environ.get("BIORBD_ANIM_REPORT_DIR", "")
LOGO_PATH = S.ASSETS_DIR / "biorbd_logo.png"
LOGO_HEIGHT = 0.5
LOGO_MARGIN = 0.25
LOGO_PAD = 0.15
FINAL_HOLD = 2.5  # seconds, after slow-down, before the final card
CARD_DURATION = 4.5  # seconds, the final card (fade in + hold + fade out)
READ_BASE, READ_PER_CHAR, READ_PER_CODE_CHAR = 0.5, 0.03, 0.02
BIG_FACTOR = 4  # small text is built this much larger then scaled down
BIG_BELOW = 40  # font sizes under this value use the trick

_TEXT_CLASSES = (Text, MarkupText)
_originals: dict[str, object] = {}
_report: dict = {"no_catalog_entry": False}


# ---------------------------------------------------------------------------------------------
# text: default font, translation, 4x construction
# ---------------------------------------------------------------------------------------------
def _patch_text_class(cls) -> None:
    original = cls.__init__

    def __init__(self, text, *args, **kwargs):
        kwargs.setdefault("font", S.TEXT_FONT)
        code = i18n.is_code_font(kwargs["font"])
        if not code:
            text = i18n.translate(text, LANG)
        font_size = kwargs.get("font_size", manim.DEFAULT_FONT_SIZE)
        sized_by_box = kwargs.get("height") is not None or kwargs.get("width") is not None
        big = font_size < BIG_BELOW and not sized_by_box
        if big:
            kwargs["font_size"] = font_size * BIG_FACTOR
        # Pango wraps lines at config.pixel_width: lift the limit while the text is built
        saved_width = config.pixel_width
        config.pixel_width = 10**6
        try:
            original(self, text, *args, **kwargs)
        finally:
            config.pixel_width = saved_width
        if big:
            self.scale(1 / BIG_FACTOR)

    cls.__init__ = __init__


# ---------------------------------------------------------------------------------------------
# sober text transitions
# ---------------------------------------------------------------------------------------------
def _is_texty(mobject) -> bool:
    if isinstance(mobject, (Text, MarkupText, Paragraph)):
        return True
    subs = getattr(mobject, "submobjects", [])
    return bool(subs) and all(_is_texty(sub) for sub in subs)


def _text_of(mobject) -> str:
    return getattr(mobject, "text", None) or "".join(_text_of(sub) for sub in getattr(mobject, "submobjects", []))


def _sober_write(mobject, *args, **kwargs):
    kept = {key: value for key, value in kwargs.items() if key in ("run_time", "rate_func")}
    if _is_texty(mobject) or any(_is_texty(sub) for sub in mobject.submobjects):
        return FadeIn(mobject, **kept)
    return _originals["Create"](mobject, **kept)


def _sober_unwrite(mobject, *args, **kwargs):
    kept = {key: value for key, value in kwargs.items() if key in ("run_time", "rate_func")}
    return FadeOut(mobject, **kept)


def _sober_matching(source, target, *args, **kwargs):
    if _is_texty(source) and _is_texty(target):
        return ReplacementTransform(source, target)
    return _originals["TransformMatchingShapes"](source, target, *args, **kwargs)


def _patch_animations() -> None:
    for name in ("Create", "TransformMatchingShapes"):
        _originals[name] = getattr(manim, name)
    manim.Write = _sober_write
    manim.AddTextLetterByLetter = _sober_write
    manim.TypeWithCursor = _sober_write
    manim.Unwrite = _sober_unwrite
    manim.RemoveTextLetterByLetter = _sober_unwrite
    manim.UntypeWithCursor = _sober_unwrite
    manim.TransformMatchingShapes = _sober_matching
    manim.TransformMatchingTex = _sober_matching


_TEXT_TRANSFORMS = (Transform, ReplacementTransform, manim.FadeTransform)


def _split_text_transform(animation):
    """Return (fade_out, fade_in) for a text->text morph, else None."""
    if type(animation) not in _TEXT_TRANSFORMS:
        return None
    source, target = animation.mobject, getattr(animation, "target_mobject", None)
    if target is None or not (_is_texty(source) and _is_texty(target)):
        return None
    if _text_of(source) == _text_of(target):
        return None
    return FadeOut(source), FadeIn(target)


# ---------------------------------------------------------------------------------------------
# logo
# ---------------------------------------------------------------------------------------------
def _corners(logo) -> list[tuple[str, tuple[float, float, float, float]]]:
    half_w, half_h = config.frame_width / 2, config.frame_height / 2
    w, h = logo.width, logo.height
    out = []
    for name, sx, sy in (("top_right", 1, 1), ("bottom_right", 1, -1), ("top_left", -1, 1), ("bottom_left", -1, -1)):
        cx, cy = sx * (half_w - LOGO_MARGIN - w / 2), sy * (half_h - LOGO_MARGIN - h / 2)
        out.append((name, (cx - w / 2, cx + w / 2, cy - h / 2, cy + h / 2)))
    return out


def _collides(scene, logo, rect) -> bool:
    padded = (rect[0] - LOGO_PAD, rect[1] + LOGO_PAD, rect[2] - LOGO_PAD, rect[3] + LOGO_PAD)
    frame_area = config.frame_width * config.frame_height
    for mobject in scene.mobjects:
        if mobject is logo:
            continue
        box = audit.bbox(mobject)
        if box is None or audit.intersect_area(box, padded) <= 0:
            continue
        if audit.area(box) > 0.6 * frame_area:  # full-frame background
            continue
        if audit.max_opacity(mobject) < 0.1:
            continue
        return True
    return False


def _place_logo(scene, logo) -> None:
    current = next(
        (
            name
            for name, rect in _corners(logo)
            if abs(logo.get_center()[0] - (rect[0] + rect[1]) / 2) < 1e-3
            and abs(logo.get_center()[1] - (rect[2] + rect[3]) / 2) < 1e-3
        ),
        None,
    )
    corners = _corners(logo)
    rects = dict(corners)
    if current is not None and not _collides(scene, logo, rects[current]):
        return
    for name, rect in corners:
        if not _collides(scene, logo, rect):
            logo.move_to([(rect[0] + rect[1]) / 2, (rect[2] + rect[3]) / 2, 0])
            return
    if not getattr(scene, "_logo_warned", False):
        scene._logo_warned = True
        audit.findings.append(
            {"kind": "logo_no_free_corner", "text": "", "other": "", "time": round(float(scene.time), 2)}
        )


def _make_logo(scene):
    if not LOGO_PATH.exists():
        raise FileNotFoundError(f"{LOGO_PATH} is missing; run tools/fetch_logo.py")
    logo = ImageMobject(str(LOGO_PATH))
    logo.set_height(LOGO_HEIGHT)
    logo.set_opacity(0.9)
    _, rect = _corners(logo)[0]
    logo.move_to([(rect[0] + rect[1]) / 2, (rect[2] + rect[3]) / 2, 0])
    logo.add_updater(lambda mob, dt: _place_logo(scene, mob))
    return logo


def _ensure_logo(scene) -> None:
    logo = scene._layer_logo
    if logo not in scene.mobjects:
        scene.add_foreground_mobject(logo)
    elif logo not in scene.foreground_mobjects:
        scene.foreground_mobjects.append(logo)


# ---------------------------------------------------------------------------------------------
# Scene.play / Scene.wait
# ---------------------------------------------------------------------------------------------
def _visible_texts(scene):
    return [
        m
        for m in scene.get_mobject_family_members()
        if isinstance(m, _TEXT_CLASSES) and (m.submobjects or m.has_points()) and audit.max_opacity(m) > 0.3
    ]


def _reading_need(scene) -> float:
    read = scene._layer_read
    need = 0.0
    fresh = False
    for text in _visible_texts(scene):
        if id(text) in read:
            continue
        read.add(id(text))
        chars = len((getattr(text, "text", "") or "").strip())
        if chars == 0:
            continue
        fresh = True
        code = i18n.is_code_font(getattr(text, "font", ""))
        need += chars * (READ_PER_CODE_CHAR if code else READ_PER_CHAR)
    return (READ_BASE + need) if fresh else 0.0


def _patched_play(scene, *args, **kwargs):
    original = _originals["play"]
    if getattr(scene, "_layer_inner", False) or not hasattr(scene, "_layer_logo"):
        return original(scene, *args, **kwargs)
    _ensure_logo(scene)
    animations, follow = [], []
    for arg in args:
        animation = prepare_animation(arg)
        if animation.mobject is scene._layer_logo:
            continue
        split = _split_text_transform(animation)
        if split is not None:
            animations.append(split[0])
            follow.append(split[1])
        else:
            animations.append(animation)
    if not animations and not follow:
        return None
    if "run_time" in kwargs and kwargs["run_time"] is not None:
        kwargs["run_time"] = kwargs["run_time"] * SPEED
    else:
        for animation in animations:
            animation.run_time = animation.run_time * SPEED
    start = scene.time
    if animations:
        original(scene, *animations, **kwargs)
    if follow:
        for animation in follow:
            animation.run_time = animation.run_time * SPEED
        original(scene, *follow)
    audit.check(scene, scene._layer_logo)
    extra = _reading_need(scene) - (scene.time - start)
    if extra > 0.05:
        scene._layer_inner = True
        try:
            original(scene, _originals["Wait"](run_time=extra))
        finally:
            scene._layer_inner = False
    return None


# ---------------------------------------------------------------------------------------------
# final card and report
# ---------------------------------------------------------------------------------------------
def _catalog_entry(scene_name: str) -> dict | None:
    path = ROOT / "catalog.json"
    if not path.exists():
        return None
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    for entry in data.get("scenes", []):
        if entry.get("scene") == scene_name:
            return entry
    return None


def _pick(value, lang: str) -> str:
    return value.get(lang) or value.get("en", "") if isinstance(value, dict) else str(value)


def _final_card(scene) -> None:
    scene._layer_inner = True
    i18n.bypass = True
    try:
        entry = _catalog_entry(type(scene).__name__)
        if entry is None:
            _report["no_catalog_entry"] = True
        # hold the last frame, then clear everything except the logo
        original = _originals["play"]
        original(scene, _originals["Wait"](run_time=FINAL_HOLD))
        audit.check(scene, scene._layer_logo)
        clear = [FadeOut(m) for m in scene.mobjects if m is not scene._layer_logo]
        if clear:
            original(scene, *clear, run_time=0.5)
        title = Text(
            "Pour aller plus loin" if LANG == "fr" else "Go further", font=S.TEXT_FONT, font_size=48, color=S.C_TITLE
        )
        title.to_edge(UP, buff=1.0)
        rows = VGroup()
        for link in (entry or {}).get("links", []):
            label = Text(_pick(link.get("label", ""), LANG), font=S.TEXT_FONT, font_size=30, color=S.C_TEXT)
            lines = link.get("lines")
            where = link["path"] + (f":{lines[0]}-{lines[1]}" if lines else "")
            path = Text(where, font=S.CODE_FONT, font_size=24, color=S.C_MODEL)
            rows.add(VGroup(label, path).arrange(DOWN, aligned_edge=LEFT, buff=0.08))
        if len(rows):
            rows.arrange(DOWN, aligned_edge=LEFT, buff=0.32).next_to(title, DOWN, buff=0.7)
            room = rows.get_top()[1] - (-config.frame_height / 2 + 0.6)  # keep a margin under the last link
            if rows.height > room:  # many links: shrink the list to fit
                rows.scale(room / rows.height)
                rows.next_to(title, DOWN, buff=0.7)
            rows.align_on_border(LEFT, buff=1.2)
        card = VGroup(title, rows)
        original(scene, FadeIn(card), run_time=0.5)
        audit.check(scene, scene._layer_logo)
        original(scene, _originals["Wait"](run_time=CARD_DURATION - 1.0))
        original(scene, FadeOut(card), run_time=0.5)
    finally:
        i18n.bypass = False
        scene._layer_inner = False


def _write_report(scene) -> None:
    if not REPORT_DIR:
        return
    out = Path(REPORT_DIR)
    out.mkdir(parents=True, exist_ok=True)
    report = {
        "scene": type(scene).__name__,
        "lang": LANG,
        "duration_s": round(float(scene.time), 2),
        "audit": audit.findings,
        "missing_keys": sorted(i18n.missing_keys),
        "keys": sorted(i18n.seen_keys),
        **_report,
    }
    with open(out / f"report_{type(scene).__name__}_{LANG}.json", "w", encoding="utf-8") as handle:
        json.dump(report, handle, ensure_ascii=False, indent=2)


def _patched_render(scene, preview: bool = False):
    config.background_color = S.BG
    scene.camera.background_color = manim.ManimColor(S.BG)
    scene._layer_logo = _make_logo(scene)
    scene._layer_read = set()
    scene._layer_inner = False
    scene.add_foreground_mobject(scene._layer_logo)
    construct = scene.construct

    def wrapped():
        construct()
        _final_card(scene)
        _write_report(scene)

    scene.construct = wrapped
    return _originals["render"](scene, preview)


def install() -> None:
    """Patch Manim. Must run before any scene module is imported."""
    if _originals.get("installed"):
        return
    config.background_color = S.BG
    _originals["installed"] = True
    _originals["play"] = Scene.play
    _originals["render"] = Scene.render
    _originals["Wait"] = manim.Wait
    _patch_animations()
    for cls in _TEXT_CLASSES:
        _patch_text_class(cls)
    Scene.play = _patched_play
    Scene.render = _patched_render
