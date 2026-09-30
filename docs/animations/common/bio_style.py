"""Colours, fonts and layout helpers shared by every scene of the series."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from manim import DOWN, LEFT, ORIGIN, RIGHT, UP, RoundedRectangle, Text, VGroup, config

ROOT = Path(__file__).resolve().parents[1]
DATA_DIR = ROOT / "data"
ASSETS_DIR = ROOT / "assets"

# --- fonts (set in Text.__init__ by the layer when a scene does not choose one) -----------------
TEXT_FONT = "Segoe UI"
CODE_FONT = "Consolas"

# --- colour roles: the same meaning in every scene ----------------------------------------------
BG = "#0F1722"
C_TEXT = "#F2F5F7"  # body text
C_MUTED = "#9AA5B1"  # captions, footers, secondary labels
C_TITLE = "#4FC3F7"  # titles and the object being explained
C_MODEL = "#4DB6AC"  # what biorbd computes (model output)
C_DATA = "#FFB74D"  # reference / measured / input data
C_DIFF = "#EF5350"  # differences, errors, residuals
C_SYNTH = "#BA68C8"  # "synthetic data" and "redrawn interface" labels
C_OK = "#81C784"  # checks that pass
C_CODE_BG = "#1A2636"
C_CODE_TEXT = "#E0E6ED"

# --- layout (frame is 14.22 x 8 Manim units) -----------------------------------------------------
LEFT_X0 = -6.8  # left edge of the visuals area
CODE_PANEL_WIDTH = 5.2
CODE_PANEL_X = 4.0  # centre x of the code panel


def load_npz(name: str):
    """Load ``data/<name>``. Every number shown on screen must come from such a file."""
    path = DATA_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"{path} is missing; run the matching generate_*.py first")
    return np.load(path, allow_pickle=False)


def title_block(title: str, subtitle: str) -> VGroup:
    """Title (44 pt) and one-line subtitle (28 pt) at the top left."""
    head = Text(title, font_size=44, color=C_TITLE, weight="BOLD")
    sub = Text(subtitle, font_size=28, color=C_MUTED)
    group = VGroup(head, sub).arrange(DOWN, aligned_edge=LEFT, buff=0.15)
    group.to_corner(UP + LEFT, buff=0.4)
    return group


def footer(sentence: str) -> Text:
    """The single closing sentence of the scene (whole sentence, bottom centre)."""
    text = Text(sentence, font_size=30, color=C_TEXT)
    if text.width > 12.6:
        text.scale_to_fit_width(12.6)
    text.to_edge(DOWN, buff=0.35)
    return text


def code_panel(caption: str, lines: list[str], font_size: int = 22) -> VGroup:
    """Code panel on the right. The caption (legend) sits ABOVE the code box.

    The code shown must be the code that runs (see STANDARD.md, rule D5).
    """
    cap = Text(caption, font_size=26, color=C_MUTED)
    code_lines = VGroup(
        *[Text(line if line else " ", font=CODE_FONT, font_size=font_size, color=C_CODE_TEXT) for line in lines]
    )
    code_lines.arrange(DOWN, aligned_edge=LEFT, buff=0.12)
    box = RoundedRectangle(
        corner_radius=0.12,
        width=max(code_lines.width + 0.5, CODE_PANEL_WIDTH),
        height=code_lines.height + 0.5,
        fill_color=C_CODE_BG,
        fill_opacity=1,
        stroke_color=C_MUTED,
        stroke_width=1.5,
    )
    code_lines.move_to(box.get_center()).align_to(box, LEFT).shift(RIGHT * 0.25)
    body = VGroup(box, code_lines)
    cap.next_to(body, UP, buff=0.18).align_to(body, LEFT)
    panel = VGroup(cap, body)
    panel.move_to([CODE_PANEL_X, 0.2, 0])
    return panel


def synthetic_tag(kind: str = "synthetic") -> Text:
    """Label for data that are simulated or for redrawn interfaces (rule D3)."""
    label = {"synthetic": "Synthetic data", "interface": "Redrawn interface", "handmade": "Hand-written code"}[kind]
    tag = Text(label, font_size=24, color=C_SYNTH)
    tag.to_corner(DOWN + LEFT, buff=0.35).shift(UP * 1.0)
    return tag
