"""Video 04: rotations. The same three angles in two Euler sequences, then matrix -> quaternion and back.

All numbers come from data/rotations_main.npz (made by generate_rotations_data.py, biorbd 1.12.3).
The code shown is read from the generator source, so it is the code that was executed (rule D5).
"""

from pathlib import Path

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "rotations"
CENTERS = (-5.3, -1.9)  # x of the two frames
FRAME_Y = 0.5
AXIS_LEN = 0.9
# oblique projection of the world axes: x toward the viewer (down-left), y to the right, z up
PROJECTION = np.array([[-0.45, 1.0, 0.0], [-0.32, 0.0, 1.0], [0.0, 0.0, 0.0]])
TEXT_SIZE = 20


def code_blocks() -> list[list[str]]:
    """Lines between the '# code:start' and '# code:end' markers of the generator."""
    lines = (Path(__file__).with_name(f"generate_{TOPIC}_data.py")).read_text(encoding="utf-8").splitlines()
    blocks, current = [], None
    for line in lines:
        if line.startswith("# code:start"):
            current = []
        elif line.startswith("# code:end"):
            blocks.append(current)
            current = None
        elif current is not None:
            current.append(line)
    return blocks


def project(vector) -> np.ndarray:
    return PROJECTION @ np.asarray(vector, dtype=float) * AXIS_LEN


def frame_group(matrix, cx: float, reference: bool) -> VGroup:
    """Axes drawn from the columns of the rotation matrix (world coordinates of the rotated x, y, z)."""
    origin = np.array([cx, FRAME_Y, 0.0])
    group = VGroup()
    for column, name in enumerate("xyz"):
        color = S.C_MUTED if reference else S.C_MODEL
        vector = np.eye(3)[column] if reference else matrix[:, column]
        tip = origin + project(vector)
        arrow = Arrow(origin, tip, buff=0, stroke_width=2.5 if reference else 5, color=color, tip_length=0.12)
        if reference:
            arrow.set_opacity(0.4)
        label = Text(name, font=S.CODE_FONT, font_size=18, color=color)
        label.move_to(tip + (tip - origin) / max(np.linalg.norm(tip - origin), 1e-6) * 0.2)
        if reference:
            label.set_opacity(0.4)
        group.add(arrow, label)
    return group


def matrix_rows(matrix, cx: float, y0: float, color) -> VGroup:
    rows = VGroup()
    for index, row in enumerate(matrix):
        text = "  ".join(f"{value:5.2f}" for value in row)
        rows.add(Text(text, font=S.CODE_FONT, font_size=18, color=color).move_to([cx, y0 - 0.36 * index, 0]))
    return rows


class AnimRotations(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        angles, m_xyz, m_zyx = d["angles"], d["m_xyz"], d["m_zyx"]
        quat, back = d["quat_xyz"], d["back_xyz"]
        block_a, block_b = code_blocks()

        def line(text, y, color=S.C_TEXT, x=-6.6):
            return Text(text, font_size=TEXT_SIZE, color=color).align_to([x, 0, 0], LEFT).set_y(y)

        self.play(Write(S.title_block("Rotations", "Same angles, different order")))

        # point 1: one set of angles, two sequences, two rotation matrices
        angle_text = ", ".join(f"{value:.1f}" for value in angles)
        intro = line(f"Three angles: {angle_text} rad", 2.25, S.C_DATA)
        panel_a = S.code_panel("biorbd code", block_a, font_size=18)
        names = [Text(n, font=S.CODE_FONT, font_size=22, color=S.C_TITLE) for n in ("xyz", "zyx")]
        for name, cx in zip(names, CENTERS):
            name.move_to([cx, 1.9, 0])
        ref = [frame_group(None, cx, True) for cx in CENTERS]
        first = frame_group(m_xyz, CENTERS[0], False)
        second = frame_group(m_zyx, CENTERS[1], False)
        rows_a = matrix_rows(m_xyz, CENTERS[0], -0.6, S.C_MODEL)
        rows_b = matrix_rows(m_zyx, CENTERS[1], -0.6, S.C_MODEL)
        self.play(FadeIn(intro), FadeIn(panel_a))
        self.play(FadeIn(names[0]), FadeIn(ref[0]), FadeIn(first))
        self.play(FadeIn(rows_a))
        self.play(FadeIn(names[1]), FadeIn(ref[1]), FadeIn(second))
        self.play(FadeIn(rows_b))
        diff = line(f"Largest difference between the matrices: {d['matrix_diff']:.2f}", -1.75, S.C_DIFF)
        self.play(FadeIn(diff))

        # point 2: quaternion of the xyz matrix, and the way back to the angles
        panel_b = S.code_panel("biorbd code", block_b, font_size=18)
        quat_head = line("Quaternion of the xyz matrix (w, x, y, z):", -2.15, S.C_TEXT)
        quat_values = line(", ".join(f"{value:.2f}" for value in quat), -2.5, S.C_MODEL)
        back_text = ", ".join(f"{value:.1f}" for value in back)
        check = line(f"Matrix back to xyz angles: {back_text} rad", -2.85, S.C_OK)
        self.play(FadeOut(panel_a), FadeIn(panel_b), FadeIn(quat_head))
        self.play(FadeIn(quat_values))
        self.play(FadeIn(check))

        self.play(FadeIn(S.footer("The angle order changes the rotation; biorbd converts in both directions.")))
