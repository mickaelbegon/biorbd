"""Video 07: the joint-space mass matrix M(q) is symmetric, positive definite, and depends on the posture.

All numbers come from data/mass_matrix_main.npz (made by generate_mass_matrix_data.py, biorbd 1.12.3).
The code shown is read from the generator source, so it is the code that was executed (rule D5).
"""

from pathlib import Path

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "mass_matrix"
SHOULDER = np.array([-6.2, 1.5, 0.0])  # screen position of the shoulder marker
SCALE = 3.8  # screen units per metre
FMT = "{:.3f}"  # one template for every entry of a mass matrix (D2)


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


def to_screen(point) -> np.ndarray:
    """World (x, y, z) in metres -> screen: the arm hangs in the x-y plane."""
    return SHOULDER + np.array([(point[0] - ARM0[0]) * SCALE, (point[1] - ARM0[1]) * SCALE, 0.0])


ARM0 = np.zeros(3)  # set in construct from the shoulder position


def arm_group(points) -> VGroup:
    p = [to_screen(x) for x in points]
    bones = VGroup(*[Line(a, b, color=S.C_TITLE, stroke_width=8) for a, b in zip(p[:-1], p[1:])])
    joints = VGroup(*[Dot(x, radius=0.08, color=S.C_TEXT) for x in p])
    return VGroup(bones, joints)


def bracket(height: float, side: str) -> VMobject:
    """A square bracket drawn with lines (no LaTeX)."""
    sign = -1 if side == "left" else 1
    points = [[sign * -0.08, height / 2, 0], [0, height / 2, 0], [0, -height / 2, 0], [sign * -0.08, -height / 2, 0]]
    shape = VMobject(color=S.C_MUTED, stroke_width=3)
    shape.set_points_as_corners(points)
    return shape


def matrix_block(m, color: str, font_size: int = 24) -> VGroup:
    cols = []
    for j in range(m.shape[1]):
        texts = [Text(FMT.format(float(v)), font_size=font_size, color=color) for v in m[:, j]]
        cols.append(VGroup(*texts).arrange(DOWN, buff=0.16, aligned_edge=RIGHT))
    body = VGroup(*cols).arrange(RIGHT, buff=0.3, aligned_edge=UP)
    return VGroup(
        bracket(body.height + 0.2, "left").next_to(body, LEFT, buff=0.05),
        body,
        bracket(body.height + 0.2, "right").next_to(body, RIGHT, buff=0.05),
    )


class AnimMassMatrix(Scene):
    def construct(self):
        global ARM0
        d = S.load_npz(f"{TOPIC}_main.npz")
        ARM0 = d["points_a"][0]
        block_a, block_b = code_blocks()
        q_a, q_b = d["q_a"], d["q_b"]

        def text(s, size, color, x, y, code=False):
            kw = {"font": S.CODE_FONT} if code else {}
            t = Text(s, font_size=size, color=color, **kw)
            return t.move_to([x + t.width / 2, y, 0])

        head = S.title_block("Mass matrix", "It is symmetric, positive definite and depends on q.")
        self.play(Write(head))

        # point 1: M at pose A, symmetric and positive definite
        arm_a = arm_group(d["points_a"])
        q_text = ", ".join(f"{v:.1f}" for v in q_a)
        lab_pose = text(f"Pose A: shoulder and elbow angles {q_text} rad.", 22, S.C_TEXT, -4.4, 1.85)
        mat_a = matrix_block(d["M_a"], S.C_MODEL).move_to([-2.4, 0.75, 0])
        lab_m = Text("M", font=S.CODE_FONT, font_size=26, color=S.C_MODEL).next_to(mat_a, LEFT, buff=0.3)
        sym = text(f"Symmetry error: {d['asym_a']:.6f}", 22, S.C_OK, -6.8, -1.0)
        eig = text(f"Smallest eigenvalue: {d['lam_a']:.3f} (positive).", 22, S.C_OK, -6.8, -1.5)
        meaning = text("Inertial part of the joint torques: M(q) times the accelerations.", 22, S.C_TEXT, -6.8, -2.2)
        panel_a = S.code_panel("biorbd code", block_a, font_size=20)
        self.play(FadeIn(arm_a), FadeIn(lab_pose))
        self.play(FadeIn(panel_a), FadeIn(VGroup(lab_m, mat_a)))
        self.play(FadeIn(sym))
        self.play(FadeIn(eig))
        self.play(FadeIn(meaning))

        # point 2: q changes -> ghost of pose A, matrix at pose B, difference (A6)
        ghost = arm_a.copy().set_opacity(0.3)
        arm_b = arm_group(d["points_b"])
        ghost_m = mat_a.copy().set_opacity(0.3)
        q_text_b = ", ".join(f"{v:.1f}" for v in q_b)
        lab_pose_b = text(f"Pose B: shoulder and elbow angles {q_text_b} rad.", 22, S.C_TEXT, -4.4, 1.85)
        lab_a = text("Pose A", 20, S.C_MUTED, -3.55, 1.6)
        lab_b = text("Pose B", 20, S.C_MODEL, -1.45, 1.6)
        mat_b = matrix_block(d["M_b"], S.C_MODEL).move_to([-0.6, 0.75, 0])
        ghost_m.move_to([-3.0, 0.75, 0])
        lab_a.next_to(ghost_m, UP, buff=0.12)
        lab_b.next_to(mat_b, UP, buff=0.12)
        mat_d = matrix_block(d["dM"], S.C_DIFF).move_to([-3.0, -0.75, 0])
        lab_d = text("Change from A to B", 20, S.C_DIFF, -6.8, -0.65)
        lab_d.next_to(mat_d, UP, buff=0.12).align_to(mat_d, LEFT)
        eig_b = text(f"Smallest eigenvalue at B: {d['lam_b']:.3f}, still above zero.", 22, S.C_OK, -6.8, -1.75)
        legend = text("Faded: previous pose.", 20, S.C_MUTED, -6.8, -2.45)
        panel_b = S.code_panel("biorbd code", block_b, font_size=20)
        self.play(
            FadeOut(VGroup(arm_a, lab_pose, sym, eig, meaning, lab_m, mat_a, panel_a)),
            FadeIn(ghost),
            FadeIn(ghost_m),
            FadeIn(lab_a),
        )
        self.play(FadeIn(arm_b), FadeIn(lab_pose_b), FadeIn(panel_b), FadeIn(VGroup(lab_b, mat_b)), FadeIn(legend))
        self.play(FadeIn(VGroup(lab_d, mat_d)))
        self.play(FadeIn(eig_b))

        # A4
        self.play(FadeIn(S.footer("The same joint is heavier to accelerate in one posture than in another.")))
