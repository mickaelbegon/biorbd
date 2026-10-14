"""Video 18: the CasADi build of biorbd gives symbolic functions, here the mass matrix of a 2-link pendulum."""

import re

from manim import *

from common import bio_style as S

TOPIC = "casadi_symbolic"
BRACKET_COLOR = S.C_MUTED


def code_lines() -> list[str]:
    """The lines the generator runs (rule D5): read between its CODE-BEGIN and CODE-END markers."""
    source = (S.ROOT / f"generate_{TOPIC}_data.py").read_text(encoding="utf-8")
    body = re.search(r"# CODE-BEGIN\n(.*?)\n\s*# CODE-END", source, re.DOTALL).group(1)
    return body.splitlines()


def bracket(height: float, side: str) -> VMobject:
    """A square bracket drawn with lines (no LaTeX)."""
    sign = -1 if side == "left" else 1
    points = [[sign * -0.08, height / 2, 0], [sign * 0.0, height / 2, 0], [sign * 0.0, -height / 2, 0]]
    points.append([sign * -0.08, -height / 2, 0])
    bracket_shape = VMobject(color=BRACKET_COLOR, stroke_width=3)
    bracket_shape.set_points_as_corners(points)
    return bracket_shape


def matrix_block(rows, fmt: str, color: str, font_size: int = 26) -> VGroup:
    cols = [
        VGroup(*[Text(fmt.format(float(v)), font_size=font_size, color=color) for v in rows[:, j]]).arrange(
            DOWN, buff=0.2, aligned_edge=RIGHT
        )
        for j in range(rows.shape[1])
    ]
    body = VGroup(*cols).arrange(RIGHT, buff=0.35, aligned_edge=UP)
    return VGroup(
        bracket(body.height + 0.2, "left").next_to(body, LEFT, buff=0.05),
        body,
        bracket(body.height + 0.2, "right").next_to(body, RIGHT, buff=0.05),
    )


class AnimCasadiSymbolic(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")

        # A1
        head = S.title_block("Symbolic functions", "A separate build of biorbd gives symbolic functions.")
        self.play(Write(head))
        panel = S.code_panel("biorbd code", code_lines(), font_size=15)
        self.play(FadeIn(panel))

        # Stage 1 (point 1): the mass matrix of the pendulum as a CasADi function
        intro = Text("The CasADi build is installed separately.", font_size=24, color=S.C_TEXT)
        intro.move_to([S.LEFT_X0 + intro.width / 2, 1.75, 0])
        cap_f = Text("The mass matrix becomes a function of q.", font_size=24, color=S.C_TEXT)
        cap_f.move_to([S.LEFT_X0 + cap_f.width / 2, 0.9, 0])
        func = Text(str(d["func_repr"]), font=S.CODE_FONT, font_size=24, color=S.C_MODEL)
        func.next_to(cap_f, DOWN, buff=0.2).align_to(cap_f, LEFT)
        cap_e = Text("One entry of the matrix, in terms of q.", font_size=24, color=S.C_TEXT)
        cap_e.next_to(func, DOWN, buff=0.5).align_to(cap_f, LEFT)
        expr = Text("M01 = " + str(d["m01_expr"]), font=S.CODE_FONT, font_size=24, color=S.C_MODEL)
        expr.next_to(cap_e, DOWN, buff=0.2).align_to(cap_f, LEFT)
        self.play(FadeIn(intro))
        self.play(FadeIn(VGroup(cap_f, func)))
        self.play(FadeIn(VGroup(cap_e, expr)))

        # Stage 2 (point 2): evaluated, then compared with the default (Eigen) build
        self.play(FadeOut(VGroup(intro, cap_f, func, cap_e, expr)))
        q0 = d["q0"]
        at = Text(f"Both builds evaluated at q = [{q0[0]:.2f}, {q0[1]:.2f}] rad.", font_size=24, color=S.C_TEXT)
        if at.width > 7.5:  # stay clear of the code panel caption, even with a longer French string
            at.scale_to_fit_width(7.5)
        at.move_to([S.LEFT_X0 + at.width / 2, 1.75, 0])
        m_cas = matrix_block(d["M_cas"], "{:.4f}", S.C_MODEL)
        m_eig = matrix_block(d["M_eig"], "{:.4f}", S.C_DATA)
        both = VGroup(m_cas, m_eig).arrange(RIGHT, buff=0.9)
        both.move_to([-3.6, 0.3, 0])
        lab_c = Text("CasADi build", font_size=22, color=S.C_MODEL).next_to(m_cas, UP, buff=0.2)
        lab_e = Text("Default build", font_size=22, color=S.C_DATA).next_to(m_eig, UP, buff=0.2)
        lab_e.set_y(lab_c.get_y())  # same height in both languages (a descender would shift one label)
        max_diff = float(d["max_diff"])
        assert max_diff < 1e-12 and bool(d["eigen_backend"]) and bool(d["casadi_backend"])
        diff = Text(f"Largest difference: {max_diff:.6f}", font_size=26, color=S.C_DIFF)
        diff.move_to([-3.6, -1.6, 0])
        self.play(FadeIn(at))
        self.play(FadeIn(VGroup(m_cas, lab_c)))
        self.play(FadeIn(VGroup(m_eig, lab_e)))
        self.play(FadeIn(diff))

        # A4
        self.play(FadeIn(S.footer("Both builds give the same mass matrix for this pendulum.")))
