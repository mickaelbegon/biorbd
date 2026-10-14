"""Video 09: inverse dynamics gives the torques of a motion, tau = M(q) qddot + N(q, qdot)."""

import re

from manim import *

from common import bio_style as S

TOPIC = "inverse_dynamics"
BRACKET_COLOR = S.C_MUTED


def code_lines() -> list[str]:
    """The lines the generator runs (rule D5): read between its CODE-BEGIN and CODE-END markers."""
    source = (S.ROOT / f"generate_{TOPIC}_data.py").read_text(encoding="utf-8")
    body = re.search(r"# CODE-BEGIN\n(.*?)\n\s*# CODE-END", source, re.DOTALL).group(1)
    return [line[4:] for line in body.splitlines()]  # the loop body is indented by 4 spaces in the generator


def bracket(height: float, side: str) -> VMobject:
    """A square bracket drawn with lines (no LaTeX)."""
    sign = -1 if side == "left" else 1
    points = [[sign * -0.08, height / 2, 0], [sign * 0.0, height / 2, 0], [sign * 0.0, -height / 2, 0]]
    points.append([sign * -0.08, -height / 2, 0])
    bracket_shape = VMobject(color=BRACKET_COLOR, stroke_width=3)
    bracket_shape.set_points_as_corners(points)
    return bracket_shape


def vector_block(values, fmt: str, color: str, font_size: int = 24) -> VGroup:
    """One column of numbers between brackets (D2: one template per quantity)."""
    body = VGroup(*[Text(fmt.format(float(v)), font_size=font_size, color=color) for v in values]).arrange(
        DOWN, buff=0.16, aligned_edge=RIGHT
    )
    return VGroup(
        bracket(body.height + 0.2, "left").next_to(body, LEFT, buff=0.05),
        body,
        bracket(body.height + 0.2, "right").next_to(body, RIGHT, buff=0.05),
    )


class AnimInverseDynamics(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        TAU = "{:.2f}"

        # A1
        head = S.title_block("Inverse dynamics", "The torques that produce a motion.")
        self.play(Write(head))
        synth = S.synthetic_tag("synthetic")

        # Stage 1 (point 1): tau = M qddot + N at one instant
        k = int(d["shown_index"])
        intro = Text(f"Arm model, motion at t = {float(d['shown_time']):.2f} s.", font_size=24, color=S.C_TEXT)
        intro.move_to([S.LEFT_X0 + intro.width / 2, 1.75, 0])
        tau = vector_block(d["tau_id"][k], TAU, S.C_MODEL)
        m_qddot = vector_block(d["m_qddot"][k], TAU, S.C_MODEL)
        nle = vector_block(d["nle"][k], TAU, S.C_MODEL)
        equals, plus = (Text(s, font_size=28, color=S.C_TEXT) for s in ("=", "+"))
        row = VGroup(tau, equals, m_qddot, plus, nle).arrange(RIGHT, buff=0.3)
        row.move_to([-3.6, 0.3, 0])

        def label(text, block):
            tag = Text(text, font_size=22, color=S.C_MODEL, font=S.CODE_FONT)
            return tag.next_to(block, UP, buff=0.15)

        lab_t = label("tau", tau)
        lab_m = label("M @ qddot", m_qddot)
        lab_n = label("N", nle)
        unit = Text("All torques are in N·m.", font_size=22, color=S.C_MUTED)
        unit.next_to(row, DOWN, buff=0.5).align_to(intro, LEFT)

        panel = S.code_panel("biorbd code", code_lines(), font_size=16)
        self.play(FadeIn(intro), FadeIn(synth))
        self.play(FadeIn(panel))
        self.play(FadeIn(VGroup(tau, lab_t)))
        self.play(FadeIn(VGroup(equals, m_qddot, lab_m, plus, nle, lab_n, unit)))

        # Stage 2 (point 2): the check over the whole motion, with the difference axis (A6)
        self.play(FadeOut(VGroup(intro, row, lab_t, lab_m, lab_n, unit)))
        time, t_id, t_chk = d["time"], d["tau_id"], d["tau_check"]
        diff, d_max = d["diff_abs"], float(d["diff_max"])
        t_max = float(abs(t_id).max())
        cfg = {"include_numbers": False, "color": S.C_MUTED, "include_ticks": False}
        xr = [float(time[0]), float(time[-1]), 0.25]
        axes = Axes(
            x_range=xr,
            y_range=[-t_max * 1.1, t_max * 1.1, t_max],
            x_length=5.6,
            y_length=1.8,
            tips=False,
            axis_config=cfg,
        )
        axes.move_to([-3.7, 0.5, 0])
        axes_d = Axes(
            x_range=xr, y_range=[0, d_max * 1.3, d_max], x_length=5.6, y_length=0.9, tips=False, axis_config=cfg
        )
        axes_d.next_to(axes, DOWN, buff=0.6).align_to(axes, LEFT)
        title_top = Text(f"Torques of the two joints, up to {t_max:.1f} N·m.", font_size=24, color=S.C_TEXT)
        title_top.next_to(axes, UP, buff=0.15).align_to(axes, LEFT)
        title_diff = Text(f"Difference, at most {d_max:.1e} N·m.", font_size=22, color=S.C_DIFF)
        title_diff.next_to(axes_d, UP, buff=0.1).align_to(axes_d, LEFT)

        def line(ax, y, **kw):
            return ax.plot_line_graph(time, y, add_vertex_dots=False, **kw)["line_graph"]

        ghost = VGroup(*[line(axes, t_id[:, j], line_color=S.C_DATA, stroke_width=6) for j in range(t_id.shape[1])])
        ghost.set_stroke(opacity=0.5)  # inverse_dynamics kept visible behind M @ qddot + N
        check = VGroup(*[line(axes, t_chk[:, j], line_color=S.C_MODEL, stroke_width=3) for j in range(t_chk.shape[1])])
        diff_curve = line(axes_d, diff, line_color=S.C_DIFF, stroke_width=3)
        key_id = Text("inverse_dynamics", font_size=22, color=S.C_DATA, font=S.CODE_FONT)
        key_mn = Text("M @ qddot + N", font_size=22, color=S.C_MODEL, font=S.CODE_FONT)
        keys = VGroup(key_id, key_mn).arrange(RIGHT, buff=0.4)
        keys.move_to([-3.7, 2.15, 0])
        title_top.next_to(keys, DOWN, buff=0.15).align_to(axes, LEFT)

        self.play(FadeIn(VGroup(axes, axes_d, title_top, title_diff, keys)))
        self.play(Create(ghost), run_time=1.5)
        self.play(Create(check), run_time=1.5)
        self.play(Create(diff_curve), run_time=1.5)

        # A4
        self.play(
            FadeIn(S.footer("The torques equal the mass matrix times the acceleration plus the nonlinear effects."))
        )
