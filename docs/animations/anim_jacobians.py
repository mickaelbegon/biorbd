"""Video 05: the marker Jacobian maps generalized velocities to the marker velocity, v = J(q) * qdot."""

import re

from manim import *

from common import bio_style as S

TOPIC = "jacobians"
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


def number_column(values, fmt: str, color: str, font_size: int = 24) -> VGroup:
    """One column of numbers (D2: one template per quantity)."""
    return VGroup(*[Text(fmt.format(float(v)), font_size=font_size, color=color) for v in values]).arrange(
        DOWN, buff=0.16, aligned_edge=RIGHT
    )


def matrix_block(rows, fmt: str, color: str) -> VGroup:
    cols = [number_column(rows[:, j], fmt, color) for j in range(rows.shape[1])]
    body = VGroup(*cols).arrange(RIGHT, buff=0.3, aligned_edge=UP)
    return VGroup(
        bracket(body.height + 0.2, "left").next_to(body, LEFT, buff=0.05),
        body,
        bracket(body.height + 0.2, "right").next_to(body, RIGHT, buff=0.05),
    )


def vector_block(values, fmt: str, color: str) -> VGroup:
    return matrix_block(values.reshape(-1, 1), fmt, color)


class AnimJacobians(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")

        # A1
        head = S.title_block("Marker Jacobians", "The Jacobian maps joint velocities to marker velocity.")
        self.play(Write(head))

        # Stage 1 (point 1): the small matrix of one marker, J times qdot
        t_shown = float(d["shown_time"])
        intro = Text(f"One marker of the arm at t = {t_shown:.2f} s.", font_size=24, color=S.C_TEXT)
        intro.move_to([S.LEFT_X0 + intro.width / 2, 1.75, 0])
        self.play(FadeIn(intro))

        jac = matrix_block(d["jac_shown"], "{:.2f}", S.C_MODEL)
        qdot = vector_block(d["qdot_shown"], "{:.2f}", S.C_DATA)
        v_jac = vector_block(d["v_jac_shown"], "{:.3f}", S.C_MODEL)
        v_fd = vector_block(d["v_fd_shown"], "{:.3f}", S.C_DATA)
        times, equals, approx = (Text(s, font_size=28, color=S.C_TEXT) for s in ("×", "=", "≈"))
        row = VGroup(jac, times, qdot, equals, v_jac, approx, v_fd).arrange(RIGHT, buff=0.2)
        row.move_to([-3.6, 0.2, 0])
        if row.width > 6.2:
            row.scale_to_fit_width(6.2)
            row.move_to([-3.6, 0.2, 0])

        def label(text, block, color, code=True):
            kw = {"font": S.CODE_FONT} if code else {}
            tag = Text(text, font_size=22, color=color, **kw)
            return tag.next_to(block, UP, buff=0.15)

        lab_j = label("J", jac, S.C_MODEL)
        lab_q = label("qdot", qdot, S.C_DATA)
        lab_v = label("J @ qdot", v_jac, S.C_MODEL)
        lab_f = label("Finite differences", v_fd, S.C_DATA, code=False)
        lab_f.next_to(v_fd, DOWN, buff=0.15)

        panel = S.code_panel("biorbd code", code_lines(), font_size=20)
        self.play(FadeIn(VGroup(jac, lab_j)))
        self.play(FadeIn(panel))
        self.play(FadeIn(VGroup(times, qdot, lab_q)))
        self.play(FadeIn(VGroup(equals, v_jac, lab_v)))
        self.play(FadeIn(VGroup(approx, v_fd, lab_f)))

        # Stage 2 (point 2): the check over a whole trajectory, with the difference axis (A6)
        self.play(FadeOut(VGroup(intro, lab_j, lab_q, lab_v, lab_f, jac, times, qdot, equals, v_jac, approx, v_fd)))
        time = d["time"]
        s_jac, s_fd, diff = d["speed_jac"], d["speed_fd"], d["diff_norm"]
        s_max, d_max = float(s_jac.max()), float(diff.max())
        cfg = {"include_numbers": False, "color": S.C_MUTED, "include_ticks": False}
        xr = [float(time[0]), float(time[-1]), 0.25]
        axes = Axes(
            x_range=xr, y_range=[0, s_max * 1.1, s_max], x_length=5.6, y_length=1.8, tips=False, axis_config=cfg
        )
        axes.move_to([-3.7, 0.35, 0])
        axes_d = Axes(
            x_range=xr, y_range=[0, d_max * 1.1, d_max], x_length=5.6, y_length=0.9, tips=False, axis_config=cfg
        )
        axes_d.next_to(axes, DOWN, buff=0.7).align_to(axes, LEFT)
        title_top = Text(f"Speed of the marker, up to {s_max:.2f} m/s.", font_size=24, color=S.C_TEXT)
        title_top.next_to(axes, UP, buff=0.15).align_to(axes, LEFT)
        title_diff = Text(f"Difference, at most {d_max:.4f} m/s.", font_size=22, color=S.C_DIFF)
        title_diff.next_to(axes_d, UP, buff=0.1).align_to(axes_d, LEFT)
        synth = Text("Synthetic trajectory", font_size=22, color=S.C_SYNTH)
        synth.next_to(axes_d, DOWN, buff=0.12).align_to(axes_d, LEFT)

        def line(*a, **k):
            return axes.plot_line_graph(*a, add_vertex_dots=False, **k)["line_graph"]

        fd_curve = line(time, s_fd, line_color=S.C_DATA, stroke_width=6)
        fd_curve.set_stroke(opacity=0.5)  # reference curve kept visible behind J @ qdot
        jac_curve = line(time, s_jac, line_color=S.C_MODEL, stroke_width=3)
        diff_curve = axes_d.plot_line_graph(time, diff, add_vertex_dots=False, line_color=S.C_DIFF, stroke_width=3)[
            "line_graph"
        ]
        key_fd = Text("Finite differences", font_size=22, color=S.C_DATA)
        key_j = Text("J @ qdot", font_size=22, color=S.C_MODEL, font=S.CODE_FONT)
        keys = VGroup(key_fd, key_j).arrange(RIGHT, buff=0.5)
        keys.move_to([-3.7, 2.15, 0])
        title_top.next_to(axes, UP, buff=0.15).align_to(axes, LEFT)

        self.play(FadeIn(VGroup(axes, axes_d, title_top, title_diff, synth, keys)))
        self.play(Create(fd_curve), run_time=1.5)
        self.play(Create(jac_curve), run_time=1.5)
        self.play(Create(diff_curve), run_time=1.5)

        # A4
        self.play(FadeIn(S.footer("Multiplying by the Jacobian matches the finite differences.")))
