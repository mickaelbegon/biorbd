"""Video 10: forward dynamics gives qddot; integrating it is our job, and the energy drift shows the quality."""

import textwrap

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "forward_dynamics"
GENERATOR = S.ROOT / "generate_forward_dynamics_data.py"


def generator_code(tag: str) -> list[str]:
    """The lines between CODE<tag>-BEGIN and CODE<tag>-END of the generator that runs them (rule D5)."""
    lines = GENERATOR.read_text(encoding="utf-8").splitlines()
    begin = next(i for i, line in enumerate(lines) if f"# CODE{tag}-BEGIN" in line)
    end = next(i for i, line in enumerate(lines) if f"# CODE{tag}-END" in line)
    body = textwrap.dedent("\n".join(lines[begin + 1 : end])).splitlines()
    body = [line for line in body if line.strip()]  # blank lines are not displayed
    return body


def code_panel(lines: list[str], font_size: int = 16) -> VGroup:
    """Like bio_style.code_panel, but keeps the indentation (Text drops leading spaces, so lines are shifted)."""
    cap = Text("biorbd code", font_size=26, color=S.C_MUTED)
    texts = VGroup(
        *[Text(line.lstrip(), font=S.CODE_FONT, font_size=font_size, color=S.C_CODE_TEXT) for line in lines]
    ).arrange(DOWN, aligned_edge=LEFT, buff=0.12)
    indent = 0.11 * font_size / 16 * 4  # width of four characters
    for txt, line in zip(texts, lines):
        txt.shift(RIGHT * indent * (len(line) - len(line.lstrip())) / 4)
    box = RoundedRectangle(
        corner_radius=0.12,
        width=max(texts.width + 0.5, S.CODE_PANEL_WIDTH),
        height=texts.height + 0.5,
        fill_color=S.C_CODE_BG,
        fill_opacity=1,
        stroke_color=S.C_MUTED,
        stroke_width=1.5,
    )
    texts.move_to(box.get_center()).align_to(box, LEFT).shift(RIGHT * 0.25)
    body = VGroup(box, texts)
    cap.next_to(body, UP, buff=0.18).align_to(body, LEFT)
    panel = VGroup(cap, body).move_to([S.CODE_PANEL_X, 0.2, 0])
    return panel.shift(LEFT * max(0.0, panel.get_right()[0] - 6.95))


def sentence(text: str, y: float = -1.7, color=S.C_TEXT) -> Text:
    out = Text(text, font_size=22, color=color)
    if out.width > 6.8:
        out.scale_to_fit_width(6.8)
    return out.move_to([-3.4, y, 0])


class AnimForwardDynamics(Scene):
    def construct(self):
        data = S.load_npz(f"{TOPIC}_main.npz")
        t = data["t"]
        drift_eu, drift_rk = data["drift_euler_pct"], data["drift_rk4_pct"]
        qddot0, dt = data["qddot0"], float(data["dt"])

        self.play(Write(S.title_block("Forward dynamics", "Integrate qddot to get a motion")))
        tag_a = S.synthetic_tag("synthetic")
        tag_b = S.synthetic_tag("handmade").next_to(tag_a, RIGHT, buff=0.6)
        tag_b.set_y(tag_a.get_y())
        panel1 = code_panel(generator_code("1"))
        self.play(FadeIn(panel1), FadeIn(tag_a), FadeIn(tag_b))

        # --- point 1: biorbd returns qddot, it does not integrate ---------------------------------
        s1 = sentence(f"At t = 0 forward dynamics returns qddot = {qddot0[0]:.1f}, {qddot0[1]:.1f}, {qddot0[2]:.1f}.")
        s1b = sentence("biorbd does not integrate it: we write the integrator ourselves.", y=-2.1)
        self.play(FadeIn(s1), run_time=1.0)
        self.play(FadeIn(s1b), run_time=1.0)
        self.wait(1.0)

        # --- point 2: the energy drift of a passive system ------------------------------------------
        panel2 = code_panel(generator_code("2"))
        self.play(FadeOut(s1), FadeOut(s1b), ReplacementTransform(panel1, panel2))
        w = 4.8
        o1, h1 = np.array([-6.0, 0.6, 0.0]), 1.3
        o2, h2 = np.array([-6.0, -0.75, 0.0]), 0.7
        top = [Line(o1, o1 + RIGHT * w, color=S.C_MUTED), Line(o1, o1 + UP * h1, color=S.C_MUTED)]
        bot = [Line(o2, o2 + RIGHT * w, color=S.C_MUTED), Line(o2, o2 + UP * h2, color=S.C_MUTED)]
        top_title = Text("Energy drift (%)", font_size=20, color=S.C_MUTED).next_to(top[1], UP, buff=0.08)
        top_title.align_to(top[1], LEFT)
        d_max = float(drift_eu.max())
        top_top = Text(f"{d_max:.0f}", font_size=18, color=S.C_MUTED).next_to(top[1], LEFT, buff=0.06)
        top_top.align_to(top[1], UP)
        bot_title = Text("Euler minus RK4 (%)", font_size=20, color=S.C_DIFF).next_to(bot[1], UP, buff=0.08)
        bot_title.align_to(bot[1], LEFT).shift(RIGHT * 2.3)
        bot_top = Text(f"{d_max:.0f}", font_size=18, color=S.C_MUTED).next_to(bot[1], LEFT, buff=0.06)
        bot_top.align_to(bot[1], UP)
        time_lab = Text("Time (s)", font_size=20, color=S.C_MUTED).next_to(bot[0], DOWN, buff=0.15)
        t_end = Text(f"{t[-1]:.0f}", font_size=18, color=S.C_MUTED).next_to(bot[0], DOWN, buff=0.15)
        t_end.align_to(bot[0], RIGHT)
        time_lab.next_to(t_end, LEFT, buff=1.4)

        def curve(values, origin, height, color, opacity=1.0, width=4):
            pts = [origin + np.array([w * tt / t[-1], height * v / d_max, 0.0]) for tt, v in zip(t, values)]
            out = VMobject(stroke_color=color, stroke_width=width, stroke_opacity=opacity)
            out.set_points_as_corners(pts)
            return out

        c_eu = curve(drift_eu, o1, h1, S.C_DATA)
        c_rk = curve(drift_rk, o1, h1, S.C_MODEL)
        c_diff = curve(drift_eu - drift_rk, o2, h2, S.C_DIFF)
        key_e = Text("Euler", font_size=20, color=S.C_DATA).move_to([-2.9, 1.95, 0], aligned_edge=LEFT)
        key_r = Text("RK4", font_size=20, color=S.C_MODEL).next_to(key_e, RIGHT, buff=0.4)
        frame = [*top, *bot, top_title, bot_title, top_top, bot_top, time_lab, t_end]
        self.play(*[FadeIn(m) for m in frame], Create(c_eu), FadeIn(key_e), run_time=1.6)
        s2 = sentence(f"With Euler the total energy drifts by {np.abs(drift_eu).max():.2f} % in {t[-1]:.0f} s.")
        self.play(FadeIn(s2))
        self.wait(0.6)
        s3 = sentence(f"With RK4 and the same {dt:.2f} s step, it stays below {np.abs(drift_rk).max():.2f} %.")
        self.play(
            c_eu.animate.set_stroke(opacity=0.3),
            Create(c_rk),
            Create(c_diff),
            FadeIn(key_r),
            ReplacementTransform(s2, s3),
            run_time=1.6,
        )
        self.wait(0.6)
        self.play(FadeIn(S.footer("A better integrator keeps the energy of a passive system nearly constant.")))
