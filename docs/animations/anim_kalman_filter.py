"""Video 16: an extended Kalman filter reconstructs q from noisy synthetic markers of the arm26 model."""

import textwrap

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "kalman_filter"
GENERATOR = S.ROOT / "generate_kalman_filter_data.py"


def generator_code() -> list[str]:
    """The lines of the filter loop, read from the generator that runs them (rule D5)."""
    lines = GENERATOR.read_text(encoding="utf-8").splitlines()
    begin = next(i for i, line in enumerate(lines) if "# CODE-BEGIN" in line)
    end = next(i for i, line in enumerate(lines) if "# CODE-END" in line)
    return textwrap.dedent("\n".join(lines[begin + 1 : end])).splitlines()


def keep_right_margin(panel, limit: float = 6.7):
    """Shift a wide code panel left so that its right edge keeps a margin to the frame edge (frame half-width 7.11)."""
    excess = panel.get_right()[0] - limit
    if excess > 0:
        panel.shift(LEFT * excess)
    return panel


def sci(x) -> str:
    mant, exp = f"{float(x):.0e}".split("e")
    return f"{mant}e{int(exp)}"


def sentence(text: str, y: float = -1.95, color=S.C_TEXT) -> Text:
    out = Text(text, font_size=24, color=color)
    if out.width > 7.0:
        out.scale_to_fit_width(7.0)
    return out.move_to([S.LEFT_X0, y, 0], aligned_edge=LEFT)


class AnimKalmanFilter(Scene):
    def construct(self):
        data = S.load_npz(f"{TOPIC}_main.npz")
        t = data["t"]
        q_true, q_kal, q_alone = (np.degrees(data[k]) for k in ("q_true", "q_kal", "q_alone"))
        err_kal = np.sqrt(np.mean((q_kal - q_true) ** 2, axis=1))
        err_alone = np.sqrt(np.mean((q_alone - q_true) ** 2, axis=1))
        rmse_kal, rmse_alone = float(data["rmse_q_kal_deg"]), float(data["rmse_q_alone_deg"])

        self.play(Write(S.title_block("Kalman filter", "Reconstruct q from noisy markers")))
        tag_a = S.synthetic_tag("synthetic")
        tag_b = S.synthetic_tag("handmade").next_to(tag_a, RIGHT, buff=0.6)
        panel = keep_right_margin(S.code_panel("biorbd code", generator_code(), font_size=16))
        settings = Text(
            "\n".join(
                (
                    f"FREQ = {int(data['frequency'])}",
                    f"NOISE = {sci(data['noise_factor'])}",
                    f"ERROR = {sci(data['error_factor'])}",
                )
            ),
            font=S.CODE_FONT,
            font_size=18,
            color=S.C_MUTED,
        )
        settings.next_to(panel, DOWN, buff=0.3).align_to(panel, LEFT)
        self.play(FadeIn(panel), FadeIn(settings), FadeIn(tag_a), FadeIn(tag_b))

        # axes: joint angles on top, difference with the truth below (rule A6)
        w = 5.0
        o1, h1 = np.array([-6.0, 0.15, 0.0]), 1.6
        o2, h2 = np.array([-6.0, -1.25, 0.0]), 0.75
        top = [Line(o1, o1 + RIGHT * w, color=S.C_MUTED), Line(o1, o1 + UP * h1, color=S.C_MUTED)]
        bot = [Line(o2, o2 + RIGHT * w, color=S.C_MUTED), Line(o2, o2 + UP * h2, color=S.C_MUTED)]
        top_title = Text("Joint angles", font_size=20, color=S.C_MUTED).next_to(top[1], UP, buff=0.08)
        top_title.align_to(top[1], LEFT)
        bot_title = Text("Difference with the truth", font_size=20, color=S.C_DIFF).next_to(bot[1], UP, buff=0.1)
        bot_title.align_to(bot[1], LEFT).shift(RIGHT * 2.2)
        d_max = float(err_alone.max())
        bot_top = Text(f"{d_max:.1f}°", font_size=18, color=S.C_MUTED).next_to(bot[1], LEFT, buff=0.06)
        bot_top.align_to(bot[1], UP)
        top_top = Text("90°", font_size=18, color=S.C_MUTED).next_to(top[1], LEFT, buff=0.06).align_to(top[1], UP)
        time_lab = Text("Time (s)", font_size=20, color=S.C_MUTED).next_to(bot[0], DOWN, buff=0.06)

        def curve(values, origin, height, vmax, color, opacity=1.0, width=3):
            pts = [
                origin + np.array([w * (tt - t[0]) / (t[-1] - t[0]), height * v / vmax, 0.0])
                for tt, v in zip(t, values)
            ]
            out = VMobject(stroke_color=color, stroke_width=width, stroke_opacity=opacity)
            out.set_points_as_corners(pts)
            return out

        truth = [curve(q_true[:, j], o1, h1, 90.0, S.C_DATA, width=4) for j in range(2)]
        alone = [curve(q_alone[:, j], o1, h1, 90.0, S.C_MODEL, opacity=0.75, width=2) for j in range(2)]
        kal = [curve(q_kal[:, j], o1, h1, 90.0, S.C_MODEL, width=4) for j in range(2)]
        d_alone = curve(err_alone, o2, h2, d_max, S.C_DIFF, opacity=0.75, width=2)
        d_kal = curve(err_kal, o2, h2, d_max, S.C_DIFF, width=4)
        key_t = Text("True q", font_size=18, color=S.C_DATA).move_to([-5.9, 2.3, 0], aligned_edge=LEFT)
        key_a = Text("Each frame alone", font_size=18, color=S.C_MODEL).next_to(key_t, RIGHT, buff=0.3)
        key_k = Text("Kalman filter", font_size=18, color=S.C_MODEL).next_to(key_a, RIGHT, buff=0.3)
        frame = [*top, *bot, top_title, bot_title, bot_top, top_top, time_lab]

        # --- point 1: each frame solved alone follows the noise ------------------------------
        self.play(*[FadeIn(m) for m in frame], *[Create(c) for c in truth], FadeIn(key_t))
        self.play(*[Create(c) for c in alone], Create(d_alone), FadeIn(key_a), run_time=1.2)
        s1 = sentence(f"Solving each frame alone leaves {rmse_alone:.2f} degrees of error (RMSE).")
        self.play(FadeIn(s1))

        # --- point 2: the filter uses the previous frames --------------------------------------
        self.play(
            *[c.animate.set_stroke(opacity=0.3) for c in (*alone, d_alone)],
            *[Create(c) for c in (*kal, d_kal)],
            FadeIn(key_k),
            run_time=1.4,
        )
        s2 = sentence(
            f"The Kalman filter uses past frames and lowers the error to {rmse_kal:.2f} degrees.", color=S.C_DIFF
        )
        self.play(ReplacementTransform(s1, s2))
        self.play(
            FadeIn(S.footer("The filter smooths the marker noise by combining each frame with what came before."))
        )
