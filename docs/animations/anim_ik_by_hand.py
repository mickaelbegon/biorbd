"""Video 15: inverse kinematics by hand, a Gauss-Newton loop on synthetic markers of the arm26 model."""

import textwrap

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "ik_by_hand"
GENERATOR = S.ROOT / "generate_ik_by_hand_data.py"
ORIGIN_PT = np.array([-5.6, 1.85, 0.0])  # scene position of the acromion marker
SCALE = 5.3  # scene units per metre
SHOW_MARKERS = [1, 2, 3, 4]  # the acromion is fixed to the ground, it carries no information


def generator_code() -> list[str]:
    """The lines of the Gauss-Newton step, read from the generator that runs them (rule D5)."""
    lines = GENERATOR.read_text(encoding="utf-8").splitlines()
    begin = next(i for i, line in enumerate(lines) if "# CODE-BEGIN" in line)
    end = next(i for i, line in enumerate(lines) if "# CODE-END" in line)
    return textwrap.dedent("\n".join(lines[begin + 1 : end])).splitlines()


def to_scene(xyz) -> np.ndarray:
    """Project a world point (x right, y up, the arm moves in the x-y plane) to the scene."""
    return ORIGIN_PT + SCALE * np.array([xyz[0] - ACROMION[0], xyz[1] - ACROMION[1], 0.0])


ACROMION = np.zeros(3)


def rms_mm(errors) -> float:
    return 1000.0 * float(np.sqrt(np.mean(np.asarray(errors)[SHOW_MARKERS] ** 2)))


def sentence(text: str, y: float = -1.75, color=S.C_TEXT) -> Text:
    out = Text(text, font_size=24, color=color)
    if out.width > 6.2:
        out.scale_to_fit_width(6.2)
    return out.move_to([-3.6, y, 0])


class AnimIkByHand(Scene):
    def construct(self):
        global ACROMION
        data = S.load_npz(f"{TOPIC}_main.npz")
        demo_pose, demo_err = data["demo_pose"], data["demo_err"]
        n_it = len(data["demo_q"]) - 1
        ACROMION = demo_pose[0, 0]
        measured = data["demo_measured"]
        errs = [rms_mm(e) for e in demo_err]

        self.play(Write(S.title_block("Inverse kinematics by hand", "Find q from markers with Gauss-Newton")))
        tag_a = S.synthetic_tag("synthetic")
        tag_b = S.synthetic_tag("handmade").next_to(tag_a, RIGHT, buff=0.6)
        panel = S.code_panel("biorbd code", generator_code(), font_size=18)
        self.play(FadeIn(panel), FadeIn(tag_a), FadeIn(tag_b))

        # --- point 1: the loop on one frame ---------------------------------------------------
        def arm(pose, opacity=1.0, width=6):
            pts = [to_scene(pose[i]) for i in (0, 1, 3)]
            line = VMobject(stroke_color=S.C_MODEL, stroke_width=width, stroke_opacity=opacity)
            line.set_points_as_corners(pts)
            dots = VGroup(
                *[Dot(to_scene(pose[i]), radius=0.07, color=S.C_MODEL, fill_opacity=opacity) for i in SHOW_MARKERS]
            )
            return VGroup(line, dots)

        target = VGroup(*[Dot(to_scene(measured[i]), radius=0.09, color=S.C_DATA) for i in SHOW_MARKERS])
        target_label = Text("Measured markers", font_size=22, color=S.C_DATA).move_to([-3.4, 2.0, 0], aligned_edge=LEFT)
        model_label = Text("Model markers", font_size=22, color=S.C_MODEL).next_to(
            target_label, DOWN, aligned_edge=LEFT
        )

        # error axis (difference axis, rule A6)
        ax_o = np.array([-2.5, -0.85, 0.0])
        ax_w, ax_h = 1.9, 1.6
        x_axis = Line(ax_o, ax_o + RIGHT * ax_w, color=S.C_MUTED, stroke_width=2)
        y_axis = Line(ax_o, ax_o + UP * ax_h, color=S.C_MUTED, stroke_width=2)
        ax_title = (
            Text("Marker error (mm)", font_size=20, color=S.C_MUTED)
            .next_to(y_axis, UP, buff=0.1)
            .align_to(y_axis, LEFT)
        )
        ax_x = Text("Iteration", font_size=20, color=S.C_MUTED).next_to(x_axis, DOWN, buff=0.08)
        y_top = (
            Text(f"{errs[0]:.0f}", font_size=18, color=S.C_MUTED).next_to(y_axis, LEFT, buff=0.06).align_to(y_axis, UP)
        )

        def err_point(k):
            return ax_o + np.array([ax_w * (k + 0.3) / (n_it + 0.6), ax_h * errs[k] / errs[0] * 0.95, 0.0])

        err_dots = [Dot(err_point(k), radius=0.06, color=S.C_DIFF) for k in range(n_it + 1)]
        err_links = [Line(err_point(k), err_point(k + 1), color=S.C_DIFF, stroke_width=3) for k in range(n_it)]

        model = arm(demo_pose[0])
        p1 = [target, target_label, model_label, x_axis, y_axis, ax_title, ax_x, y_top, *err_dots, *err_links]
        sent = sentence(f"At iteration 0 the marker error is {errs[0]:.1f} mm.")
        self.play(
            FadeIn(target),
            FadeIn(target_label),
            FadeIn(model_label),
            FadeIn(model),
            FadeIn(VGroup(x_axis, y_axis, ax_title, ax_x, y_top)),
            FadeIn(err_dots[0]),
            FadeIn(sent),
            run_time=1.0,
        )
        ghosts = []
        for k in (1, 2):
            ghost = arm(demo_pose[k - 1], opacity=0.3, width=4)
            self.add(ghost)
            ghosts.append(ghost)
            new_model = arm(demo_pose[k])
            moves = [ReplacementTransform(model, new_model), Create(err_links[k - 1]), FadeIn(err_dots[k])]
            if k == 1:
                new_sent = sentence(f"After iteration {k} it falls to {errs[k]:.1f} mm.")
                moves.append(ReplacementTransform(sent, new_sent))
                sent = new_sent
            self.play(*moves, run_time=0.9)
            model = new_model
        rest = VGroup(*[m for k in range(3, n_it + 1) for m in (err_links[k - 1], err_dots[k])])
        end_model = arm(demo_pose[n_it])
        end_sent = sentence(f"It stops decreasing after {n_it} iterations, at {errs[n_it]:.1f} mm.")
        self.play(
            ReplacementTransform(model, end_model), FadeIn(rest), ReplacementTransform(sent, end_sent), run_time=0.9
        )

        # --- point 2: every frame, compared with the known truth ------------------------------
        self.play(*[FadeOut(m) for m in [*p1, *ghosts, end_model, end_sent]])
        t, q_true, q_est, q_one = (
            data["t"],
            np.degrees(data["q_true"]),
            np.degrees(data["q_est"]),
            np.degrees(data["q_one"]),
        )
        diff = np.sqrt(np.mean((q_est - q_true) ** 2, axis=1))
        o1, w, h1 = np.array([-6.0, 0.2, 0.0]), 4.6, 1.5
        o2, h2 = np.array([-6.0, -1.2, 0.0]), 0.7
        top = [Line(o1, o1 + RIGHT * w, color=S.C_MUTED), Line(o1, o1 + UP * h1, color=S.C_MUTED)]
        bot = [Line(o2, o2 + RIGHT * w, color=S.C_MUTED), Line(o2, o2 + UP * h2, color=S.C_MUTED)]
        top_title = Text("Joint angles", font_size=20, color=S.C_MUTED).next_to(top[1], UP, buff=0.08)
        top_title.align_to(top[1], LEFT)
        bot_title = Text("Difference with the truth", font_size=20, color=S.C_DIFF).next_to(bot[1], UP, buff=0.12)
        bot_title.align_to(bot[1], LEFT).shift(RIGHT * 2.4)
        d_max = float(diff.max())
        bot_top = (
            Text(f"{d_max:.1f}", font_size=18, color=S.C_MUTED).next_to(bot[1], LEFT, buff=0.06).align_to(bot[1], UP)
        )
        top_top = Text("90°", font_size=18, color=S.C_MUTED).next_to(top[1], LEFT, buff=0.06).align_to(top[1], UP)
        time_lab = Text("Time (s)", font_size=20, color=S.C_MUTED).next_to(bot[0], DOWN, buff=0.06)

        def curve(values, origin, height, vmax, color, opacity=1.0, width=4):
            pts = [
                origin + np.array([w * (tt - t[0]) / (t[-1] - t[0]), height * v / vmax, 0.0])
                for tt, v in zip(t, values)
            ]
            out = VMobject(stroke_color=color, stroke_width=width, stroke_opacity=opacity)
            out.set_points_as_corners(pts)
            return out

        truth = [curve(q_true[:, j], o1, h1, 90.0, S.C_DATA) for j in range(2)]
        one = [curve(q_one[:, j], o1, h1, 90.0, S.C_MODEL) for j in range(2)]
        final = [curve(q_est[:, j], o1, h1, 90.0, S.C_MODEL) for j in range(2)]
        dcurve = curve(diff, o2, h2, d_max, S.C_DIFF)
        key_t = Text("True q", font_size=18, color=S.C_DATA).move_to([-2.9, 1.92, 0], aligned_edge=LEFT)
        key_e = Text("Estimated q", font_size=18, color=S.C_MODEL).next_to(key_t, RIGHT, buff=0.3)
        frame2 = [*top, *bot, top_title, bot_title, bot_top, top_top, time_lab]
        self.play(*[FadeIn(m) for m in frame2], Create(truth[0]), Create(truth[1]), FadeIn(key_t))
        self.play(*[Create(c) for c in one], FadeIn(key_e), run_time=1.0)
        s2 = sentence("Iterating on every frame recovers the true motion.", y=-1.9)
        self.play(
            *[c.animate.set_stroke(opacity=0.3) for c in one],
            *[Create(c) for c in final],
            Create(dcurve),
            FadeIn(s2),
            run_time=1.2,
        )
        rmse = float(data["rmse_q_deg"])
        s3 = sentence(f"The reconstruction error is {rmse:.2f} degrees (RMSE).", y=-1.9, color=S.C_DIFF)
        self.play(ReplacementTransform(s2, s3))
        self.play(
            FadeIn(S.footer("Gauss-Newton turns marker positions into joint angles by repeating one linear step."))
        )
