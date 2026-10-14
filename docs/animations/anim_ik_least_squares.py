"""Video 14: biorbd.InverseKinematics (scipy least squares) on synthetic markers, with a missing (NaN) marker.

All numbers come from data/ik_least_squares_main.npz (made by generate_ik_least_squares_data.py, biorbd 1.12.3).
The code shown is read from the generator source between marker comments, so it is the code that was executed (D5).
"""

import textwrap

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "ik_least_squares"
GENERATOR = S.ROOT / "generate_ik_least_squares_data.py"
ORIGIN_PT = np.array([-6.0, 1.75, 0.0])  # screen position of world (y = 0, z = Z_TOP)
Z_TOP = 0.52
SCALE = 2.3  # screen units per metre
BONES = [(0, 1), (1, 2), (1, 3), (3, 4), (1, 5), (5, 6), (0, 7), (7, 8), (8, 9), (0, 10), (10, 11), (11, 12)]
TEXT_X = -4.3  # left edge of the text column
METHODS = ["lm", "trf", "only_lm"]


def generator_code(begin_tag: str, end_tag: str) -> list[str]:
    """The lines between two marker comments of the generator (they are the lines that ran, rule D5)."""
    lines = GENERATOR.read_text(encoding="utf-8").splitlines()
    begin = next(i for i, line in enumerate(lines) if begin_tag in line)
    end = next(i for i, line in enumerate(lines) if end_tag in line)
    return textwrap.dedent("\n".join(lines[begin + 1 : end])).splitlines()


def keep_right_margin(panel, limit: float = 6.7):
    """Shift a wide code panel left so that its right edge keeps a margin to the frame edge (frame half-width 7.11)."""
    excess = panel.get_right()[0] - limit
    if excess > 0:
        panel.shift(LEFT * excess)
    return panel


def to_screen(yz) -> np.ndarray:
    """World (y, z) in metres -> screen (the motion is drawn in the sagittal plane: y to the right, z up)."""
    return ORIGIN_PT + SCALE * np.array([yz[0], yz[1] - Z_TOP, 0.0])


def line_of(text, y, color=S.C_TEXT, size=22, x=TEXT_X, max_width=5.3) -> Text:
    out = Text(text, font_size=size, color=color)
    if out.width > max_width:
        out.scale_to_fit_width(max_width)
    return out.move_to([x, y, 0], aligned_edge=LEFT)


class AnimIkLeastSquares(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        n_frames = len(d["t"])
        gap = d["gap_frames"]
        gap_a, gap_b = int(gap[0]), int(gap[-1])
        missing = int(d["missing"])
        miss_name = str(d["marker_names"][missing])
        n_marker = int(d["n_marker"])
        noise = float(d["noise_std_mm"])

        def frame_group(k, skel, markers, est_missing=None):
            """Estimated stick figure (teal), measured markers (orange), ring of the missing marker."""
            bones = VGroup(
                *[
                    Line(to_screen(skel[k][a][1:]), to_screen(skel[k][b][1:]), color=S.C_MODEL, stroke_width=5)
                    for a, b in BONES
                ]
            )
            dots = VGroup(
                *[
                    Dot(to_screen(markers[1:, i, k]), radius=0.03, color=S.C_DATA)
                    for i in range(n_marker)
                    if np.isfinite(markers[0, i, k])
                ]
            )
            group = VGroup(bones, dots)
            if est_missing is not None and not np.isfinite(markers[0, missing, k]):
                group.add(Circle(radius=0.11, color=S.C_DIFF, stroke_width=4).move_to(to_screen(est_missing[1:, k])))
            return group

        head = S.title_block("Inverse kinematics in biorbd", "Least squares with the InverseKinematics class")
        tag = S.synthetic_tag("synthetic")
        code_a = generator_code("# CODE-BEGIN", "# CODE-END")
        code_b = generator_code("# CODE2-BEGIN", "# CODE2-END")
        panel_a = keep_right_margin(S.code_panel("biorbd code", code_a, font_size=15))
        self.play(FadeIn(head), FadeIn(panel_a), FadeIn(tag))

        # --- point 1: one call, three methods --------------------------------------------------
        tracker = ValueTracker(0)
        key_data = Text("Noisy markers", font_size=20, color=S.C_DATA).move_to([TEXT_X, 1.85, 0], aligned_edge=LEFT)
        key_est = Text("Estimated pose", font_size=20, color=S.C_MODEL).next_to(key_data, DOWN, aligned_edge=LEFT)
        full = frame_group(0, d["skel_est_full"], d["markers"])
        full.add_updater(
            lambda m: m.become(frame_group(int(round(tracker.get_value())), d["skel_est_full"], d["markers"]))
        )
        self.add(full)
        self.play(FadeIn(key_data), FadeIn(key_est), run_time=0.6)
        self.play(tracker.animate.set_value(n_frames - 1), run_time=3.5, rate_func=linear)
        full.clear_updaters()

        col = [TEXT_X, TEXT_X + 1.35, TEXT_X + 2.65]
        h1 = Text("Error (deg)", font_size=18, color=S.C_MUTED).move_to([col[1], 0.75, 0], aligned_edge=LEFT)
        h2 = Text("Over range (deg)", font_size=18, color=S.C_MUTED).move_to([col[2], 0.75, 0], aligned_edge=LEFT)
        for header, limit in ((h1, 1.15), (h2, 1.9)):
            if header.width > limit:
                header.scale_to_fit_width(limit).move_to([header.get_left()[0], 0.75, 0], aligned_edge=LEFT)
        rows = VGroup(h1, h2)
        for r, method in enumerate(METHODS):
            y = 0.3 - 0.45 * r
            name = Text(method, font=S.CODE_FONT, font_size=22, color=S.C_TEXT).move_to(
                [col[0], y, 0], aligned_edge=LEFT
            )
            err = Text(f"{float(d[f'rmse_q_{method}_full']):.3f}", font_size=22, color=S.C_DIFF)
            out = Text(f"{float(d[f'max_violation_deg_{method}_full']):.3f}", font_size=22, color=S.C_DIFF)
            rows.add(
                name, err.move_to([col[1], y, 0], aligned_edge=LEFT), out.move_to([col[2], y, 0], aligned_edge=LEFT)
            )
        s1 = line_of(f"Noise added: {noise:.0f} mm per coordinate.", -1.2, size=20)
        self.play(FadeIn(rows), FadeIn(s1), run_time=1.2)
        s1b = line_of("Only trf keeps every joint inside its range.", -1.6, size=20, color=S.C_TEXT)
        self.play(FadeIn(s1b), run_time=0.8)

        # --- point 2: one marker becomes NaN ---------------------------------------------------
        panel_b = keep_right_margin(S.code_panel("biorbd code", [*code_a, *code_b], font_size=15))
        self.play(
            FadeOut(rows),
            FadeOut(s1),
            FadeOut(s1b),
            FadeOut(key_data),
            FadeOut(key_est),
            FadeOut(full),
            FadeOut(panel_a),
        )
        self.play(FadeIn(panel_b), run_time=0.6)
        tracker.set_value(0)
        nan_grp = frame_group(0, d["skel_est_nan"], d["markers_nan"], d["est_missing_nan"])
        nan_grp.add_updater(
            lambda m: m.become(
                frame_group(int(round(tracker.get_value())), d["skel_est_nan"], d["markers_nan"], d["est_missing_nan"])
            )
        )
        self.add(nan_grp)
        name_lab = Text(miss_name, font=S.CODE_FONT, font_size=20, color=S.C_DIFF).move_to(
            [TEXT_X, 1.9, 0], aligned_edge=LEFT
        )
        sent = Text(f"is NaN from frame {gap_a} to {gap_b}.", font_size=20, color=S.C_TEXT).next_to(
            name_lab, RIGHT, buff=0.12
        )
        ring_key = Text("Red ring: where the model puts it", font_size=20, color=S.C_DIFF).next_to(
            name_lab, DOWN, aligned_edge=LEFT
        )
        self.play(FadeIn(name_lab), FadeIn(sent), FadeIn(ring_key), run_time=0.8)

        # error axis of the missing marker (difference axis, rule A6): ghost = all markers measured
        o = np.array([TEXT_X + 0.2, -0.75, 0.0])
        w, h = 3.4, 1.3
        e_full, e_nan = d["missing_err_lm_full"], d["missing_err_lm_nan"]
        e_max = float(max(e_full.max(), e_nan.max()))

        def pt(k, value):
            return o + np.array([w * k / (n_frames - 1), h * value / e_max, 0.0])

        x_axis, y_axis = Line(o, o + RIGHT * w, color=S.C_MUTED), Line(o, o + UP * h, color=S.C_MUTED)
        shade = Rectangle(
            width=w * (gap_b - gap_a) / (n_frames - 1),
            height=h,
            stroke_width=0,
            fill_color=S.C_MUTED,
            fill_opacity=0.18,
        ).move_to(pt(gap_a, 0), aligned_edge=DOWN + LEFT)
        ax_t = Text("Error of this marker (mm)", font_size=18, color=S.C_MUTED).next_to(y_axis, UP, buff=0.08)
        ax_t.align_to(y_axis, LEFT)
        ax_x = Text("Frame", font_size=18, color=S.C_MUTED).next_to(x_axis, DOWN, buff=0.06)
        y_top = (
            Text(f"{e_max:.1f}", font_size=16, color=S.C_MUTED).next_to(y_axis, LEFT, buff=0.05).align_to(y_axis, UP)
        )

        def curve(values, color, opacity=1.0):
            out = VMobject(stroke_color=color, stroke_width=3, stroke_opacity=opacity)
            out.set_points_as_corners([pt(k, v) for k, v in enumerate(values)])
            return out

        ghost, red = curve(e_full, S.C_DIFF, 0.3), curve(e_nan, S.C_DIFF)
        self.add(shade)
        self.play(FadeIn(VGroup(x_axis, y_axis, ax_t, ax_x, y_top)), FadeIn(ghost), run_time=0.8)
        self.play(tracker.animate.set_value(n_frames - 1), Create(red), run_time=3.5, rate_func=linear)
        nan_grp.clear_updaters()
        mean_full = float(e_full[gap].mean())
        mean_nan = float(e_nan[gap].mean())
        rm_full, rm_nan = float(d["rmse_marker_lm_full"]), float(d["rmse_marker_lm_nan"])
        s2 = line_of(f"Its error rises from {mean_full:.2f} to {mean_nan:.2f} mm.", -1.5, size=20, color=S.C_DIFF)
        s3 = line_of(f"On all markers it rises from {rm_full:.2f} to {rm_nan:.2f} mm.", -1.85, size=20, color=S.C_TEXT)
        self.play(FadeIn(s2), FadeIn(s3), run_time=1.0)
        self.play(FadeIn(S.footer("The solver skips NaN markers, and trf also respects the joint ranges.")))
