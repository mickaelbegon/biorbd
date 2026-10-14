"""Video 17: static optimization, choosing muscle activations among many that give the same joint torque."""

import textwrap

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "static_optimization"
GENERATOR = S.ROOT / "generate_static_optimization_data.py"


def block(begin_tag: str, end_tag: str) -> list[str]:
    """Lines between two marker comments of the generator (displayed code == executed code, rule D5)."""
    lines = GENERATOR.read_text(encoding="utf-8").splitlines()
    begin = next(i for i, line in enumerate(lines) if line.strip() == begin_tag)
    end = next(i for i, line in enumerate(lines) if line.strip() == end_tag)
    return textwrap.dedent("\n".join(lines[begin + 1 : end])).splitlines()


def code_lines() -> list[str]:
    lines = block("# CODE-BEGIN", "# CODE-END") + block("# CODE-BEGIN-CHECK", "# CODE-END-CHECK")
    # Pango drops leading spaces: keep the indentation with non-breaking spaces
    return [line.replace(" ", " ", len(line) - len(line.lstrip(" "))) for line in lines]


def sentence(text: str, y: float, color=S.C_TEXT, size: int = 22) -> Text:
    out = Text(text, font_size=size, color=color)
    if out.width > 6.5:  # the visuals area ends where the code panel starts
        out.scale_to_fit_width(6.5)
    return out.move_to([-6.8, y, 0], aligned_edge=LEFT)


class AnimStaticOptimization(Scene):
    def construct(self):
        data = S.load_npz(f"{TOPIC}_main.npz")
        names = [str(n) for n in data["muscle_names"]]
        sets, set_tau, set_cost = data["set_activations"], data["set_tau"], data["set_cost"]
        demo_tau = data["demo_tau"]
        nb_mus, nb_dof = int(data["nb_muscles"]), int(data["nb_dof"])

        self.play(
            Write(S.title_block("Static optimization", "Static optimization chooses the muscle activations.")),
            run_time=0.8,
        )
        tag_a = S.synthetic_tag("synthetic")
        tag_b = S.synthetic_tag("handmade").next_to(tag_a, RIGHT, buff=0.6, aligned_edge=DOWN)
        panel = S.code_panel("biorbd code", code_lines(), font_size=14).shift(LEFT * 0.5)
        self.play(FadeIn(panel), FadeIn(tag_a), FadeIn(tag_b), run_time=0.6)

        # --- point 1: many activation sets, one joint torque ------------------------------------
        base_y, scale = -0.45, 1.7
        x0, dx, bw = -6.2, 0.95, 0.5
        axis = Line([x0 - 0.3, base_y, 0], [x0 + dx * (nb_mus - 1) + 0.6, base_y, 0], color=S.C_MUTED, stroke_width=2)
        y_axis = Line([x0 - 0.3, base_y, 0], [x0 - 0.3, base_y + scale, 0], color=S.C_MUTED, stroke_width=2)
        top_lab = Text("1", font_size=18, color=S.C_MUTED).next_to(y_axis, LEFT, buff=0.06).align_to(y_axis, UP)
        y_title = Text("Muscle activation", font_size=20, color=S.C_MUTED).next_to(y_axis, UP, buff=0.1)
        y_title.align_to(y_axis, LEFT)
        labels = VGroup(
            *[
                Text(n, font=S.CODE_FONT, font_size=14, color=S.C_MUTED).move_to([x0 + dx * i, base_y - 0.22, 0])
                for i, n in enumerate(names)
            ]
        )

        def bars(values, color, opacity=1.0):
            return VGroup(
                *[
                    Rectangle(
                        width=bw,
                        height=max(v * scale, 0.02),
                        fill_color=color,
                        fill_opacity=0.85 * opacity,
                        stroke_width=0,
                    ).move_to([x0 + dx * i, base_y + max(v * scale, 0.02) / 2, 0])
                    for i, v in enumerate(values)
                ]
            )

        def torque_text(k, color):
            tau = set_tau[k]
            return Text(f"Muscle torque: {tau[0]:.2f} and {tau[1]:.2f} N·m", font_size=24, color=color).move_to(
                [-6.8, -1.1, 0], aligned_edge=LEFT
            )

        def cost_text(k, color=S.C_TEXT):
            return Text(f"Sum of squared activations: {set_cost[k]:.2f}", font_size=24, color=color).move_to(
                [-6.8, -1.55, 0], aligned_edge=LEFT
            )

        target = Text(f"Target joint torque: {demo_tau[0]:.2f} and {demo_tau[1]:.2f} N·m", font_size=24, color=S.C_DATA)
        if target.width > 5.6:
            target.scale_to_fit_width(5.6)
        target.move_to([-6.8, 2.0, 0], aligned_edge=LEFT)

        cur = bars(sets[0], S.C_MODEL)
        t_txt, c_txt = torque_text(0, S.C_MODEL), cost_text(0)
        s1 = sentence(f"{nb_mus} muscles act on {nb_dof} joints, so many sets give this torque.", -2.0)
        self.play(
            FadeIn(VGroup(axis, y_axis, top_lab, y_title, labels)),
            FadeIn(target),
            FadeIn(cur),
            FadeIn(t_txt),
            FadeIn(c_txt),
            FadeIn(s1),
            run_time=1.0,
        )
        ghosts = []
        for k in (1, 2):
            ghost = bars(sets[k - 1], S.C_MUTED, opacity=0.3)
            self.add(ghost)
            ghosts.append(ghost)
            last = k == 2
            new_bars = bars(sets[k], S.C_OK if last else S.C_MODEL)
            new_t = torque_text(k, S.C_MODEL)
            new_c = cost_text(k, S.C_OK if last else S.C_TEXT)
            moves = [ReplacementTransform(cur, new_bars), ReplacementTransform(t_txt, new_t)]
            moves.append(ReplacementTransform(c_txt, new_c))
            if last:
                new_s = sentence("Static optimization keeps the set with the smallest sum of squares.", -2.0)
                moves.append(ReplacementTransform(s1, new_s))
                s1 = new_s
            self.play(*moves, run_time=0.7)
            cur, t_txt, c_txt = new_bars, new_t, new_c

        # --- point 2: the whole motion ----------------------------------------------------------
        self.play(
            *[FadeOut(m) for m in [axis, y_axis, top_lab, y_title, labels, target, cur, t_txt, c_txt, s1, *ghosts]],
            run_time=0.6,
        )
        t = data["t"]
        acts, tau, tau_m, diff = data["activations"], data["tau"], data["tau_muscles"], data["diff"]
        w = 4.6
        ox = -6.0

        def curve(values, origin, height, vmin, vmax, color, width=3.5):
            pts = [
                origin + np.array([w * (tt - t[0]) / (t[-1] - t[0]), height * (v - vmin) / (vmax - vmin), 0.0])
                for tt, v in zip(t, values)
            ]
            out = VMobject(stroke_color=color, stroke_width=width)
            out.set_points_as_corners(pts)
            return out

        def axes(origin, height, title, color=S.C_MUTED):
            xa = Line(origin, origin + RIGHT * w, color=S.C_MUTED, stroke_width=2)
            ya = Line(origin, origin + UP * height, color=S.C_MUTED, stroke_width=2)
            ti = Text(title, font_size=20, color=color).next_to(ya, UP, buff=0.05).align_to(ya, LEFT)
            return xa, ya, ti

        oa, ob, oc = np.array([ox, 0.95, 0.0]), np.array([ox, -0.55, 0.0]), np.array([ox, -1.65, 0.0])
        ha, hb, hc = 0.85, 0.95, 0.4
        a_axes = axes(oa, ha, "Muscle activations")
        b_axes = axes(ob, hb, "Joint torque")
        d_max = float(np.abs(diff).max())
        c_axes = axes(oc, hc, "Muscle torque minus target", color=S.C_DIFF)
        lab_b = Text(f"{tau.max():.0f}", font_size=16, color=S.C_MUTED).next_to(b_axes[1], LEFT, buff=0.05)
        lab_b.align_to(b_axes[1], UP)
        lab_a = Text("1", font_size=16, color=S.C_MUTED).next_to(a_axes[1], LEFT, buff=0.05).align_to(a_axes[1], UP)
        lab_c = Text(f"{d_max:.4f}", font_size=16, color=S.C_MUTED).next_to(c_axes[1], LEFT, buff=0.05)
        lab_c.align_to(c_axes[1], UP)
        lab_b0 = Text(f"{tau.min():.0f}", font_size=16, color=S.C_MUTED).next_to(b_axes[1], LEFT, buff=0.05)
        lab_b0.align_to(b_axes[1], DOWN)
        t_lab = Text("Time (s)", font_size=16, color=S.C_MUTED).next_to(c_axes[0], RIGHT, buff=0.1)

        act_curves = [curve(acts[:, j], oa, ha, 0.0, 1.0, S.C_MODEL, 3) for j in range(nb_mus)]
        b_min, b_max = float(tau.min()), float(tau.max())
        ref_curves = [curve(tau[:, j], ob, hb, b_min, b_max, S.C_DATA, 5) for j in range(nb_dof)]
        mus_curves = [curve(tau_m[:, j], ob, hb, b_min, b_max, S.C_MODEL, 2) for j in range(nb_dof)]
        diff_curves = [curve(diff[:, j], oc, hc, -d_max, d_max, S.C_DIFF, 3) for j in range(nb_dof)]
        key_t = Text("Target", font_size=16, color=S.C_DATA).move_to([-1.2, 0.1, 0], aligned_edge=LEFT)
        key_m = Text("Muscles", font_size=16, color=S.C_MODEL).next_to(key_t, DOWN, buff=0.1, aligned_edge=LEFT)
        frame2 = [*a_axes, *b_axes, *c_axes, lab_a, lab_b, lab_c, lab_b0, t_lab]
        self.play(*[FadeIn(m) for m in frame2], *[Create(c) for c in act_curves], run_time=1.2)
        self.play(
            LaggedStart(
                AnimationGroup(*[Create(c) for c in ref_curves]),
                AnimationGroup(*[Create(c) for c in mus_curves]),
                lag_ratio=0.6,
            ),
            FadeIn(key_t),
            FadeIn(key_m),
            run_time=1.4,
        )
        s2 = sentence("Muscle torque matches the target at every frame.", -2.0)
        self.play(*[Create(c) for c in diff_curves], FadeIn(s2), run_time=0.7)
        s3 = sentence(f"The largest difference is {d_max:.4f} N·m.", -2.0, color=S.C_DIFF)
        self.play(ReplacementTransform(s2, s3), run_time=0.6)
        self.play(
            FadeIn(S.footer("Static optimization picks, among many, the activations that cost the least.")),
            run_time=0.6,
        )
