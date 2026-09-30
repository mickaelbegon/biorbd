"""Video 11: a muscle's length depends on the joint angle; the joint torque is -(dLength/dq) * force."""

import re

from manim import *

from common import bio_style as S

TOPIC = "muscles"
X_LEN = 5.6


def code_lines() -> list[str]:
    """The lines the generator runs (rule D5): read between its CODE-BEGIN and CODE-END markers."""
    source = (S.ROOT / f"generate_{TOPIC}_data.py").read_text(encoding="utf-8")
    body = re.search(r"# CODE-BEGIN\n(.*?)\n\s*# CODE-END", source, re.DOTALL).group(1)
    return [line[4:] for line in body.splitlines()]  # the loop body is indented by 4 spaces in the generator


class AnimMuscles(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        elbow = d["elbow"]
        k = int(d["shown"])
        name = str(d["muscle"])
        cfg = {"include_numbers": False, "color": S.C_MUTED, "include_ticks": False}
        xr = [float(elbow[0]), float(elbow[-1]), 0.5]

        def make_axes(values, length, top, pad=0.1):
            lo, hi = float(values.min()), float(values.max())
            margin = pad * (hi - lo)
            return Axes(
                x_range=xr,
                y_range=[lo - margin, hi + margin, hi - lo],
                x_length=X_LEN,
                y_length=length,
                tips=False,
                axis_config=cfg,
            ).move_to([-3.7, top, 0])

        def curve(axes, y, color, width=4, opacity=1.0):
            line = axes.plot_line_graph(elbow, y, add_vertex_dots=False, line_color=color, stroke_width=width)
            return line["line_graph"].set_stroke(opacity=opacity)

        def caption(text, axes, color, code=False):
            kw = {"font": S.CODE_FONT} if code else {}
            out = Text(text, font_size=24, color=color, **kw)
            if out.width > 7.2:  # keep clear of the code panel caption, even with a longer French string
                out.scale_to_fit_width(7.2)
            return out.next_to(axes, UP, buff=0.12).align_to(axes, LEFT)

        # A1
        head = S.title_block("Muscles", "A muscle turns its force into a joint torque.")
        self.play(Write(head))
        panel = S.code_panel("biorbd code", code_lines(), font_size=20)
        self.play(FadeIn(panel))

        # Stage 1 (point 1): length and moment arm of one muscle over an elbow sweep
        length, arm = d["length"], d["moment_arm"]
        ax_l = make_axes(length, 1.45, 0.85)
        ax_a = make_axes(arm, 1.45, -1.15)
        cap_l = caption(f"Length of {name}: {length.min():.2f} to {length.max():.2f} m.", ax_l, S.C_MODEL)
        cap_a = caption(f"Moment arm, up to {arm.max() * 100:.1f} cm.", ax_a, S.C_MODEL)
        sweep = Text(
            f"Elbow angle from {elbow[0]:.2f} to {elbow[-1]:.2f} rad, shoulder fixed at {float(d['shoulder']):.2f} rad.",
            font_size=22,
            color=S.C_MUTED,
        )
        sweep.move_to([S.LEFT_X0 + sweep.width / 2, -2.65, 0])
        self.play(FadeIn(VGroup(ax_l, ax_a, cap_l, cap_a, sweep)))
        c_len = curve(ax_l, length, S.C_MODEL)
        self.play(Create(c_len), run_time=1.5)
        c_arm = curve(ax_a, arm, S.C_MODEL)
        self.play(Create(c_arm), run_time=1.5)
        dots = VGroup(
            Dot(ax_l.c2p(elbow[k], length[k]), color=S.C_TITLE, radius=0.07),
            Dot(ax_a.c2p(elbow[k], arm[k]), color=S.C_TITLE, radius=0.07),
        )
        read = Text(
            f"At {elbow[k]:.2f} rad, dLength/dq = {float(d['dl_dq'][k]):.3f} m/rad and the moment arm is its opposite.",
            font_size=22,
            color=S.C_TITLE,
        )
        read.move_to([S.LEFT_X0 + read.width / 2, -3.05, 0])
        self.play(FadeIn(dots), FadeIn(read))

        # Stage 2 (point 2): the joint torque computed by biorbd against -J^T F computed by hand (A6)
        self.play(FadeOut(VGroup(ax_l, ax_a, cap_l, cap_a, sweep, c_len, c_arm, dots, read)))
        tau, tau_h, diff = d["tau_elbow"], d["tau_hand_elbow"], d["diff_elbow"]
        ax_t = make_axes(tau, 2.0, 0.75)
        ax_d = Axes(x_range=xr, y_range=[-1, 1, 2], x_length=X_LEN, y_length=0.9, tips=False, axis_config=cfg).move_to(
            [-3.7, -1.65, 0]
        )
        cap_t = caption(f"Elbow torque, six muscles at activation {float(d['activation']):.1f}.", ax_t, S.C_TEXT)
        cap_d = Text(f"Difference, at most {float(d['max_diff']):.1e} N·m.", font_size=22, color=S.C_DIFF)
        cap_d.next_to(ax_d, UP, buff=0.1).align_to(ax_d, LEFT)
        key_m = Text("joint_torque", font_size=22, color=S.C_MODEL, font=S.CODE_FONT)
        key_h = Text("-J.T @ forces", font_size=22, color=S.C_DATA, font=S.CODE_FONT)
        keys = VGroup(key_m, key_h).arrange(RIGHT, buff=0.5).move_to([-3.7, -2.75, 0])
        hand = curve(ax_t, tau_h, S.C_DATA, width=7, opacity=0.5)
        mine = curve(ax_t, tau, S.C_MODEL, width=3)
        scale = max(float(np.abs(diff).max()), 1e-300)
        zero = ax_d.plot_line_graph(
            elbow, diff / scale * 0.8, add_vertex_dots=False, line_color=S.C_DIFF, stroke_width=3
        )
        self.play(FadeIn(VGroup(ax_t, ax_d, cap_t, cap_d, keys)))
        self.play(Create(hand), run_time=1.5)
        self.play(Create(mine), run_time=1.5)
        self.play(Create(zero["line_graph"]), run_time=1.5)

        # A4
        self.play(FadeIn(S.footer("The muscle torque is minus the length Jacobian times the force.")))
