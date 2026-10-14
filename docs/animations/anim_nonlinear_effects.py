"""Video 08: non_linear_effect(q, qdot) = gravity at zero velocity + a part that grows with the velocity squared."""

import re

from manim import *

from common import bio_style as S

TOPIC = "nonlinear_effects"


def code_lines() -> list[str]:
    """The lines the generator runs (rule D5): read between its CODE-BEGIN and CODE-END markers."""
    source = (S.ROOT / f"generate_{TOPIC}_data.py").read_text(encoding="utf-8")
    body = re.search(r"# CODE-BEGIN\n(.*?)\n# CODE-END", source, re.DOTALL).group(1)
    return body.splitlines()


class AnimNonlinearEffects(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        shown = d["shown"]
        names = [str(n) for n in d["shown_names"]]

        # A1
        head = S.title_block("Nonlinear effects", "Gravity and velocity terms in one vector.")
        self.play(Write(head))

        # Stage 1 (point 1): at zero velocity the vector is pure gravity
        intro = Text("Posture and velocities chosen by hand.", font_size=24, color=S.C_TEXT)
        intro.move_to([S.LEFT_X0 + intro.width / 2, 2.0, 0])
        synth = S.synthetic_tag("synthetic")

        fmt = "{:.2f}"
        x_cols = [-3.6, -2.55, -1.05, 0.35]  # right edges of the four number columns
        y0, dy = 0.85, 0.5

        def cell(text, col, y, color, code=False, size=22):
            kw = {"font": S.CODE_FONT} if code else {}
            t = Text(text, font_size=size, color=color, **kw)
            return t.move_to([x_cols[col] - t.width / 2, y, 0])

        labels = VGroup(
            *[
                Text(n, font_size=20, color=S.C_TITLE, font=S.CODE_FONT).move_to([S.LEFT_X0 + 1.0, y0 - i * dy, 0])
                for i, n in enumerate(names)
            ]
        )
        for lab in labels:
            lab.align_to([S.LEFT_X0, 0, 0], LEFT)
        col_g = VGroup(*[cell(fmt.format(d["g"][k]), 0, y0 - i * dy, S.C_MODEL) for i, k in enumerate(shown)])
        head_g = cell("gravity", 0, y0 + 0.5, S.C_MUTED, size=22)

        panel = S.code_panel("biorbd code", code_lines(), font_size=16)
        weight_a = Text(
            f"Vertical force {fmt.format(d['g_transz'])} N = {fmt.format(d['mass'])} kg × {fmt.format(d['g_norm'])} m/s².",
            font_size=22,
            color=S.C_TEXT,
        )
        weight_b = Text("It is positive: it pushes the pelvis up, against gravity.", font_size=22, color=S.C_TEXT)
        weight = VGroup(weight_a, weight_b).arrange(DOWN, aligned_edge=LEFT, buff=0.12)
        weight.move_to([S.LEFT_X0 + weight.width / 2, -1.65, 0])

        self.play(FadeIn(VGroup(intro, synth)))
        self.play(FadeIn(panel))
        self.play(FadeIn(VGroup(labels, head_g, col_g)))
        self.play(FadeIn(weight))

        # Stage 2 (point 2): the velocity part, for qdot and 2 qdot (A6: the gravity column stays as a ghost)
        note = Text("biorbd has no separate Coriolis function.", font_size=24, color=S.C_TEXT)
        note.move_to([S.LEFT_X0 + note.width / 2, 2.0, 0])
        col_g.set_opacity(0.3)
        head_g.set_opacity(0.3)
        head_1 = cell("qdot", 1, y0 + 0.5, S.C_MUTED, code=True, size=18)
        head_2 = cell("2 * qdot", 2, y0 + 0.5, S.C_MUTED, code=True, size=18)
        head_r = cell("ratio", 3, y0 + 0.5, S.C_MUTED)
        col_1 = VGroup(*[cell(fmt.format(d["v1"][k]), 1, y0 - i * dy, S.C_DIFF) for i, k in enumerate(shown)])
        col_2 = VGroup(*[cell(fmt.format(d["v2"][k]), 2, y0 - i * dy, S.C_DIFF) for i, k in enumerate(shown)])
        col_r = VGroup(*[cell("{:.4f}".format(d["ratio"][i]), 3, y0 - i * dy, S.C_OK) for i in range(len(shown))])
        cap_a = Text("Velocity part: N(q, qdot) − N(q, 0).", font_size=22, color=S.C_TEXT)
        cap_b = Text(
            f"The ratio is 4 on all {int(d['n_active'])} degrees of freedom, within {d['ratio_error']:.1e}.",
            font_size=22,
            color=S.C_TEXT,
        )
        cap = VGroup(cap_a, cap_b).arrange(DOWN, aligned_edge=LEFT, buff=0.12)
        cap.move_to([S.LEFT_X0 + cap.width / 2, -1.65, 0])

        self.play(FadeOut(VGroup(intro, weight)), FadeIn(note))
        self.play(FadeIn(VGroup(head_1, col_1, cap_a)))
        self.play(FadeIn(VGroup(head_2, col_2)))
        self.play(FadeIn(VGroup(head_r, col_r, cap_b)))

        # A4
        self.play(FadeIn(S.footer("Doubling the velocity multiplies the velocity part by four.")))
