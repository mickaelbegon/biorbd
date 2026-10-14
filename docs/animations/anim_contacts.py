"""Video 12: rigid contacts in forward dynamics, the contact points stop accelerating and biorbd returns the forces."""

import re

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "contacts"
X0 = S.LEFT_X0


def code_lines() -> list[str]:
    """The lines the generator runs (rule D5): read between its CODE-BEGIN and CODE-END markers."""
    source = (S.ROOT / f"generate_{TOPIC}_data.py").read_text(encoding="utf-8")
    body = re.search(r"# CODE-BEGIN\n(.*?)\n\s*# CODE-END", source, re.DOTALL).group(1)
    return body.splitlines()


def vec(values) -> str:
    """One template for a vector of accelerations (D2); -0.00 is written 0.00."""
    return "[" + ", ".join(f"{float(v) + 0.0 if abs(v) >= 0.005 else 0.0:.2f}" for v in values) + "]"


def left_text(text: str, y: float, color=S.C_TEXT, size: int = 22, **kw) -> Text:
    out = Text(text, font_size=size, color=color, **kw)
    return out.move_to([X0 + out.width / 2, y, 0])


def row(label: str, values, y: float, color) -> VGroup:
    """A code-font name followed by the vector it holds."""
    name = Text(label, font=S.CODE_FONT, font_size=22, color=color)
    value = Text(vec(values), font=S.CODE_FONT, font_size=22, color=color)
    group = VGroup(name, Text("=", font_size=22, color=S.C_MUTED), value).arrange(RIGHT, buff=0.2)
    return group.move_to([X0 + group.width / 2, y, 0])


class AnimContacts(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        forces, weight = d["forces"], float(d["weight"])
        acc_free, acc = d["acc_free"], d["acc"]

        self.play(Write(S.title_block("Rigid contacts", "A contact forbids the motion of its point.")))
        panel = S.code_panel("biorbd code", code_lines(), font_size=17)
        # the long code lines need a smaller font: scale the code box only, the caption keeps its 26 pt
        cap, body = panel
        body.scale_to_fit_width(5.4)
        cap.next_to(body, UP, buff=0.18).align_to(body, LEFT)
        # top right: the wide sentences of the left column stay below the panel or clear of it
        panel.move_to([4.05, 1.1, 0])
        self.play(FadeIn(panel))

        # Point 1: forward dynamics that ignores the contacts
        head1 = left_text("Contacts ignored: the cube is in free fall.", 1.75, S.C_MUTED)
        r1 = row("free", d["qddot_free"], 1.2, S.C_DATA)
        s1 = left_text(f"Contact point 1 accelerates at {acc_free[0, 2]:.2f} m/s² along z.", 0.65, S.C_DATA)
        self.play(FadeIn(head1))
        self.play(FadeIn(r1))
        self.play(FadeIn(s1))

        # Point 2: the same call with the contacts (A6: the free result stays, faded)
        self.play(VGroup(head1, r1, s1).animate.set_opacity(0.45))
        head2 = left_text("Contacts active: they cancel that motion.", 0.0, S.C_MUTED)
        r2 = row("qdd", d["qddot"], -0.55, S.C_MODEL)
        worst = float(np.abs(acc).max())
        s2 = left_text(f"Contact point acceleration is at most {worst:.0e} m/s².", -1.1, S.C_MODEL)
        f_txt = left_text(f"Contact forces along z: {forces[1]:.1f} N and {forces[2]:.1f} N.", -1.65, S.C_MODEL)
        t_txt = left_text(f"They sum to {float(d['sum_z']):.2f} N, the weight of the cube.", -2.2, S.C_OK)
        self.play(FadeIn(head2))
        self.play(FadeIn(r2))
        self.play(FadeIn(s2))
        self.play(FadeIn(f_txt))
        self.play(FadeIn(t_txt))

        self.play(FadeIn(S.footer("The contact forces hold the cube where the contacts forbid motion.")))
