"""Layout self-test of the render layer. It shows no biorbd result: the numbers are formatting checks only."""

from manim import *

from common import bio_style as S


class SelfTestLayer(Scene):
    def construct(self):
        head = S.title_block("Layer self-test", "Layout check, not a result")
        self.play(Write(head))
        first = Text("First sentence, soon replaced.", font_size=26, color=S.C_TEXT).move_to(LEFT * 3.6 + UP * 0.5)
        self.play(Write(first))
        second = Text("Second sentence in its place.", font_size=26, color=S.C_TEXT).move_to(first)
        self.play(ReplacementTransform(first, second))
        value = Text("Format check: 3.14, 12, -0.5", font_size=24, color=S.C_DATA).next_to(second, DOWN, buff=0.5)
        self.play(FadeIn(value))
        panel = S.code_panel("biorbd code", ["import biorbd", "model = biorbd.Biorbd(path)", "print(model.nb_q)"])
        self.play(FadeIn(panel))
        self.play(FadeIn(S.synthetic_tag("synthetic")))
        note = S.footer("The logo stays in a free corner and the text never overlaps.")
        self.play(FadeIn(note))
