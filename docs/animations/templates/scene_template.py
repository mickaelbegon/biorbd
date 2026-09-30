"""Template of a biorbd scene. Copy to ``anim_<topic>.py`` and ``generate_<topic>_data.py``.

Rules: see STANDARD.md. Numbers shown on screen come from ``data/<topic>_*.npz`` (rule D2),
the code shown in the panel is the code that ``generate_<topic>_data.py`` runs (rule D5).
Do not add a final pause or end card: the render layer does it.
"""

from manim import *

from common import bio_style as S

TOPIC = "topic"  # replace: data/<TOPIC>_*.npz


class AnimTopic(Scene):  # replace: Anim<Topic>, one scene class per file
    def construct(self):
        data = S.load_npz(f"{TOPIC}_main.npz")

        # A1: title and subtitle
        head = S.title_block("Title of the video", "One line that says what you will learn")
        self.play(Write(head))

        # A2: visuals on the left (x between -6.8 and -0.5); D2: numbers computed from the npz
        value = float(data["value"])
        label = Text(f"The result is {value:.2f} metres.", font_size=28, color=S.C_MODEL).move_to(LEFT * 3.6)
        self.play(FadeIn(label))

        # A3: code panel on the right, caption above the code (D5: same lines as the generator)
        panel = S.code_panel("biorbd code", ["import biorbd", "model = biorbd.Biorbd(path)"])
        self.play(FadeIn(panel))

        # D3: label anything that is not a biorbd output
        # self.play(FadeIn(S.synthetic_tag("synthetic")))

        # A4: one closing sentence
        self.play(FadeIn(S.footer("A whole sentence that sums up the idea.")))
