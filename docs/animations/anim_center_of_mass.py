"""Video 06: the centre of mass of the whole body is the mass-weighted average of the segment centres of mass.

All numbers come from data/center_of_mass_main.npz (made by generate_center_of_mass_data.py, biorbd 1.12.3).
The code shown is read from the generator source, so it is the code that was executed (rule D5).
"""

import re

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "center_of_mass"
ORIGIN_PT = np.array([-5.2, 0.55, 0.0])  # screen position of the world origin (pelvis)
SCALE = 2.6  # screen units per metre
COLUMN_X = -4.0
Y1, Y2, Y3 = 1.9, 0.85, -0.2  # rows of the text column
TEXT_SIZE = 20


def code_lines() -> list[str]:
    """The lines the generator runs (rule D5): read between its CODE-BEGIN and CODE-END markers."""
    source = (S.ROOT / f"generate_{TOPIC}_data.py").read_text(encoding="utf-8")
    body = re.search(r"# CODE-BEGIN\n(.*?)\n\s*# CODE-END", source, re.DOTALL).group(1)
    return [line[4:] for line in body.splitlines()]  # the loop body is indented by 4 spaces in the generator


def to_screen(point) -> np.ndarray:
    """World (x, y, z) in metres -> screen. Sagittal view: y to the right, z up."""
    return ORIGIN_PT + np.array([point[1] * SCALE, point[2] * SCALE, 0.0])


def body_group(origins, coms, masses, parents) -> VGroup:
    """Each segment: its frame origin, a link to its centre of mass, and a dot whose area follows its mass."""
    links = VGroup(*[Line(to_screen(o), to_screen(c), color=S.C_TITLE, stroke_width=4) for o, c in zip(origins, coms)])
    for index, parent in enumerate(parents):
        if parent >= 0:
            links.add(Line(to_screen(origins[parent]), to_screen(origins[index]), color=S.C_TITLE, stroke_width=4))
    dots = VGroup(*[Dot(to_screen(c), radius=0.045 * float(np.sqrt(m)), color=S.C_DATA) for c, m in zip(coms, masses)])
    return VGroup(links, dots)


def com_marker(point, color) -> VGroup:
    centre = to_screen(point)
    ring = Circle(radius=0.2, color=color, stroke_width=5).move_to(centre)
    cross = VGroup(
        Line(centre + LEFT * 0.3, centre + RIGHT * 0.3, color=color, stroke_width=3),
        Line(centre + UP * 0.3, centre + DOWN * 0.3, color=color, stroke_width=3),
    )
    return VGroup(ring, cross)


class AnimCenterOfMass(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        masses = d["masses"]
        total = float(d["total_mass"])

        def yz(vector) -> str:
            return f"y = {vector[1]:.3f} m, z = {vector[2]:.3f} m"

        def line(text, y, color=S.C_TEXT, font=None):
            kw = {"font": font} if font else {}
            return Text(text, font_size=TEXT_SIZE, color=color, **kw).align_to([COLUMN_X, 0, 0], LEFT).set_y(y)

        head = S.title_block("Centre of mass", "A mass-weighted average that follows the posture")
        self.play(Write(head))

        # point 1: average of the segment centres of mass = centre of mass of the model
        body_a = body_group(d["origins_a"], d["coms_a"], masses, d["parents"])
        text_1 = line("Each dot is a segment's centre", Y1)
        text_1b = line("of mass; larger means heavier.", Y1 - 0.35)
        self.play(FadeIn(body_a), FadeIn(VGroup(text_1, text_1b)))
        lines = code_lines()
        panel = S.code_panel("biorbd code", lines, font_size=17)
        # the average is computed by hand-written code (rule D3): label it under the code panel
        handmade = S.synthetic_tag("handmade").next_to(panel[1], DOWN, buff=0.3).align_to(panel[1], LEFT)
        self.play(FadeIn(panel), FadeIn(handmade))
        mass_head = line("Total mass:", Y2)
        mass_value = line(f"{total:.2f} kg", Y2 - 0.38, S.C_MODEL)
        self.play(FadeIn(VGroup(mass_head, mass_value)))
        mean_head = line("Average of the segments:", Y3)
        mean_value = line(yz(d["mean_a"]), Y3 - 0.38, S.C_DATA)
        model_head = line("model.center_of_mass(q):", Y3 - 0.95, S.C_MUTED, font=S.CODE_FONT)
        model_value = line(yz(d["com_model_a"]), Y3 - 1.33, S.C_MODEL)
        marker_a = com_marker(d["com_model_a"], S.C_MODEL)
        self.play(FadeIn(mean_head), FadeIn(mean_value))
        self.play(FadeIn(model_head), FadeIn(model_value), FadeIn(marker_a))

        # point 2: new posture (ghost of the first), the centre of mass moves; difference axis (A6)
        ghost = VGroup(body_a.copy(), marker_a.copy()).set_opacity(0.3)
        body_b = body_group(d["origins_b"], d["coms_b"], masses, d["parents"])
        marker_b = com_marker(d["com_model_b"], S.C_MODEL)
        text_2 = line("Same code, new posture q:", Y1)
        text_2b = line("arm raised, both legs bent.", Y1 - 0.35)
        mean_value_b = line(yz(d["mean_b"]), Y3 - 0.38, S.C_DATA)
        model_value_b = line(yz(d["com_model_b"]), Y3 - 1.33, S.C_MODEL)
        legend = VGroup(
            Text("Faded:", font_size=20, color=S.C_MUTED), Text("previous posture.", font_size=20, color=S.C_MUTED)
        ).arrange(DOWN, aligned_edge=LEFT, buff=0.08)
        legend.move_to([S.LEFT_X0 + legend.width / 2, -2.15, 0])
        self.play(
            FadeOut(body_a),
            FadeOut(marker_a),
            FadeIn(ghost),
            FadeOut(VGroup(text_1, text_1b)),
            FadeOut(VGroup(mean_value, model_value)),
        )
        self.play(FadeIn(body_b), FadeIn(marker_b), FadeIn(VGroup(text_2, text_2b)), FadeIn(legend))
        self.play(FadeIn(mean_value_b), FadeIn(model_value_b))

        scale = 1400.0  # screen units per metre on the small axis: its full length is 1 mm
        x0 = COLUMN_X + 0.4
        caption = Text("Average minus biorbd, per posture:", font_size=TEXT_SIZE, color=S.C_MUTED)
        caption.align_to([COLUMN_X, 0, 0], LEFT).set_y(-2.05)
        axis = Line([x0, -2.35, 0], [x0, -3.0, 0], color=S.C_MUTED, stroke_width=2)
        rows = VGroup(axis)
        for row, (name, tag) in enumerate((("1", "a"), ("2", "b"))):
            worst = float(np.abs(d[f"diff_{tag}"]).max())
            y = -2.55 - 0.3 * row
            bar = Rectangle(width=worst * scale + 0.02, height=0.16, color=S.C_DIFF, fill_opacity=0.9, stroke_width=0)
            bar.move_to([x0 + bar.width / 2, y, 0])
            axis_name = Text(name, font_size=TEXT_SIZE, color=S.C_DIFF).next_to(axis, LEFT, buff=0.1).set_y(y)
            amount = Text(f"{worst:.1e} m", font_size=TEXT_SIZE, color=S.C_DIFF).next_to(bar, RIGHT, buff=0.15)
            rows.add(bar, axis_name, amount)
        self.play(FadeIn(caption), FadeIn(rows))

        self.play(FadeIn(S.footer("The centre of mass is a mass-weighted average, so it moves with the posture.")))
