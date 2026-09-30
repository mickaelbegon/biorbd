"""Video 01: anatomy of a .bioMod file. Each tag of the text file becomes something in the biorbd model.

All numbers and the file excerpts come from data/biomod_anatomy_main.npz (made by generate_biomod_anatomy_data.py,
biorbd 1.12.3). The excerpts are real lines of examples/pyomecaman.bioMod. The code shown is read from the generator
source, so it is the code that was executed (rule D5).
"""

from pathlib import Path

from manim import *

from common import bio_style as S

TOPIC = "biomod_anatomy"
BOX_X0 = -6.8  # left edge of the file box
BOX_WIDTH = 6.0
ROW_SIZE = 17
RESULT_SIZE = 22
BOX_TOP = 1.75


def code_lines_of_generator() -> list[str]:
    """Lines between the '# code:start' and '# code:end' markers of the generator."""
    lines = (Path(__file__).with_name(f"generate_{TOPIC}_data.py")).read_text(encoding="utf-8").splitlines()
    block, current = [], None
    for line in lines:
        if line.startswith("# code:start"):
            current = []
        elif line.startswith("# code:end"):
            block, current = current, None
        elif current is not None:
            current.append(line)
    return block


def file_box(name: str, rows: list[str], highlight: set[int], gaps: set[int]) -> VGroup:
    """The text of the .bioMod excerpt: highlighted rows in the data colour, '...' rows where lines are skipped."""
    texts = VGroup()
    for index, row in enumerate(rows):
        color = S.C_DATA if index in highlight else S.C_CODE_TEXT
        texts.add(Text(row if row else " ", font=S.CODE_FONT, font_size=ROW_SIZE, color=color))
    # gap rows ("...") come from the skipped lines of the file
    items = []
    for index, text in enumerate(texts):
        if index in gaps:
            items.append(Text("...", font=S.CODE_FONT, font_size=ROW_SIZE, color=S.C_MUTED))
        items.append(text)
    column = VGroup(*items).arrange(DOWN, aligned_edge=LEFT, buff=0.1)
    width = max(column.width + 0.5, BOX_WIDTH)
    box = RoundedRectangle(
        corner_radius=0.12,
        width=width,
        height=column.height + 0.4,
        fill_color=S.C_CODE_BG,
        fill_opacity=1,
        stroke_color=S.C_MUTED,
        stroke_width=1.5,
    )
    column.move_to(box.get_center()).align_to(box, LEFT).shift(RIGHT * 0.25)
    caption = (
        Text(name, font=S.CODE_FONT, font_size=20, color=S.C_MUTED).next_to(box, UP, buff=0.12).align_to(box, LEFT)
    )
    group = VGroup(caption, box, column)
    group.move_to([BOX_X0 + width / 2, 0, 0]).set_y(BOX_TOP - group.height / 2)
    return group


class AnimBiomodAnatomy(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        rows_a, rows_b = [str(r) for r in d["rows_a"]], [str(r) for r in d["rows_b"]]
        dofs = [str(n) for n in d["dof_names"]]
        mass, com, local = float(d["mass"]), d["com"], d["marker_local"]
        model_file = str(d["model_name"])
        code = code_lines_of_generator()

        def result(text, y, color=S.C_TEXT, size=RESULT_SIZE, font=None):
            kwargs = {"font": font} if font else {}
            return Text(text, font_size=size, color=color, **kwargs).align_to([BOX_X0, 0, 0], LEFT).set_y(y)

        head = S.title_block("Anatomy of a .bioMod file", "Every tag becomes part of the model")
        self.play(Write(head))

        # point 1: the translations and rotations tags define the generalized coordinates q
        box_a = file_box(model_file, rows_a, highlight={2, 3}, gaps={1})
        panel = S.code_panel("biorbd code", code, font_size=17)
        code_rows = panel[1][1]
        code_rows[5:].set_opacity(0)
        q_top = box_a.get_bottom()[1] - 0.45
        text_1 = result("The highlighted tags create these coordinates q:", q_top)
        names = VGroup(
            *[result(name, q_top - 0.5 - 0.38 * i, S.C_MODEL, 19, S.CODE_FONT) for i, name in enumerate(dofs)]
        )
        self.play(FadeIn(box_a), FadeIn(panel))
        self.play(FadeIn(text_1))
        self.play(FadeIn(names))

        # point 2: mass, com and the marker become numbers that biorbd gives back
        box_b = file_box(model_file, rows_b, highlight={0, 1, 5}, gaps={2, 3})
        b_top = box_b.get_bottom()[1] - 0.45
        text_2 = result("The highlighted tags come back from biorbd:", b_top)
        value_mass = result(f"Mass: {mass:.5f} kg", b_top - 0.5, S.C_MODEL)
        value_com = result(f"Centre of mass: ({com[0]:.4f}, {com[1]:.4f}, {com[2]:.4f}) m", b_top - 0.9, S.C_MODEL)
        value_marker = result(
            f"Marker position: ({local[0]:.4f}, {local[1]:.4f}, {local[2]:.4f}) m", b_top - 1.3, S.C_MODEL
        )
        self.play(FadeOut(box_a), FadeOut(text_1), FadeOut(names))
        self.play(FadeIn(box_b), code_rows[5:].animate.set_opacity(1))
        self.play(FadeIn(text_2))
        self.play(FadeIn(value_mass), FadeIn(value_com), FadeIn(value_marker))

        self.play(FadeIn(S.footer("The text file describes the model, and biorbd gives its content back as numbers.")))
