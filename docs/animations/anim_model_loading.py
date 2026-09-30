"""Video 02: loading a model with biorbd.Biorbd(path) and inspecting what the model object contains."""

import textwrap

from manim import *

from common import bio_style as S

TOPIC = "model_loading"
GENERATOR = S.ROOT / "generate_model_loading_data.py"
X0 = -6.6  # left edge of the text rows
ROW_W = 6.1


def generator_code() -> list[str]:
    """The lines run by the generator between the markers (rule D5)."""
    lines = GENERATOR.read_text(encoding="utf-8").splitlines()
    begin = next(i for i, line in enumerate(lines) if "# CODE-BEGIN" in line)
    end = next(i for i, line in enumerate(lines) if "# CODE-END" in line)
    return textwrap.dedent("\n".join(lines[begin + 1 : end])).splitlines()


def code_text(text: str, color=S.C_MODEL, size: int = 18) -> Text:
    out = Text(text, font=S.CODE_FONT, font_size=size, color=color)
    if out.width > ROW_W:
        out.scale_to_fit_width(ROW_W)
    return out


def names_lines(names, per_line: int, more: bool = False) -> VGroup:
    """Names in code font, `per_line` per line, one Text per line."""
    chunks = [list(names[i : i + per_line]) for i in range(0, len(names), per_line)]
    texts = []
    for k, chunk in enumerate(chunks):
        s = ", ".join(chunk)
        if k < len(chunks) - 1:
            s += ","
        elif more:
            s += ", ..."
        texts.append(code_text(s))
    return VGroup(*texts).arrange(DOWN, aligned_edge=LEFT, buff=0.08)


class AnimModelLoading(Scene):
    def construct(self):
        data = S.load_npz(f"{TOPIC}_main.npz")
        name = str(data["model_name"])
        nb_q, n_seg, n_mark, n_mus = (int(data[k]) for k in ("nb_q", "n_segments", "n_markers", "n_muscles"))
        dof, segs, marks, mus = (list(map(str, data[k])) for k in ("dof_names", "segments", "markers", "muscles"))

        self.play(Write(S.title_block("Loading a model", "A path goes in, a model comes out")))
        panel = S.code_panel("biorbd code", generator_code(), font_size=15)
        panel.shift(LEFT * 0.25)  # the widest code line would otherwise come within 0.15 of the frame edge
        code_lines = panel[1][1]
        self.play(FadeIn(panel))

        def highlight(*idx):
            group = VGroup(*[code_lines[i] for i in idx])
            box = SurroundingRectangle(group, color=S.C_MODEL, buff=0.05, stroke_width=2)
            return box

        # --- point 1: the path goes in, the model object comes out -----------------------------
        path_txt = code_text("examples/arm26.bioMod", S.C_DATA, 22)
        arrow = Arrow(LEFT, RIGHT, buff=0, color=S.C_MUTED, stroke_width=4, max_tip_length_to_length_ratio=0.3)
        arrow.set_width(0.8)
        model_txt = code_text(f"Biorbd ({name})", S.C_MODEL, 22)
        head = VGroup(path_txt, arrow, model_txt).arrange(RIGHT, buff=0.25).move_to([X0 + 3.0, 1.75, 0])
        head.align_to([X0, 0, 0], LEFT)
        box = highlight(0)
        sent = Text("Loading the file builds a model.", font_size=26, color=S.C_TEXT)
        if sent.width > 6.0:  # keep clear of the code panel caption, also in French
            sent.scale_to_fit_width(6.0)
        sent.next_to(head, DOWN, aligned_edge=LEFT, buff=0.3)
        self.play(FadeIn(path_txt), FadeIn(box))
        self.play(GrowArrow(arrow), FadeIn(model_txt), FadeIn(sent), run_time=1.0)

        # --- point 2: inspect its parts -----------------------------------------------------------
        self.play(FadeOut(sent))
        rows = [
            (f"Degrees of freedom: {nb_q}", names_lines(dof, 1), (1, 2)),
            (f"Segments: {n_seg}", names_lines(segs[:3], 3, more=True), (3,)),
            (f"Markers: {n_mark}", names_lines(marks, 3), (4,)),
            (f"Muscles: {n_mus}", names_lines(mus, 6), (5,)),
        ]
        y = 1.0
        for label, names, idx in rows:
            lab = Text(label, font_size=26, color=S.C_TITLE)
            lab.move_to([X0, y, 0], aligned_edge=LEFT)
            names.next_to(lab, DOWN, aligned_edge=LEFT, buff=0.1)
            new_box = highlight(*idx)
            self.play(ReplacementTransform(box, new_box), FadeIn(lab), FadeIn(names), run_time=0.9)
            box = new_box
            y -= 0.62 + 0.27 * len(names)
        self.play(FadeIn(S.footer("One call loads the file, then the model lists what it contains.")))
