"""Video 03: forward kinematics. From generalized coordinates q to marker positions in the world.

All numbers come from data/forward_kinematics_main.npz (made by generate_forward_kinematics_data.py, biorbd 1.12.3).
The code shown is read from the generator source, so it is the code that was executed (rule D5).
"""

from pathlib import Path

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "forward_kinematics"
HIP = np.array([-6.1, 1.9, 0.0])  # screen position of the world origin (pelvis)
SCALE = 3.0  # screen units per metre
COLUMN_X = -4.3
Y1, Y2 = 1.8, 0.6  # rows of the text column  # left edge of the text column
TEXT_SIZE = 20


def code_blocks() -> list[list[str]]:
    """Lines between the '# code:start' and '# code:end' markers of the generator."""
    lines = (Path(__file__).with_name(f"generate_{TOPIC}_data.py")).read_text(encoding="utf-8").splitlines()
    blocks, current = [], None
    for line in lines:
        if line.startswith("# code:start"):
            current = []
        elif line.startswith("# code:end"):
            blocks.append(current)
            current = None
        elif current is not None:
            current.append(line)
    return blocks


def to_screen(point) -> np.ndarray:
    """World (x, y, z) in metres -> screen. The leg is drawn in the sagittal plane: y to the right, z up."""
    return HIP + np.array([point[1] * SCALE, point[2] * SCALE, 0.0])


def pose_group(origins, axes, marker, marker_local_ok=True) -> VGroup:
    points = [to_screen(p) for p in origins]
    segments = VGroup(*[Line(a, b, color=S.C_TITLE, stroke_width=6) for a, b in zip(points[:-1], points[1:])])
    arrows = VGroup()
    for point, frame_axes in zip(points, axes):
        for column in range(2):
            direction = np.array([frame_axes[1, column], frame_axes[2, column], 0.0]) * 0.3
            arrows.add(Arrow(point, point + direction, buff=0, stroke_width=3, color=S.C_MODEL, tip_length=0.1))
    dots = VGroup(*[Dot(p, radius=0.05, color=S.C_TEXT) for p in points])
    target = to_screen(marker)
    link = DashedLine(points[-1], target, color=S.C_MUTED, stroke_width=2, dash_length=0.06)
    spot = Dot(target, radius=0.09, color=S.C_DATA)
    return VGroup(segments, link, arrows, dots, spot)


class AnimForwardKinematics(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        origins_a, origins_b = d["origins_a"], d["origins_b"]
        marker_a, marker_b, marker_local = d["marker_a"], d["marker_b"], d["marker_local"]
        diff = d["marker_diff"]
        q_leg = d["q_b"][7:10]
        block_a, block_b = code_blocks()

        def yz(vector) -> tuple[str, str]:
            return f"{vector[1]:.2f}", f"{vector[2]:.2f}"

        def line(text, y, color=S.C_TEXT):
            return Text(text, font_size=TEXT_SIZE, color=color).align_to([COLUMN_X, 0, 0], LEFT).set_y(y)

        head = S.title_block("Forward kinematics", "From joint angles q to positions in the world")
        self.play(Write(head))

        # point 1: the frames of the chain, stacked from the pelvis to the foot
        pose_a = pose_group(origins_a, d["axes_a"], marker_a)
        names = VGroup()
        for index, name in ((1, "CuisseD"), (2, "JambeD"), (3, "PiedD")):
            label = Text(name, font=S.CODE_FONT, font_size=16, color=S.C_MUTED)
            label.next_to(to_screen(origins_a[index]), RIGHT, buff=0.42)
            names.add(label)
        panel_a = S.code_panel("biorbd code", block_a)
        code_lines = panel_a[1][1]
        code_lines[5:].set_opacity(0)
        foot_y, foot_z = yz(origins_a[3])
        text_1 = line("Each frame sits on its parent.", Y1)
        text_2 = VGroup(
            line("Foot frame origin:", Y2, S.C_TEXT), line(f"y = {foot_y} m, z = {foot_z} m", Y2 - 0.42, S.C_MODEL)
        )
        self.play(FadeIn(pose_a[0]), FadeIn(pose_a[2]), FadeIn(pose_a[3]), FadeIn(names))
        self.play(FadeIn(panel_a), FadeIn(text_1))
        self.play(FadeIn(text_2))

        # point 2: a marker fixed in the foot frame, and its position in the world
        local_y, local_z = yz(marker_local)
        world_y, world_z = yz(marker_a)
        text_3 = VGroup(
            line("Marker in the foot frame:", Y1, S.C_TEXT),
            line(f"y = {local_y} m, z = {local_z} m", Y1 - 0.42, S.C_DATA),
        )
        head_4 = line("Marker in the world:", Y2, S.C_TEXT)
        value_4 = line(f"y = {world_y} m, z = {world_z} m", Y2 - 0.42, S.C_MODEL)
        tag = Text("piedd1", font=S.CODE_FONT, font_size=16, color=S.C_DATA).next_to(
            to_screen(marker_a), RIGHT, buff=0.15
        )
        self.play(FadeOut(text_1), FadeOut(text_2), FadeOut(names))
        self.play(FadeIn(pose_a[1]), FadeIn(pose_a[4]), FadeIn(tag), FadeIn(text_3))
        self.play(FadeIn(head_4), FadeIn(value_4), code_lines[5:].animate.set_opacity(1))

        # q changes: ghost of the previous pose, new pose, difference of the marker position
        ghost = pose_a.copy().set_opacity(0.3)
        pose_b = pose_group(origins_b, d["axes_b"], marker_b)
        tag_b = Text("piedd1", font=S.CODE_FONT, font_size=16, color=S.C_DATA).next_to(
            to_screen(marker_b), RIGHT, buff=0.15
        )
        q_text = ", ".join(f"{value:.1f}" for value in q_leg)
        text_5 = VGroup(line("Hip, knee, ankle angles:", Y1, S.C_TEXT), line(f"{q_text} rad", Y1 - 0.42, S.C_TEXT))
        new_y, new_z = yz(marker_b)
        value_6 = line(f"y = {new_y} m, z = {new_z} m", Y2 - 0.42, S.C_MODEL)
        legend = Text("Faded: previous pose.", font_size=20, color=S.C_MUTED)
        legend.to_corner(DOWN + LEFT, buff=0.35).shift(UP * 0.7)
        panel_b = S.code_panel("biorbd code", block_b)
        self.play(
            FadeOut(pose_a),
            FadeIn(ghost),
            FadeOut(tag),
            FadeOut(text_3),
            FadeOut(panel_a),
            FadeIn(panel_b),
        )
        self.play(FadeIn(pose_b), FadeIn(tag_b), FadeIn(text_5), FadeIn(legend))
        self.play(FadeOut(value_4))
        self.play(FadeIn(value_6))

        # difference axis (A6)
        scale = 17.0  # screen units per metre on the small axis
        x0 = COLUMN_X + 0.4
        caption = line("Change of the marker position:", -0.6, S.C_MUTED)
        axis = Line([x0, -0.95, 0], [x0, -2.35, 0], color=S.C_MUTED, stroke_width=2)
        rows = VGroup(axis)
        for row, (name, value) in enumerate((("y", diff[1]), ("z", diff[2]))):
            y = -1.3 - 0.7 * row
            bar = Rectangle(width=float(value) * scale, height=0.32, color=S.C_DIFF, fill_opacity=0.9, stroke_width=0)
            bar.move_to([x0 + bar.width / 2, y, 0])
            axis_name = Text(name, font_size=TEXT_SIZE, color=S.C_DIFF).next_to(axis, LEFT, buff=0.1).set_y(y)
            amount = Text(f"{value:.2f} m", font_size=TEXT_SIZE, color=S.C_DIFF).next_to(bar, RIGHT, buff=0.15)
            rows.add(bar, axis_name, amount)
        self.play(FadeIn(caption), FadeIn(rows))

        self.play(FadeIn(S.footer("When q changes, every frame moves and biorbd updates the marker.")))
