"""Video 13: an IMU is a frame fixed in a segment; biorbd gives its orientation in the world from q.

All numbers come from data/imu_main.npz (made by generate_imu_data.py, biorbd 1.12.3).
The code shown is read from the generator source, so it is the code that was executed (rule D5).
"""

from pathlib import Path

import numpy as np
from manim import *

from common import bio_style as S

TOPIC = "imu"
SHOULDER = np.array([-5.0, 1.3, 0.0])  # screen position of the BrasD frame origin at q = 0
SCALE = 3.2  # screen units per metre
COLUMN_X = -3.4
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


def frame_group(rt, origin_world, color, length=0.55, width=4) -> VGroup:
    """Axes y and z of a 4x4 frame drawn in the sagittal plane (y to the right, z up), relative to the shoulder."""
    start = SHOULDER + np.array([(rt[1, 3] - origin_world[1]) * SCALE, (rt[2, 3] - origin_world[2]) * SCALE, 0.0])
    group = VGroup()
    for column in (1, 2):
        direction = np.array([rt[1, column], rt[2, column], 0.0]) * length
        group.add(Arrow(start, start + direction, buff=0, stroke_width=width, color=color, tip_length=0.12))
    group.add(Dot(start, radius=0.06, color=color))
    return group


class AnimImu(Scene):
    def construct(self):
        d = S.load_npz(f"{TOPIC}_main.npz")
        local, world_a, world_b = d["local"], d["world_a"], d["world_b"]
        seg_a, seg_b = d["seg_a"], d["seg_b"]
        block_a, block_b = code_blocks()
        origin = seg_a[:3, 3]  # the shoulder at q = 0 is the drawing anchor

        def line(text, y, color=S.C_TEXT, x=COLUMN_X):
            return Text(text, font_size=TEXT_SIZE, color=color).align_to([x, 0, 0], LEFT).set_y(y)

        def matrix(world, top, color):
            grid = VGroup()
            for row in range(3):
                for col in range(3):
                    cell = Text(f"{world[row, col]:.2f}", font_size=TEXT_SIZE, color=color)
                    cell.move_to([COLUMN_X + 0.45 + 0.85 * col, top - 0.42 * row, 0])
                    grid.add(cell)
            return grid

        def pose(seg, world):
            body = frame_group(seg, origin, S.C_MUTED, length=0.7, width=3)
            imu = frame_group(world, origin, S.C_MODEL)
            link = DashedLine(
                body[-1].get_center(), imu[-1].get_center(), color=S.C_MUTED, stroke_width=2, dash_length=0.06
            )
            return VGroup(body, link, imu)

        head = S.title_block("Inertial measurement unit", "An IMU is a frame fixed in a segment.")
        self.play(Write(head))

        # point 1: the IMU sits in the segment (local rt), and its world orientation follows from q
        pose_a = pose(seg_a, world_a)
        name_seg = Text(str(d["segment_name"]), font=S.CODE_FONT, font_size=16, color=S.C_MUTED)
        name_seg.next_to(pose_a[0][-1], LEFT, buff=0.15)
        name_imu = Text(str(d["imu_name"]), font=S.CODE_FONT, font_size=16, color=S.C_MODEL)
        name_imu.next_to(pose_a[2][-1], DOWN, buff=0.35)
        panel_a = S.code_panel("biorbd code", block_a, font_size=18)
        text_1 = line("The IMU is fixed in the segment.", 1.6)
        text_2 = VGroup(
            line("Offset in the segment:", 1.1),
            line(f"y = {local[1, 3]:.2f} m, z = {local[2, 3]:.2f} m", 0.68, S.C_DATA),
        )
        self.play(FadeIn(pose_a[0]), FadeIn(name_seg), FadeIn(panel_a), FadeIn(text_1))
        self.play(FadeIn(pose_a[1]), FadeIn(pose_a[2]), FadeIn(name_imu), FadeIn(text_2))

        text_3 = line("World rotation of the IMU:", -0.1)
        mat_a = matrix(world_a, -0.55, S.C_MODEL)
        self.play(FadeOut(text_1), FadeOut(text_2), FadeIn(text_3), FadeIn(mat_a))

        # point 2: q changes, the segment moves, the IMU follows
        ghost = pose_a.copy().set_opacity(0.35)
        pose_b = pose(seg_b, world_b)
        name_imu_b = name_imu.copy().next_to(pose_b[2][-1], DOWN, buff=0.55)
        name_seg_b = name_seg.copy()
        panel_b = S.code_panel("biorbd code", block_b, font_size=18)
        mat_b = matrix(world_b, -0.55, S.C_MODEL)
        legend = Text("Faded frames show the previous posture.", font_size=20, color=S.C_MUTED)
        legend.to_corner(DOWN + LEFT, buff=0.35).shift(UP * 0.55)
        self.play(
            FadeOut(pose_a),
            FadeIn(ghost),
            FadeOut(name_imu),
            FadeOut(panel_a),
            FadeIn(panel_b),
            FadeOut(mat_a),
        )
        self.play(FadeIn(pose_b), FadeIn(name_imu_b), FadeIn(mat_b), FadeIn(legend))

        check_1 = line("World = segment × local rotation:", -2.0)
        check_2 = line(f"The largest difference is {float(d['error']):.1e}.", -2.4, S.C_DIFF)
        self.play(FadeIn(check_1), FadeIn(check_2))

        self.play(FadeIn(S.footer("The IMU orientation follows its segment, and biorbd computes it from q.")))
