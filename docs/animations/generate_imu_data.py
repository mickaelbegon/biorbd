"""Data for video 13 (IMU): the right-hand IMU of pyomecaman_withIMUs.bioMod in two postures.

Run with the biorbd_anim environment. The lines between the ``# code:`` markers are the ones shown in the
code panel of the video (rule D5); everything else only stores the results.
"""

from pathlib import Path

import biorbd
import numpy as np

ANIM = Path(__file__).resolve().parent
path = str(ANIM.parents[1] / "test" / "models" / "IMUandCustomRT" / "pyomecaman_withIMUs.bioMod")
SEGMENT = "BrasD"
CHAIN = ["Tronc", "BrasD"]

# code:start
model = biorbd.Biorbd(path)
imus = model.internal
local = imus.IMU()[0].to_array()
q = np.zeros(model.nb_q)
world = imus.IMU(q, 0).to_array()
# code:end
q_a = q.copy()
world_a = np.array(world)
frame_a = model.segment_frames(q_a)[SEGMENT]
seg_a = np.array(frame_a.world)

# code:start
q[3:5] = [0.5, -1.2]
world = imus.IMU(q, 0).to_array()
seg = model.segment_frames(q)["BrasD"]
expected = seg.world @ local
err = abs(world - expected).max()
# code:end
q_b = q.copy()
world_b = np.array(world)
seg_b = np.array(seg.world)
assert imus.IMUsNames()[0].to_string() == "rightHand"
assert model.segments[SEGMENT].name == SEGMENT

np.savez(
    ANIM / "data" / "imu_main.npz",
    q_a=q_a,
    q_b=q_b,
    imu_name=np.array("rightHand"),
    segment_name=np.array(SEGMENT),
    model_name=np.array("pyomecaman_withIMUs.bioMod"),
    nb_imus=imus.nbIMUs(),
    local=np.array(local),
    world_a=world_a,
    world_b=world_b,
    seg_a=seg_a,
    seg_b=seg_b,
    error=err,
)
print("local\n", local, "\nworld_a\n", world_a, "\nworld_b\n", world_b, "\nerror", err)
