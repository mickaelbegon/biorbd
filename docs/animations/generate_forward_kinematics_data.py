"""Data for video 03 (forward kinematics): right-leg chain of pyomecaman.bioMod in two poses.

Run with the biorbd_anim environment. The lines between the two ``# code:`` markers are the ones shown
in the code panel of the video (rule D5); everything else only stores the results.
"""

from pathlib import Path

import biorbd
import numpy as np

ANIM = Path(__file__).resolve().parent
path = str(ANIM.parents[1] / "examples" / "pyomecaman.bioMod")
CHAIN = ["Pelvis", "CuisseD", "JambeD", "PiedD"]


def collect(model, q, frames, mk):
    """Store the world origin and the y/z axes of each frame of the chain (not shown in the video)."""
    origins = np.array([frames[name].world_translation for name in CHAIN])
    axes = np.array([frames[name].world_rotation[:, 1:] for name in CHAIN])
    return origins, axes, np.array(mk.world), np.array(mk.local)


# code:start
model = biorbd.Biorbd(path)
q = np.zeros(model.nb_q)
frames = model.segment_frames(q)
foot = frames["PiedD"]
origin = foot.world_translation
mk = model.markers(q)["piedd1"]
pos = mk.world
# code:end
q_a = q.copy()
origins_a, axes_a, marker_a, marker_local = collect(model, q, frames, mk)
origin_a, pos_a = np.array(origin), np.array(pos)

# code:start
q[7:10] = [0.6, -0.9, 0.3]
frames = model.segment_frames(q)
foot = frames["PiedD"]
origin = foot.world_translation
mk = model.markers(q)["piedd1"]
pos = mk.world
# code:end
q_b = q.copy()
origins_b, axes_b, marker_b, _ = collect(model, q, frames, mk)
assert np.allclose(origins_a[-1], origin_a) and np.allclose(origins_b[-1], origin) and np.allclose(marker_b, pos)

np.savez(
    ANIM / "data" / "forward_kinematics_main.npz",
    nb_q=model.nb_q,
    q_a=q_a,
    q_b=q_b,
    chain=np.array(CHAIN),
    origins_a=origins_a,
    origins_b=origins_b,
    axes_a=axes_a,
    axes_b=axes_b,
    marker_a=marker_a,
    marker_b=marker_b,
    marker_local=marker_local,
    marker_diff=marker_b - marker_a,
    marker_name=np.array("piedd1"),
    model_name=np.array("pyomecaman.bioMod"),
)
print("nb_q", model.nb_q, "marker a", marker_a, "b", marker_b, "diff", marker_b - marker_a)
