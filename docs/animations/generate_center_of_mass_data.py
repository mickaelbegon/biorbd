"""Data for video 06 (centre of mass): mass-weighted average of the segment centres of mass, two postures.

Run with the biorbd_anim environment:  python generate_center_of_mass_data.py
The lines between the CODE-BEGIN and CODE-END markers are displayed as is in the code panel (rule D5).
"""

from pathlib import Path

import biorbd
import numpy as np

HERE = Path(__file__).resolve().parent
MODEL_PATH = HERE.parents[1] / "examples" / "pyomecaman.bioMod"

model = biorbd.Biorbd(str(MODEL_PATH))
names = [segment.name for segment in model.segments]
# Parent of each segment, read from the bioMod text (only used to draw the links between segments).
parent_of, current = {}, None
for words in (line.split() for line in MODEL_PATH.read_text().splitlines()):
    if len(words) == 2 and words[0] == "segment":
        current = words[1]
    elif len(words) == 2 and words[0] == "parent" and current in names and current not in parent_of:
        parent_of[current] = words[1]
parents = np.array([names.index(parent_of[n]) if n in parent_of else -1 for n in names])
q_a = np.zeros(model.nb_q)  # standing posture
q_b = q_a.copy()
q_b[4] = -1.4  # BrasD_RotX: right arm raised forward
q_b[7:10] = [0.6, -0.9, 0.3]  # right leg: hip, knee, ankle
q_b[10:13] = [0.3, -0.4, 0.1]  # left leg: hip, knee, ankle

out = {}
for tag, q in (("a", q_a), ("b", q_b)):
    # CODE-BEGIN
    frames = model.segment_frames(q)
    weighted = np.zeros(3)
    for seg in model.segments:
        f = frames[seg.name]
        R = f.world_rotation
        t = f.world_translation
        com = R @ seg.center_of_mass + t
        weighted += seg.mass * com
    mean = weighted / model.mass
    com_model = model.center_of_mass(q)
    # CODE-END
    # Stored for the drawing (not shown): origin and world centre of mass of every segment, and the masses.
    origins = np.array([frames[n].world_translation for n in names])
    coms = np.array(
        [frames[s.name].world_translation + frames[s.name].world_rotation @ s.center_of_mass for s in model.segments]
    )
    masses = np.array([s.mass for s in model.segments])
    assert np.allclose(masses @ coms / masses.sum(), mean) and np.isclose(masses.sum(), model.mass)
    out.update(
        {
            f"origins_{tag}": origins,
            f"coms_{tag}": coms,
            f"mean_{tag}": np.array(mean),
            f"com_model_{tag}": np.array(com_model),
            f"diff_{tag}": np.array(mean) - np.array(com_model),
        }
    )

np.savez(
    HERE / "data" / "center_of_mass_main.npz",
    q_a=q_a,
    q_b=q_b,
    names=np.array(names),
    parents=parents,
    masses=masses,
    total_mass=float(model.mass),
    model_name=np.array("pyomecaman.bioMod"),
    **out,
)
print("mass", float(model.mass), "model a", out["com_model_a"], "b", out["com_model_b"])
print("diff a", out["diff_a"], "b", out["diff_b"])
