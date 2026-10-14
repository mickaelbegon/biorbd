"""Data for video 01 (anatomy of a .bioMod file): what biorbd reads from the Pelvis segment of pyomecaman.bioMod.

Run with the biorbd_anim environment. The lines between the ``# code:start`` / ``# code:end`` markers are the ones
shown in the code panel of the video (rule D5). The file excerpts displayed in the video are copied here, line by
line, from the real file (they are checked against the values biorbd returned).
"""

from pathlib import Path

import biorbd
import numpy as np

ANIM = Path(__file__).resolve().parent
MODEL_FILE = ANIM.parents[1] / "examples" / "pyomecaman.bioMod"
path = str(MODEL_FILE)

# (first line, last line) of the excerpts shown, 1-based, in examples/pyomecaman.bioMod
EXCERPT_A = [(1, 1), (9, 11)]  # version, segment Pelvis, translations, rotations
EXCERPT_B = [(22, 22), (27, 27), (35, 35), (38, 41)]  # mass, com, endsegment, marker pelv1 ... endmarker

# code:start
model = biorbd.Biorbd(path)
seg = model.segments["Pelvis"]
trans = seg.translations
rot = seg.rotations
dofs = model.dof_names[:3]
mass = seg.mass
com = seg.center_of_mass
q = np.zeros(model.nb_q)
local = model.markers(q)["pelv1"].local
# code:end

file_lines = [line.rstrip("\r\n") for line in MODEL_FILE.read_text(encoding="utf-8").splitlines()]


def excerpt(ranges):
    """Lines copied from the real file, with the common left indentation removed (display only)."""
    rows = [file_lines[i - 1] for a, b in ranges for i in range(a, b + 1)]
    cut = min(len(r) - len(r.lstrip()) for r in rows if r.strip())
    return [r[cut:] for r in rows], [i for a, b in ranges for i in range(a, b + 1)]


rows_a, numbers_a = excerpt(EXCERPT_A)
rows_b, numbers_b = excerpt(EXCERPT_B)

# the excerpt really contains what biorbd returned
assert rows_a[0].split() == ["version", "4"]
assert [r.split() for r in rows_a[1:]] == [["segment", "Pelvis"], ["translations", "yz"], ["rotations", "x"]]
assert trans == "yz" and rot == "x" and list(dofs) == ["Pelvis_TransY", "Pelvis_TransZ", "Pelvis_RotX"]
assert np.isclose(float(rows_b[0].split()[1]), float(mass))
assert np.allclose([float(v) for v in rows_b[1].split()[1:]], com)
assert rows_b[2].split() == ["endsegment"]
assert rows_b[3].split() == ["marker", "pelv1"] and rows_b[4].split() == ["parent", "Pelvis"]
assert rows_b[5].split()[0] == "position" and np.allclose([float(v) for v in rows_b[5].split()[1:]], local)
assert rows_b[6].split() == ["endmarker"]

np.savez(
    ANIM / "data" / "biomod_anatomy_main.npz",
    model_name=np.array(MODEL_FILE.name),
    rows_a=np.array(rows_a),
    rows_b=np.array(rows_b),
    numbers_a=np.array(numbers_a),
    numbers_b=np.array(numbers_b),
    nb_q=model.nb_q,
    translations=np.array(trans),
    rotations=np.array(rot),
    dof_names=np.array(list(dofs)),
    mass=np.array(mass),
    com=np.array(com),
    marker_local=np.array(local),
    marker_name=np.array("pelv1"),
    segment_name=np.array("Pelvis"),
)
print("nb_q", model.nb_q, dofs, trans, rot, "mass", mass, "com", com, "pelv1 local", local)
print(rows_a, rows_b)
