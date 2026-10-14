"""Data for the video "model-loading": load examples/arm26.bioMod and list what the model object contains.

Run with the ``biorbd_anim`` Python. The numbers and names come from the real biorbd model.
"""

import os
from pathlib import Path

import biorbd
import numpy as np

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parents[1]
OUT = ROOT / "data" / "model_loading_main.npz"


def main():
    os.chdir(REPO)  # the displayed path is relative to the repository root
    # CODE-BEGIN
    model = biorbd.Biorbd("examples/arm26.bioMod")
    nb_q = model.nb_q
    dof_names = model.dof_names
    segments = [s.name for s in model.segments]
    markers = [m.name for m in model.markers]
    muscles = [m.name for m in model.muscles]
    # CODE-END
    np.savez(
        OUT,
        model_name=model.name,
        nb_q=nb_q,
        dof_names=np.array(dof_names),
        segments=np.array(segments),
        markers=np.array(markers),
        muscles=np.array(muscles),
        n_segments=len(segments),
        n_markers=len(markers),
        n_muscles=len(muscles),
    )
    print(model.name, nb_q, dof_names, len(segments), segments, len(markers), markers, len(muscles), muscles)


if __name__ == "__main__":
    main()
