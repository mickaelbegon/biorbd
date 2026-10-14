"""Data for the video "ik-least-squares": biorbd.InverseKinematics (scipy least squares) on synthetic markers.

The true motion q(t) is simulated (inside the joint ranges of the model), the "measured" markers are biorbd forward
kinematics plus Gaussian noise, and one marker is set to NaN for some frames. Run with the ``biorbd_anim`` Python.

Why the joint dampings of the pelvis are NOT set to zero here (the biorbd test does it): joint dampings only enter the
dynamics (torques); I checked that the solution of InverseKinematics is identical with and without them.
"""

from pathlib import Path

import biorbd
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT.parents[1] / "examples" / "pyomecaman.bioMod"
OUT = ROOT / "data" / "ik_least_squares_main.npz"
SEED = 14
N_FRAMES = 61
NOISE_STD = 0.002  # metres, added to every marker coordinate
MISSING_NAME = "jambed4"  # marker set to NaN
GAP = slice(20, 41)  # frames where it is missing (frames 20 to 40)
METHODS = ["lm", "trf", "only_lm"]


def simulate_q(t):
    """A squat-like motion with swinging arms, inside the joint ranges of the model (13 q, one column per frame)."""
    s = np.sin(2 * np.pi * t)
    c = 0.5 * (1 - np.cos(2 * np.pi * t))
    return np.stack(
        [
            -0.10 * c,  # pelvis y
            -0.30 * c,  # pelvis z
            0.10 * s,  # pelvis rotation
            0.30 * s,  # right arm z
            1.15 + 0.40 * s,  # right arm x
            -0.30 * s,  # left arm z
            1.15 - 0.40 * s,  # left arm x
            0.20 + 1.00 * c,  # right hip
            -1.40 * c,  # right knee
            0.30 * c,  # right ankle
            0.20 + 0.95 * c,  # left hip
            -1.30 * c,  # left knee
            0.25 * c,  # left ankle
        ]
    )


def marker_positions(model, q):
    return np.array([m.to_array() for m in model.markers(q)]).T  # (3, nb_marker)


def solve(model, markers, method):
    # CODE-BEGIN
    ik = biorbd.InverseKinematics(model, markers)
    q_est = ik.solve(method=method)
    # CODE-END
    return q_est, ik


def hide_marker(markers, missing, gap):
    # CODE2-BEGIN
    markers_nan = markers.copy()
    markers_nan[:, missing, gap] = np.nan
    # CODE2-END
    return markers_nan


def skeleton(model, q, names):
    """A few points (hip, knee, ankle, shoulder, hand, head...) for drawing a stick figure: all from biorbd."""
    jcs = lambda i: model.globalJCS(q, i).trans().to_array()
    mk = marker_positions(model, q)
    idx = {n: i for i, n in enumerate(names)}
    pelvis = jcs(0)
    shoulder_r, shoulder_l = jcs(3), jcs(4)
    head = mk[:, [idx[n] for n in names if n.startswith("tete")]].mean(axis=1)
    neck = 0.5 * (shoulder_r + shoulder_l)
    pts = [pelvis, neck, head, shoulder_r, mk[:, idx["brasd17"]], shoulder_l, mk[:, idx["brasg17"]]]
    pts += [jcs(i) for i in (5, 6, 7, 8, 9, 10)]
    return np.array(pts)


def main():
    model = biorbd.Model(str(MODEL_PATH))
    names = [model.markerNames()[i].to_string() for i in range(model.nbMarkers())]
    missing = names.index(MISSING_NAME)
    rng = np.random.default_rng(SEED)
    t = np.linspace(0.0, 1.0, N_FRAMES)
    q_true = simulate_q(t)
    truth = np.stack(
        [marker_positions(model, q_true[:, k]) for k in range(N_FRAMES)], axis=2
    )  # (3, nb_marker, nb_frame)
    markers = truth + rng.normal(0.0, NOISE_STD, truth.shape)
    markers_nan = hide_marker(markers, missing, GAP)
    gap_frames = np.arange(N_FRAMES)[GAP]

    out = dict(
        t=t,
        q_true=q_true,
        marker_names=np.array(names),
        missing=missing,
        gap_frames=gap_frames,
        noise_std_mm=1000 * NOISE_STD,
        n_marker=model.nbMarkers(),
        n_q=model.nbQ(),
        markers=markers,
        markers_nan=markers_nan,
        truth_missing=truth[:, missing, :],
        q_min=np.array(biorbd.utils.get_range_q(model)[0]),
        q_max=np.array(biorbd.utils.get_range_q(model)[1]),
    )
    for method in METHODS:
        for label, data in (("full", markers), ("nan", markers_nan)):
            q_est, ik = solve(model, data, method)
            est = np.stack([marker_positions(model, q_est[:, k]) for k in range(N_FRAMES)], axis=2)
            err_q = np.degrees(np.sqrt(np.mean((q_est - q_true) ** 2, axis=0)))  # per frame, deg
            err_mk = 1000 * np.sqrt(np.mean(np.sum((est - truth) ** 2, axis=0), axis=0))  # per frame, mm (all markers)
            key = f"{method}_{label}"
            out[f"q_{key}"] = q_est
            out[f"err_q_{key}"] = err_q
            out[f"err_marker_{key}"] = err_mk
            out[f"rmse_q_{key}"] = np.degrees(np.sqrt(np.mean((q_est - q_true) ** 2)))
            out[f"rmse_marker_{key}"] = np.sqrt(np.mean(err_mk**2))
            out[f"missing_err_{key}"] = 1000 * np.linalg.norm(est[:, missing, :] - truth[:, missing, :], axis=0)
            out[f"within_bounds_{key}"] = bool(
                np.all(q_est >= out["q_min"][:, None] - 1e-9) and np.all(q_est <= out["q_max"][:, None] + 1e-9)
            )
            over = np.maximum(out["q_min"][:, None] - q_est, q_est - out["q_max"][:, None])
            out[f"max_violation_deg_{key}"] = np.degrees(max(over.max(), 0.0))  # 0 if every q is inside its range
            out[f"max_nfev_{key}"] = max(s.nfev for s in ik.list_sol)
            print(
                key,
                "rmse q %.3f deg" % out[f"rmse_q_{key}"],
                "rmse marker %.2f mm" % out[f"rmse_marker_{key}"],
                "missing marker in gap %.2f mm" % out[f"missing_err_{key}"][GAP].mean(),
                "max violation %.3f deg" % out[f"max_violation_deg_{key}"],
            )
            if method == "lm":
                out[f"est_missing_{label}"] = est[:, missing, :]
                out[f"skel_est_{label}"] = np.stack([skeleton(model, q_est[:, k], names) for k in range(N_FRAMES)])
    out["skel_true"] = np.stack([skeleton(model, q_true[:, k], names) for k in range(N_FRAMES)])
    out["measured_missing_err_mm"] = 1000 * np.linalg.norm(markers[:, missing, :] - truth[:, missing, :], axis=0)
    np.savez(OUT, **out)
    print("saved", OUT)


if __name__ == "__main__":
    main()
