"""Data for the video "ik-by-hand": a hand-written Gauss-Newton inverse kinematics on synthetic markers.

The true motion is simulated (fixed seed), the "measured" markers are biorbd forward kinematics plus noise,
then q is recovered by the Gauss-Newton loop below. Run with the ``biorbd_anim`` Python.
"""

from pathlib import Path

import biorbd
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT.parents[1] / "examples" / "arm26.bioMod"
OUT = ROOT / "data" / "ik_by_hand_main.npz"
SEED = 15
N_FRAMES = 101
NOISE_STD = 0.002  # metres, added to every marker coordinate
DEMO_FRAME = 40
MAX_ITER = 20
Q_INIT = np.zeros(2)


def gauss_newton_step(model, q, measured):
    # CODE-BEGIN
    pos = [m.world for m in model.markers(q)]
    res = (np.array(pos) - measured).ravel()
    J = np.vstack(model.markers.jacobian(q))
    q = q - np.linalg.pinv(J) @ res
    # CODE-END
    return q, res, J


def marker_positions(model, q):
    return np.array([m.world for m in model.markers(q)])


def solve_frame(model, measured, q0, max_iter=MAX_ITER):
    """Return the q of each iteration (row 0 = q0), the residual norms and the marker positions per iteration."""
    qs, errors, poses = [np.array(q0, dtype=float)], [], []
    for _ in range(max_iter):
        q_new, res, J = gauss_newton_step(model, qs[-1], measured)
        errors.append(np.linalg.norm(res.reshape(-1, 3), axis=1))  # per-marker distance before the step
        poses.append(marker_positions(model, qs[-1]))
        qs.append(q_new)
        if np.linalg.norm(q_new - qs[-2]) < 1e-9:
            break
    errors.append(np.linalg.norm((marker_positions(model, qs[-1]) - measured), axis=1))
    poses.append(marker_positions(model, qs[-1]))
    return np.array(qs), np.array(errors), np.array(poses)


def main():
    model = biorbd.Biorbd(str(MODEL_PATH))
    rng = np.random.default_rng(SEED)
    t = np.linspace(0.0, 1.0, N_FRAMES)
    ph = rng.uniform(0, 2 * np.pi, 2)
    amp = rng.uniform(0.3, 0.45, 2)
    q_true = np.stack(
        [0.35 + amp[0] * np.sin(2 * np.pi * 1.0 * t + ph[0]), 1.1 + amp[1] * np.sin(2 * np.pi * 1.5 * t + ph[1])],
        axis=1,
    )
    truth_markers = np.array([marker_positions(model, q) for q in q_true])
    measured = truth_markers + rng.normal(0.0, NOISE_STD, truth_markers.shape)

    q_est = np.zeros_like(q_true)
    q_one = np.zeros_like(q_true)
    n_iter = np.zeros(N_FRAMES, dtype=int)
    for k in range(N_FRAMES):
        qs, _, _ = solve_frame(model, measured[k], Q_INIT)
        q_est[k], q_one[k], n_iter[k] = qs[-1], qs[1], len(qs) - 1
    demo_q, demo_err, demo_pose = solve_frame(model, measured[DEMO_FRAME], Q_INIT)
    est_markers = np.array([marker_positions(model, q) for q in q_est])
    jac = np.vstack(model.markers.jacobian(q_true[DEMO_FRAME]))
    sv = np.linalg.svd(jac, compute_uv=False)

    np.savez(
        OUT,
        t=t,
        q_true=q_true,
        q_est=q_est,
        q_one=q_one,
        n_iter=n_iter,
        demo_frame=DEMO_FRAME,
        demo_q=demo_q,
        demo_err=demo_err,
        demo_pose=demo_pose,
        demo_measured=measured[DEMO_FRAME],
        marker_names=np.array([m.name for m in model.markers]),
        rmse_q_deg=np.degrees(np.sqrt(np.mean((q_est - q_true) ** 2))),
        rmse_marker_mm=1000 * np.sqrt(np.mean(np.sum((est_markers - truth_markers) ** 2, axis=2))),
        noise_std_mm=1000 * NOISE_STD,
        singular_values=sv,
        max_iter_used=n_iter.max(),
    )
    print("frames", N_FRAMES, "iterations max", n_iter.max(), "demo iters", len(demo_q) - 1)
    print("demo marker error per iteration (mm):", (1000 * np.sqrt((demo_err**2).mean(axis=1))).round(2))
    print("rmse q deg", np.degrees(np.sqrt(np.mean((q_est - q_true) ** 2))), "singular values", sv)
    print("q ranges true", q_true.min(0), q_true.max(0))


if __name__ == "__main__":
    main()
