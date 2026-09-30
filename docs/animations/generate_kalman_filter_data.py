"""Data for the video "kalman-filter": an extended Kalman filter reconstructs q from noisy synthetic markers.

The true motion is simulated (fixed seed), the "measured" markers are biorbd forward kinematics plus noise.
The filter is biorbd's ExtendedKalmanFilterMarkers. For comparison, each frame is also solved alone by a
hand-written Gauss-Newton step (the method of the video "ik-by-hand"). Run with the ``biorbd_anim`` Python.
"""

from pathlib import Path

import biorbd
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT.parents[1] / "examples" / "arm26.bioMod"
OUT = ROOT / "data" / "kalman_filter_main.npz"
SEED = 16
N_FRAMES = 151
FREQ = 100  # Hz
NOISE_STD = 0.002  # metres, added to every marker coordinate
NOISE = 1e-10  # biorbd default (KalmanParam), not tuned
ERROR = 1e-5  # biorbd default (KalmanParam), not tuned


def marker_positions(model, q):
    return np.array([m.world for m in model.markers(q)])


def run_kalman(model, measured):
    # CODE-BEGIN
    Kalman = biorbd.ExtendedKalmanFilterMarkers
    kalman = Kalman(model, FREQ, NOISE, ERROR)
    frames = [y.T for y in measured]
    results = kalman.reconstruct_frames(frames)
    q, qdot, qddot = zip(*results)
    # CODE-END
    return np.array(q).reshape(len(measured), -1), np.array(qdot).reshape(len(measured), -1)


def solve_alone(model, y, q0=np.zeros(2), n_iter=20):
    """Per-frame Gauss-Newton (as in ik-by-hand), no memory between frames."""
    q = np.array(q0, dtype=float)
    for _ in range(n_iter):
        res = (marker_positions(model, q) - y).ravel()
        q = q - np.linalg.pinv(np.vstack(model.markers.jacobian(q))) @ res
    return q


def rmse(a, b):
    return float(np.sqrt(np.mean((a - b) ** 2)))


def main():
    model = biorbd.Biorbd(str(MODEL_PATH))
    rng = np.random.default_rng(SEED)
    t = np.arange(N_FRAMES) / FREQ
    ph = rng.uniform(0, 2 * np.pi, 2)
    amp = np.array([0.4, 0.4])
    w = 2 * np.pi * np.array([1.0, 1.5])
    off = np.array([0.35, 1.1])
    q_true = off + amp * np.sin(np.outer(t, w) + ph)
    qdot_true = amp * w * np.cos(np.outer(t, w) + ph)
    truth_markers = np.array([marker_positions(model, q) for q in q_true])
    measured = truth_markers + rng.normal(0.0, NOISE_STD, truth_markers.shape)

    q_kal, qdot_kal = run_kalman(model, measured)
    q_alone = np.array([solve_alone(model, y) for y in measured])
    # qdot of the per-frame estimate by finite differences, for information
    qdot_alone = np.gradient(q_alone, t, axis=0)

    np.savez(
        OUT,
        t=t,
        q_true=q_true,
        qdot_true=qdot_true,
        q_kal=q_kal,
        qdot_kal=qdot_kal,
        q_alone=q_alone,
        qdot_alone=qdot_alone,
        rmse_q_kal_deg=np.degrees(rmse(q_kal, q_true)),
        rmse_q_alone_deg=np.degrees(rmse(q_alone, q_true)),
        rmse_qdot_kal_deg=np.degrees(rmse(qdot_kal, qdot_true)),
        rmse_qdot_alone_deg=np.degrees(rmse(qdot_alone, qdot_true)),
        noise_std_mm=1000 * NOISE_STD,
        noise_factor=NOISE,
        error_factor=ERROR,
        frequency=FREQ,
        seed=SEED,
    )
    print("q rmse deg: kalman", np.degrees(rmse(q_kal, q_true)), "alone", np.degrees(rmse(q_alone, q_true)))
    print(
        "qdot rmse deg/s: kalman",
        np.degrees(rmse(qdot_kal, qdot_true)),
        "alone(fd)",
        np.degrees(rmse(qdot_alone, qdot_true)),
    )
    print(
        "q rmse after frame 20: kalman",
        np.degrees(rmse(q_kal[20:], q_true[20:])),
        "alone",
        np.degrees(rmse(q_alone[20:], q_true[20:])),
    )


if __name__ == "__main__":
    main()
