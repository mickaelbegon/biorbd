"""Data for the video "static-optimization": muscle redundancy and the minimum-activation solution on arm26.

The motion is SYNTHETIC (two sinusoids), its joint torques come from biorbd inverse dynamics, then
biorbd's StaticOptimization finds the activations. Alternative activation sets that give the same joint torque are
built by adding null-space activations of the (linear) muscle torque map, computed with biorbd. Run with the
``biorbd_anim`` Python.
"""

from pathlib import Path

import biorbd
import numpy as np
from biorbd.biorbd import GeneralizedCoordinates, GeneralizedTorque, GeneralizedVelocity, StaticOptimization

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT.parents[1] / "examples" / "arm26.bioMod"
OUT = ROOT / "data" / "static_optimization_main.npz"
N_FRAMES = 101
DEMO_FRAME = 50
A_MIN, A_MAX = 1e-4, 0.9999  # activation bounds used by biorbd's optimizer
SEED = 17


def synthetic_motion(t):
    """Two sinusoids for (shoulder, elbow): q, qdot, qddot."""
    w1, w2 = 2 * np.pi, 3 * np.pi
    q = np.stack([0.35 + 0.4 * np.sin(w1 * t), 1.1 + 0.4 * np.sin(w2 * t + 1)], axis=1)
    qdot = np.stack([0.4 * w1 * np.cos(w1 * t), 0.4 * w2 * np.cos(w2 * t + 1)], axis=1)
    qddot = np.stack([-0.4 * w1**2 * np.sin(w1 * t), -0.4 * w2**2 * np.sin(w2 * t + 1)], axis=1)
    return q, qdot, qddot


def static_optimization(model, q_traj, qdot_traj, tau_traj):
    # CODE-BEGIN
    qs = [GeneralizedCoordinates(q) for q in q_traj]
    qdots = [GeneralizedVelocity(v) for v in qdot_traj]
    taus = [GeneralizedTorque(t) for t in tau_traj]
    so = StaticOptimization(model.internal, qs, qdots, taus)
    so.run(False)
    acts = [a.to_array() for a in so.finalSolution()]
    # CODE-END
    return np.array(acts)


def muscle_torques(model, acts, q_traj, qdot_traj):
    # CODE-BEGIN-CHECK
    frames = zip(acts, q_traj, qdot_traj)
    tau_m = [model.muscles.joint_torque(*f) for f in frames]
    # CODE-END-CHECK
    return np.array(tau_m)


def main():
    model = biorbd.Biorbd(str(MODEL_PATH))
    t = np.linspace(0.0, 1.0, N_FRAMES)
    q, qdot, qddot = synthetic_motion(t)
    tau = np.array([model.inverse_dynamics(q[i], qdot[i], qddot[i]) for i in range(N_FRAMES)])
    activations = static_optimization(model, q, qdot, tau)
    tau_muscles = muscle_torques(model, activations, q, qdot)
    diff = tau_muscles - tau

    # --- redundancy at one frame: the muscle torque is linear in the activations (passive part + M a) -----------
    k = DEMO_FRAME
    zero = model.muscles.joint_torque(np.zeros(6), q[k], qdot[k])
    mat = np.stack([model.muscles.joint_torque(np.eye(6)[i], q[k], qdot[k]) - zero for i in range(6)], axis=1)
    rng = np.random.default_rng(SEED)
    alternatives = []
    while len(alternatives) < 2:
        a = rng.uniform(0.05, 0.8, 6)
        a = a + np.linalg.pinv(mat) @ (tau[k] - zero - mat @ a)  # project onto {M a = tau - passive}
        if a.min() > A_MIN and a.max() < 0.9 and (a @ a) > 1.3 * (activations[k] @ activations[k]):
            alternatives.append(a)
    sets = np.vstack([alternatives[0], alternatives[1], activations[k]])
    set_tau = np.array([model.muscles.joint_torque(a, q[k], qdot[k]) for a in sets])
    set_cost = np.sum(sets**2, axis=1)
    nullity = 6 - np.linalg.matrix_rank(mat)

    np.savez(
        OUT,
        t=t,
        q=q,
        tau=tau,
        activations=activations,
        tau_muscles=tau_muscles,
        diff=diff,
        max_abs_diff=np.abs(diff).max(),
        rms_diff=np.sqrt(np.mean(diff**2)),
        cost=np.sum(activations**2, axis=1),
        muscle_names=np.array([m.name for m in model.muscles]),
        demo_frame=k,
        demo_time=t[k],
        demo_tau=tau[k],
        set_activations=sets,
        set_tau=set_tau,
        set_cost=set_cost,
        nb_muscles=6,
        nb_dof=model.nb_q,
        nullity=nullity,
        bound_min=A_MIN,
    )
    print("max |diff| Nm", np.abs(diff).max(), "rms", np.sqrt(np.mean(diff**2)), "nullity", nullity)
    print("demo tau", tau[k], "set_tau", set_tau, "set_cost", set_cost)
    print("activations min/max", activations.min(), activations.max(), "names", [m.name for m in model.muscles])
    print("sets", sets.round(3))


if __name__ == "__main__":
    main()
