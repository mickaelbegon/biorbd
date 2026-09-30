"""Data for the video "forward-dynamics": forward dynamics of a passive double pendulum, integrated by hand.

biorbd has no integrator in its core: ``forward_dynamics`` returns qddot and the two schemes below (explicit Euler
and Runge-Kutta 4, both hand-written) are ours. The quality of the integrator shows in the drift of the total
energy of a passive system (tau = 0, no damping). Run with the ``biorbd_anim`` Python.
"""

from pathlib import Path

import biorbd
import numpy as np

ROOT = Path(__file__).resolve().parent
MODEL_PATH = ROOT.parents[1] / "test" / "models" / "pendulum.bioMod"
OUT = ROOT / "data" / "forward_dynamics_main.npz"
DT = 0.01  # s, the same time step for both schemes
DURATION = 2.0  # s
X0 = np.array([0.0, 0.3, 0.2, 0.0, 0.0, 0.0])  # q (3) then qdot (3): released at rest


def make_dynamics(model):
    # CODE1-BEGIN
    tau = np.zeros(model.nb_tau)

    def f(x):
        q, v = x[:3], x[3:]
        a = model.forward_dynamics(q, v, tau)
        return np.concatenate([v, a])

    def energy(x):
        q, v = x[:3], x[3:]
        return model.internal.TotalEnergy(q, v)

    # CODE1-END
    return f, energy


def make_steps(f):
    # CODE2-BEGIN
    def euler(x, dt):
        return x + dt * f(x)

    def rk4(x, dt):
        k1 = f(x)
        k2 = f(x + dt / 2 * k1)
        k3 = f(x + dt / 2 * k2)
        k4 = f(x + dt * k3)
        return x + dt / 6 * (k1 + 2 * (k2 + k3) + k4)

    # CODE2-END
    return euler, rk4


def simulate(step, energy, n):
    x = X0.copy()
    xs, es = [x.copy()], [energy(x)]
    for _ in range(n):
        x = step(x, DT)
        xs.append(x.copy())
        es.append(energy(x))
    return np.array(xs), np.array(es)


def main():
    model = biorbd.Biorbd(str(MODEL_PATH))
    f, energy = make_dynamics(model)
    euler, rk4 = make_steps(f)
    n = int(round(DURATION / DT))
    x_eu, e_eu = simulate(euler, energy, n)
    x_rk, e_rk = simulate(rk4, energy, n)
    e0 = energy(X0)
    drift_eu = 100.0 * (e_eu - e0) / abs(e0)
    drift_rk = 100.0 * (e_rk - e0) / abs(e0)
    qddot0 = f(X0)[3:]
    # cross-check: kinetic energy 1/2 qdot' M qdot from the mass matrix equals the internal KineticEnergy
    qd_test = np.array([0.0, 1.0, 0.5])
    ke_mass = 0.5 * qd_test @ model.mass_matrix(X0[:3]) @ qd_test
    ke_internal = model.internal.KineticEnergy(X0[:3], qd_test)
    np.savez(
        OUT,
        t=np.arange(n + 1) * DT,
        dt=DT,
        x_euler=x_eu,
        x_rk4=x_rk,
        e_euler=e_eu,
        e_rk4=e_rk,
        e0=e0,
        drift_euler_pct=drift_eu,
        drift_rk4_pct=drift_rk,
        qddot0=qddot0,
        nb_q=model.nb_q,
        gravity=model.gravity,
        ke_mass_matrix=ke_mass,
        ke_internal=ke_internal,
    )
    print("E0", e0, "qddot0", qddot0, "KE check", ke_mass, ke_internal)
    print("max |drift| % euler", np.abs(drift_eu).max(), "rk4", np.abs(drift_rk).max(), "final euler", drift_eu[-1])


if __name__ == "__main__":
    main()
