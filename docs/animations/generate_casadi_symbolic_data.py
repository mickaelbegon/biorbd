"""Data for the video 18 (casadi_symbolic): a symbolic mass matrix from the CasADi backend of biorbd.

Run with the biorbd_anim environment:  python generate_casadi_symbolic_data.py
Needs both the default build (``biorbd``, Eigen backend) and the CasADi build (``biorbd_casadi``).
The lines between the CODE-BEGIN and CODE-END markers are displayed as is in the code panel (rule D5).
"""

from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
path = str(HERE / "data" / "casadi_symbolic_pendulum.bioMod")
q0 = np.array([0.3, 0.8])

# fmt: off
# CODE-BEGIN
import biorbd
import biorbd_casadi as bc
from casadi import MX, SX
m = bc.Model(path)
q = MX.sym("q", 2)
Mf = bc.to_casadi_func("M", m.massMatrix, q)
M01 = Mf.expand()(SX.sym("q", 2))[0, 1]
M_cas = np.array(Mf(q0))
M_eig = biorbd.Biorbd(path).mass_matrix(q0)
# CODE-END
# fmt: on

assert biorbd.backend == biorbd.EIGEN3 and bc.backend == bc.CASADI
diff = np.abs(M_cas - M_eig)
print("M_cas =", M_cas, "\nM_eig =", M_eig, "\nmax diff =", diff.max(), "\nfunc =", str(Mf), "\nM_01 =", str(M01))
np.savez(
    HERE / "data" / "casadi_symbolic_main.npz",
    q0=q0,
    M_cas=M_cas,
    M_eig=M_eig,
    max_diff=diff.max(),
    func_repr=str(Mf),
    m01_expr=str(M01),
    eigen_backend=biorbd.backend == biorbd.EIGEN3,
    casadi_backend=bc.backend == bc.CASADI,
)
