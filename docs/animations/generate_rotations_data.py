"""Data for video 04 (rotations): the same three Euler angles in two sequences, matrix, quaternion, and back.

Run with the biorbd_anim environment. The lines between the ``# code:`` markers are the ones shown
in the code panel of the video (rule D5); everything else only stores the results.
"""

from pathlib import Path

import biorbd
import numpy as np

ANIM = Path(__file__).resolve().parent

# code:start
Rot = biorbd.Rotation
a = np.array([0.5, 0.3, 0.8])
r_xyz = Rot.fromEulerAngles(a, "xyz")
r_zyx = Rot.fromEulerAngles(a, "zyx")
# code:end
m_xyz, m_zyx = r_xyz.to_array(), r_zyx.to_array()

# code:start
Q = biorbd.Quaternion
q = Q.fromMatrix(r_xyz)
back = Rot.toEulerAngles(r_xyz, "xyz")
# code:end
quat = np.array([q.w(), q.x(), q.y(), q.z()])
back = back.to_array()
back_zyx = Rot.toEulerAngles(r_zyx, "zyx").to_array()
m_from_quat = q.toMatrix().to_array()


# independent checks (not shown): the sequence "xyz" is R = Rx(a0) Ry(a1) Rz(a2)
def elementary(axis, t):
    c, s = np.cos(t), np.sin(t)
    return {
        "x": np.array([[1, 0, 0], [0, c, -s], [0, s, c]]),
        "y": np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]),
        "z": np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]]),
    }[axis]


assert np.allclose(m_xyz, elementary("x", a[0]) @ elementary("y", a[1]) @ elementary("z", a[2]))
assert np.allclose(m_zyx, elementary("z", a[0]) @ elementary("y", a[1]) @ elementary("x", a[2]))
assert np.allclose(back, a) and np.allclose(back_zyx, a) and np.allclose(m_from_quat, m_xyz)
assert np.isclose(np.linalg.norm(quat), 1.0)

np.savez(
    ANIM / "data" / "rotations_main.npz",
    angles=a,
    m_xyz=m_xyz,
    m_zyx=m_zyx,
    quat_xyz=quat,
    back_xyz=back,
    back_zyx=back_zyx,
    back_error=float(np.max(np.abs(back - a))),
    matrix_diff=float(np.max(np.abs(m_xyz - m_zyx))),
    quat_matrix_error=float(np.max(np.abs(m_from_quat - m_xyz))),
)
print(
    "xyz", m_xyz.round(3), "zyx", m_zyx.round(3), "quat", quat, "back", back, "maxdiff", np.max(np.abs(m_xyz - m_zyx))
)
