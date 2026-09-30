# Video 13: IMU (AnimImu)

## API verified
- bioMod: `imu <name> / parent / rt <angles> xyz <translation> / endimu` in test/models/IMUandCustomRT/pyomecaman_withIMUs.bioMod lines 853-876 (read-only); 4 IMUs, the first is `rightHand` on `BrasD`.
- The high-level wrapper (binding/python3/wrapper/) has no IMU class (grep "imu" finds nothing there), so the video uses `model.internal` (SWIG `biorbd.Model`): `IMU()` (local rt, tuple of IMU), `IMU(q, idx)` (world), `IMU.to_array()` (4x4). Checked by running generate_imu_data.py in biorbd_anim (1.12.3).
- src/RigidBody/IMUs.cpp: `IMU(Q, idx)` = `globalJCS(Q, parent) * local`, which is what the generator checks numerically.
- `SegmentFrame.world` (4x4) of the wrapper is used for the segment world frame.

## Data
data/imu_main.npz. Posture A: q = 0; posture B: q[3:5] = [0.5, -1.2] (BrasD_RotZ, BrasD_RotX).
Check world IMU == segment world @ local rt: largest absolute difference 5.6e-17 (shown on screen, from the npz).
The drawing is a projection on the y-z plane (y right, z up) of the y and z axes only; the x axis is not drawn.

## Not verified / open
- Only the 'rightHand' IMU is shown; the other three were not displayed.
- At q = 0 the BrasD world rotation is the identity, so the world rotation equals the local one in posture A.
- Difference axis (A6) not drawn: the ghost is the previous posture, the matrix is replaced.
