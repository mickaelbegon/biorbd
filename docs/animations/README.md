# biorbd video series

Short Manim videos (about 35 s each, English and French) that explain how biorbd works. Every number on screen comes from a
real biorbd 1.12.3 computation; synthetic data and hand-written algorithms are labelled as such in the videos.
Français : [README.fr.md](README.fr.md).

The videos themselves are not stored in the repository (they are rendered locally, see below).

## Start here

1. **Models**: 01 anatomy of a `.bioMod`, then 02 loading a model.
2. **Kinematics**: 03 forward kinematics, 04 rotations, 05 Jacobians.
3. **Inverse kinematics**: 15 by hand, then 14 with `biorbd.InverseKinematics`, then 16 with the Kalman filter.
4. **Mass and dynamics**: 06 centre of mass, 07 mass matrix, 08 nonlinear effects, 09 inverse dynamics, 10 forward dynamics.
5. **Muscles**: 11 muscles, then 17 static optimization.
6. **Contacts, sensors, CasADi**: 12 contacts, 13 IMU, 18 CasADi.

## The videos

<!-- TABLE:BEGIN -->
| # | Video | What it shows | Level | Scene file | Code examples |
|---|---|---|---|---|---|
| 01 | **Anatomy of a .bioMod file** | A .bioMod file is plain text: segment tags give the coordinates q, the mass and the centre of mass, marker tags give points fixed in a segment, and biorbd reads them back. | 1 (basics) | [`anim_biomod_anatomy.py`](anim_biomod_anatomy.py) | [`README.md`](../../README.md), [`README.md`](../../README.md), [`pyomecaman.bioMod`](../../examples/pyomecaman.bioMod), [`ModelReader.cpp`](../../src/ModelReader.cpp), [`ModelReader.cpp`](../../src/ModelReader.cpp) |
| 02 | **Loading a model** | biorbd.Biorbd(path) reads a .bioMod file and returns a model object whose degrees of freedom, segments, markers and muscles can be listed. | 1 (basics) | [`anim_model_loading.py`](anim_model_loading.py) | [`forward_dynamics_from_muscles.py`](../../examples/python3/forward_dynamics_from_muscles.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`arm26.bioMod`](../../examples/arm26.bioMod) |
| 03 | **Forward kinematics** | From generalized coordinates q, biorbd chains the segment frames of a leg and gives the world position of a marker. | 1 (basics) | [`anim_forward_kinematics.py`](anim_forward_kinematics.py) | [`forward_kinematics.py`](../../examples/python3/forward_kinematics.py), [`marker.py`](../../binding/python3/wrapper/marker.py), [`segment_frame.py`](../../binding/python3/wrapper/segment_frame.py), [`Joints.h`](../../include/RigidBody/Joints.h) |
| 04 | **Rotations** | The same three Euler angles give different rotation matrices in the xyz and zyx sequences; biorbd converts between angles, matrices and quaternions. | 2 | [`anim_rotations.py`](anim_rotations.py) | [`Rotation.h`](../../include/Utils/Rotation.h), [`Rotation.h`](../../include/Utils/Rotation.h), [`Quaternion.h`](../../include/Utils/Quaternion.h), [`segment_frame.py`](../../binding/python3/wrapper/segment_frame.py), [`manipulating_frames_of_reference.py`](../../examples/python3/manipulating_frames_of_reference.py) |
| 05 | **Marker Jacobians** | The Jacobian of a marker maps generalized velocities to the marker velocity in the world frame, v = J(q) qdot. The video shows the small matrix of a forearm marker of the arm model and checks the product against finite differences of marker positions along a short trajectory. | 2 | [`anim_jacobians.py`](anim_jacobians.py) | [`marker.py`](../../binding/python3/wrapper/marker.py), [`Markers.h`](../../include/RigidBody/Markers.h), [`test_wrapper_markers.py`](../../test/binding/python3/tests_wrapper/test_wrapper_markers.py), [`forward_kinematics.py`](../../examples/python3/forward_kinematics.py) |
| 06 | **Centre of mass** | The centre of mass of the whole body is the mass-weighted average of the segment centres of mass, and it moves when the posture q changes. | 1 (basics) | [`anim_center_of_mass.py`](anim_center_of_mass.py) | [`manipulation_model.py`](../../examples/python3/manipulation_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`segment.py`](../../binding/python3/wrapper/segment.py), [`Joints.h`](../../include/RigidBody/Joints.h) |
| 07 | **The mass matrix** | The joint-space mass matrix M(q) gives the inertial part of the joint torques, M(q) times the accelerations. It is symmetric and positive definite, and it depends on the posture. The video computes M for the arm26 model at two elbow angles, checks the symmetry and the smallest eigenvalue, and shows how the matrix changes between the two postures. | 2 | [`anim_mass_matrix.py`](anim_mass_matrix.py) | [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h), [`test_wrapper_model.py`](../../test/binding/python3/tests_wrapper/test_wrapper_model.py), [`arm26.bioMod`](../../examples/arm26.bioMod) |
| 08 | **Nonlinear effects** | model.non_linear_effect(q, qdot) returns the generalized forces that cancel gravity and the velocity-dependent (Coriolis and centrifugal) terms. At zero velocity it is pure gravity; the velocity part grows with the square of the velocity. The video uses the pyomecaman model in a hand-chosen posture: the vertical pelvis force equals mass times 9.81, and doubling qdot multiplies the velocity part by exactly four. biorbd has no separate Coriolis function. | 2 | [`anim_nonlinear_effects.py`](anim_nonlinear_effects.py) | [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h), [`Joints.cpp`](../../src/RigidBody/Joints.cpp), [`test_wrapper_model.py`](../../test/binding/python3/tests_wrapper/test_wrapper_model.py) |
| 09 | **Inverse dynamics** | Inverse dynamics gives the joint torques that produce a given motion (q, qdot, qddot). The video computes them with model.inverse_dynamics on the arm model and checks that they equal M(q) qddot + N(q, qdot), using the mass matrix and the nonlinear effects of biorbd. | 2 | [`anim_inverse_dynamics.py`](anim_inverse_dynamics.py) | [`inverse_dynamics.py`](../../examples/python3/inverse_dynamics.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py) |
| 10 | **Forward dynamics and integration** | model.forward_dynamics returns the accelerations qddot from q, qdot and tau. biorbd has no integrator in its core, so a hand-written Euler and a hand-written RK4 are compared on a passive double pendulum through the drift of its total energy. | 2 | [`anim_forward_dynamics.py`](anim_forward_dynamics.py) | [`forward_dynamics.py`](../../examples/python3/forward_dynamics.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h), [`pendulum.bioMod`](../../test/models/pendulum.bioMod) |
| 11 | **Muscles: length, moment arm and torque** | The length of a muscle depends on the joint angles, and the gradient of that length with respect to the coordinates is the length Jacobian. The joint torque produced by the muscles is minus the transposed length Jacobian times the muscle forces. The video sweeps the elbow angle of the arm model, shows the length and the moment arm of the long head of the biceps, and compares the torque returned by muscles.joint_torque with minus J transposed times F computed by hand. | 2 | [`anim_muscles.py`](anim_muscles.py) | [`manipulating_muscles.py`](../../examples/python3/manipulating_muscles.py), [`muscle.py`](../../binding/python3/wrapper/muscle.py), [`muscle.py`](../../binding/python3/wrapper/muscle.py), [`Muscles.h`](../../include/InternalForces/Muscles/Muscles.h), [`test_wrapper_muscles.py`](../../test/binding/python3/tests_wrapper/test_wrapper_muscles.py) |
| 12 | **Rigid contacts** | A rigid contact declared in the bioMod file forbids the motion of its point along the chosen axes. The video runs the forward dynamics of a free cube with two contacts, first ignoring the contacts (free fall), then with them: the generalized accelerations vanish, the acceleration of the contact points is zero to numerical precision and biorbd returns the contact forces, whose vertical components add up to the weight. | 3 (advanced) | [`anim_contacts.py`](anim_contacts.py) | [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Contacts.h`](../../include/RigidBody/Contacts.h), [`cubeWithRigidContactsExternalForces.bioMod`](../../test/models/cubeWithRigidContactsExternalForces.bioMod), [`test_rigidbody.cpp`](../../test/test_rigidbody.cpp) |
| 13 | **Inertial measurement unit (IMU)** | An IMU declared in the .bioMod is a frame fixed in a segment: biorbd gives its orientation in the world from q. | 3 (advanced) | [`anim_imu.py`](anim_imu.py) | [`pyomecaman_withIMUs.bioMod`](../../test/models/IMUandCustomRT/pyomecaman_withIMUs.bioMod), [`IMUs.h`](../../include/RigidBody/IMUs.h), [`IMUs.cpp`](../../src/RigidBody/IMUs.cpp), [`test_conversion.py`](../../test/binding/python3/test_conversion.py) |
| 14 | **Inverse kinematics by least squares** | biorbd.InverseKinematics solves each frame with a scipy least-squares solver (lm, trf or only_lm), skips markers that are NaN and, with trf, respects the joint ranges. Shown on synthetic noisy markers of a 13-DoF model, with the error measured against the known truth. | 2 | [`anim_ik_least_squares.py`](anim_ik_least_squares.py) | [`rigid_body.py`](../../binding/python3/rigid_body.py), [`rigid_body.py`](../../binding/python3/rigid_body.py), [`utils.py`](../../binding/python3/utils.py), [`test_inverse_kinematics.py`](../../test/binding/python3/test_inverse_kinematics.py), [`pyomecaman.bioMod`](../../examples/pyomecaman.bioMod) |
| 15 | **Inverse kinematics by hand** | A hand-written Gauss-Newton loop recovers the joint angles of a two-degree-of-freedom arm from noisy synthetic markers, using the marker Jacobian of biorbd. | 2 | [`anim_ik_by_hand.py`](anim_ik_by_hand.py) | [`marker.py`](../../binding/python3/wrapper/marker.py), [`forward_kinematics.py`](../../examples/python3/forward_kinematics.py), [`rigid_body.py`](../../binding/python3/rigid_body.py), [`test_inverse_kinematics.py`](../../test/binding/python3/test_inverse_kinematics.py) |
| 16 | **Kalman filter** | The extended Kalman filter of biorbd reconstructs the joint angles frame by frame from noisy synthetic markers and is compared with solving each frame alone. | 3 (advanced) | [`anim_kalman_filter.py`](anim_kalman_filter.py) | [`extended_kalman_filter.py`](../../binding/python3/wrapper/extended_kalman_filter.py), [`KalmanRecons.h`](../../include/RigidBody/KalmanRecons.h), [`inverseKinematicsKalmanExample.cpp`](../../examples/inverseKinematicsKalmanExample.cpp), [`test_wrapper_kalman_filter.py`](../../test/binding/python3/tests_wrapper/test_wrapper_kalman_filter.py) |
| 17 | **Static optimization** | Six muscles act on two joints, so many activation sets give the same joint torque. Static optimization keeps the one with the smallest sum of squared activations, and the muscle joint torque reproduces the inverse-dynamics target. | 3 (advanced) | [`anim_static_optimization.py`](anim_static_optimization.py) | [`static_optimization.py`](../../binding/python3/wrapper/static_optimization.py), [`static_optimization.py`](../../examples/python3/static_optimization.py), [`muscle.py`](../../binding/python3/wrapper/muscle.py), [`test_wrapper_muscles.py`](../../test/binding/python3/tests_wrapper/test_wrapper_muscles.py), [`StaticOptimization.cpp`](../../src/InternalForces/Muscles/StaticOptimization.cpp) |
| 18 | **Symbolic functions with the CasADi build** | The CasADi build of biorbd (biorbd_casadi, installed separately from the default Eigen build) turns any model quantity into a symbolic CasADi function with to_casadi_func. The video builds the mass matrix of a two-link pendulum as a function of q, shows one symbolic entry, and compares its value at one pose with the mass matrix of the default build. | 3 (advanced) | [`anim_casadi_symbolic.py`](anim_casadi_symbolic.py) | [`__init__.py`](../../binding/python3/__init__.py), [`test_binder_python_rigidbody.py`](../../test/binding/python3/test_binder_python_rigidbody.py), [`biorbd_model.py`](../../binding/python3/wrapper/biorbd_model.py), [`Joints.h`](../../include/RigidBody/Joints.h) |
<!-- TABLE:END -->

The table is generated from [`catalog.json`](catalog.json) by `tools/make_readme_table.py` (do not edit it by hand).

## Prerequisites

- **Rendering**: Python 3.11 with `manim` 0.21 (Community Edition). Fonts used: Segoe UI and Consolas (Windows); another
  system needs these fonts or the fallback ones will change the layout.
- **Generating the data** (`generate_*_data.py`): a conda environment with biorbd 1.12.3 from conda-forge (Python 3.12; the
  high-level `biorbd.Biorbd` wrapper needs Python >= 3.12), numpy and scipy. The conda package provides both the Eigen
  backend (`biorbd`) and the CasADi backend (`biorbd_casadi`). Use OpenBLAS rather than MKL on Windows if `numpy.linalg`
  crashes.
- The committed `data/*.npz` files are enough to render without biorbd.
- The logo is downloaded, not committed: `python tools/fetch_logo.py`.

## Commands

```bash
python tools/fetch_logo.py                                  # once
python tools/render_series.py --list                        # list the scenes
python tools/render_series.py AnimJacobians --lang both --quality 480p15   # quick draft
python tools/render_series.py --all --lang both --jobs 3 --strict --collect
python tools/extract_frames.py media/AnimJacobians_fr/videos/anim_jacobians/1080p30/AnimJacobians.mp4
python tools/validate_catalog.py
python tools/make_readme_table.py --check
```

`--strict` fails on any frame-audit finding (text outside the frame, overlaps, collision with the logo), on a missing French
translation key or on a missing catalog entry. `--collect` copies the videos to `<Documents>/biorbd_animations/EN` and
`/FR` (change it with `--out` or `BIORBD_ANIM_OUT`). Rules for new videos: [STANDARD.md](STANDARD.md) and
`templates/scene_template.py`.

## Honest warnings

- The frame audit does not see everything (it runs between animations only): frames must still be looked at.
- Scenes use small, often planar models (mostly `arm26`, sometimes `pyomecaman`). The numbers illustrate the method; they are
  not a validation of biorbd on other models, 3D cases or real measurements.
- Video 17 bypasses the wrapper's `StaticOptimization.perform_frames`, which calls the linearized solver and did not match the
  target torque in our test; see `notes/static_optimization.md`.
- Some videos use a hand-written integrator or solver (10, 15) for teaching; biorbd does not contain an RK integrator.
- Each scene has a `notes/<topic>.md` file listing what was verified and what was not.
