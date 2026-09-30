# CLAUDE.md — biorbd architecture map

biorbd is a C++ library for rigid-body dynamics and musculoskeletal biomechanics, built on RBDL
(Eigen backend) or rbdl-casadi (CasADi backend), with SWIG bindings for Python (plus MATLAB and C).
Version: `project(biorbd VERSION 1.12.3)` in `CMakeLists.txt` (C++11). Read `README.md` for install
and the `.bioMod` format; this file only tells you where things are. No `AGENTS.md` exists.

## Layout

| Path | Content |
|---|---|
| `include/`, `src/` | C++ library; the two trees mirror each other |
| `binding/` | SWIG (`*.i.in`), `python3/`, `matlab/`, `c/` |
| `test/` | gtest C++ tests, `binding/{c,python3}` tests, models in `test/models/*.bioMod` |
| `examples/` | C++ (`*Example.cpp`), `python3/`, `matlab/`, `arm26.bioMod`, `pyomecaman.bioMod`, `cube.bioMod` |
| `modules/` | CMake `FindOrBuild{Eigen3,Casadi,RBDL,TinyXML2}` and `FindIPOPT` |
| `doc/` | `contributing.md`, `release_checklist.md`, Doxygen config (no Sphinx) |
| `.github/workflows/` | CI |
| `environment_{eigen,casadi}.yml` | conda dev environments |

## C++ core (`include/`, `src/`)

Everything is in namespace `BIORBD_NAMESPACE`, defined in `include/biorbdConfig.h.in` as
`Biorbd@MATH_LIBRARY_BACKEND@`, i.e. `BiorbdEigen3` or `BiorbdCasadi`. Sub-namespaces:
`utils`, `rigidbody`, `internal_forces::{muscles, actuator, ligaments, passive_torques}`.
Umbrella header: `include/biorbd.h`.

- `include/BiorbdModel.h`: `class Model`, inherits `rigidbody::Joints`, `Markers`, `IMUs`,
  `RotoTransNodes`, `Contacts`, `SoftContacts` and, depending on the module flags, `Actuators`,
  `Muscles`, `PassiveTorques`, `Ligaments`. `Joints` derives from `RigidBodyDynamics::Model`.
- `include/ModelReader.h`, `src/ModelReader.cpp`: `Reader::readModelFile`, parses `.bioMod`
  (tag comparison is case-insensitive, unknown tags are silently skipped).
- `include/ModelWriter.h`: `Writer::writeModel(Model&, const utils::Path&)`.
- `include/RigidBody/`: `Joints`, `Segment`, `SegmentCharacteristics`, `Markers`, `NodeSegment`,
  `IMU`/`IMUs`, `Contacts`, `SoftContacts`/`SoftContactSphere`, `ExternalForceSet`,
  `Generalized{Coordinates,Velocity,Acceleration,Torque}`, `KalmanRecons*`, `Mesh`.
- `include/Utils/`: `Scalar`, `Vector`, `Matrix`, `Rotation`, `RotoTrans`, `Quaternion`,
  `SpatialVector`, `Error`, `Path`, `String`, `Equation`.
- `include/InternalForces/`: `Muscles/`, `Ligaments/`, `PassiveTorques/`, `Actuators/`, plus
  `ViaPoint`, `WrappingHalfCylinder`, `WrappingSphere`, `Geometry`, `PathModifiers`.

## Where is X

| Need | Look in |
|---|---|
| Load a model | `Model(const utils::Path&)` -> `Reader::readModelFile` (`src/ModelReader.cpp`) |
| Write a model | `Writer::writeModel` (`src/ModelWriter.cpp`) |
| Update kinematics | `Joints::UpdateKinematicsCustom` (`include/RigidBody/Joints.h`) |
| Marker positions | `Markers::markers` / `marker` (`include/RigidBody/Markers.h`) |
| Segment frames | `Joints::globalJCS`, `allGlobalJCS`, `localJCS` |
| Jacobians | `Markers::markersJacobian`, `Joints::CoMJacobian`, `Joints::projectPointJacobian` |
| Centre of mass, angular momentum | `Joints::CoM`, `CoMdot`, `CalcAngularMomentum`, `angularMomentum` |
| Mass matrix | `Joints::massMatrix`, `massMatrixInverse` |
| Inverse dynamics | `Joints::InverseDynamics` |
| Coriolis + gravity terms | `Joints::NonLinearEffect` (no separate Coriolis function) |
| Forward dynamics | `Joints::ForwardDynamics`; with contacts `ForwardDynamicsConstraintsDirect`; free base `ForwardDynamicsFreeFloatingBase` |
| Muscles | `internal_forces::muscles::Muscles` (`muscularJointTorque`, `stateSet`, `muscleForces`); models in `Muscles/Hill*Type.h`, states in `Muscles/State*.h` |
| Ligaments, passive torques, actuators | `include/InternalForces/{Ligaments,PassiveTorques,Actuators}/` |
| Rigid contacts | `rigidbody::Contacts` (`AddConstraint`, `AddLoopConstraint`) |
| Soft contacts | `rigidbody::SoftContacts` |
| External forces | `rigidbody::ExternalForceSet` (`Model::externalForceSet`) |
| IMU | `rigidbody::IMUs` (`IMU`, `IMUJacobian`) |
| Kalman filters | `KalmanReconsMarkers`, `KalmanReconsIMU` (`MODULE_KALMAN`) |
| Static optimisation | `StaticOptimization` (`MODULE_STATIC_OPTIM`, needs Ipopt) |
| Inverse kinematics | C++ `Markers::inverseKinematics` (Eigen only); Python `InverseKinematics` in `binding/python3/rigid_body.py` |
| Quaternion time step | `utils::Quaternion::timeStep` |

Not in the core: no Runge-Kutta or integration code (examples leave integration to the user).

## Main flow

`.bioMod` -> `Reader::readModelFile` fills a `Model` -> set `Q`/`Qdot` (`Generalized*`) ->
kinematics (`UpdateKinematicsCustom`, called by most accessors) -> markers, Jacobians, dynamics.
Muscle torques: `Muscles::muscularJointTorque` gives a `GeneralizedTorque` that goes to `ForwardDynamics`.

## Backends

`MATH_LIBRARY_BACKEND` (`Eigen3` default, or `Casadi`) in `CMakeLists.txt` defines
`BIORBD_USE_EIGEN3_MATH` or `BIORBD_USE_CASADI_MATH` and the target name (`biorbd_eigen`,
`biorbd_casadi`). Under CasADi `utils::Scalar` wraps `casadi::MX`. `biorbdConfig.h.in` also holds
the `DECLARE_*` / `CALL_BIORBD_FUNCTION_*` macros used by tests to run on both backends.

## C++/Python binding

SWIG, not pybind. `binding/biorbd*.i.in` are templates configured by `binding/CMakeLists.txt`.
`binding/python3/__init__.py` exports the raw SWIG API (`biorbd.Model`, camelCase) and, for
Python >= 3.12 only, the wrapper in `binding/python3/wrapper/`: `Biorbd` (`biorbd_model.py`),
`Muscle`, `Marker`, `Segment`, `SegmentFrame`, `ExternalForceSet`, `ExtendedKalmanFilterMarkers`,
`StaticOptimization` (snake_case: `nb_q`, `markers`, `muscles.joint_torque`, `forward_dynamics`).
With the CasADi backend, `to_casadi_func` (in `__init__.py`) turns a call on MX arguments into a
`casadi.Function`; the Python package is `biorbd_casadi`. MATLAB: `binding/matlab/biorbd.h`
(`biorbd('cmd', ...)`); C: `binding/c/`. Both refuse the CasADi backend.

## Tests and examples

- C++: `test/test_*.cpp` (gtest from the `external/googletest` submodule), built with
  `-DBUILD_TESTS=ON`; binary `biorbd_eigen_tests` / `biorbd_casadi_tests` in `build/test`.
- Python: `pytest` in `build/test/binding/python3` (sources in `test/binding/python3/`,
  wrapper tests in `tests_wrapper/`); `test_examples.py` runs the Python examples.
- Models: `test/models/*.bioMod`, `examples/*.bioMod`.
- Examples: `examples/*.cpp`, `examples/python3/*.py`, `examples/matlab/*.m`.

## Build and CI

CMake options are listed in `README.md` (`MODULE_*`, `BINDER_*`, `BUILD_TESTS`, `SKIP_ASSERT`).
Workflows: `run_eigen_tests.yml`, `run_casadi_tests.yml`, `run_pip_tests.yml`,
`run_codecoverage.yml`, `publish_eigen_to_pypi.yml`, `publish_matlab_eigen_binaries.yml`
(`publish_casadi_to_pypi.yml` is commented out). Pip builds: `setup.py` + `pyproject.toml`
(scikit-build, Python >= 3.10).

## Conventions

- `.clang-format`: Google style, 2-space indent, 80 columns. Methods are camelCase but RBDL-derived
  ones are PascalCase (`InverseDynamics`); members use `m_` and are mostly `std::shared_ptr`;
  classes have `DeepCopy()`.
- Errors: `utils::Error::raise` / `check`. Size checks sit behind `#ifndef SKIP_ASSERT`.
- `doc/contributing.md`: add a unit test with numerical values, small commits, no binaries in
  history. Its "Testing" section still says `biorbd_tests` (stale).

## Known limitations (verified in code)

- With CasADi: no Kalman, no static optimisation, no C/MATLAB binders, no `Markers::inverseKinematics`;
  SWIG must be < 4.4.0.
- Python wrapper needs Python >= 3.12; under CasADi some accessors (`Marker.world`, ...) raise.
- `.bioMod`: `maxVelocity`, `wrappingside` and the soft-contact `muStatic`/`muDynamic`/`muViscous`
  tags are not read (see `README.md`); multiple wrapping objects per muscle are not implemented.
- `BUILD_SHARED_LIBS` is forced to static on Windows.
