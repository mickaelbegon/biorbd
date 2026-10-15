# Muscle wrapping: numerical 3-D trajectories

This implementation reproduces ten independently computed reference trajectories:
six analytical or symmetry-plane cases, a spatial ellipsoid geodesic, an oblique
torus geodesic, three tilted cylinders, and a spatial ellipsoid–cylinder–torus
chain. The examples use metres and rigid local frames. `muscleWrapping.py` checks
their native lengths and exports the computed 3-D coordinates. From a build with
Python bindings:

```sh
PYTHONPATH=build/eigen/binding/python3 python examples/python3/muscleWrapping.py --output wrapping.json
```

`three-cylinders-3d.bioMod` adds a non-coplanar chain with three independently
tilted cylinder axes. Its native length is 482.358666497 mm, compared with an
independent simultaneous solve of the entry/exit tangent directions and exact
helices. It is covered by the native and Python regression tests. The path's
RMS distance to its best-fit plane is 3.14 mm, so this case exercises spatial
coupling rather than a rigidly rotated planar chain.

## Model properties and frames

Add an object to a muscle or ligament using a `wrapping` block. The existing
`parent`, `RT` / `RTinMatrix`, `muscle` / `musclegroup`, and `ligament` fields
retain their meaning. `RT` maps the object frame into its parent segment frame.
Positions and dimensions use the model length unit, normally metres.

| `type` | Required dimensions | Object frame |
| --- | --- | --- |
| `cylinder` | `radius r`, `length h`, both positive | Axis z, lateral surface from −h/2 to +h/2 |
| `ellipsoid` | `radii a b c`, all positive | Semiaxes along x, y, z |
| `torus` | `majorradius R`, `minorradius r`, R > r > 0 | Ring in xy, axis z |

`wrappingSide 1` (default) selects the tangent branch with the greater second
planar coordinate at its midpoint; `-1` selects the opposite branch. For a
cylinder and an xy ellipsoid section this coordinate is y. For an xz/yz
ellipsoid section and a torus meridian it is z. For the outer torus equator it is
y. The branch is anatomically prescribed; its length need not be the global
shortest path.

For a general 3-D ellipsoid or torus path, the selected planar branch initializes
a continuation to the actual endpoints. Thus `wrappingSide` prescribes an initial
branch; it does not impose a global coordinate inequality along the entire arc.
Nearby calls may reuse a converged shooting seed, with all final checks repeated.

Objects and via points are traversed in their declaration order. A via point
fixes an anchor and divides the ordered objects into contact regions. All
associated solids still constrain the complete trajectory. Infeasible orders
and intersections raise an error. The example `.bioMod` files give complete
blocks for each shape.

## Hybrid solver

The default build retains the native solver and its existing dependencies. To
enable the optional spatial NGV backend, install Simbody 3.9 or newer and use
CMake 3.21 or newer:

```sh
cmake -S . -B build/eigen -DBIORBD_WITH_SIMBODY_WRAPPING=ON \
    -DSimbody_DIR=/path/to/simbody/lib/cmake/simbody
cmake --build build/eigen
```

The same option works with `-DMATH_LIBRARY_BACKEND=Casadi`. Simbody is a private
dependency of the internal-forces library; no Simbody types appear in the
installed biorbd headers. Its current imported targets require a compiler with
C++20 support. An explicitly enabled backend fails configuration if Simbody is
unavailable; with the option OFF, Simbody is not searched or linked.

Analytical cylinder helices, principal-plane ellipsoid sections, torus meridians
and equators, and free straight paths retain the native route. Spatial ellipsoid
and cold torus paths and coupled mixed spatial regions use Simbody's
`Scholz2015` algorithm when enabled. Cylinder-only chains retain the native
helical solver, including non-parallel axes; integrated benchmarks show that
Simbody setup costs outweigh its solve-time gain on these cases. Contact hints
are computed from the shape, its prescribed side and the current endpoints; reference coordinates are never supplied to the solver.

Every NGV proposal must pass surface and tangent checks, finite-cylinder bounds,
nonnegative contact reaction, exact free-chord collision checks and sampled
cross-object arc checks. A converged proposal is not sufficient on its own.
Proposals that fail these checks or the solver convergence check fall back to the
native solver, whose existing errors remain errors. `MinimumLength` is not used.

The native activation rule also applies to NGV: an object cannot retain contact
when the free chord between its neighbours is clear. In particular, a free
straight segment through the torus hole bypasses NGV. This avoids the retained
contact branch seen in the standalone Simbody motion benchmark. A switch of
contact branch is still piecewise smooth rather than globally differentiable.

NGV systems and states are local to each solve; there is no mutable shared
Simbody cache or growing solver registry. Native seeds remain value-owned and
are committed only after the complete trajectory is accepted. NGV evaluations
currently use cold initialization, including in CasADi callbacks. For a single
spatial torus with a caller-owned seed, a successful cold solve also provides
native shooting angles and normalized arc length. Subsequent nearby calls first
try the faster native warm geodesic, with the same refinement and physical checks.
A failed warm solve returns to the cold hybrid dispatch; the seed never substitutes
for validation. CasADi callbacks remain deterministic cold evaluations.

Hybrid dispatch applies to `Model` / `Geometry`; standalone calls to
`WrappingSurface::wrapPoints()` retain the native surface solver.
`Geometry` uses the same numerical path kernel for Eigen and CasADi. Eigen
publishes dense arc samples and active contacts only after successful global
validation. The shared stationary envelope derivatives retain the existing
length, velocity and force-to-torque conventions described below.

For an end-to-end timing comparison, run the following against each freshly
built Python binding, first without and then with the optional backend:

```sh
python examples/python3/muscleWrappingHybridBenchmark.py --backend eigen \
    --label native --output native-eigen.json
python examples/python3/muscleWrappingHybridBenchmark.py --backend eigen \
    --label hybrid --output hybrid-eigen.json
python examples/python3/muscleWrappingHybridBenchmark.py --backend casadi \
    --label hybrid --output hybrid-casadi.json
```

The script measures repeated model updates or CasADi `Function` evaluations
returning length, its Jacobian and velocity, and verifies the reference lengths
and `velocity = J * Qdot`. Model/graph creation is outside the timer. These times
include kinematics and Python output extraction, so they are not the standalone
geometric-kernel timings. Compare on the same machine without concurrent builds.

The measured integration comparison against commit `935de0ab` is recorded with
raw samples in [hybridBenchmarkResults.json](hybridBenchmarkResults.json). Each
row is the median of 75 evaluations at Q=0, in three serial blocks after two
untimed priming calls. Times are in milliseconds:

| Spatial fixture | Eigen native | Eigen hybrid | CasADi native | CasADi hybrid |
| --- | ---: | ---: | ---: | ---: |
| Ellipsoid | 1.4520 | 0.7342 | 7.5896 | 1.2243 |
| Torus | 1.5683 | 0.6774 | 11.1875 | 6.8361 |
| Three tilted cylinders | 6.4264 | 0.4361 | 0.3073 | 0.3023 |
| Ellipsoid + cylinder + torus | 46.8254 | 3.8633 | 68.5817 | 6.9057 |

These are Release builds on the same macOS arm64 host, collected on different
days without a reserved CPU. Eigen reuses native seeds; CasADi uses cold
callbacks. This comparison includes the shared-kernel integration and C++11 vs
C++20 build standards, so it does not isolate the external solver. Analytical
CasADi fixtures remain around 0.02–0.06 ms, with small changes in either direction;
small ratios should not be interpreted as reliable speedups. No motion-cycle
performance or global branch-optimality claim follows from these timings.

Run the Python regression suites from `test/binding/Python3` so legacy relative
model paths resolve. With the matching fresh package on `PYTHONPATH`, use
`python -m pytest -q` for Eigen and `python -m unittest test_wrapping_casadi -v`
for CasADi. The exact CasADi module avoids recursively importing unrelated Eigen
packages when both backend builds use different Python ABIs.

## Path and derivatives

The native solver handles each region by iterating the tangent contacts, using
the previous object's exit and the next object's entry. Inactive objects are reconsidered on
each sweep. Convergence requires stable activity and contact/arc-length changes
below `1e-10 * max(1, region_chord_length)` in model units, within 200 sweeps.
For several objects, admissible object-local symmetry-plane contacts provide
simultaneous initial guesses. Every contact is then re-solved using the actual
neighbouring contacts and anchors; projected guesses are never returned as a
trajectory. This avoids forcing the first object to solve an unnecessary
standalone branch across the entire region.
Free segments are checked analytically against each associated solid; sampled
surface arcs are checked against the other solids. The penetration threshold
is `1e-8` in normalized implicit surface level.

General ellipsoid and torus arcs use geodesic shooting in object coordinates
normalized by the largest semiaxis or the major radius. The unknowns are two
entry angles and log arc length; entry tangency and the outgoing unit tangent
provide the residuals. Bounded Levenberg–Marquardt iterations use 128 RK4 steps
for continuation, followed by 512- and 1024-step refinement. The final solver
checks surface level, unit speed, tangency, nonnegative normal contact reaction,
integration refinement, and exact free-segment penetration extrema. The final
defect threshold is `2e-9` in normalized coordinates; tangent fitting targets
`2e-11`. Sampled arc checks are numerical checks, not a proof of continuous
non-intersection or a global minimum. Failure raises an error.

`Geometry::pointsInGlobal()` contains anchors and active entry/exit contacts.
`Geometry::pathPointsInGlobal()` samples each new surface arc for Eigen, so it
can be plotted as a 3-D polyline. In the CasADi MX graph, both point methods
use a fixed two contact slots per wrapping object; inactive slots are duplicated
and arcs are not densely sampled. Legacy objects retain their original point
representation. Arc integration and the analytical cylinder helix determine
the native length; the plotted polyline is an approximation.

`MuscleGeometry::musculoTendonLength()` is the complete trajectory length LMT.
`MuscleGeometry::length()` remains the fibre length `(LMT - tendonSlackLength) /
cos(pennationAngle)`. The existing `jacobianLength()` and velocity convention
use LMT: `velocity = jacobianLength * Qdot` and muscle torque is
`-jacobianLength.transpose() * force`. A rotational moment arm is `-dLMT/dq`
(metres per radian). Translation derivatives are dimensionless when q uses
metres.

The length derivative uses stationarity of the tangent contact path: contact
sliding terms cancel, and a rigid surface arc has zero material length change.
Only the free-segment material point Jacobians contribute. The point Jacobian
at a contact describes its instantaneous fixed parent material point; it is
not the derivative of the sliding contact location. Length derivatives are
piecewise smooth on a fixed branch; activity/topology changes are not globally
differentiable. Failed solves invalidate geometry and velocity caches.

## Supported scope

- Eigen and CasADi MX use the same double-precision contact kernel. CasADi
  evaluates the complete input `Q` through a callback and returns the exact
  first derivative of the converged stationary length on its current contact
  branch. Higher-order derivatives are formed by centred finite differences of
  that gradient; changes of active set or topology can therefore be
  nonsmooth. The callback registry retains one immutable graph per distinct
  model/path definition for the process lifetime; numerical evaluations do not
  grow the registry. An exported CasADi `Function` remains usable after its
  source model is destroyed.
- CasADi support targets `Model` / `Geometry` length and derivative expressions.
  Direct symbolic use of `WrappingSurface::wrapPoints()` is not supported at
  this milestone. New shapes require `Q_size == dof_count`; quaternion
  configurations with a different size are rejected explicitly.
- Construct wrapping expressions from MX inputs; direct SX inputs to the model
  are not supported. CasADi 3.7 and newer can expand a `Function` containing
  these callbacks by retaining them as SX call nodes. This is tested callback
  expansion, not pure algebraic expansion or standalone code generation. With
  older CasADi versions, use `biorbd.to_casadi_func(..., expand=False)`.
- CasADi Function serialization/deserialization, JIT, and standalone C code
  generation are unsupported. The MX graph can be used directly in a CasADi
  `Function` or passed to the project helper.
- Cylinder: exact helical geodesic on the finite lateral surface. Rim and cap
  contacts are unsupported.
- Ellipsoid: principal symmetry-plane arcs and connected stationary 3-D
  geodesics reached by continuation. A bounded solve may reject a configuration
  whose prescribed branch cannot be followed from the available initialization.
- Ring torus: meridian tube arcs, exterior equator arcs, and connected stationary
  3-D geodesics reached by continuation. Passage through the hole remains straight
  when it has no contact. Multiple disconnected contacts on one torus are
  unsupported; branches requiring negative normal contact reaction are rejected.
- Several new objects, including mixed shapes, and via points. Coupled contact
  iteration has explicit convergence and feasibility checks; it is not a
  global optimization or a complete enumeration of possible contact orders.
- Existing `halfcylinder` behavior remains available. Combining legacy wrapping
  objects with the new solver is unsupported.

The existing model writer does not serialize muscles or wrapping objects;
these examples should be kept as `.bioMod` sources.

## Validation

Native and Python regressions compare independent lengths, contact geometry,
rigid frame invariance, copy isolation, multi-object/via paths, length derivatives
against centred finite differences, and failure cases. The latest reported runs
passed 177 Eigen C++ tests, 153 CasADi C++ tests, 177 C++ tests with Simbody
disabled, 78 Eigen Python tests (12 backend-specific skips), and 12 CasADi Python
tests, including callback/concurrency coverage. Four IPOPT examples also passed
with gradient finite-difference and limited-memory Hessians. CasADi Hessian
finite differences use a fixed Q step of `1e-6` in each coordinate’s model unit.
Hessian checks use an absolute tolerance of `2e-4`; symmetry checks use
`5e-5`. The six original reference lengths
are 349.210591272, 354.197814806, 144.705264266, 200.000000000, 396.450336454 and
499.027887126 mm. Unsupported cases are tested as errors, not approximated paths.

The reviewed general 3-D references use independent SciPy/DOP853 geodesic shooting,
tighter integration tolerances, and RK4 convergence checks. Their native reference
lengths are 358.918940251 mm (ellipsoid), 189.821234052 mm (oblique torus),
482.358666497 mm (three cylinders), and 499.920426804 mm (mixed chain).
Moving ellipsoid, torus, and mixed-chain fixtures each have nine DOFs: three
obstacle translations, three obstacle rotations, and three insertion translations.
Tests compare all nine length derivatives and directional speed with centred
finite differences and check a return to the initial pose after warm starts.

For a nine-DOF moving ellipsoid model, this MX graph exposes LMT, its Jacobian,
the directional velocity, the force-induced generalized torque, and the
Hessian. Keep `expand=False` when using the convenience wrapper:

```python
import casadi as ca
import biorbd_casadi as biorbd

model = biorbd.Model("test/models/wrapping_ellipsoid_3d_moving.bioMod")
q = ca.MX.sym("q", 9)
qdot = ca.MX.sym("qdot", 9)
force = ca.MX.sym("F")
model.updateMuscles(q, qdot, True)
geometry = model.muscle(0).position()
to_mx = lambda value: value.to_mx() if hasattr(value, "to_mx") else ca.MX(value)
L = to_mx(geometry.musculoTendonLength())
J = to_mx(geometry.jacobianLength())
v = to_mx(geometry.velocity())
tau = -J.T @ force
H = ca.hessian(L, q)[0]
wrapping = ca.Function("wrapping", [q, qdot, force], [L, J, v, tau, H])

# Alternative when wrapping an existing callable with the project helper
# (keep expansion disabled on CasADi versions older than 3.7):
# f = biorbd.to_casadi_func("wrapping", callable_, q, expand=False)
```

`examples/python3/muscleWrappingCasadi.py` is a reproducible IPOPT smoke test
using the moving ellipsoid and mixed-chain fixtures from this source checkout.
It increases LMT by 0.5 mm through one insertion translation and checks both
IPOPT's default Hessian interface (supplied with finite differences of the
stationary gradient) and `limited-memory`. All four local solves succeeded in
5–7 iterations with length error below `1.6e-13` m; this small smooth-branch
problem does not establish convergence for arbitrary wrapping optimization.

```sh
python examples/python3/muscleWrappingCasadi.py --output /tmp/wrapping-casadi.json
```

The independent reference generators are in `references/`. They require NumPy
and SciPy; the cylinder generator also compares with the freshly built Eigen
binding. For example:

```sh
python examples/muscleWrapping/references/geodesic_reference.py --output-dir /tmp/wrapping-references
python examples/muscleWrapping/references/mixed_geodesic_reference.py --output-dir /tmp/wrapping-references
python examples/muscleWrapping/references/cylinder_chain_reference.py --output-dir /tmp/wrapping-references
```

Development is performed with OpenAI Codex and tracked in
[pyomeca/biorbd issue #401](https://github.com/pyomeca/biorbd/issues/401).
