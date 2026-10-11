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

## Path and derivatives

Each region is solved by iterating the tangent contacts, using the previous
object's exit and the next object's entry. Inactive objects are reconsidered on
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
`Geometry::pathPointsInGlobal()` additionally samples each new surface arc, so
it can be plotted as a 3-D polyline. Legacy objects retain their original point
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

- Eigen numerical backend. CasADi still compiles; new contact evaluation raises
  an explicit unsupported-solver error.
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

Native and Python tests compare independent lengths, contact geometry, rigid
frame invariance, copy isolation, multi-object/via paths, length derivatives
against centred finite differences, and failure cases. The six original reference lengths
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
