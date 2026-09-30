# Video 18 - casadi_symbolic (AnimCasadiSymbolic)

## Constraint from the brief was wrong on this machine
The brief said `biorbd_casadi` is not installed. It is: in the `biorbd_anim` environment `import biorbd_casadi` works
(`site-packages/biorbd_casadi`, version 1.12.3, `biorbd_casadi.backend == biorbd_casadi.CASADI`), and `import biorbd`
gives the Eigen backend (`biorbd.backend == biorbd.EIGEN3`). So the video uses the real CasADi backend; no
hand-written symbolic code was needed (no "Hand-written code" label).

## What was verified (biorbd 1.12.3)
- `binding/python3/__init__.py` lines 15-49: `to_casadi_func` exists only for the CASADI backend (raises with Eigen).
  Usage pattern taken from `test/binding/python3/test_binder_python_rigidbody.py` (`brbd.to_casadi_func("...", m.ForwardDynamics, q_sym, ...)`).
  Those tests are parametrised over `biorbd` and `biorbd_casadi`; I did not run the test suite.
- `bc.to_casadi_func("M", m.massMatrix, q)` with `q = MX.sym("q", 2)` returns an `SXFunction` (expand=True); run in biorbd_anim.
- Model: `data/casadi_symbolic_pendulum.bioMod` (written for this video): two rotations about y, masses 1 and 2 kg,
  com (0,0,-0.25) and (0,0,-0.3), diagonal inertias, link offset 0.5 m (`RTinMatrix 0` / `RT 0 0 0 xyz 0 0 -0.5`).
  A first version used `rttrans`, which is NOT a keyword: it was silently ignored (the two links then had no offset) - caught by
  checking M against the analytic value M01 = I2 + m2*(d2^2 + l1*d2*cos q2) = 0.23 + 0.3 cos(q2) = 0.4390 at q2 = 0.8.
- `data/casadi_symbolic_main.npz` from `generate_casadi_symbolic_data.py`: M from `biorbd_casadi` and from `biorbd.Biorbd(path).mass_matrix(q0)`
  at q0 = [0.3, 0.8]; max |difference| = 0.0 (bit-identical on this machine). The symbolic entry M01 is printed by CasADi from the expanded SX function.
- The displayed code is read between CODE-BEGIN/CODE-END (wrapped in `# fmt: off` so black keeps the lines). The scene asserts max diff < 1e-12.

## Not verified / open
- Only one pose is compared, not a sweep; only mass matrix (not dynamics, Jacobians, etc.).
- The final card is empty until the lead merges catalog.json.
- The symbolic entry shown is the raw CasADi print (constants folded by CasADi); not simplified by hand.
- Text in a code panel loses leading spaces (manim Text), hence no continuation lines in the displayed code.
- The number template splits `0.000000` as one number ("Largest difference: {0}"); scientific notation (`1.0e-13`) would be split into several placeholders by the layer.
