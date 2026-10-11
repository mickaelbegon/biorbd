"""MX wrapping regression tests runnable with unittest, without pytest.

First derivatives use the stationary envelope. Second derivatives are numerical
central differences of that gradient, so their checks use numerical tolerances.
CasADi >= 3.7 supports expanded SX callback call nodes; callback graphs cannot
be deserialized into a fresh executable Function. Code generation is not tested.
"""

from concurrent.futures import ThreadPoolExecutor
import gc
from pathlib import Path
import tempfile
from threading import Barrier
import unittest

import numpy as np

try:
    import casadi as ca
    import biorbd_casadi as brbd
except ModuleNotFoundError:
    ca = None
    brbd = None

MODEL_DIR = Path(__file__).resolve().parents[2] / "models"
CASES = (
    ("ellipsoid", 0.3589189402510813),
    ("torus", 0.18982123405216328),
    ("mixed", 0.4999204268042869),
)


def _mx(value):
    return value.to_mx() if hasattr(value, "to_mx") else ca.MX(value)


def _array(value):
    return np.asarray(value, dtype=float)


def _function(model, name, with_hessian=False, sliced=False):
    n = model.nbQ()
    parent_q = ca.MX.sym(name + "_q", n + (2 if sliced else 0))
    q = parent_q[:n] if sliced else parent_q
    qdot = ca.MX.sym(name + "_qdot", model.nbQdot())
    force = ca.MX.sym(name + "_force", model.nbMuscles())
    model.updateMuscles(q, qdot, True)
    geometry = model.muscle(0).position()
    length = _mx(geometry.musculoTendonLength())
    gradient = _mx(geometry.jacobianLength())
    velocity = _mx(geometry.velocity())
    torque = _mx(model.muscularJointTorque(force))
    ad_gradient = ca.jacobian(length, parent_q)
    # CasADi exports fixed contact slots, including inactive duplicated slots.
    nodes = ca.horzcat(*[_mx(point) for point in geometry.pathPointsInGlobal()])
    outputs = [length, gradient, velocity, torque, ad_gradient, nodes]
    if with_hessian:
        outputs.extend(
            [
                ca.jacobian(ad_gradient[:, :n].T, parent_q)[:, :n],
                ca.jacobian(velocity, parent_q)[:, :n],
                ca.jacobian(torque, parent_q)[:, :n],
            ]
        )
    return ca.Function(name, [parent_q, qdot, force], outputs)


@unittest.skipIf(ca is None or brbd is None, "CasADi biorbd binding is unavailable")
class TestWrappingCasadi(unittest.TestCase):
    def model(self, kind):
        return brbd.Model(str(MODEL_DIR / f"wrapping_{kind}_3d_moving.bioMod"))

    def evaluate(self, function, q):
        n = len(q)
        return function(q, np.linspace(-0.3, 0.4, n), [57.0])

    def test_references_envelope_ad_and_centered_length_difference(self):
        for kind, reference in CASES:
            with self.subTest(kind=kind):
                model = self.model(kind)
                self.assertEqual(model.nbQ(), 9)
                function = _function(model, "reference_" + kind)
                q = np.zeros(9)
                length, gradient, _, _, ad_gradient, nodes = self.evaluate(function, q)
                self.assertAlmostEqual(float(length), reference, delta=1e-9)
                gradient = _array(gradient).ravel()
                np.testing.assert_allclose(
                    _array(ad_gradient).ravel(), gradient, atol=1e-10, rtol=0
                )
                self.assertEqual(nodes.shape, (3, 8 if kind == "mixed" else 4))
                h = 1e-6
                for coordinate in range(9):
                    plus, minus = q.copy(), q.copy()
                    plus[coordinate] += h
                    minus[coordinate] -= h
                    difference = (
                        float(self.evaluate(function, plus)[0])
                        - float(self.evaluate(function, minus)[0])
                    ) / (2 * h)
                    self.assertAlmostEqual(
                        gradient[coordinate],
                        difference,
                        delta=2e-6,
                        msg=f"{kind} coordinate {coordinate}",
                    )

    def test_velocity_torque_and_virtual_power(self):
        for kind, _ in CASES:
            with self.subTest(kind=kind):
                function = _function(self.model(kind), "power_" + kind)
                q = 1e-4 * np.linspace(-0.3, 0.4, 9)
                qdot = np.linspace(-0.3, 0.4, 9)
                _, gradient, velocity, torque, _, _ = self.evaluate(function, q)
                gradient = _array(gradient).ravel()
                torque = _array(torque).ravel()
                self.assertAlmostEqual(
                    float(velocity), float(gradient @ qdot), delta=1e-11
                )
                np.testing.assert_allclose(torque, -57 * gradient, atol=1e-10, rtol=0)
                self.assertAlmostEqual(
                    float(torque @ qdot), -57 * float(velocity), delta=1e-10
                )
                h = 1e-6
                difference = (
                    float(self.evaluate(function, q + h * qdot)[0])
                    - float(self.evaluate(function, q - h * qdot)[0])
                ) / (2 * h)
                self.assertAlmostEqual(float(velocity), difference, delta=2e-6)

    def test_numerical_hessian_and_symmetry(self):
        for kind, _ in CASES:
            with self.subTest(kind=kind):
                function = _function(
                    self.model(kind), "hessian_" + kind, with_hessian=True
                )
                q = np.zeros(9)
                outputs = self.evaluate(function, q)
                hessian = _array(outputs[6])
                gradient_function = _function(self.model(kind), "hessian_fd_" + kind)
                h = 2e-5
                finite_difference = np.empty((9, 9))
                for coordinate in range(9):
                    plus, minus = q.copy(), q.copy()
                    plus[coordinate] += h
                    minus[coordinate] -= h
                    finite_difference[:, coordinate] = (
                        _array(self.evaluate(gradient_function, plus)[1]).ravel()
                        - _array(self.evaluate(gradient_function, minus)[1]).ravel()
                    ) / (2 * h)
                np.testing.assert_allclose(
                    hessian, finite_difference, atol=2e-4, rtol=0
                )
                np.testing.assert_allclose(hessian, hessian.T, atol=5e-5, rtol=0)
                qdot = np.linspace(-0.3, 0.4, 9)
                np.testing.assert_allclose(
                    _array(outputs[7]).ravel(), qdot @ hessian, atol=2e-4, rtol=0
                )
                np.testing.assert_allclose(
                    _array(outputs[8]), -57 * hessian, atol=57 * 2e-4, rtol=0
                )

    def test_function_lifetime_geometry_copy_and_interleaving(self):
        for kind, reference in CASES:
            with self.subTest(kind=kind):
                model = self.model(kind)
                function = _function(model, "lifetime_" + kind)
                geometry_copy = model.muscle(0).position().DeepCopy()
                copied_length = _mx(geometry_copy.musculoTendonLength())
                inputs = ca.symvar(copied_length)
                self.assertEqual(len(inputs), 1)
                self.assertEqual(inputs[0].shape, (9, 1))
                copied_function = ca.Function(
                    "geometry_copy_" + kind, inputs, [copied_length]
                )
                del copied_length, inputs, geometry_copy, model
                gc.collect()
                q_a = np.zeros(9)
                q_b = 1e-4 * np.linspace(-0.3, 0.4, 9)
                first = self.evaluate(function, q_a)
                self.evaluate(function, q_b)
                copied_function(q_b)
                self.assertAlmostEqual(
                    float(copied_function(q_a)), reference, delta=1e-9
                )
                last = self.evaluate(function, q_a)
                self.assertAlmostEqual(float(last[0]), reference, delta=1e-9)
                for before, after in zip(first, last):
                    np.testing.assert_allclose(
                        _array(before), _array(after), atol=1e-10, rtol=0
                    )

    def test_shared_exported_function_concurrent_evaluations(self):
        for kind in ("ellipsoid", "mixed"):
            with self.subTest(kind=kind):
                function = _function(self.model(kind), "concurrent_" + kind)
                q_a = np.zeros(9)
                q_b = 1e-4 * np.linspace(-0.3, 0.4, 9)
                references = [
                    tuple(_array(value).copy() for value in self.evaluate(function, q))
                    for q in (q_a, q_b)
                ]
                barrier = Barrier(2)

                def repeat(q):
                    results = []
                    for _ in range(4):
                        barrier.wait(timeout=30)
                        results.append(
                            tuple(
                                _array(value).copy()
                                for value in self.evaluate(function, q)
                            )
                        )
                    return results

                with ThreadPoolExecutor(max_workers=2) as executor:
                    jobs = [executor.submit(repeat, q) for q in (q_a, q_b)]
                    for job, reference in zip(jobs, references):
                        for result in job.result(timeout=60):
                            for actual, expected in zip(result, reference):
                                np.testing.assert_allclose(
                                    actual, expected, atol=1e-10, rtol=0
                                )

    @unittest.skipIf(
        ca is None or tuple(map(int, ca.__version__.split(".")[:2])) < (3, 7),
        "SX callback call nodes require CasADi >= 3.7",
    )
    def test_expanded_sx_function_and_length_gradient(self):
        for kind, reference in CASES:
            with self.subTest(kind=kind):
                function = _function(self.model(kind), "mx_expand_" + kind)
                expanded = function.expand()
                q = np.zeros(9)
                original = self.evaluate(function, q)
                result = self.evaluate(expanded, q)
                self.assertAlmostEqual(float(result[0]), reference, delta=1e-9)
                for actual, expected in zip(result, original):
                    np.testing.assert_allclose(
                        _array(actual), _array(expected), atol=1e-10, rtol=0
                    )
                sx_q = ca.SX.sym("sx_q_" + kind, 9)
                sx_qdot = ca.SX.sym("sx_qdot_" + kind, 9)
                sx_force = ca.SX.sym("sx_force_" + kind, 1)
                sx_outputs = expanded(sx_q, sx_qdot, sx_force)
                derivative = ca.Function(
                    "sx_gradient_" + kind,
                    [sx_q, sx_qdot, sx_force],
                    [ca.jacobian(sx_outputs[0], sx_q)],
                )
                sx_gradient = derivative(q, np.linspace(-0.3, 0.4, 9), [57.0])
                np.testing.assert_allclose(
                    _array(sx_gradient), _array(original[1]), atol=1e-10, rtol=0
                )

    def test_callback_graph_deserialization_is_explicitly_unsupported(self):
        function = _function(self.model("mixed"), "serialize_mixed")
        serialized = function.serialize()
        self.assertIsInstance(serialized, str)
        self.assertGreater(len(serialized), 0)
        with self.assertRaisesRegex(RuntimeError, "CallbackInternal|[Cc]allback"):
            ca.Function.deserialize(serialized)

    def test_constant_q_and_graph_slice(self):
        model = self.model("ellipsoid")
        model.updateMuscles(ca.MX.zeros(9, 1), True)
        constant = ca.Function(
            "constant_length",
            [],
            [_mx(model.muscle(0).position().musculoTendonLength())],
        )
        self.assertAlmostEqual(float(constant.call([])[0]), CASES[0][1], delta=1e-9)
        sliced = _function(model, "sliced_length", sliced=True)
        outputs = sliced(np.zeros(11), np.zeros(9), [57.0])
        self.assertAlmostEqual(float(outputs[0]), CASES[0][1], delta=1e-9)
        np.testing.assert_allclose(
            _array(outputs[4])[:, :9], _array(outputs[1]), atol=1e-10, rtol=0
        )
        np.testing.assert_array_equal(_array(outputs[4])[:, 9:], np.zeros((1, 2)))

    def test_inactive_slots_and_via_path(self):
        model = self.model("ellipsoid")
        function = _function(model, "inactive_ellipsoid")
        q = np.zeros(9)
        q[1] = 0.2  # Move obstacle away from the fixed origin-to-insertion chord.
        length, gradient, _, _, _, nodes = self.evaluate(function, q)
        nodes = _array(nodes).T
        self.assertEqual(nodes.shape, (4, 3))
        self.assertAlmostEqual(
            float(length), float(np.linalg.norm(nodes[-1] - nodes[0])), delta=1e-10
        )
        self.assertGreaterEqual(
            np.count_nonzero(np.linalg.norm(np.diff(nodes, axis=0), axis=1) < 1e-10), 2
        )
        np.testing.assert_allclose(
            _array(gradient).ravel()[:6], np.zeros(6), atol=1e-10, rtol=0
        )
        via_model = brbd.Model(str(MODEL_DIR / "wrapping_moving_path.bioMod"))
        via_function = _function(via_model, "moving_via")
        q = np.zeros(via_model.nbQ())
        length, gradient, _, _, ad_gradient, nodes = self.evaluate(via_function, q)
        self.assertGreater(float(length), 0)
        np.testing.assert_allclose(
            _array(gradient), _array(ad_gradient), atol=1e-10, rtol=0
        )
        self.assertTrue(
            np.any(np.linalg.norm(_array(nodes).T - [0, -0.02, 0], axis=1) < 1e-10)
        )

    def test_invalid_evaluation_does_not_poison_later_valid_evaluation(self):
        function = _function(self.model("ellipsoid"), "invalid_then_valid")
        q = np.zeros(9)
        invalid = q.copy()
        invalid[:3] = [-0.17, -0.01, -0.025]  # Obstacle center at the origin endpoint.
        with self.assertRaises(RuntimeError):
            self.evaluate(function, invalid)
        self.assertAlmostEqual(
            float(self.evaluate(function, q)[0]), CASES[0][1], delta=1e-9
        )
        via_function = _function(
            brbd.Model(str(MODEL_DIR / "wrapping_moving_path.bioMod")), "invalid_via"
        )
        invalid = np.zeros(8)
        invalid[-2:] = [-0.065, -0.005]
        with self.assertRaises(RuntimeError):
            self.evaluate(via_function, invalid)
        self.assertGreater(float(self.evaluate(via_function, np.zeros(8))[0]), 0)

    def test_quaternion_q_dimension_is_explicitly_unsupported(self):
        text = (MODEL_DIR / "wrapping_ellipsoid_3d_moving.bioMod").read_text()
        text = text.replace(
            "    translations xyz\n    rotations xyz", "    rotations q", 1
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "quaternion_wrapping.bioMod"
            path.write_text(text)
            model = brbd.Model(str(path))
            self.assertNotEqual(model.nbQ(), model.nbQdot())
            q = ca.MX.sym("unsupported_quaternion_q", model.nbQ())
            with self.assertRaisesRegex(
                RuntimeError, "[Qq]uaternion|nbQ|dimension|dof"
            ):
                model.updateMuscles(q, True)

    def test_standalone_symbolic_wrapping_is_explicitly_unsupported(self):
        surface = brbd.WrappingEllipsoid(
            brbd.RotoTrans(), brbd.Vector3d(0.07, 0.045, 0.055)
        )
        a_mx = ca.MX.sym("standalone_a", 3)
        b_mx = ca.MX.sym("standalone_b", 3)
        a = brbd.Vector3d(a_mx[0], a_mx[1], a_mx[2])
        b = brbd.Vector3d(b_mx[0], b_mx[1], b_mx[2])
        with self.assertRaisesRegex(
            RuntimeError, "unsupported|not supported|numerical|Eigen|symbolic"
        ):
            surface.wrapPoints(brbd.RotoTrans(), a, b, brbd.Vector3d(), brbd.Vector3d())


if __name__ == "__main__":
    unittest.main()
