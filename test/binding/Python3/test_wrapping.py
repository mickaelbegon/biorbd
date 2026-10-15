from pathlib import Path

import numpy as np
import pytest

try:
    import biorbd
except ModuleNotFoundError:
    biorbd = None

MODEL_DIR = Path(__file__).resolve().parents[2] / "models"


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
@pytest.mark.parametrize(
    "fixture, reference, tolerance",
    [
        ("wrapping_cylinder.bioMod", 0.3492105912719195, 1e-9),
        ("wrapping_ellipsoid.bioMod", 0.35419781480619866, 1e-9),
        ("wrapping_ellipsoid_3d.bioMod", 0.3589189402510813, 1e-9),
        ("wrapping_torus_3d.bioMod", 0.18982123405216328, 1e-9),
        ("wrapping_torus_tube.bioMod", 0.14470526426579974, 1e-9),
        ("wrapping_torus_hole.bioMod", 0.2, 1e-10),
        ("wrapping_two_cylinders.bioMod", 0.39645033645390365, 1e-9),
        ("wrapping_three_cylinders_3d.bioMod", 0.48235866649711817, 1e-9),
        ("wrapping_mixed.bioMod", 0.49902788712585605, 1e-9),
        ("wrapping_mixed_3d.bioMod", 0.4999204268042869, 1e-9),
    ],
)
def test_wrapping_model_path_length(fixture, reference, tolerance):
    model = biorbd.Model(str(MODEL_DIR / fixture))
    q = np.zeros(model.nbQ())
    model.updateMuscles(q, True)
    geometry = model.muscle(0).position()

    assert float(geometry.musculoTendonLength()) == pytest.approx(
        reference, abs=tolerance
    )
    points = np.array([point.to_array() for point in geometry.pathPointsInGlobal()])
    assert points.ndim == 2 and points.shape[1] == 3
    assert np.linalg.norm(np.diff(points, axis=0), axis=1).sum() == pytest.approx(
        reference, abs=1e-7
    )


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
def test_wrapping_path_with_via_point_is_sampled_in_order():
    model = biorbd.Model(str(MODEL_DIR / "wrapping_via.bioMod"))
    model.updateMuscles(np.zeros(model.nbQ()), True)
    geometry = model.muscle(0).position()
    path = np.array([point.to_array() for point in geometry.pathPointsInGlobal()])
    nodes = np.array([point.to_array() for point in geometry.pointsInGlobal()])

    assert path.shape[0] > nodes.shape[0]
    assert np.any(np.linalg.norm(path - np.array([0.0, -0.02, 0.0]), axis=1) < 1e-12)
    assert np.linalg.norm(np.diff(path, axis=0), axis=1).sum() == pytest.approx(
        float(geometry.musculoTendonLength()), abs=1e-7
    )


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
@pytest.mark.parametrize("coordinate", [0, 1])
def test_wrapping_length_jacobian_matches_insertion_finite_difference(coordinate):
    model = biorbd.Model(str(MODEL_DIR / "wrapping_moving_insertion.bioMod"))
    q = np.zeros(model.nbQ())
    h = 1e-6
    model.updateMuscles(q, True)
    geometry = model.muscle(0).position()
    analytic = np.asarray(geometry.jacobianLength().to_array()).reshape(-1)[coordinate]

    plus = q.copy()
    minus = q.copy()
    plus[coordinate] += h
    minus[coordinate] -= h
    model.updateMuscles(plus, True)
    length_plus = float(model.muscle(0).position().musculoTendonLength())
    model.updateMuscles(minus, True)
    length_minus = float(model.muscle(0).position().musculoTendonLength())
    finite_difference = (length_plus - length_minus) / (2 * h)

    assert analytic == pytest.approx(finite_difference, abs=2e-7)


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
def test_moving_wrapping_objects_via_and_speed_match_jacobian():
    model = biorbd.Model(str(MODEL_DIR / "wrapping_moving_path.bioMod"))
    q = np.zeros(model.nbQ())
    qdot = np.linspace(-0.3, 0.4, model.nbQ())
    model.updateMuscles(q, qdot, True)
    geometry = model.muscle(0).position()
    jacobian = np.asarray(geometry.jacobianLength().to_array()).reshape(-1)
    velocity = float(geometry.velocity())
    assert velocity == pytest.approx(float(jacobian @ qdot), abs=1e-12)

    velocity_step = 1e-6
    model.updateMuscles(q + velocity_step * qdot, True)
    length_along_plus = float(model.muscle(0).position().musculoTendonLength())
    model.updateMuscles(q - velocity_step * qdot, True)
    length_along_minus = float(model.muscle(0).position().musculoTendonLength())
    assert velocity == pytest.approx(
        (length_along_plus - length_along_minus) / (2 * velocity_step), abs=2e-7
    )

    step = 1e-6
    for coordinate in range(model.nbQ()):
        plus = q.copy()
        minus = q.copy()
        plus[coordinate] += step
        minus[coordinate] -= step
        model.updateMuscles(plus, True)
        length_plus = float(model.muscle(0).position().musculoTendonLength())
        model.updateMuscles(minus, True)
        length_minus = float(model.muscle(0).position().musculoTendonLength())
        finite_difference = (length_plus - length_minus) / (2 * step)
        assert jacobian[coordinate] == pytest.approx(finite_difference, abs=2e-7)


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
def test_crossing_another_surface_after_via_invalidates_geometry_cache():
    model = biorbd.Model(str(MODEL_DIR / "wrapping_cross_via.bioMod"))
    with pytest.raises(RuntimeError, match="outside its assigned via-point region"):
        model.updateMuscles(np.zeros(model.nbQ()), True)
    geometry = model.muscle(0).position()
    with pytest.raises(RuntimeError):
        geometry.pathPointsInGlobal()


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
def test_failed_update_clears_previous_geometry_cache():
    model = biorbd.Model(str(MODEL_DIR / "wrapping_moving_path.bioMod"))
    q = np.zeros(model.nbQ())
    model.updateMuscles(q, True)
    geometry = model.muscle(0).position()
    assert float(geometry.musculoTendonLength()) > 0

    invalid = q.copy()
    invalid[-2:] = [-0.065, -0.005]
    with pytest.raises(RuntimeError):
        model.updateMuscles(invalid, True)
    with pytest.raises(RuntimeError):
        geometry.musculoTendonLength()
    with pytest.raises(RuntimeError):
        geometry.pathPointsInGlobal()


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
@pytest.mark.parametrize(
    "kind,reference",
    [("ellipsoid", 0.3589189402510813), ("torus", 0.18982123405216328)],
)
def test_general_3d_contacts_surface_tangency_and_nonplanarity(kind, reference):
    model = biorbd.Model(str(MODEL_DIR / f"wrapping_{kind}_3d.bioMod"))
    model.updateMuscles(np.zeros(model.nbQ()), True)
    geometry = model.muscle(0).position()
    nodes = np.array([point.to_array() for point in geometry.pointsInGlobal()])
    path = np.array([point.to_array() for point in geometry.pathPointsInGlobal()])
    assert nodes.shape == (4, 3)
    assert float(geometry.musculoTendonLength()) == pytest.approx(
        reference, abs=1e-9, rel=0
    )
    assert np.linalg.norm(np.diff(path, axis=0), axis=1).sum() == pytest.approx(
        reference, abs=1e-7, rel=0
    )
    # Display path contains only origin, contact samples (including both contacts), insertion.
    curve = path[1:-1]
    assert np.linalg.norm(curve[0] - nodes[1]) < 1e-10
    assert np.linalg.norm(curve[-1] - nodes[2]) < 1e-10
    if kind == "ellipsoid":
        radii = np.array([0.07, 0.045, 0.055])
        assert np.max(np.abs(np.sum((curve / radii) ** 2, axis=1) - 1)) < 1e-7
        normals = nodes[1:3] / radii**2
    else:
        rho = np.linalg.norm(curve[:, :2], axis=1)
        assert np.max(np.abs((rho - 0.065) ** 2 + curve[:, 2] ** 2 - 0.021**2)) < 1e-10
        contact_rho = np.linalg.norm(nodes[1:3, :2], axis=1)
        normals = nodes[1:3].copy()
        normals[:, :2] *= ((contact_rho - 0.065) / contact_rho)[:, None]
    directions = np.array([nodes[1] - nodes[0], nodes[3] - nodes[2]])
    directions /= np.linalg.norm(directions, axis=1)[:, None]
    normals /= np.linalg.norm(normals, axis=1)[:, None]
    assert np.max(np.abs(np.sum(directions * normals, axis=1))) < 2e-7
    centered = curve - curve.mean(axis=0)
    _, _, planes = np.linalg.svd(centered, full_matrices=False)
    assert np.max(np.abs(centered @ planes[-1])) > 1e-6


@pytest.mark.skipif(biorbd is None, reason="biorbd Python binding is unavailable")
@pytest.mark.parametrize(
    "kind,reference",
    [
        ("ellipsoid", 0.3589189402510813),
        ("torus", 0.18982123405216328),
        ("mixed", 0.4999204268042869),
    ],
)
def test_general_3d_nine_dof_jacobian_speed_and_warm_return(kind, reference):
    model = biorbd.Model(str(MODEL_DIR / f"wrapping_{kind}_3d_moving.bioMod"))
    assert model.nbQ() == 9
    q = np.zeros(9)
    qdot = np.linspace(-0.3, 0.4, 9)
    model.updateMuscles(q, qdot, True)
    geometry = model.muscle(0).position()
    assert float(geometry.musculoTendonLength()) == pytest.approx(
        reference, abs=1e-9, rel=0
    )
    original_nodes = np.array([point.to_array() for point in geometry.pointsInGlobal()])
    jacobian = np.asarray(geometry.jacobianLength().to_array()).reshape(-1).copy()
    velocity = float(geometry.velocity())
    assert velocity == pytest.approx(float(jacobian @ qdot), abs=1e-12, rel=0)
    h = 1e-6
    for coordinate in range(9):
        plus, minus = q.copy(), q.copy()
        plus[coordinate] += h
        minus[coordinate] -= h
        model.updateMuscles(plus, True)
        length_plus = float(geometry.musculoTendonLength())
        model.updateMuscles(minus, True)
        length_minus = float(geometry.musculoTendonLength())
        assert jacobian[coordinate] == pytest.approx(
            (length_plus - length_minus) / (2 * h), abs=2e-6, rel=0
        ), f"coordinate {coordinate}"
    model.updateMuscles(q + h * qdot, True)
    length_plus = float(geometry.musculoTendonLength())
    model.updateMuscles(q - h * qdot, True)
    length_minus = float(geometry.musculoTendonLength())
    assert velocity == pytest.approx(
        (length_plus - length_minus) / (2 * h), abs=2e-6, rel=0
    )
    model.updateMuscles(q + 1e-4 * qdot, True)
    model.updateMuscles(q, True)
    assert float(geometry.musculoTendonLength()) == pytest.approx(
        reference, abs=1e-9, rel=0
    )
    returned_nodes = np.array([point.to_array() for point in geometry.pointsInGlobal()])
    assert np.max(np.linalg.norm(returned_nodes - original_nodes, axis=1)) < 1e-8
