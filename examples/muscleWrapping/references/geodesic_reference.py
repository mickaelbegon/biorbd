"""Numerical 3-D stationary wrapping references; all exported lengths are metres.

The geodesic uses unit-speed shooting on an implicit surface. Contact angles and
log arc length are fitted to entry tangency and outgoing direction continuity.
Continuation starts on independently known symmetry-plane branches. This does
not establish global optimality or choose between different homotopy branches.

Reusable API: Surface(...), planar_seed(...), solve_contact(surface,A,B,seed).
The returned seed can be reused for nearby endpoints and rigid surface frames.
For a chain, use neighbouring exit/entry as A/B and retain one seed per surface;
continue endpoints/frames from a validated initial chain in small increments.
A chain still requires its own convergence and collision checks against ALL
associated obstacles, including inactive ones and regions separated by vias.
"""

from dataclasses import dataclass
from pathlib import Path
import json
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import least_squares

OUT = Path(__file__).resolve().parent


@dataclass
class Surface:
    kind: str
    dimensions: np.ndarray
    center: np.ndarray = None
    rotation: np.ndarray = None

    def __post_init__(self):
        self.dimensions = np.asarray(self.dimensions, dtype=float)
        self.center = (
            np.zeros(3) if self.center is None else np.asarray(self.center, dtype=float)
        )
        self.rotation = (
            np.eye(3)
            if self.rotation is None
            else np.asarray(self.rotation, dtype=float)
        )
        self.scale = float(max(self.dimensions))
        self.d = self.dimensions / self.scale
        if self.kind not in ("ellipsoid", "torus"):
            raise ValueError("Supported reference surfaces: ellipsoid and torus")
        if not np.all(self.dimensions > 0):
            raise ValueError("Dimensions must be positive")
        if self.kind == "torus" and not self.dimensions[0] > self.dimensions[1]:
            raise ValueError("Requires a ring torus: major > minor")
        if not np.allclose(self.rotation.T @ self.rotation, np.eye(3), atol=1e-13):
            raise ValueError("Surface rotation must be orthogonal")

    def local(self, global_points):
        return (np.asarray(global_points) - self.center) @ self.rotation / self.scale

    def global_(self, normalized_points):
        return (
            np.asarray(normalized_points) @ self.rotation.T * self.scale + self.center
        )

    def parameter_point(self, angles):
        u, v = angles
        if self.kind == "ellipsoid":
            return self.d * np.array(
                [np.cos(u) * np.cos(v), np.sin(u) * np.cos(v), np.sin(v)]
            )
        major, minor = self.d
        return np.array(
            [
                (major + minor * np.cos(v)) * np.cos(u),
                (major + minor * np.cos(v)) * np.sin(u),
                minor * np.sin(v),
            ]
        )

    def geometry(self, point):
        """Return F, gradient(F), Hessian(F) in normalized local coordinates."""
        x = np.asarray(point)
        if self.kind == "ellipsoid":
            inverse = 1 / self.d**2
            return (
                float((x * x * inverse).sum() - 1),
                2 * x * inverse,
                np.diag(2 * inverse),
            )
        major, minor = self.d
        rho = np.hypot(x[0], x[1])
        if rho < 1e-12:
            raise ValueError("Torus gradient undefined on its symmetry axis")
        q = rho - major
        g = 2 * np.array([q * x[0] / rho, q * x[1] / rho, x[2]])
        h = np.zeros((3, 3))
        h[:2, :2] = 2 * (q / rho * np.eye(2) + major / rho**3 * np.outer(x[:2], x[:2]))
        h[2, 2] = 2
        return q * q + x[2] * x[2] - minor * minor, g, h

    def level(self, points):
        p = np.asarray(points)
        if self.kind == "ellipsoid":
            return np.linalg.norm(p / self.d, axis=-1) - 1
        major, minor = self.d
        return np.hypot(np.hypot(p[..., 0], p[..., 1]) - major, p[..., 2]) - minor

    def rhs(self, unused_s, state):
        p, v = state[:3], state[3:]
        _, g, h = self.geometry(p)
        acceleration = -float(v @ h @ v) / float(g @ g) * g
        return np.r_[v, acceleration]

    def chord_minimum_torus_clearance(self, a, b):
        """Minimum signed tube clearance in metres, by stationary-root isolation."""
        if self.kind != "torus":
            raise ValueError("Euclidean clearance minimization here is torus-only")
        a, delta = np.asarray(a), np.asarray(b) - np.asarray(a)
        major, minor = self.d
        radial = np.polynomial.Polynomial(
            [a[:2] @ a[:2], 2 * a[:2] @ delta[:2], delta[:2] @ delta[:2]]
        )
        scalar = np.polynomial.Polynomial([a @ delta, delta @ delta])
        radial_dot = np.polynomial.Polynomial(
            [a[:2] @ delta[:2], delta[:2] @ delta[:2]]
        )
        stationary = scalar * scalar * radial - major * major * radial_dot * radial_dot
        ts = [0.0, 1.0]
        ts.extend(
            float(t.real)
            for t in stationary.roots()
            if abs(t.imag) < 1e-9 and 0 < t.real < 1
        )
        # The signed distance has a possible nonsmooth point on the symmetry axis.
        if radial.coef[-1] > 0:
            t_axis = -radial.coef[1] / (2 * radial.coef[-1])
            if 0 < t_axis < 1 and abs(radial(t_axis)) < 1e-12:
                ts.append(t_axis)
        return min(float(self.level(a + t * delta)) for t in ts) * self.scale

    def chord_minimum_level(self, a, b):
        """Exact line minimization, up to polynomial root roundoff."""
        a, delta = np.asarray(a), np.asarray(b) - np.asarray(a)
        if self.kind == "ellipsoid":
            p, d = a / self.d, delta / self.d
            t = np.clip(-p @ d / (d @ d), 0, 1) if d @ d else 0
            return float(np.linalg.norm(p + t * d) - 1)
        major, minor = self.d
        q = np.array([a @ a + major**2 - minor**2, 2 * a @ delta, delta @ delta])
        radial = np.array([a[:2] @ a[:2], 2 * a[:2] @ delta[:2], delta[:2] @ delta[:2]])
        polynomial = np.polynomial.Polynomial(
            np.convolve(q, q)
        ) - 4 * major**2 * np.polynomial.Polynomial(radial)
        # Implicit polynomial sign equals tube penetration for a ring torus.
        ts = [0.0, 1.0]
        ts.extend(
            float(t.real)
            for t in polynomial.deriv().roots()
            if abs(t.imag) < 1e-10 and 0 < t.real < 1
        )
        minimum = min(float(polynomial(t)) for t in ts)
        # Return polynomial level, normalized by the obstacle's fourth power.
        return minimum / (major + minor) ** 4


def planar_seed(surface, start, end):
    """Upper planar analytic circle/ellipse tangent branch, for continuation only."""
    a, b = surface.local(start), surface.local(end)
    if surface.kind == "ellipsoid":
        rx, ry = surface.d[:2]
        pa, pb = a[:2], b[:2]
    else:
        rx = ry = surface.d[1]
        center = np.array([surface.d[0], 0.0])
        pa, pb = a[[0, 2]] - center, b[[0, 2]] - center
    scaled_a, scaled_b = pa / [rx, ry], pb / [rx, ry]
    entry = np.arctan2(scaled_a[1], scaled_a[0]) - np.arccos(
        1 / np.linalg.norm(scaled_a)
    )
    exit_ = np.arctan2(scaled_b[1], scaled_b[0]) + np.arccos(
        1 / np.linalg.norm(scaled_b)
    )
    while exit_ > entry:
        exit_ -= 2 * np.pi
    from scipy.integrate import quad

    length = (
        quad(
            lambda t: np.hypot(rx * np.sin(t), ry * np.cos(t)),
            exit_,
            entry,
            epsabs=1e-13,
            epsrel=1e-13,
        )[0]
        * surface.scale
    )
    angles = [entry, 0.0] if surface.kind == "ellipsoid" else [0.0, entry]
    return {"angles": angles, "arc_length_m": length}


def solve_contact(surface, start, end, seed, rtol=2e-11, atol=2e-13, count=801):
    """Fit one connected contact geodesic. Return globals plus a reusable seed.

    Always raises when fitting fails, endpoints penetrate, reaction curvature is
    negative, a free piece penetrates, or the numerical surface defect is large.
    No automatic activation or global branch selection is performed.
    """
    a, b = surface.local(start), surface.local(end)
    if min(float(surface.level(a)), float(surface.level(b))) < -1e-11:
        raise ValueError("Contact endpoints must be outside the solid")

    def initial_state(parameters):
        p = surface.parameter_point(parameters[:2])
        _, normal, _ = surface.geometry(p)
        incoming = p - a
        incoming /= np.linalg.norm(incoming)
        velocity = incoming - (incoming @ normal) / (normal @ normal) * normal
        velocity /= np.linalg.norm(velocity)
        return np.r_[p, velocity], incoming @ normal / np.linalg.norm(normal)

    def integrate(parameters, dense=False):
        state, tangent_error = initial_state(parameters)
        length = float(np.exp(parameters[2]))
        fit = solve_ivp(
            surface.rhs,
            [0, length],
            state,
            method="DOP853",
            rtol=rtol,
            atol=atol,
            dense_output=dense,
        )
        if not fit.success:
            raise RuntimeError("Geodesic integration failed: " + fit.message)
        return fit, tangent_error

    def residual(parameters):
        fit, entry_error = integrate(parameters)
        p, velocity = fit.y[:3, -1], fit.y[3:, -1]
        outgoing = (b - p) / np.linalg.norm(b - p)
        return np.r_[entry_error, velocity - outgoing]

    initial = np.r_[seed["angles"], np.log(seed["arc_length_m"] / surface.scale)]
    lower, upper = np.array([-np.inf, -np.inf, -12.0]), np.array([np.inf, np.inf, 5.0])
    if surface.kind == "ellipsoid":
        lower[1], upper[1] = -np.pi / 2 + 1e-7, np.pi / 2 - 1e-7
    fit = least_squares(
        residual,
        initial,
        bounds=(lower, upper),
        xtol=2e-13,
        ftol=2e-13,
        gtol=2e-13,
        max_nfev=150,
    )
    error = float(np.max(np.abs(residual(fit.x))))
    if not fit.success or error > 2e-8:
        raise RuntimeError(f"Shooting failed: {fit.message}, residual={error:.4g}")
    integration, entry_error = integrate(fit.x, dense=True)
    arc_normalized = float(np.exp(fit.x[2]))
    states = integration.sol(np.linspace(0, arc_normalized, count)).T
    curve = states[:, :3]
    velocities = states[:, 3:]
    surface_defect = max(abs(surface.geometry(p)[0]) for p in curve)
    unit_speed_defect = float(np.max(np.abs(np.linalg.norm(velocities, axis=1) - 1)))
    normal_velocity_defect = max(
        abs(surface.geometry(p)[1] @ v) / np.linalg.norm(surface.geometry(p)[1])
        for p, v in zip(curve, velocities)
    )
    reaction_curvature = min(
        float(v @ surface.geometry(p)[2] @ v) / np.linalg.norm(surface.geometry(p)[1])
        for p, v in zip(curve, velocities)
    )
    free_clearance = min(
        surface.chord_minimum_level(a, curve[0]),
        surface.chord_minimum_level(curve[-1], b),
    )
    if (
        surface_defect > 2e-8
        or unit_speed_defect > 2e-8
        or free_clearance < -2e-8
        or reaction_curvature < -1e-9
    ):
        raise RuntimeError(
            "Geodesic validation failed: "
            + str(
                (surface_defect, unit_speed_defect, free_clearance, reaction_curvature)
            )
        )
    h = arc_normalized / (count - 1)
    numerical_acceleration = (
        velocities[:-4] - 8 * velocities[1:-3] + 8 * velocities[3:-1] - velocities[4:]
    ) / (12 * h)
    exact_acceleration = np.array([surface.rhs(0, state)[3:] for state in states[2:-2]])
    differential_defect = float(
        np.max(np.linalg.norm(numerical_acceleration - exact_acceleration, axis=1))
    )
    torus_clearance = None
    if surface.kind == "torus":
        torus_clearance = min(
            surface.chord_minimum_torus_clearance(a, curve[0]),
            surface.chord_minimum_torus_clearance(curve[-1], b),
        )
        if torus_clearance < -2e-10:
            raise RuntimeError("Free piece penetrates torus by signed distance check")
    outgoing = (b - curve[-1]) / np.linalg.norm(b - curve[-1])
    exit_normal = surface.geometry(curve[-1])[1]
    exit_error = abs(float(outgoing @ exit_normal) / np.linalg.norm(exit_normal))
    arc_m = arc_normalized * surface.scale
    total = (
        np.linalg.norm(curve[0] - a) + arc_normalized + np.linalg.norm(b - curve[-1])
    ) * surface.scale
    output = dict(
        seed={"angles": fit.x[:2].tolist(), "arc_length_m": arc_m},
        parameters=fit.x,
        local_states=states,
        curve=surface.global_(curve),
        contacts=surface.global_(curve[[0, -1]]),
        length=float(total),
        arc_length=arc_m,
        endpoints=np.array([start, end]),
        normalized_arc_length=arc_normalized,
        metrics=dict(
            shooting_residual=error,
            entry_tangency_residual=abs(float(entry_error)),
            exit_tangency_residual=exit_error,
            surface_implicit_defect=float(surface_defect),
            unit_speed_defect=unit_speed_defect,
            normal_velocity_defect=float(normal_velocity_defect),
            minimum_free_segment_implicit_level=float(free_clearance),
            minimum_reaction_curvature_per_normalized_length=float(reaction_curvature),
            geodesic_differential_defect_normalized=float(differential_defect),
            shooting_nfev=int(fit.nfev),
        ),
    )
    if torus_clearance is not None:
        output["metrics"][
            "minimum_free_segment_exact_torus_clearance_m"
        ] = torus_clearance
    output["metrics"]["direct_chord_implicit_level"] = surface.chord_minimum_level(a, b)
    return output


def rk4_endpoint(surface, state, length, steps):
    x, h = state.copy(), length / steps
    for i in range(steps):
        k1 = surface.rhs(0, x)
        k2 = surface.rhs(0, x + h * k1 / 2)
        k3 = surface.rhs(0, x + h * k2 / 2)
        k4 = surface.rhs(0, x + h * k3)
        x += h * (k1 + 2 * k2 + 2 * k3 + k4) / 6
    return x


def convergence_metrics(surface, result, start, end):
    refined = solve_contact(
        surface, start, end, result["seed"], rtol=2e-13, atol=2e-15, count=1601
    )
    refined["metrics"]["refinement_length_difference_m"] = abs(
        refined["length"] - result["length"]
    )
    refined["metrics"]["refinement_contact_difference_m"] = float(
        np.max(np.linalg.norm(refined["contacts"] - result["contacts"], axis=1))
    )
    errors = []
    for steps in (64, 128, 256, 512):
        rk_end = rk4_endpoint(
            surface, refined["local_states"][0], refined["normalized_arc_length"], steps
        )
        errors.append(float(np.linalg.norm(rk_end - refined["local_states"][-1])))
    refined["metrics"]["independent_rk4_steps"] = [64, 128, 256, 512]
    refined["metrics"]["independent_rk4_terminal_state_errors"] = errors
    refined["metrics"]["independent_rk4_observed_orders"] = [
        float(np.log2(a / b)) for a, b in zip(errors[:-1], errors[1:])
    ]
    assert refined["metrics"]["refinement_length_difference_m"] < 2e-10
    assert refined["metrics"]["refinement_contact_difference_m"] < 2e-9
    assert errors[-1] < 2e-8
    assert min(refined["metrics"]["independent_rk4_observed_orders"][:2]) > 3.7
    return refined


def mesh(surface):
    u, v = np.meshgrid(
        np.linspace(0, 2 * np.pi, 65),
        (
            np.linspace(-np.pi / 2, np.pi / 2, 33)
            if surface.kind == "ellipsoid"
            else np.linspace(0, 2 * np.pi, 33)
        ),
    )
    points = np.array(
        [surface.parameter_point(angles) for angles in zip(u.ravel(), v.ravel())]
    ).reshape(u.shape + (3,))
    return surface.global_(points)


def export_case(key, title, surface, result, continuation):
    a, b = result["endpoints"]
    p, q = result["contacts"]
    path = np.vstack(
        [np.linspace(a, p, 401)[:-1], result["curve"], np.linspace(q, b, 401)[1:]]
    )
    polyline_length = float(np.linalg.norm(np.diff(path, axis=0), axis=1).sum())
    result["metrics"]["dense_polyline_length_difference_m"] = abs(
        polyline_length - result["length"]
    )
    result["metrics"]["minimum_sampled_surface_level"] = float(
        np.min(surface.level(surface.local(result["curve"])))
    )
    result["metrics"]["minimum_sampled_path_level"] = float(
        np.min(surface.level(surface.local(path)))
    )
    clearance_scale = (
        min(surface.dimensions) if surface.kind == "ellipsoid" else surface.scale
    )
    result["metrics"]["minimum_sampled_scaled_level_m"] = (
        result["metrics"]["minimum_sampled_path_level"] * clearance_scale
    )
    result["metrics"]["continuation_steps"] = continuation
    assert result["metrics"]["minimum_sampled_path_level"] >= -1e-8
    assert result["metrics"]["dense_polyline_length_difference_m"] < 1e-7
    xyz = mesh(surface)
    obj = dict(
        kind=surface.kind,
        center=surface.center.tolist(),
        rotation=surface.rotation.tolist(),
        axes=surface.dimensions.tolist() if surface.kind == "ellipsoid" else None,
        major=float(surface.dimensions[0]) if surface.kind == "torus" else None,
        minor=float(surface.dimensions[1]) if surface.kind == "torus" else None,
        x=xyz[..., 0].tolist(),
        y=xyz[..., 1].tolist(),
        z=xyz[..., 2].tolist(),
    )
    return dict(
        key=key,
        title=title,
        reference="Branche géodésique stationnaire 3D obtenue par continuation ; minimum global non établi",
        detail="Shooting sur surface implicite · backend SciPy DOP853 · toutes dimensions en mètres",
        length=result["length"],
        direct_length=float(np.linalg.norm(b - a)),
        arc_length=result["arc_length"],
        objects=[obj],
        points=path.tolist(),
        contacts=result["contacts"].tolist(),
        endpoints=result["endpoints"].tolist(),
        endpoint_labels=["Origine", "Insertion"],
        metrics=result["metrics"],
        seed=result["seed"],
    )


def main():
    baseline_specs = [
        (
            "sphere",
            Surface("ellipsoid", [0.05, 0.05, 0.05]),
            np.array([-0.15, -0.01, 0]),
            np.array([0.15, -0.01, 0]),
            None,
        ),
        (
            "ellipsoid-planar",
            Surface("ellipsoid", [0.07, 0.045, 0.055]),
            np.array([-0.17, -0.01, 0]),
            np.array([0.17, 0.005, 0]),
            0.35419781480619866,
        ),
        (
            "torus-planar",
            Surface("torus", [0.065, 0.021]),
            np.array([0, 0, -0.01]),
            np.array([0.13, 0, -0.01]),
            0.14470526426579974,
        ),
    ]
    baselines = []
    starts = {}
    for key, surface, a, b, expected in baseline_specs:
        seed = planar_seed(surface, a, b)
        if expected is None:
            p = surface.parameter_point(seed["angles"])
            # Independent planar developed-circle total length.
            external = np.sqrt(np.linalg.norm(surface.local(a)) ** 2 - 1) + np.sqrt(
                np.linalg.norm(surface.local(b)) ** 2 - 1
            )
            expected = external * surface.scale + seed["arc_length_m"]
        result = solve_contact(surface, a, b, seed)
        difference = abs(result["length"] - expected)
        assert difference < 2e-10, (key, difference)
        baselines.append(
            dict(
                key=key,
                length=result["length"],
                reference_length=expected,
                difference_m=difference,
                metrics=result["metrics"],
            )
        )
        starts[key] = (surface, a, b, result)
    cases = []
    for key, title, source, target_a, target_b in [
        (
            "ellipsoid-3d",
            "Ellipsoïde : géodésique hors plan principal",
            "ellipsoid-planar",
            [-0.17, -0.01, -0.025],
            [0.17, 0.005, 0.035],
        ),
        (
            "torus-3d",
            "Tore : géodésique oblique hors plan de symétrie",
            "torus-planar",
            [0, -0.038, -0.03],
            [0.13, 0.07, -0.01],
        ),
    ]:
        surface, original_a, original_b, result = starts[source]
        target_a, target_b = np.array(target_a), np.array(target_b)
        for fraction in np.linspace(0, 1, 13)[1:]:
            a = original_a + fraction * (target_a - original_a)
            b = original_b + fraction * (target_b - original_b)
            result = solve_contact(surface, a, b, result["seed"])
        result = convergence_metrics(surface, result, target_a, target_b)
        if surface.kind == "torus":
            theta = np.unwrap(
                np.arctan2(result["local_states"][:, 1], result["local_states"][:, 0])
            )
            theta_span = float(np.ptp(theta))
            result["metrics"]["torus_azimuth_span_rad"] = theta_span
            result["metrics"]["endpoint_xy_cross_product_m2"] = float(
                target_a[0] * target_b[1] - target_a[1] * target_b[0]
            )
            assert (
                0.15 < theta_span < 0.4
            ), "The supposed 3D torus reference is still meridional"
        # Independent covariance test under a tilted rigid transform.
        cx, sx, cz, sz = np.cos(0.51), np.sin(0.51), np.cos(0.7), np.sin(0.7)
        rotation = np.array(
            [[cz, -sz * cx, sz * sx], [sz, cz * cx, -cz * sx], [0, sx, cx]]
        )
        translation = np.array([0.73, -0.31, 0.22])
        moved = Surface(surface.kind, surface.dimensions, translation, rotation)
        moved_result = solve_contact(
            moved,
            target_a @ rotation.T + translation,
            target_b @ rotation.T + translation,
            result["seed"],
            rtol=2e-13,
            atol=2e-15,
        )
        frame_length_error = abs(moved_result["length"] - result["length"])
        frame_contact_error = float(
            np.max(
                np.linalg.norm(
                    moved_result["contacts"]
                    - (result["contacts"] @ rotation.T + translation),
                    axis=1,
                )
            )
        )
        result["metrics"]["rigid_frame_length_difference_m"] = frame_length_error
        result["metrics"]["rigid_frame_contact_difference_m"] = frame_contact_error
        assert frame_length_error < 1e-12 and frame_contact_error < 1e-12
        if surface.kind == "torus":
            angular_momentum = np.cross(
                result["local_states"][:, :3], result["local_states"][:, 3:]
            )[:, 2]
            result["metrics"]["torus_axial_angular_momentum_normalized"] = float(
                angular_momentum[0]
            )
            result["metrics"]["torus_axial_angular_momentum_defect_normalized"] = float(
                np.ptp(angular_momentum)
            )
            assert abs(angular_momentum[0]) > 1e-3
        cases.append(export_case(key, title, surface, result, 12))
        print(
            f"{key}: length={result['length']:.15f} m, residual={result['metrics']['shooting_residual']:.3e}, defect={result['metrics']['surface_implicit_defect']:.3e}"
        )
    (OUT / "geodesic-data.json").write_text(
        json.dumps(cases, ensure_ascii=False, separators=(",", ":"), allow_nan=False)
    )
    (OUT / "geodesic-validation.json").write_text(
        json.dumps(
            dict(
                baselines=baselines,
                cases=[
                    {
                        k: v
                        for k, v in case.items()
                        if k
                        in (
                            "key",
                            "length",
                            "arc_length",
                            "metrics",
                            "seed",
                            "reference",
                        )
                    }
                    for case in cases
                ],
            ),
            ensure_ascii=False,
            indent=2,
            allow_nan=False,
        )
    )
    print(
        "All independent planar baselines, full-3D contacts, and mesh-refinement checks passed."
    )


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=Path.cwd() / "wrapping-reference-output"
    )
    OUT = parser.parse_args().output_dir
    OUT.mkdir(parents=True, exist_ok=True)
    main()
