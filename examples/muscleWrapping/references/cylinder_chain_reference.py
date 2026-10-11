"""Independent simultaneous helix contact solve, compared with biorbd Eigen.

All lengths are metres, object rotations map local coordinates to world.
The prescribed upper contact branch is continued from parallel cylinders.
"""

import json
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

OUT = Path(__file__).resolve().parent


def rotation(x=0, y=0, z=0):
    x, y, z = np.deg2rad([x, y, z])
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


def point(obj, angle, z):
    return (
        obj["rotation"]
        @ np.array([obj["radius"] * np.cos(angle), obj["radius"] * np.sin(angle), z])
        + obj["center"]
    )


def contacts_and_tangents(objects, parameters):
    contacts, tangents, arcs = [], [], []
    for obj, (a, b, z0, z1) in zip(objects, parameters.reshape(-1, 4)):
        arc = np.hypot(obj["radius"] * (b - a), z1 - z0)
        pair = [point(obj, a, z0), point(obj, b, z1)]
        directions = [
            obj["rotation"]
            @ np.array(
                [
                    -obj["radius"] * np.sin(t) * (b - a),
                    obj["radius"] * np.cos(t) * (b - a),
                    z1 - z0,
                ]
            )
            / arc
            for t in (a, b)
        ]
        contacts.append(pair)
        tangents.append(directions)
        arcs.append(arc)
    return np.array(contacts), np.array(tangents), np.array(arcs)


def solve(objects, start, end, seed):
    def residual(parameters):
        contacts, tangents, unused = contacts_and_tangents(objects, parameters)
        errors = []
        for i, (pair, tangent) in enumerate(zip(contacts, tangents)):
            previous = start if i == 0 else contacts[i - 1, 1]
            following = end if i == len(objects) - 1 else contacts[i + 1, 0]
            incoming, outgoing = pair[0] - previous, following - pair[1]
            errors.extend(incoming / np.linalg.norm(incoming) - tangent[0])
            errors.extend(outgoing / np.linalg.norm(outgoing) - tangent[1])
        return errors

    lower = np.array(
        [
            [1e-5, 1e-5, -o["length"] / 2 + 1e-5, -o["length"] / 2 + 1e-5]
            for o in objects
        ]
    ).ravel()
    upper = np.array(
        [
            [np.pi - 1e-5, np.pi - 1e-5, o["length"] / 2 - 1e-5, o["length"] / 2 - 1e-5]
            for o in objects
        ]
    ).ravel()
    fit = least_squares(
        residual,
        seed.ravel(),
        bounds=(lower, upper),
        xtol=1e-14,
        ftol=1e-14,
        gtol=1e-14,
        max_nfev=300,
    )
    error = float(np.max(np.abs(residual(fit.x))))
    if not fit.success or error > 1e-10:
        raise RuntimeError(f"Simultaneous helix solve failed: {fit.message}, {error}")
    assert np.all(fit.x.reshape(-1, 4)[:, 0] > fit.x.reshape(-1, 4)[:, 1])
    contacts, tangents, arcs = contacts_and_tangents(objects, fit.x)
    pieces, previous = [], start
    for obj, pars, pair in zip(objects, fit.x.reshape(-1, 4), contacts):
        pieces.append(np.linspace(previous, pair[0], 401))
        a, b, z0, z1 = pars
        pieces.append(
            np.array(
                [
                    point(obj, t, z)
                    for t, z in zip(np.linspace(a, b, 1601), np.linspace(z0, z1, 1601))
                ]
            )
        )
        previous = pair[1]
    pieces.append(np.linspace(previous, end, 401))
    length = float(sum(arcs) + sum(np.linalg.norm(p[-1] - p[0]) for p in pieces[::2]))
    return dict(
        parameters=fit.x,
        contacts=contacts.reshape(-1, 3),
        points=np.vstack(pieces),
        pieces=pieces,
        length=length,
        residual=error,
    )


def chord_minimum(obj, a, b):
    """Exact radial distance minimum over the finite cylinder's axial range."""
    p = (a - obj["center"]) @ obj["rotation"]
    d = (b - a) @ obj["rotation"]
    lo, hi = 0.0, 1.0
    if abs(d[2]) < 1e-16:
        if abs(p[2]) > obj["length"] / 2:
            return np.inf
    else:
        limits = sorted(
            [(-obj["length"] / 2 - p[2]) / d[2], (obj["length"] / 2 - p[2]) / d[2]]
        )
        lo, hi = max(lo, limits[0]), min(hi, limits[1])
        if lo > hi:
            return np.inf
    d2 = float(d[:2] @ d[:2])
    t = np.clip(-p[:2] @ d[:2] / d2, lo, hi) if d2 else lo
    return float(np.linalg.norm((p + t * d)[:2]) - obj["radius"])


def validate(objects, result, start, end):
    clearance = np.inf
    for i, piece in enumerate(result["pieces"]):
        for j, obj in enumerate(objects):
            if i % 2 == 1 and j == (i - 1) // 2:
                local = (piece - obj["center"]) @ obj["rotation"]
                assert (
                    np.max(np.abs(np.linalg.norm(local[:, :2], axis=1) - obj["radius"]))
                    < 1e-14
                )
                assert np.max(np.abs(local[:, 2])) < obj["length"] / 2
                continue
            endpoints = (
                [(piece[0], piece[-1])] if i % 2 == 0 else zip(piece[:-1], piece[1:])
            )
            clearance = min(
                clearance, min(chord_minimum(obj, a, b) for a, b in endpoints)
            )
    assert clearance > -1e-10, clearance
    polyline = np.linalg.norm(np.diff(result["points"], axis=0), axis=1).sum()
    assert abs(polyline - result["length"]) < 1e-8
    singular = np.linalg.svd(
        result["points"] - result["points"].mean(axis=0), compute_uv=False
    )
    assert singular[-1] > 1e-3
    # Repeat the stationary solve after an arbitrary rigid-frame change.
    rt, shift = rotation(23, -37, 16), np.array([0.7, -0.4, 0.3])
    shifted = [
        dict(o, center=rt @ o["center"] + shift, rotation=rt @ o["rotation"])
        for o in objects
    ]
    invariant = solve(
        shifted, rt @ start + shift, rt @ end + shift, result["parameters"]
    )
    assert abs(invariant["length"] - result["length"]) < 1e-12
    return dict(
        tangent_direction_residual=result["residual"],
        minimum_other_object_or_free_clearance_m=clearance,
        dense_polyline_length_error_m=float(abs(polyline - result["length"])),
        noncoplanarity_rms_m=float(singular[-1] / np.sqrt(len(result["points"]))),
        rigid_frame_length_error_m=abs(invariant["length"] - result["length"]),
    )


def model_text(objects, start, end):
    number = lambda values: " ".join(f"{float(x):.17g}" for x in values)
    text = """version 4
segment base
endsegment
musclegroup path
    OriginParent base
    InsertionParent base
endmusclegroup
muscle route
    Type hill
    musclegroup path
"""
    text += f"    OriginPosition {number(start)}\n    InsertionPosition {number(end)}\n"
    text += """    optimalLength 0.5
    maximalForce 100
    tendonSlackLength 0.01
    pennationAngle 0
    maxVelocity 10
endmuscle
"""
    for i, obj in enumerate(objects):
        rt = np.eye(4)
        rt[:3, :3], rt[:3, 3] = obj["rotation"], obj["center"]
        text += f"wrapping cylinder-{i+1}\n    parent base\n    muscle route\n    musclegroup path\n    type cylinder\n    RTinMatrix 1\n    RT\n"
        text += "".join(f"        {number(row)}\n" for row in rt)
        text += f"    radius {obj['radius']}\n    length {obj['length']}\n    wrappingSide 1\nendwrapping\n"
    return text


def main():
    import biorbd

    centers = [[-0.10, 0.005, -0.008], [0, 0.017, 0.01], [0.10, -0.002, -0.005]]
    radii, heights = [0.032, 0.03, 0.027], [0.17, 0.18, 0.17]
    tilts = [[20, -8, 0], [-18, 12, 0], [10, 17, 0]]
    start, end = np.array([-0.22, -0.026, -0.055]), np.array([0.22, -0.018, 0.06])
    seed = np.tile([2.0, 1.0, 0.0, 0.0], (3, 1))
    for fraction in np.linspace(0, 1, 21):
        objects = [
            dict(
                kind="cylinder",
                center=np.array(c) * [1, 1, fraction],
                radius=r,
                length=h,
                rotation=rotation(*(fraction * np.array(t))),
            )
            for c, r, h, t in zip(centers, radii, heights, tilts)
        ]
        a, b = start * [1, 1, fraction], end * [1, 1, fraction]
        result = solve(objects, a, b, seed)
        seed = result["parameters"]
    metrics = validate(objects, result, start, end)
    path = OUT / "noncoplanar-cylinders.bioMod"
    path.write_text(model_text(objects, start, end), encoding="utf-8")
    model = biorbd.Model(str(path))
    model.updateMuscles(np.zeros(model.nbQ()), True)
    geometry = model.muscle(0).position()
    native_length = float(geometry.musculoTendonLength())
    native_nodes = np.array([p.to_array() for p in geometry.pointsInGlobal()])
    native_path = np.array([p.to_array() for p in geometry.pathPointsInGlobal()])
    metrics["native_reference_length_error_m"] = abs(native_length - result["length"])
    metrics["native_reference_contact_error_m"] = float(
        np.max(np.linalg.norm(native_nodes[1:-1] - result["contacts"], axis=1))
    )
    assert metrics["native_reference_length_error_m"] < 1e-9
    assert metrics["native_reference_contact_error_m"] < 2e-8
    meshes = []
    for i, obj in enumerate(objects):
        angle, axial = np.meshgrid(
            np.linspace(0, 2 * np.pi, 49),
            np.linspace(-obj["length"] / 2, obj["length"] / 2, 17),
        )
        xyz = np.array(
            [point(obj, t, z) for t, z in zip(angle.ravel(), axial.ravel())]
        ).reshape(angle.shape + (3,))
        meshes.append(
            dict(
                obj,
                center=obj["center"].tolist(),
                rotation=obj["rotation"].tolist(),
                name=f"Cylindre {i+1}",
                x=xyz[..., 0].tolist(),
                y=xyz[..., 1].tolist(),
                z=xyz[..., 2].tolist(),
            )
        )
    case = dict(
        key="cylinders-3d",
        reference_length=result["length"],
        reference_contacts=result["contacts"].tolist(),
        title="Trois cylindres : chaîne non coplanaire",
        detail="Axes inclinés indépendamment · rayons 32 / 30 / 27 mm",
        reference="C++ Eigen comparé à une résolution simultanée indépendante des hélices",
        length=native_length,
        direct_length=float(np.linalg.norm(end - start)),
        objects=meshes,
        points=native_path.tolist(),
        contacts=native_nodes[1:-1].tolist(),
        endpoints=[start.tolist(), end.tolist()],
        endpoint_labels=["Origine", "Insertion"],
        metrics=metrics,
    )
    (OUT / "cylinder-chain-data.json").write_text(
        json.dumps(case, ensure_ascii=False, separators=(",", ":"), allow_nan=False),
        encoding="utf-8",
    )
    (OUT / "cylinder-chain-validation.json").write_text(
        json.dumps(metrics, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(dict(length=native_length, **metrics), indent=2))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=Path.cwd() / "wrapping-reference-output"
    )
    OUT = parser.parse_args().output_dir
    OUT.mkdir(parents=True, exist_ok=True)
    main()
