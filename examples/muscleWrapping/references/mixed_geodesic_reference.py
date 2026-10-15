"""Independent continuation of a spatial ellipsoid/cylinder/torus chain, metres.

Reference-only SciPy shooting and an exact developed-cylinder helix. A fixed
upper branch/contact order is followed; no global shortest-path claim is made.
"""

import json
from pathlib import Path

import numpy as np
from scipy.integrate import quad
from scipy.optimize import least_squares

from geodesic_reference import Surface, solve_contact
from cylinder_chain_reference import rotation, chord_minimum

OUT = Path(__file__).resolve().parent


def initial_chain():
    centers = np.array([[-0.12, 0], [0, 0.017], [0.12, 0]])
    axes = np.array([[0.048, 0.032], [0.034, 0.034], [0.045, 0.045]])
    a, b = np.array([-0.23, -0.025]), np.array([0.23, -0.025])

    def contacts(angles):
        return centers[:, None, :] + axes[:, None, :] * np.stack(
            [np.cos(angles), np.sin(angles)], axis=-1
        )

    def residual(parameters):
        angles = parameters.reshape(3, 2)
        points = contacts(angles)
        normal = np.stack([np.cos(angles), np.sin(angles)], axis=-1) / axes[:, None, :]
        errors = []
        for i in range(3):
            incoming = points[i, 0] - (a if i == 0 else points[i - 1, 1])
            outgoing = (b if i == 2 else points[i + 1, 0]) - points[i, 1]
            errors.extend(
                [
                    incoming
                    @ normal[i, 0]
                    / np.linalg.norm(incoming)
                    / np.linalg.norm(normal[i, 0]),
                    outgoing
                    @ normal[i, 1]
                    / np.linalg.norm(outgoing)
                    / np.linalg.norm(normal[i, 1]),
                ]
            )
        return errors

    fit = least_squares(
        residual,
        np.tile([2.0, 1.0], 3),
        bounds=(1e-5, np.pi - 1e-5),
        xtol=1e-14,
        ftol=1e-14,
        gtol=1e-14,
    )
    assert np.max(np.abs(residual(fit.x))) < 1e-12
    angles = fit.x.reshape(3, 2)
    points = np.concatenate([contacts(angles), np.zeros((3, 2, 1))], axis=2)
    seeds = []
    for i in (0, 2):
        rx, ry = axes[i]
        length = quad(
            lambda t: np.hypot(rx * np.sin(t), ry * np.cos(t)),
            angles[i, 1],
            angles[i, 0],
            epsabs=1e-13,
            epsrel=1e-13,
        )[0]
        seeds.append(
            dict(angles=[float(angles[i, 0]), 0.0], arc_length_m=float(length))
        )
    return points, seeds


def cylinder_contact(obj, a, b):
    a, b = (a - obj["center"]) @ obj["rotation"], (b - obj["center"]) @ obj["rotation"]
    r = obj["radius"]
    na, nb = np.linalg.norm(a[:2]) / r, np.linalg.norm(b[:2]) / r
    assert min(na, nb) > 1
    entry = np.arctan2(a[1], a[0]) - np.arccos(1 / na)
    exit_ = np.arctan2(b[1], b[0]) + np.arccos(1 / nb)
    while exit_ > entry:
        exit_ -= 2 * np.pi
    xy0, xy1 = r * np.array([np.cos(entry), np.sin(entry)]), r * np.array(
        [np.cos(exit_), np.sin(exit_)]
    )
    free0, free1 = np.linalg.norm(xy0 - a[:2]), np.linalg.norm(b[:2] - xy1)
    arc = r * (entry - exit_)
    planar = free0 + arc + free1
    z0, z1 = (
        a[2] + (b[2] - a[2]) * free0 / planar,
        a[2] + (b[2] - a[2]) * (free0 + arc) / planar,
    )
    assert max(abs(z0), abs(z1)) < obj["length"] / 2
    t, z = np.linspace(entry, exit_, 801), np.linspace(z0, z1, 801)
    curve = (
        np.column_stack([r * np.cos(t), r * np.sin(t), z]) @ obj["rotation"].T
        + obj["center"]
    )
    return dict(
        curve=curve, contacts=curve[[0, -1]], arc_length=float(np.hypot(arc, z1 - z0))
    )


def configuration(fraction):
    ellipsoid = Surface(
        "ellipsoid",
        [0.048, 0.032, 0.044],
        [-0.12, 0, -0.006 * fraction],
        rotation(3 * fraction, -5 * fraction),
    )
    cylinder = dict(
        kind="cylinder",
        center=np.array([0, 0.017, 0.004 * fraction]),
        radius=0.034,
        length=0.1,
        rotation=rotation(-5 * fraction, 3 * fraction),
    )
    torus = Surface(
        "torus",
        [0.03, 0.015],
        [0.12, 0, -0.002 * fraction],
        rotation(5 * fraction, -3 * fraction),
    )
    a, b = np.array([-0.23, -0.025, -0.015 * fraction]), np.array(
        [0.23, -0.025, 0.015 * fraction]
    )
    return [ellipsoid, cylinder, torus], a, b


def chain_solve(objects, a, b, contacts, seeds, tolerance=2e-11):
    contacts, seeds = contacts.copy(), [dict(s) for s in seeds]
    results = [None] * 3
    for sweep in range(80):
        previous_contacts = contacts.copy()
        for i in range(3):
            previous, following = a if i == 0 else contacts[i - 1, 1], (
                b if i == 2 else contacts[i + 1, 0]
            )
            if i == 1:
                result = cylinder_contact(objects[i], previous, following)
            else:
                seed_index = 0 if i == 0 else 1
                result = solve_contact(
                    objects[i],
                    previous,
                    following,
                    seeds[seed_index],
                    rtol=2e-12,
                    atol=2e-14,
                    count=401,
                )
                seeds[seed_index] = result["seed"]
            results[i] = result
            contacts[i] = result["contacts"]
        if np.max(np.linalg.norm(contacts - previous_contacts, axis=2)) <= tolerance:
            break
    else:
        raise RuntimeError("Independent mixed contact sweep did not converge")
    lengths, pieces, previous = [], [], a
    for i, result in enumerate(results):
        lengths.extend(
            [np.linalg.norm(contacts[i, 0] - previous), result["arc_length"]]
        )
        pieces.extend([np.linspace(previous, contacts[i, 0], 101), result["curve"]])
        previous = contacts[i, 1]
    lengths.append(np.linalg.norm(b - previous))
    pieces.append(np.linspace(previous, b, 101))
    return dict(
        contacts=contacts,
        seeds=seeds,
        results=results,
        pieces=pieces,
        points=np.vstack(pieces),
        length=float(sum(lengths)),
        sweeps=sweep + 1,
    )


def validate(objects, a, b, result):
    minimum_level, minimum_cylinder_clearance = np.inf, np.inf
    for piece_index, piece in enumerate(result["pieces"]):
        for i, obj in enumerate(objects):
            if piece_index % 2 and i == (piece_index - 1) // 2:
                continue
            chords = (
                [(piece[0], piece[-1])]
                if piece_index % 2 == 0
                else zip(piece[:-1], piece[1:])
            )
            for p, q in chords:
                level = (
                    chord_minimum(obj, p, q)
                    if i == 1
                    else obj.chord_minimum_level(obj.local(p), obj.local(q))
                )
                if i == 1:
                    minimum_cylinder_clearance = min(minimum_cylinder_clearance, level)
                else:
                    minimum_level = min(minimum_level, level)
    assert minimum_level >= -2e-8, minimum_level
    assert minimum_cylinder_clearance >= -1e-10, minimum_cylinder_clearance
    errors = []
    for i, r in enumerate(result["results"]):
        p, q = result["contacts"][i]
        previous, following = a if i == 0 else result["contacts"][i - 1, 1], (
            b if i == 2 else result["contacts"][i + 1, 0]
        )
        if i != 1:
            s = objects[i]
            for point, direction in ((p, p - previous), (q, following - q)):
                normal = s.geometry(s.local(point))[1] @ s.rotation.T
                errors.append(
                    abs(
                        normal
                        @ direction
                        / np.linalg.norm(normal)
                        / np.linalg.norm(direction)
                    )
                )
        else:
            obj = objects[i]
            for point, direction in ((p, p - previous), (q, following - q)):
                local = (point - obj["center"]) @ obj["rotation"]
                normal = np.array([local[0], local[1], 0.0]) @ obj["rotation"].T
                errors.append(
                    abs(
                        normal
                        @ direction
                        / np.linalg.norm(normal)
                        / np.linalg.norm(direction)
                    )
                )
    assert max(errors) < 1e-8, errors
    return dict(
        tangency_residual=max(errors),
        minimum_normalized_implicit_level=minimum_level,
        minimum_cylinder_clearance_m=minimum_cylinder_clearance,
        independent_contact_sweeps=result["sweeps"],
    )


def model_text(objects, a, b, moving=False):
    num = lambda values: " ".join(f"{float(x):.17g}" for x in values)
    text = "version 4\nsegment base\nendsegment\n"
    if moving:
        text += f"segment assembly\n    parent base\n    translations xyz\n    rotations xyz\nendsegment\nsegment insertion\n    parent base\n    RT 0 0 0 xyz {num(b)}\n    translations xyz\nendsegment\n"
    text += f'musclegroup path\n    OriginParent base\n    InsertionParent {"insertion" if moving else "base"}\nendmusclegroup\nmuscle route\n    Type hill\n    musclegroup path\n    OriginPosition {num(a)}\n    InsertionPosition {"0 0 0" if moving else num(b)}\n    optimalLength 0.5\n    maximalForce 100\n    tendonSlackLength 0.01\n    pennationAngle 0\n    maxVelocity 10\nendmuscle\n'
    for i, obj in enumerate(objects):
        kind = "cylinder" if i == 1 else obj.kind
        center, rot = (
            (obj["center"], obj["rotation"]) if i == 1 else (obj.center, obj.rotation)
        )
        rt = np.eye(4)
        rt[:3, :3], rt[:3, 3] = rot, center
        text += (
            f'wrapping object-{i+1}\n    parent {"assembly" if moving else "base"}\n    muscle route\n    musclegroup path\n    type {kind}\n    RTinMatrix 1\n    RT\n'
            + "".join(f"        {num(row)}\n" for row in rt)
        )
        if i == 0:
            text += f"    radii {num(obj.dimensions)}\n"
        elif i == 1:
            text += f"    radius {obj['radius']}\n    length {obj['length']}\n"
        else:
            text += f"    majorradius {obj.dimensions[0]}\n    minorradius {obj.dimensions[1]}\n"
        text += "    wrappingSide 1\nendwrapping\n"
    return text


def main():
    contacts, seeds = initial_chain()
    for fraction in np.linspace(0, 1, 13):
        objects, a, b = configuration(fraction)
        result = chain_solve(objects, a, b, contacts, seeds)
        contacts, seeds = result["contacts"], result["seeds"]
        if fraction == 0:
            assert abs(result["length"] - 0.49902788712585605) < 1e-9
        print(
            f'continuation={fraction:.3f} L={result["length"]:.15f} sweeps={result["sweeps"]}',
            flush=True,
        )
    refined = chain_solve(objects, a, b, contacts, seeds, tolerance=2e-13)
    metrics = validate(objects, a, b, refined)
    metrics["refined_length_difference_m"] = abs(refined["length"] - result["length"])
    assert metrics["refined_length_difference_m"] < 1e-10
    for moving in (False, True):
        (OUT / f'mixed-3d{"-moving" if moving else ""}.bioMod').write_text(
            model_text(objects, a, b, moving), encoding="utf-8"
        )
    data = dict(
        key="mixed-3d",
        length=refined["length"],
        contacts=refined["contacts"].reshape(-1, 3).tolist(),
        points=refined["points"].tolist(),
        endpoints=[a.tolist(), b.tolist()],
        metrics=metrics,
    )
    (OUT / "mixed-geodesic-data.json").write_text(
        json.dumps(data, indent=2, allow_nan=False), encoding="utf-8"
    )
    print(json.dumps(dict(length=data["length"], **metrics), indent=2))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir", type=Path, default=Path.cwd() / "wrapping-reference-output"
    )
    OUT = parser.parse_args().output_dir
    OUT.mkdir(parents=True, exist_ok=True)
    main()
