"""Reproduce ten wrapping references with the Eigen backend, in metres.

Run from the repository with its freshly compiled Python module:
    PYTHONPATH=build/eigen/binding/python3 python examples/python3/muscleWrapping.py
Use --output /path/to/results.json for coordinates suitable for a 3-D viewer.
The generated .bioMod examples are stored beside the script in ../muscleWrapping.
"""

import argparse
import json
from pathlib import Path

import biorbd
import numpy as np


def rotation(x=0, y=0, z=0):
    x, y, z = np.deg2rad([x, y, z])
    rx = np.array([[1, 0, 0], [0, np.cos(x), -np.sin(x)], [0, np.sin(x), np.cos(x)]])
    ry = np.array([[np.cos(y), 0, np.sin(y)], [0, 1, 0], [-np.sin(y), 0, np.cos(y)]])
    rz = np.array([[np.cos(z), -np.sin(z), 0], [np.sin(z), np.cos(z), 0], [0, 0, 1]])
    return rz @ ry @ rx


def cases():
    return [
        dict(
            key="cylinder",
            rotation=rotation(y=18, z=-10),
            start=[-0.15, -0.01, -0.065],
            end=[0.15, -0.01, 0.065],
            reference=0.3492105912719195,
            objects=[dict(kind="cylinder", center=[0, 0, 0], radius=0.05, length=0.18)],
        ),
        dict(
            key="ellipsoid",
            rotation=rotation(x=25, z=-8),
            start=[-0.17, -0.01, 0],
            end=[0.17, 0.005, 0],
            reference=0.35419781480619866,
            objects=[
                dict(kind="ellipsoid", center=[0, 0, 0], radii=[0.07, 0.045, 0.055])
            ],
        ),
        dict(
            key="torus-tube",
            rotation=rotation(),
            start=[0, 0, -0.01],
            end=[0.13, 0, -0.01],
            reference=0.14470526426579974,
            objects=[
                dict(
                    kind="torus", center=[0, 0, 0], majorradius=0.065, minorradius=0.021
                )
            ],
        ),
        dict(
            key="torus-hole",
            rotation=rotation(),
            start=[0, 0, -0.1],
            end=[0, 0, 0.1],
            reference=0.2,
            objects=[
                dict(
                    kind="torus", center=[0, 0, 0], majorradius=0.065, minorradius=0.021
                )
            ],
        ),
        dict(
            key="two-cylinders",
            rotation=rotation(x=22, z=-7),
            start=[-0.18, -0.025, 0],
            end=[0.18, -0.015, 0],
            reference=0.39645033645390365,
            objects=[
                dict(
                    kind="cylinder",
                    center=[-0.065, -0.005, 0],
                    radius=0.04,
                    length=0.12,
                ),
                dict(
                    kind="cylinder", center=[0.065, 0.015, 0], radius=0.038, length=0.12
                ),
            ],
        ),
        dict(
            key="mixed",
            rotation=rotation(x=24, z=-6),
            start=[-0.23, -0.025, 0],
            end=[0.23, -0.025, 0],
            reference=0.49902788712585605,
            objects=[
                dict(
                    kind="ellipsoid", center=[-0.12, 0, 0], radii=[0.048, 0.032, 0.044]
                ),
                dict(kind="cylinder", center=[0, 0.017, 0], radius=0.034, length=0.10),
                dict(
                    kind="torus",
                    center=[0.12, 0, 0],
                    majorradius=0.030,
                    minorradius=0.015,
                ),
            ],
        ),
        dict(
            key="ellipsoid-3d",
            rotation=rotation(),
            start=[-0.17, -0.01, -0.025],
            end=[0.17, 0.005, 0.035],
            reference=0.3589189402510813,
            objects=[
                dict(kind="ellipsoid", center=[0, 0, 0], radii=[0.07, 0.045, 0.055])
            ],
        ),
        dict(
            key="torus-3d",
            rotation=rotation(),
            start=[0, -0.038, -0.03],
            end=[0.13, 0.07, -0.01],
            reference=0.18982123405216328,
            objects=[
                dict(
                    kind="torus", center=[0, 0, 0], majorradius=0.065, minorradius=0.021
                )
            ],
        ),
        dict(
            key="three-cylinders-3d",
            rotation=rotation(),
            start=[-0.22, -0.026, -0.055],
            end=[0.22, -0.018, 0.06],
            reference=0.48235866649711817,
            objects=[
                dict(
                    kind="cylinder",
                    center=[-0.10, 0.005, -0.008],
                    radius=0.032,
                    length=0.17,
                    rotation=rotation(20, -8),
                ),
                dict(
                    kind="cylinder",
                    center=[0, 0.017, 0.01],
                    radius=0.03,
                    length=0.18,
                    rotation=rotation(-18, 12),
                ),
                dict(
                    kind="cylinder",
                    center=[0.10, -0.002, -0.005],
                    radius=0.027,
                    length=0.17,
                    rotation=rotation(10, 17),
                ),
            ],
        ),
        dict(
            key="mixed-3d",
            rotation=rotation(),
            start=[-0.23, -0.025, -0.015],
            end=[0.23, -0.025, 0.015],
            reference=0.4999204268042869,
            objects=[
                dict(
                    kind="ellipsoid",
                    center=[-0.12, 0, -0.006],
                    radii=[0.048, 0.032, 0.044],
                    rotation=rotation(3, -5),
                ),
                dict(
                    kind="cylinder",
                    center=[0, 0.017, 0.004],
                    radius=0.034,
                    length=0.10,
                    rotation=rotation(-5, 3),
                ),
                dict(
                    kind="torus",
                    center=[0.12, 0, -0.002],
                    majorradius=0.030,
                    minorradius=0.015,
                    rotation=rotation(5, -3),
                ),
            ],
        ),
    ]


def numbers(values):
    return " ".join(f"{float(v):.17g}" for v in values)


def model_text(case):
    rot = case["rotation"]
    start, end = rot @ case["start"], rot @ case["end"]
    text = """version 4

segment base
    RTinMatrix 1
    RT
        1 0 0 0
        0 1 0 0
        0 0 1 0
        0 0 0 1
endsegment

musclegroup path
    OriginParent base
    InsertionParent base
endmusclegroup

muscle route
    Type hill
    musclegroup path
"""
    text += (
        f"    OriginPosition {numbers(start)}\n    InsertionPosition {numbers(end)}\n"
    )
    text += """    optimalLength 0.5
    maximalForce 100
    tendonSlackLength 0.01
    pennationAngle 0
    maxVelocity 10
endmuscle
"""
    for index, obj in enumerate(case["objects"]):
        rt = np.eye(4)
        rt[:3, :3] = rot @ obj.get("rotation", np.eye(3))
        rt[:3, 3] = rot @ obj["center"]
        text += f"\nwrapping obstacle-{index + 1}\n    parent base\n    muscle route\n    musclegroup path\n    type {obj['kind']}\n    RTinMatrix 1\n    RT\n"
        text += "".join(f"        {numbers(row)}\n" for row in rt)
        for key in ("radius", "length", "radii", "majorradius", "minorradius"):
            if key in obj:
                value = obj[key] if isinstance(obj[key], list) else [obj[key]]
                text += f"    {key} {numbers(value)}\n"
        text += "    wrappingSide 1\nendwrapping\n"
    return text


def evaluate(case, path):
    model = biorbd.Model(str(path))
    model.updateMuscles(np.zeros(model.nbQ()), True)
    geometry = model.muscle(0).position()
    length = float(geometry.musculoTendonLength())
    points = np.array([p.to_array() for p in geometry.pathPointsInGlobal()])
    nodes = np.array([p.to_array() for p in geometry.pointsInGlobal()])
    error = abs(length - case["reference"])
    if error > 1e-9:
        raise AssertionError(f"{case['key']}: reference length error {error:.3g} m")
    if abs(np.linalg.norm(np.diff(points, axis=0), axis=1).sum() - length) > 1e-7:
        raise AssertionError(
            f"{case['key']}: sampled trajectory does not reproduce length"
        )
    return dict(
        key=case["key"],
        length=length,
        reference_length=case["reference"],
        reference_error_m=error,
        points=points.tolist(),
        contacts=nodes[1:-1].tolist(),
        endpoints=nodes[[0, -1]].tolist(),
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Save native trajectories as JSON")
    parser.add_argument(
        "--models-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "muscleWrapping",
    )
    args = parser.parse_args()
    args.models_dir.mkdir(parents=True, exist_ok=True)
    results = []
    for case in cases():
        path = args.models_dir / f"{case['key']}.bioMod"
        text = model_text(case)
        if not path.exists() or path.read_text() != text:
            path.write_text(text)
        result = evaluate(case, path)
        results.append(result)
        print(
            f"{case['key']}: L={1000 * result['length']:.9f} mm; "
            f"reference error={result['reference_error_m']:.3g} m; "
            f"contacts={len(result['contacts'])}"
        )
    if args.output:
        args.output.write_text(json.dumps(results, indent=2))


if __name__ == "__main__":
    main()
