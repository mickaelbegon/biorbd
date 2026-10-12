"""Time real Eigen updates or CasADi Function evaluations, including kinematics.

Run before and after the hybrid build using the matching fresh Python binding.
No plotting or model/function creation is included in the timed evaluations.
"""

import argparse
import importlib
import json
from pathlib import Path
import platform
import time

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("eigen", "casadi"), default="eigen")
    parser.add_argument(
        "--models",
        type=Path,
        default=Path(__file__).resolve().parents[2] / "test" / "models",
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--label", default="current-build")
    parser.add_argument("--repeats", type=int, default=25)
    args = parser.parse_args()
    if args.repeats < 2:
        parser.error("--repeats must be >= 2")
    br = importlib.import_module(
        "biorbd" if args.backend == "eigen" else "biorbd_casadi"
    )
    expected_backend = 0 if args.backend == "eigen" else 1
    if br.currentLinearAlgebraBackend() != expected_backend:
        raise RuntimeError("The imported binding does not match --backend")
    definitions = [
        ("cylinder", 0.3492105912719195),
        ("ellipsoid", 0.35419781480619866),
        ("torus_tube", 0.14470526426579974),
        ("torus_hole", 0.2),
        ("ellipsoid_3d_moving", 0.3589189402510813),
        ("torus_3d_moving", 0.18982123405216328),
        ("three_cylinders_3d", 0.48235866649711817),
        ("mixed_3d_moving", 0.4999204268042869),
    ]
    rows = []
    for name, reference in definitions:
        model = br.Model(str(args.models / f"wrapping_{name}.bioMod"))
        q = np.zeros(model.nbQ())
        v = np.linspace(-0.3, 0.4, model.nbQ())
        if args.backend == "eigen":

            def evaluate():
                model.updateMuscles(q, v, True)
                geom = model.muscle(0).position()
                return (
                    float(geom.musculoTendonLength()),
                    np.asarray(geom.jacobianLength().to_array()).reshape(-1),
                    float(geom.velocity()),
                )

        else:
            import casadi as ca

            qs = ca.MX.sym(name + "_q", model.nbQ())
            vs = ca.MX.sym(name + "_v", model.nbQ())
            model.updateMuscles(qs, vs, True)
            geom = model.muscle(0).position()

            def mx(value):
                return value.to_mx() if hasattr(value, "to_mx") else ca.MX(value)

            fun = ca.Function(
                name,
                [qs, vs],
                [
                    mx(geom.musculoTendonLength()),
                    mx(geom.jacobianLength()),
                    mx(geom.velocity()),
                ],
            )

            def evaluate():
                length, jac, velocity = fun(q, v)
                return float(length), np.asarray(jac).reshape(-1), float(velocity)

        evaluate()
        evaluate()
        measurements = []
        block_medians = []
        for block in range(3):
            block_times = []
            for index in range(args.repeats):
                start = time.perf_counter_ns()
                L, J, V = evaluate()
                duration = (time.perf_counter_ns() - start) * 1e-6
                if abs(L - reference) > 1e-9:
                    raise RuntimeError(f"{name}: wrong length {L}")
                if abs(V - J @ v) > 1e-10:
                    raise RuntimeError(f"{name}: velocity/gradient mismatch")
                block_times.append(duration)
            measurements.extend(block_times)
            block_medians.append(float(np.median(block_times)))
        rows.append(
            dict(
                case=name,
                nq=model.nbQ(),
                length_m=L,
                gradient=J.tolist(),
                velocity=V,
                n=len(measurements),
                median_ms=float(np.median(measurements)),
                p10_ms=float(np.percentile(measurements, 10)),
                p90_ms=float(np.percentile(measurements, 90)),
                block_medians_ms=block_medians,
                samples_ms=measurements,
            )
        )
        print(
            args.label,
            args.backend,
            name,
            round(rows[-1]["median_ms"], 4),
            "ms",
            flush=True,
        )
    args.output.write_text(
        json.dumps(
            dict(
                label=args.label,
                backend=args.backend,
                scope="Model update + output extraction or CasADi Function L,J,v evaluation; repeated zero Q; three serial blocks; setup/graph creation excluded; native seeds reused when accepted; SG state is cold per call.",
                platform=platform.platform(),
                machine=platform.machine(),
                python=platform.python_version(),
                binding=br.__file__,
                cases=rows,
            ),
            indent=2,
        )
        + "\n"
    )


if __name__ == "__main__":
    main()
