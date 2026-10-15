"""Optimize a small insertion displacement with the shared wrapping solver.

Run from a source checkout with CasADi, its IPOPT plugin, and the freshly built
biorbd_casadi Python package. The callback supplies a stationary length gradient;
IPOPT's 'exact' Hessian interface receives finite differences of that gradient.
"""

import argparse
import json
from pathlib import Path

import biorbd_casadi as biorbd
import casadi as ca
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Optional JSON validation report")
    args = parser.parse_args()
    models = Path(__file__).resolve().parents[2] / "test" / "models"
    results = []
    for kind, baseline in (
        ("ellipsoid", 0.3589189402510813),
        ("mixed", 0.4999204268042869),
    ):
        model = biorbd.Model(str(models / f"wrapping_{kind}_3d_moving.bioMod"))
        q = ca.MX.sym("q", model.nbQ())
        model.updateMuscles(q, True)
        length = model.muscle(0).position().musculoTendonLength().to_mx()
        length_function = ca.Function("length_" + kind, [q], [length])

        # Move only the insertion's x coordinate. All object poses stay fixed.
        displacement = ca.MX.sym("dx")
        physical_q = ca.vertcat(ca.MX.zeros(6, 1), displacement, ca.MX.zeros(2, 1))
        target = baseline + 0.0005  # Increase length by 0.5 mm.
        objective = ((length_function(physical_q) - target) / 0.001) ** 2
        for method in ("exact", "limited-memory"):
            solver = ca.nlpsol(
                "solve_" + kind + "_" + method.replace("-", "_"),
                "ipopt",
                {"x": displacement, "f": objective},
                {
                    "ipopt.print_level": 0,
                    "print_time": False,
                    "ipopt.tol": 1e-9,
                    "ipopt.max_iter": 50,
                    "ipopt.hessian_approximation": method,
                },
            )
            optimum = solver(x0=0, lbx=-0.002, ubx=0.002)
            stats = solver.stats()
            numeric_q = np.zeros(9)
            numeric_q[6] = float(optimum["x"])
            error = float(length_function(numeric_q)) - target
            if not stats["success"] or abs(error) > 1e-8:
                raise RuntimeError(
                    f"{kind}/{method}: {stats['return_status']}, error {error} m"
                )
            results.append(
                {
                    "shape": kind,
                    "hessian": (
                        "gradient_finite_difference" if method == "exact" else method
                    ),
                    "status": stats["return_status"],
                    "iterations": stats["iter_count"],
                    "insertion_dx_m": numeric_q[6],
                    "length_error_m": error,
                }
            )
    report = json.dumps({"casadi": ca.__version__, "results": results}, indent=2)
    if args.output:
        args.output.write_text(report + "\n", encoding="utf-8")
    print(report)


if __name__ == "__main__":
    main()
