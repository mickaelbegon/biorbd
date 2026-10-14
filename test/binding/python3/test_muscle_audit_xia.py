import numpy as np
import pytest
from muscle_audit_utils import backends, model_file, numeric


@pytest.mark.parametrize("b", backends)
@pytest.mark.parametrize("family", ["thelen", "degroote"])
def test_xia_force_uses_recruited_compartment(b, tmp_path, family):
    regular_path = model_file(tmp_path, **{"Type thelen": "Type " + family})
    regular = b.Model(regular_path)
    xia_path = model_file(
        tmp_path,
        **{
            "Type thelen": f"Type {family}fatigable\n    fatigueParameters\n        Type xia\n        fatigueRate 0\n    endfatigueParameters"
        },
    )
    xia = b.Model(xia_path)
    q, v = np.array([0.2]), np.array([0.0])
    regular.updateMuscles(q, v, True)
    xia.updateMuscles(q, v, True)
    cls = (
        b.HillThelenTypeFatigable if family == "thelen" else b.HillDeGrooteTypeFatigable
    )
    muscle = cls(xia.muscle(0))
    for recruited in [0.0, 0.2, 0.5, 1.0]:
        compartment = b.FatigueDynamicStateXia(recruited, 0, 1 - recruited)
        muscle.fatigueState().DeepCopy(compartment)
        reference = numeric(regular.muscle(0).force(b.State(0, recruited))).item()
        for command in [recruited, 1 - recruited]:
            np.testing.assert_allclose(
                numeric(muscle.force(b.State(0, command))), reference, atol=1e-9
            )
