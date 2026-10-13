import numpy as np
import pytest
from muscle_audit_utils import backends, model_file, numeric


@pytest.mark.parametrize("b", backends)
@pytest.mark.parametrize("family", ["hill", "thelen", "degroote"])
def test_explicit_velocity_units_and_length_scale(b, tmp_path, family):
    forces = []
    for scale, units, speed in [
        (1, "optimal_lengths_per_second", 10),
        (1, "length_units_per_second", 1),
        (2, "optimal_lengths_per_second", 10),
    ]:
        path = model_file(
            tmp_path,
            **{
                "Type thelen": "Type " + family,
                "optimalLength 0.1": "optimalLength " + str(0.1 * scale),
                "tendonSlackLength 0.1": "tendonSlackLength " + str(0.1 * scale),
                "maxShorteningSpeed 10": f"maxShorteningSpeed {speed}\n    maxShorteningSpeedUnits {units}\n    useDamping 1",
            },
        )
        model = b.Model(path)
        model.updateMuscles(np.array([0.2 * scale]), np.array([0.1 * scale]), True)
        state = model.stateSet()
        state[0].setActivation(0.5)
        forces.append(numeric(model.muscleForces(state)).item())
        cls = {
            "hill": b.HillType,
            "thelen": b.HillThelenType,
            "degroote": b.HillDeGrooteType,
        }[family]
        muscle = cls(model.muscle(0))
        assert numeric(muscle.maxShorteningSpeedInLengthUnits()).item() == scale
        characteristic = muscle.characteristics()
        characteristic.setMaxShorteningSpeedUnits(b.LENGTH_UNITS_PER_SECOND)
        assert (
            characteristic.DeepCopy().maxShorteningSpeedUnits()
            == b.LENGTH_UNITS_PER_SECOND
        )
    np.testing.assert_allclose(forces, forces[0], atol=1e-9)


@pytest.mark.parametrize("b", backends)
def test_invalid_velocity_units(b, tmp_path):
    path = model_file(
        tmp_path,
        **{
            "maxShorteningSpeed 10": "maxShorteningSpeed 10\n    maxShorteningSpeedUnits invalid"
        },
    )
    with pytest.raises(RuntimeError, match="units"):
        b.Model(path)


@pytest.mark.parametrize("b", backends)
def test_wrapper_velocity_units_property(b):
    import sys
    import importlib

    if sys.version_info < (3, 12):
        pytest.skip("Wrappers require Python 3.12 or newer")
    from muscle_audit_utils import fixture

    wrapper = importlib.import_module(b.__package__ + ".wrapper")
    model = wrapper.Biorbd(str(fixture))
    muscle = model.muscles[0]
    assert muscle.maximal_contraction_velocity_units == b.LEGACY_SPEED_UNITS
    muscle.maximal_contraction_velocity_units = b.OPTIMAL_LENGTHS_PER_SECOND
    assert muscle.maximal_contraction_velocity_units == b.OPTIMAL_LENGTHS_PER_SECOND
