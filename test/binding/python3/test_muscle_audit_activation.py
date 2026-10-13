import numpy as np
import pytest
from muscle_audit_utils import backends, model_file, numeric


@pytest.mark.parametrize("b", backends)
@pytest.mark.parametrize("raw_excitation", [0.8, 1.6])
def test_raw_degroot_excitation_matches_normalized(b, tmp_path, raw_excitation):
    path = model_file(
        tmp_path,
        **{
            "Type thelen": "Type thelen\n    StateType degroote\n    maximalExcitation 2"
        },
    )
    model = b.Model(path)
    raw = model.stateSet()
    # stateSet shares the internal states, so use independent models.
    normalized_model = b.Model(path)
    normalized = normalized_model.stateSet()
    raw[0].setActivation(0.5)
    raw[0].setExcitation(raw_excitation)
    normalized[0].setActivation(0.5)
    normalized[0].setExcitation(raw_excitation / 2)
    actual = numeric(model.activationDot(raw, False)).item()
    expected = numeric(normalized_model.activationDot(normalized, True)).item()
    np.testing.assert_allclose(actual, expected, atol=1e-12)
    assert (actual > 0) == (raw_excitation / 2 > 0.5)
    assert numeric(raw[0].excitation()).item() == raw_excitation
