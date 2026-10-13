import pytest
from muscle_audit_utils import backends, model_file, numeric


@pytest.mark.parametrize("b", backends)
@pytest.mark.parametrize("tag", ["maxShorteningSpeed", "maxVelocity"])
def test_nondefault_velocity_parameter(b, tmp_path, tag):
    path = model_file(tmp_path, **{"maxShorteningSpeed 10": tag + " 5"})
    model = b.Model(path)
    assert numeric(model.muscle(0).characteristics().maxShorteningSpeed()).item() == 5


@pytest.mark.parametrize("b", backends)
def test_unknown_muscle_property_reports_typo(b, tmp_path):
    path = model_file(tmp_path, **{"maxShorteningSpeed 10": "maxShortningSpeed 5"})
    with pytest.raises(RuntimeError, match="maxShortningSpeed"):
        b.Model(path)
