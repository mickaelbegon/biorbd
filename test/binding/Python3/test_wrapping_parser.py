"""Regression coverage for wrapping dimensions and legacy parser expressions."""

import math
from pathlib import Path

import pytest

try:
    import biorbd
except ModuleNotFoundError:
    biorbd = None

MODEL_DIR = Path(__file__).resolve().parents[2] / "models"
pytestmark = pytest.mark.skipif(
    biorbd is None, reason="biorbd Python binding is unavailable"
)


VALID_CASES = [
    ("cylinder-equation", "cylinder", "radius $r\nlength $l"),
    ("cylinder-arithmetic", "cylinder", "radius pi/100\nlength 2*0.06"),
    ("ellipsoid-equation", "ellipsoid", "radii $r $r*2 ($l/3)"),
    ("torus-equation", "torus", "majorradius $l\nminorradius $r"),
    ("legacy-equation", "halfcylinder", "radius $r\nlength $l"),
    ("legacy-arithmetic", "halfcylinder", "radius pi/100\nlength 2*0.06"),
    ("legacy-zero", "halfcylinder", "radius 0.04\nlength 0"),
    ("legacy-length-omitted", "halfcylinder", "radius 0.04"),
]


INVALID_CASES = [
    (
        "cylinder-missing-radius",
        "cylinder",
        "length 0.12",
        "requires radius and length",
    ),
    (
        "cylinder-missing-length",
        "cylinder",
        "radius 0.04",
        "requires radius and length",
    ),
    ("cylinder-nan", "cylinder", "radius nan\nlength 0.12", "finite"),
    ("cylinder-inf", "cylinder", "radius inf\nlength 0.12", "finite"),
    ("cylinder-nonnum", "cylinder", "radius invalid\nlength 0.12", "Invalid numeric"),
    (
        "cylinder-partialnum",
        "cylinder",
        "radius 0.04oops\nlength 0.12",
        "Invalid numeric",
    ),
    (
        "cylinder-typo",
        "cylinder",
        "radiuss 0.04\nlength 0.12",
        "Unknown wrapping property: radiuss",
    ),
    ("cylinder-zero", "cylinder", "radius 0.04\nlength 0", "strictly positive"),
    ("ellipsoid-missing", "ellipsoid", "", "requires three radii"),
    ("ellipsoid-missing-component", "ellipsoid", "radii 0.03 0.04", "Invalid numeric"),
    ("ellipsoid-nan", "ellipsoid", "radii 0.03 nan 0.05", "finite"),
    ("ellipsoid-nonnum", "ellipsoid", "radii 0.03 invalid 0.05", "Invalid numeric"),
    ("ellipsoid-zero", "ellipsoid", "radii 0.03 0 0.05", "strictly positive"),
    (
        "ellipsoid-wrong-field",
        "ellipsoid",
        "radii 0.03 0.04 0.05\nradius 0.04",
        "only supports radii",
    ),
    ("torus-missing-major", "torus", "minorradius 0.04", "requires majorradius"),
    ("torus-missing-minor", "torus", "majorradius 0.12", "requires majorradius"),
    ("torus-nan", "torus", "majorradius nan\nminorradius 0.04", "finite"),
    (
        "torus-nonnum",
        "torus",
        "majorradius invalid\nminorradius 0.04",
        "Invalid numeric",
    ),
    (
        "torus-typo",
        "torus",
        "majorradius 0.12\nminorradiuss 0.04",
        "Unknown wrapping property: minorradiuss",
    ),
    (
        "torus-equal-radii",
        "torus",
        "majorradius 0.04\nminorradius 0.04",
        "majorradius > minorradius",
    ),
    (
        "torus-negative",
        "torus",
        "majorradius 0.12\nminorradius -0.04",
        "majorradius > minorradius",
    ),
]


@pytest.fixture
def wrapping_model_file(tmp_path):
    fixture = (MODEL_DIR / "wrapping_two_cylinders.bioMod").read_text()
    base = fixture[: fixture.index("wrapping left")]
    base = base.replace(
        "version 4", "version 4\nvariables\n $r 0.04\n $l 0.12\nendvariables"
    )

    def write_model(shape, dimensions, side="1"):
        path = tmp_path / "wrapping.bioMod"
        path.write_text(
            base
            + f"wrapping shape\n parent base\n muscle route\n musclegroup path\n type {shape}\n"
            + f"{dimensions}\nwrappingside {side}\nendwrapping\n"
        )
        return str(path)

    return write_model


@pytest.mark.parametrize(
    "name,shape,dimensions", VALID_CASES, ids=[case[0] for case in VALID_CASES]
)
def test_valid_wrapping_dimensions(wrapping_model_file, name, shape, dimensions):
    model = biorbd.Model(wrapping_model_file(shape, dimensions))
    modifiers = model.muscleGroup(0).muscle(0).pathModifier()
    assert modifiers.nbObjects() == 1
    if shape == "halfcylinder":
        cylinder = biorbd.WrappingHalfCylinder(modifiers.object(0))
        expected_radius = math.pi / 100 if name == "legacy-arithmetic" else 0.04
        expected_length = (
            0 if name in ("legacy-zero", "legacy-length-omitted") else 0.12
        )
        assert cylinder.radius() == pytest.approx(expected_radius, abs=1e-8, rel=0)
        assert cylinder.length() == pytest.approx(expected_length, abs=1e-8, rel=0)


@pytest.mark.parametrize(
    "name,shape,dimensions,expected",
    INVALID_CASES,
    ids=[case[0] for case in INVALID_CASES],
)
def test_invalid_wrapping_dimensions(
    wrapping_model_file, name, shape, dimensions, expected
):
    with pytest.raises(RuntimeError) as error:
        biorbd.Model(wrapping_model_file(shape, dimensions))
    assert expected in str(error.value)


@pytest.mark.parametrize("side", ["0", "1.0", "-2", "invalid"])
def test_invalid_new_wrapping_side(wrapping_model_file, side):
    with pytest.raises(RuntimeError) as error:
        biorbd.Model(wrapping_model_file("cylinder", "radius 0.04\nlength 0.12", side))
    assert "wrappingSide must be +1 or -1" in str(error.value)


@pytest.mark.parametrize("side", ["0", "1.0", "-2", "invalid"])
def test_ignored_legacy_wrapping_side(wrapping_model_file, side):
    biorbd.Model(wrapping_model_file("halfcylinder", "radius 0.04\nlength 0.12", side))
