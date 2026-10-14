"""Shared numeric conversions for the native muscle binding regressions."""

import importlib
from pathlib import Path

import numpy as np
import pytest

try:
    import casadi as ca
except ModuleNotFoundError:
    ca = None

backends = []
for name in ("biorbd", "biorbd_casadi"):
    try:
        # Use the native module even when the package exports the wrapper API.
        backends.append(importlib.import_module(name + ".biorbd"))
    except ModuleNotFoundError:
        pass
if not backends:
    raise RuntimeError("No biorbd backend could be imported")

fixture = Path(__file__).resolve().parents[2] / "models/muscle_audit.bioMod"


def numeric(value):
    if hasattr(value, "to_array"):
        return np.asarray(value.to_array())
    if hasattr(value, "to_mx"):
        value = value.to_mx()
    if ca is not None and isinstance(value, ca.MX):
        return np.asarray(ca.Function("evaluate", [], [value])()["o0"])
    return np.asarray(value)


def model_file(tmp_path, **changes):
    text = fixture.read_text()
    for old, new in changes.items():
        assert old in text
        text = text.replace(old, new)
    path = tmp_path / "muscle_audit.bioMod"
    path.write_text(text)
    return str(path)
