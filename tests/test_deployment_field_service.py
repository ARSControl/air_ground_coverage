"""Contracts for the transport-independent simulated field service."""

import numpy as np
import pytest

from src.core.deployment_field_service import FieldService, SensorSpec
from src.core.observations import Fidelity


def service():
    obstacle = np.zeros((21, 21), dtype=bool)
    obstacle[:, 11] = True
    return FieldService(
        scenario_id="scenario-a",
        high_field=np.ones((21, 21)),
        low_field=np.full((21, 21), 2.0),
        obstacle_map=obstacle,
        origin=(100, 200),
        resolution=2,
        sensors={
            "ground": SensorSpec(Fidelity.HIGH, 100, 20, 90, 0, 42, True),
            "air": SensorSpec(Fidelity.LOW, 100, 20, 360, 0, 43, False),
        },
    )


def test_fidelity_geometry_units_and_unoccluded_low():
    svc = service()
    high = svc.sample("scenario-a", "ground", "h1", 1, (120, 220), 0)
    low = svc.sample("scenario-a", "air", "l1", 1, (120, 220), 0)
    assert high and low
    assert all(o.position[0] < 121 for o in high)
    assert any(o.position[0] > 123 for o in low)
    assert all(o.fidelity is Fidelity.HIGH and o.value == 1 for o in high)
    assert all(o.fidelity is Fidelity.LOW and o.value == 2 for o in low)


def test_idempotency_and_conflicting_duplicate():
    svc = service()
    one = svc.sample("scenario-a", "ground", "1", 1, (120, 220), 0)
    assert svc.sample("scenario-a", "ground", "1", 1, (120, 220), 0) == one
    with pytest.raises(ValueError, match="conflict"):
        svc.sample("scenario-a", "ground", "1", 2, (120, 220), 0)
    with pytest.raises(ValueError, match="scenario"):
        svc.sample("wrong", "ground", "2", 1, (120, 220), 0)


def test_bad_inputs_fail_without_advancing_random_stream():
    a, b = service(), service()
    with pytest.raises(ValueError):
        a.sample("scenario-a", "ground", "bad", 1, (99, 220), 0)
    assert a.sample("scenario-a", "ground", "ok", 1, (120, 220), 0) == b.sample(
        "scenario-a", "ground", "ok", 1, (120, 220), 0
    )
