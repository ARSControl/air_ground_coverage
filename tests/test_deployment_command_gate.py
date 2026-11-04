import pytest

from docker.deployment_command_gate import accepted_velocity


def packet():
    return dict(
        scenario_hash="s",
        vehicle_id="u",
        timestamp=10,
        expires_at=12,
        velocity=[0.3, 0.4],
        heading=0.2,
    )


def test_required_mission_lease_must_be_current():
    assert (
        accepted_velocity(
            packet(),
            "s",
            "u",
            10.5,
            registered=True,
            mission=True,
            lease_required=True,
            lease_expiry=10,
        )
        is None
    )
    assert (
        accepted_velocity(
            packet(),
            "s",
            "u",
            10.5,
            registered=True,
            mission=True,
            lease_required=True,
            lease_expiry=11,
        )
        is not None
    )


def test_accepts_fresh_registered_mission_and_preserves_velocity():
    assert accepted_velocity(
        packet(), "s", "u", 10.5, registered=True, mission=True
    ) == (0.3, 0.4, 0.2)


@pytest.mark.parametrize(
    "field,value",
    [
        ("scenario_hash", "wrong"),
        ("vehicle_id", "wrong"),
        ("timestamp", 11),
        ("expires_at", 10),
        ("velocity", [1, 0]),
        ("velocity", [float("nan"), 0]),
    ],
)
def test_rejects_invalid_plan(field, value):
    p = packet()
    p[field] = value
    assert accepted_velocity(p, "s", "u", 10.5, registered=True, mission=True) is None


def test_requires_registration_and_mission():
    assert (
        accepted_velocity(packet(), "s", "u", 10.5, registered=False, mission=True)
        is None
    )


def test_command_gate_checks_current_and_projected_map_bounds():
    args = dict(
        registered=True,
        mission=True,
        position=(9.8, 5),
        bounds=((0, 10), (0, 10)),
        margin=0.5,
        horizon=2,
    )
    assert accepted_velocity(packet(), "s", "u", 10.5, **args) is None
    args["position"] = (9, 5)
    assert accepted_velocity(packet(), "s", "u", 10.5, **args) is None
    args["position"] = (5, 5)
    assert accepted_velocity(packet(), "s", "u", 10.5, **args) == (0.3, 0.4, 0.2)


def test_command_gate_rejects_plan_from_prior_registration_session():
    p = packet()
    p["frame_session"] = "old"
    assert (
        accepted_velocity(
            p, "s", "u", 10.5, registered=True, mission=True, frame_session="new"
        )
        is None
    )
    p["frame_session"] = "new"
    assert (
        accepted_velocity(
            p, "s", "u", 10.5, registered=True, mission=True, frame_session="new"
        )
        is not None
    )
    assert (
        accepted_velocity(packet(), "s", "u", 10.5, registered=True, mission=False)
        is None
    )
