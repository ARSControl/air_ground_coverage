from docker.mission_lease import lease_expiry


def lease():
    return dict(
        scenario_hash="s",
        session="run-a",
        vehicle_ids=["a", "b"],
        timestamp=10,
        expires_at=10.5,
        enabled=True,
    )


def test_accepts_only_short_current_matching_lease():
    assert lease_expiry(lease(), "s", "a", 10.1) == 10.5
    assert lease_expiry(lease(), "s", "other", 10.1) == 0
    assert lease_expiry(lease(), "wrong", "a", 10.1) == 0
    assert lease_expiry(lease(), "s", "a", 10.6) == 0
    assert lease_expiry(lease(), "s", "a", 9.9) == 0
    bad = lease()
    bad["expires_at"] = 20
    assert lease_expiry(bad, "s", "a", 10.1) == 0
    bad = lease()
    bad["enabled"] = False
    assert lease_expiry(bad, "s", "a", 10.1) == 0
    assert lease_expiry({}, "s", "a", 10.1) == 0
