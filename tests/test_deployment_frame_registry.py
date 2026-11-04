"""Common-frame startup registration and invalidation tests."""

import pytest

from docker.deployment_frame_registry import FrameRegistry, FrameSample


def s(time, x, y, lat, lon, sequence):
    return FrameSample(time, (x, y), (lat, lon), sequence)


def test_stationary_synchronized_pairs_establish_offset():
    registry = FrameRegistry(["a", "b"], "a", min_pairs=3)
    for i in range(3):
        t = 10.0 + i * 0.2
        registry.observe("a", s(t, 0, 0, 0, 0, i), t)
        registry.observe("b", s(t + 0.01, 0, 0, 0, 0.00001, i), t + 0.01)
    offsets = registry.offsets(10.5)
    assert offsets["a"] == pytest.approx((0, 0))
    assert offsets["b"] == pytest.approx((0, 1.1132), abs=0.001)
    registry.invalidate("b")
    assert "b" not in registry.offsets(10.5)
    assert registry.offsets(20) == {}


def test_inconsistent_origin_change_invalidates_registration():
    registry = FrameRegistry(["a", "b"], "a", min_pairs=3)
    for i in range(3):
        t = 10.0 + i * 0.2
        registry.observe("a", s(t, 0, 0, 0, 0, i), t)
        registry.observe("b", s(t, 0, 0, 0, 0.00001, i), t)
    assert "b" in registry.offsets(10.5)
    registry.observe("b", s(10.6, 4, 0, 0, 0.00001, 3), 10.6)
    assert "b" not in registry.offsets(10.6)


def test_bad_or_unsynchronized_data_never_registers():
    registry = FrameRegistry(["a", "b"], "a", min_pairs=3)
    for i in range(4):
        registry.observe("a", s(10 + i, 0, 0, 0, 0, i), 10 + i)
        registry.observe("b", s(10 + i + 0.5, 0, 0, 0, 0.00001, i), 10 + i + 0.5)
    assert "b" not in registry.offsets(13.5)
    with pytest.raises(ValueError):
        registry.observe("other", s(14, 0, 0, 0, 0, 0), 14)


def test_reference_origin_change_requires_explicit_registry_restart():
    registry = FrameRegistry(["a", "b"], "a", min_pairs=3)
    for i in range(3):
        t = 10.0 + i * 0.2
        registry.observe("a", s(t, 0, 0, 0, 0, i), t)
        registry.observe("b", s(t, 0, 0, 0, 0.00001, i), t)
    assert "b" in registry.offsets(10.5)
    registry.observe("a", s(10.6, 4, 0, 0, 0, 3), 10.6)
    assert registry.offsets(10.6) == {}
    assert registry.reference_fault
    registry.observe("a", s(10.7, 4, 0, 0, 0, 4), 10.7)
    assert registry.offsets(10.7) == {}
