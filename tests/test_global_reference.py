"""WGS84 shared reference conversion for frame registration."""

import numpy as np
import pytest

from src.core.global_reference import geodetic_to_local_ne


def test_reference_is_zero_and_north_east_are_oriented_correctly():
    ref = (0.0, 0.0, 0.0)
    np.testing.assert_allclose(geodetic_to_local_ne(ref, ref), (0, 0), atol=1e-9)
    north = geodetic_to_local_ne((0.00001, 0.0, 0.0), ref)
    east = geodetic_to_local_ne((0.0, 0.00001, 0.0), ref)
    assert north[0] == pytest.approx(1.1057, abs=0.001)
    assert abs(north[1]) < 1e-6
    assert east[1] == pytest.approx(1.1132, abs=0.001)
    assert abs(east[0]) < 1e-6


@pytest.mark.parametrize("bad", [(91, 0, 0), (0, 181, 0), (0, 0, float("nan"))])
def test_invalid_geodetic_coordinates_are_rejected(bad):
    with pytest.raises(ValueError):
        geodetic_to_local_ne(bad, (0, 0, 0))
