"""Convert WGS84 geodetic positions into a local tangent north/east frame.

Altitude inputs are ellipsoidal metres, not automatically MSL heights. Source
accuracy, time synchronization and frame-reset handling are caller duties.
"""

import math

import numpy as np

_SEMI_MAJOR_M = 6378137.0
_ECCENTRICITY_SQUARED = 6.6943799901413165e-3


def _geodetic(value):
    try:
        lat, lon, height = (float(x) for x in value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            "position must contain latitude, longitude and altitude"
        ) from exc
    if (
        not all(math.isfinite(v) for v in (lat, lon, height))
        or abs(lat) > 90
        or abs(lon) > 180
    ):
        raise ValueError("invalid WGS84 geodetic position")
    return math.radians(lat), math.radians(lon), height


def _ecef(position):
    lat, lon, height = position
    sin_lat, cos_lat = math.sin(lat), math.cos(lat)
    radius = _SEMI_MAJOR_M / math.sqrt(1 - _ECCENTRICITY_SQUARED * sin_lat**2)
    return np.array(
        [
            (radius + height) * cos_lat * math.cos(lon),
            (radius + height) * cos_lat * math.sin(lon),
            (radius * (1 - _ECCENTRICITY_SQUARED) + height) * sin_lat,
        ]
    )


def geodetic_to_local_ne(position, reference):
    """Project ECEF displacement onto north/east at the reference location."""
    ref = _geodetic(reference)
    lat, lon, _ = ref
    difference = _ecef(_geodetic(position)) - _ecef(ref)
    north = np.array(
        [-math.sin(lat) * math.cos(lon), -math.sin(lat) * math.sin(lon), math.cos(lat)]
    )
    east = np.array([-math.sin(lon), math.cos(lon), 0.0])
    return np.array([np.dot(north, difference), np.dot(east, difference)])
