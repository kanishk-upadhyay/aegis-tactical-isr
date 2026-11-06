"""
AEGIS-TACTICAL-ISR: Georeferencing Engine Unit Tests
===================================================
Tests mathematical precision, rotation matrices, pinhole raycasting,
WGS-84 geodetic conversions, and inverse reprojection accuracy.
"""

import math
import pytest
import numpy as np
from telemetry.georeference import GeoreferenceEngine


@pytest.fixture
def engine() -> GeoreferenceEngine:
    return GeoreferenceEngine(
        image_width=1920,
        image_height=1080,
        hfov_deg=60.0,
        vfov_deg=40.0,
    )


def test_nadir_projection(engine: GeoreferenceEngine):
    """
    When gimbal is pointing straight down (tilt = -90 deg, zero roll/pitch),
    the optical center pixel (cx, cy) must project directly below the UAV.
    """
    uav_lat = 34.000000
    uav_lon = 77.000000
    uav_alt_msl = 5000.0
    uav_alt_agl = 1000.0

    res = engine.georeference_pixel(
        u=engine.cx,
        v=engine.cy,
        uav_lat=uav_lat,
        uav_lon=uav_lon,
        uav_alt_msl=uav_alt_msl,
        uav_alt_agl=uav_alt_agl,
        uav_roll_deg=0.0,
        uav_pitch_deg=0.0,
        uav_yaw_deg=0.0,
        gimbal_roll_deg=0.0,
        gimbal_pitch_deg=-90.0,
        gimbal_yaw_deg=0.0,
    )

    assert res is not None
    # Target latitude and longitude should be identical to UAV position within 1e-6 degrees (~0.1m)
    assert pytest.approx(res["lat"], abs=1e-6) == uav_lat
    assert pytest.approx(res["lon"], abs=1e-6) == uav_lon
    assert pytest.approx(res["ground_distance_m"], abs=0.1) == 0.0
    assert pytest.approx(res["slant_range_m"], abs=0.1) == uav_alt_agl


def test_forward_tilt_distance(engine: GeoreferenceEngine):
    """
    When gimbal tilts forward by 45 degrees (tilt = -45 deg, heading = 0 deg North),
    optical center should project exactly h * tan(45) = h meters North of UAV.
    """
    uav_lat = 34.000000
    uav_lon = 77.000000
    uav_alt_agl = 1000.0

    res = engine.georeference_pixel(
        u=engine.cx,
        v=engine.cy,
        uav_lat=uav_lat,
        uav_lon=uav_lon,
        uav_alt_msl=5000.0,
        uav_alt_agl=uav_alt_agl,
        uav_roll_deg=0.0,
        uav_pitch_deg=0.0,
        uav_yaw_deg=0.0,
        gimbal_roll_deg=0.0,
        gimbal_pitch_deg=-45.0,
        gimbal_yaw_deg=0.0,
    )

    assert res is not None
    expected_dist = uav_alt_agl * math.tan(math.radians(45.0))  # 1000.0 meters
    assert pytest.approx(res["ground_distance_m"], abs=1.0) == expected_dist
    assert pytest.approx(res["delta_north_m"], abs=1.0) == expected_dist
    assert pytest.approx(res["delta_east_m"], abs=1.0) == 0.0
    assert pytest.approx(res["azimuth_deg"], abs=0.5) == 0.0  # Due North


def test_altitude_scaling(engine: GeoreferenceEngine):
    """
    Ground offset distance must scale linearly with flight altitude AGL.
    """
    res_500 = engine.georeference_pixel(
        u=engine.cx,
        v=engine.cy,
        uav_lat=34.0,
        uav_lon=77.0,
        uav_alt_msl=5000.0,
        uav_alt_agl=500.0,
        gimbal_pitch_deg=-45.0,
    )
    res_1000 = engine.georeference_pixel(
        u=engine.cx,
        v=engine.cy,
        uav_lat=34.0,
        uav_lon=77.0,
        uav_alt_msl=5000.0,
        uav_alt_agl=1000.0,
        gimbal_pitch_deg=-45.0,
    )

    assert res_500 is not None and res_1000 is not None
    ratio = res_1000["ground_distance_m"] / res_500["ground_distance_m"]
    assert pytest.approx(ratio, rel=1e-3) == 2.0


def test_reprojection_roundtrip(engine: GeoreferenceEngine):
    """
    Inverse Projection: Projecting 2D pixel -> 3D GPS -> 2D pixel must match within 0.1 pixel.
    """
    test_u = 640.0
    test_v = 360.0
    uav_lat = 34.150000
    uav_lon = 77.580000
    uav_alt_msl = 4800.0
    uav_alt_agl = 850.0

    # 1. Forward Raycast
    geo = engine.georeference_pixel(
        u=test_u,
        v=test_v,
        uav_lat=uav_lat,
        uav_lon=uav_lon,
        uav_alt_msl=uav_alt_msl,
        uav_alt_agl=uav_alt_agl,
        uav_roll_deg=5.0,
        uav_pitch_deg=-2.0,
        uav_yaw_deg=45.0,
        gimbal_pitch_deg=-50.0,
        gimbal_yaw_deg=-15.0,
    )
    assert geo is not None

    # 2. Inverse Back-Projection
    reproj = engine.project_gps_to_pixel(
        target_lat=geo["lat"],
        target_lon=geo["lon"],
        target_alt_msl=geo["alt"],
        uav_lat=uav_lat,
        uav_lon=uav_lon,
        uav_alt_msl=uav_alt_msl,
        uav_roll_deg=5.0,
        uav_pitch_deg=-2.0,
        uav_yaw_deg=45.0,
        gimbal_pitch_deg=-50.0,
        gimbal_yaw_deg=-15.0,
    )
    assert reproj is not None
    reproj_u, reproj_v, in_frame = reproj

    assert in_frame
    assert pytest.approx(reproj_u, abs=0.1) == test_u
    assert pytest.approx(reproj_v, abs=0.1) == test_v


def test_above_horizon_rejection(engine: GeoreferenceEngine):
    """
    If gimbal tilts above the horizon (e.g. +10 deg look-up),
    ray should not intersect the ground (returns None).
    """
    res = engine.georeference_pixel(
        u=engine.cx,
        v=engine.cy,
        uav_lat=34.0,
        uav_lon=77.0,
        uav_alt_msl=5000.0,
        uav_alt_agl=1000.0,
        gimbal_pitch_deg=10.0,  # Looking up at sky
    )
    assert res is None


def test_ground_footprint_polygon(engine: GeoreferenceEngine):
    """
    Footprint calculation should generate a valid closed 5-vertex GeoJSON polygon ring.
    """
    poly = engine.compute_sensor_ground_footprint(
        uav_lat=34.15,
        uav_lon=77.58,
        uav_alt_msl=4800.0,
        uav_alt_agl=900.0,
        gimbal_pitch_deg=-60.0,
    )
    assert len(poly) == 5
    # First and last coordinate should match (closed loop)
    assert poly[0] == poly[-1]
