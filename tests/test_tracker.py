"""
AEGIS-TACTICAL-ISR: Multi-Object Tracker Unit Tests
===================================================
Tests Kalman box state estimation, ByteTrack association continuity,
ID switch prevention, low-confidence occlusion recovery, and dead reckoning.
"""

import math
import pytest
import numpy as np
from models.detector import Detection
from models.tracker import ByteTracker, KalmanBoxTracker, TrackState
from telemetry.dead_reckoning import DeadReckoningEngine


def test_kalman_box_tracker_kinematics():
    """Validates Kalman filter state prediction and velocity updates."""
    init_bbox = [100.0, 100.0, 150.0, 150.0]
    tracker = KalmanBoxTracker(init_bbox)

    # Initial state center is (125, 125)
    state = tracker.get_state()
    assert pytest.approx((state[0] + state[2]) / 2.0, abs=1.0) == 125.0
    assert pytest.approx((state[1] + state[3]) / 2.0, abs=1.0) == 125.0

    # Feed moving measurements (dx = +10, dy = +5 per frame)
    for frame in range(1, 6):
        tracker.predict()
        meas_bbox = [100.0 + frame * 10, 100.0 + frame * 5, 150.0 + frame * 10, 150.0 + frame * 5]
        tracker.update(meas_bbox)

    # Predict next state without update: should continue moving forward
    pred_bbox = tracker.predict()
    pred_cx = (pred_bbox[0] + pred_bbox[2]) / 2.0
    expected_cx = 125.0 + 6 * 10
    assert pytest.approx(pred_cx, abs=5.0) == expected_cx


def test_bytetrack_track_persistence():
    """Ensures a moving target retains a consistent persistent Track ID without switches."""
    tracker = ByteTracker(
        track_high_thresh=0.6,
        track_low_thresh=0.15,
        new_track_thresh=0.7,
        min_hits=2,
    )

    track_ids = []
    # Feed 10 consecutive frames with high confidence detection
    for frame in range(10):
        bbox = [200.0 + frame * 5, 200.0 + frame * 3, 260.0 + frame * 5, 240.0 + frame * 3]
        det = Detection(
            bbox=bbox,
            score=0.92,
            class_id=1,
            class_name="ARMORED_VEHICLE",
            modality="RGB",
        )
        tracks = tracker.update([det])
        if len(tracks) > 0:
            track_ids.append(tracks[0].track_id)

    # All confirmed frames must share the exact same track ID
    assert len(track_ids) >= 8
    assert len(set(track_ids)) == 1, f"Track ID switched unexpectedly: {set(track_ids)}"


def test_bytetrack_low_confidence_occlusion_recovery():
    """
    Validates ByteTrack's core capability: recovering an occluded target
    whose detection score falls into the secondary low-confidence band [0.15, 0.60].
    """
    tracker = ByteTracker(
        track_high_thresh=0.6,
        track_low_thresh=0.15,
        new_track_thresh=0.7,
        min_hits=2,
    )

    # 1. Establish solid track for 3 frames (High confidence)
    initial_id = None
    for frame in range(3):
        bbox = [100.0 + frame * 4, 100.0 + frame * 2, 150.0 + frame * 4, 140.0 + frame * 2]
        det = Detection(bbox=bbox, score=0.90, class_id=0, class_name="DRONE", modality="FLIR")
        tracks = tracker.update([det])
        if tracks:
            initial_id = tracks[0].track_id

    assert initial_id is not None

    # 2. Simulate combat smoke screen: Detection score drops to 0.25 (below track_high_thresh)
    occluded_bbox = [112.0, 106.0, 162.0, 146.0]
    low_conf_det = Detection(
        bbox=occluded_bbox,
        score=0.25,  # Low confidence detection recovered by Stage 2 association
        class_id=0,
        class_name="DRONE",
        modality="FLIR",
    )
    recovered_tracks = tracker.update([low_conf_det])

    assert len(recovered_tracks) == 1
    assert recovered_tracks[0].track_id == initial_id, "ByteTrack failed to associate low-confidence occluded target!"


def test_dead_reckoning_uncertainty_growth():
    """Validates that dead reckoning propagates trajectory and expands uncertainty bounds."""
    dr = DeadReckoningEngine(uncertainty_growth_rate=2.0, max_blackout_sec=10.0)

    # Register initial track
    dr.register_or_update(
        track_id=1,
        lat=34.1500,
        lon=77.5800,
        alt=4000.0,
        speed_kmh=72.0,  # 20 m/s
        heading_deg=0.0, # North
    )

    # Step dead reckoning forward by 2.0 seconds
    res = dr.step_dead_reckoning([1], dt=2.0)
    assert 1 in res
    t1 = res[1]

    # Latitude should have advanced North (+delta_lat)
    assert t1["lat"] > 34.1500
    # Uncertainty should have grown: initial (5.0) + 2.0 s * 2.0 m/s = 9.0 m
    assert pytest.approx(t1["uncertainty_m"], abs=0.5) == 9.0

    # Re-observation occurs: uncertainty must reset
    dr.register_or_update(
        track_id=1,
        lat=t1["lat"],
        lon=t1["lon"],
        alt=t1["alt"],
        speed_kmh=72.0,
        heading_deg=0.0,
    )
    assert dr.tracks[1].uncertainty_radius_m == 5.0


def test_air_defence_cpa_head_on_intercept():
    """
    Validates Kinematic Intercept and Closest Point of Approach (CPA) calculations.
    A hostile UAV heading directly towards the Air Defence battery must trigger INTERCEPT_WARNING.
    """
    from models.tracker import STrack, AirDefenceAsset

    battery = AirDefenceAsset(
        callsign="AKASH-BATTERY-01",
        lat=34.1520,
        lon=77.5760,
        alt_m=3950.0,
        mez_radius_m=2500.0,
        ewz_radius_m=5000.0,
    )

    det = Detection(
        bbox=[100.0, 100.0, 150.0, 140.0],
        score=0.95,
        class_id=0,
        class_name="DRONE",
        modality="FLIR",
    )
    track = STrack(det)

    # Position target 3500m North of battery, heading South (180 deg) at 108 km/h (30 m/s)
    r_earth = 6378137.0
    d_north = 3500.0
    target_lat = battery.lat + (d_north / r_earth) * (180.0 / math.pi)
    target_lon = battery.lon

    track.current_lat = target_lat
    track.current_lon = target_lon
    track.current_alt = 4200.0
    track.speed_kmh = 108.0
    track.heading_deg = 180.0  # Heading directly South into the battery

    cpa = track.compute_cpa(battery)

    # Head-on intercept: CPA distance should be close to 0 meters
    assert cpa["intercept_warning"] is True
    assert pytest.approx(cpa["cpa_distance_m"], abs=10.0) == 0.0
    # Expected time-to-CPA: 3500m / (30 m/s) = 116.7 seconds
    assert pytest.approx(cpa["time_to_cpa_sec"], abs=1.0) == (3500.0 / 30.0)
    assert cpa["warning_level"] in ("WARNING", "INTERCEPT_IMMINENT")


def test_air_defence_cpa_diverging_target():
    """
    Validates that a hostile target flying away from the defended asset does not
    generate a false closing warning.
    """
    from models.tracker import STrack, AirDefenceAsset

    battery = AirDefenceAsset(callsign="AKASH-BATTERY-01", lat=34.1520, lon=77.5760)
    det = Detection(bbox=[100.0, 100.0, 150.0, 140.0], score=0.95, class_id=0, class_name="DRONE", modality="FLIR")
    track = STrack(det)

    # Target 3000m North of battery, flying further North (0 deg) away from battery
    r_earth = 6378137.0
    track.current_lat = battery.lat + (3000.0 / r_earth) * (180.0 / math.pi)
    track.current_lon = battery.lon
    track.current_alt = 4200.0
    track.speed_kmh = 108.0
    track.heading_deg = 0.0  # Heading away

    cpa = track.compute_cpa(battery)

    assert cpa["intercept_warning"] is False
    assert cpa["time_to_cpa_sec"] == 0.0
    assert cpa["warning_level"] == "CLEAR"

