"""
AEGIS-TACTICAL-ISR: Persistent Multi-Object Tracker (ByteTrack + Kalman Filter)
=============================================================================
Maintains persistent target identities during occlusion, combat smoke/fog,
sensor mode handovers (RGB to FLIR), and rapid platform maneuvers.

Key Algorithms:
- 8-Dimensional Extended Kalman Filter for 2D bounding box kinematics
- Two-Stage ByteTrack Association (High-confidence + Low-confidence recovery)
- Tactical Threat Scoring & IFF (Identification Friend or Foe) classification
"""

from __future__ import annotations
from dataclasses import dataclass, field
from enum import IntEnum
import math
import time
from typing import Any, Dict, List, Optional, Tuple
import numpy as np
from scipy.optimize import linear_sum_assignment

from models.detector import Detection


@dataclass
class AirDefenceAsset:
    """Simulated Air Defence Battery or Forward Operating Base."""
    callsign: str = "AKASH-BATTERY-01"
    lat: float = 34.1520
    lon: float = 77.5760
    alt_m: float = 3950.0
    mez_radius_m: float = 2500.0   # Missile Engagement Zone (Weapon Engagement Zone - WEZ)
    ewz_radius_m: float = 5000.0   # Early Warning Acquisition Zone
    kill_radius_m: float = 1000.0  # CIWS / Point-Defence Kill Zone


class TrackState(IntEnum):
    NEW = 0
    TRACKED = 1
    LOST = 2
    REMOVED = 3


class KalmanBoxTracker:
    """
    8-dimensional Kalman filter for tracking bounding boxes in image space:
    State: [cx, cy, a, h, v_cx, v_cy, v_a, v_h]^T
    where (cx, cy) is box center, a is aspect ratio (w/h), h is height.
    """

    count = 0

    def __init__(self, bbox: List[float]) -> None:
        # State vector [8x1]
        self.dim_x = 8
        self.dim_z = 4

        # State transition matrix F
        self.F = np.eye(self.dim_x, dtype=np.float64)
        for i in range(4):
            self.F[i, i + 4] = 1.0  # dt = 1 frame

        # Measurement matrix H
        self.H = np.eye(self.dim_z, self.dim_x, dtype=np.float64)

        # State covariance P
        self.P = np.eye(self.dim_x, dtype=np.float64) * 10.0
        self.P[4:, 4:] *= 100.0  # High initial uncertainty in velocities

        # Process noise covariance Q
        self.Q = np.eye(self.dim_x, dtype=np.float64)
        self.Q[:4, :4] *= 0.05
        self.Q[4:, 4:] *= 0.005

        # Measurement noise covariance R
        self.R = np.eye(self.dim_z, dtype=np.float64) * 1.0

        # Initial state from bbox [x1, y1, x2, y2]
        self.x = np.zeros((self.dim_x, 1), dtype=np.float64)
        self.x[:4] = self._bbox_to_z(bbox)

        self.age = 0
        self.time_since_update = 0

    @staticmethod
    def _bbox_to_z(bbox: List[float]) -> np.ndarray:
        """Converts [x1, y1, x2, y2] to [cx, cy, aspect_ratio, height]."""
        w = max(1.0, bbox[2] - bbox[0])
        h = max(1.0, bbox[3] - bbox[1])
        cx = bbox[0] + w / 2.0
        cy = bbox[1] + h / 2.0
        aspect_ratio = w / h
        return np.array([[cx], [cy], [aspect_ratio], [h]], dtype=np.float64)

    @staticmethod
    def _z_to_bbox(z: np.ndarray) -> List[float]:
        """Converts [cx, cy, aspect_ratio, height] to [x1, y1, x2, y2]."""
        cx, cy, a, h = float(z[0, 0]), float(z[1, 0]), float(z[2, 0]), float(z[3, 0])
        w = max(1.0, a * h)
        h = max(1.0, h)
        return [cx - w / 2.0, cy - h / 2.0, cx + w / 2.0, cy + h / 2.0]

    def predict(self) -> List[float]:
        """Advances state vector forward by one time step."""
        if self.x[6, 0] + self.x[2, 0] <= 0:
            self.x[6, 0] = 0.0

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.age += 1
        self.time_since_update += 1
        return self._z_to_bbox(self.x)

    def update(self, bbox: List[float]) -> None:
        """Corrects state vector using incoming observation."""
        self.time_since_update = 0
        z = self._bbox_to_z(bbox)

        # Innovation / Measurement residual
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state and covariance
        self.x = self.x + K @ y
        I = np.eye(self.dim_x, dtype=np.float64)
        self.P = (I - K @ self.H) @ self.P

    def get_state(self) -> List[float]:
        """Returns current bounding box estimate [x1, y1, x2, y2]."""
        return self._z_to_bbox(self.x)

    @property
    def velocity(self) -> Tuple[float, float]:
        """Returns pixel velocity (v_cx, v_cy)."""
        return float(self.x[4, 0]), float(self.x[5, 0])


class STrack:
    """Tactical Track Representation with Military Metadata."""

    _count = 0

    def __init__(self, detection: Detection, min_hits: int = 3) -> None:
        STrack._count += 1
        self.track_id: int = STrack._count
        self.kalman_tracker = KalmanBoxTracker(detection.bbox)
        self.state: TrackState = TrackState.NEW
        self.class_name: str = detection.class_name
        self.class_id: int = detection.class_id
        self.modality: str = detection.modality
        self.score: float = detection.score
        self.min_hits: int = min_hits

        self.hits: int = 1
        self.age: int = 1
        self.time_since_update: int = 0

        # Geodetic state history
        self.current_lat: Optional[float] = None
        self.current_lon: Optional[float] = None
        self.current_alt: Optional[float] = None
        self.speed_kmh: float = 0.0
        self.heading_deg: float = 0.0
        self.trajectory_history: List[Tuple[float, float]] = []

        # Tactical assessment & Air Defence Intercept
        self.iff_status: str = "UNKNOWN"
        self.threat_priority: str = "LOW"
        self.threat_score: int = 20
        self.uncertainty_radius_m: float = 3.0

        # Terrain & CPA Intercept Kinematics
        self.terrain_elevation_msl: Optional[float] = None
        self.terrain_slope_deg: Optional[float] = None
        self.cpa_distance_m: Optional[float] = None
        self.time_to_cpa_sec: Optional[float] = None
        self.distance_to_asset_m: Optional[float] = None
        self.bearing_to_asset_deg: Optional[float] = None
        self.intercept_warning: bool = False
        self.warning_level: str = "CLEAR"
        self.projected_30s_pos: Optional[List[float]] = None

        self._assign_initial_iff(detection.class_name)

    def _assign_initial_iff(self, class_name: str) -> None:
        """Determines baseline IFF and Threat Priority."""
        if class_name in ("DRONE", "RADAR_AIR_DEFENCE"):
            self.iff_status = "HOSTILE"
            self.threat_priority = "CRITICAL"
            self.threat_score = 95
        elif class_name in ("ARMORED_VEHICLE", "CONVOY"):
            self.iff_status = "HOSTILE"
            self.threat_priority = "HIGH"
            self.threat_score = 80
        elif class_name == "FRIENDLY_UAV":
            self.iff_status = "FRIENDLY"
            self.threat_priority = "NONE"
            self.threat_score = 0
        else:
            self.iff_status = "SUSPECT"
            self.threat_priority = "MEDIUM"
            self.threat_score = 45

    def predict(self) -> List[float]:
        """Kalman state projection."""
        bbox = self.kalman_tracker.predict()
        self.age += 1
        self.time_since_update = self.kalman_tracker.time_since_update
        return bbox

    def update(self, detection: Detection) -> None:
        """Updates track with matching detection."""
        self.kalman_tracker.update(detection.bbox)
        self.score = detection.score
        self.modality = detection.modality
        self.hits += 1
        self.time_since_update = 0

        if self.state == TrackState.NEW and self.hits >= self.min_hits:
            self.state = TrackState.TRACKED
        elif self.state == TrackState.LOST:
            self.state = TrackState.TRACKED

    def update_georeference(
        self,
        lat: float,
        lon: float,
        alt: float,
        dt: float = 0.1,
        terrain_elevation_msl: Optional[float] = None,
        terrain_slope_deg: Optional[float] = None,
    ) -> None:
        """Updates geographic position and calculates real ground speed / heading."""
        if self.current_lat is not None and self.current_lon is not None and dt > 0.001:
            r_earth = 6378137.0
            lat_rad = math.radians((self.current_lat + lat) / 2.0)
            dn = math.radians(lat - self.current_lat) * r_earth
            de = math.radians(lon - self.current_lon) * (r_earth * math.cos(lat_rad))

            dist_m = math.hypot(dn, de)
            calc_speed_kmh = (dist_m / dt) * 3.6

            # Alpha filter for smooth speed readout
            self.speed_kmh = 0.7 * calc_speed_kmh + 0.3 * self.speed_kmh
            if dist_m > 0.3:
                self.heading_deg = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0

        self.current_lat = lat
        self.current_lon = lon
        self.current_alt = alt
        if terrain_elevation_msl is not None:
            self.terrain_elevation_msl = terrain_elevation_msl
        if terrain_slope_deg is not None:
            self.terrain_slope_deg = terrain_slope_deg

        self.trajectory_history.append((lat, lon))
        if len(self.trajectory_history) > 40:
            self.trajectory_history.pop(0)

        # Dynamic threat score escalation for high-speed counter-UAV tracks
        if self.class_name == "DRONE":
            if self.speed_kmh > 60.0:
                self.threat_priority = "CRITICAL"
                self.threat_score = min(100, int(90 + (self.speed_kmh / 10.0)))

    def compute_cpa(self, asset: AirDefenceAsset) -> Dict[str, Any]:
        """
        Computes Kinematic Intercept and Closest Point of Approach (CPA)
        relative to a defended Air Defence asset (SAM Battery / FOB).
        """
        if self.current_lat is None or self.current_lon is None:
            return {
                "cpa_distance_m": None,
                "time_to_cpa_sec": None,
                "distance_to_asset_m": None,
                "bearing_to_asset_deg": None,
                "intercept_warning": False,
                "warning_level": "CLEAR",
                "projected_30s_pos": None,
                "closing_speed_kmh": 0.0,
            }

        r_earth = 6378137.0
        lat_mid = math.radians((asset.lat + self.current_lat) / 2.0)
        dn = math.radians(self.current_lat - asset.lat) * r_earth
        de = math.radians(self.current_lon - asset.lon) * (r_earth * math.cos(lat_mid))

        dist_curr = math.hypot(dn, de)
        bearing_from_asset = (math.degrees(math.atan2(de, dn)) + 360.0) % 360.0

        speed_ms = self.speed_kmh / 3.6
        h_rad = math.radians(self.heading_deg)
        vn = speed_ms * math.cos(h_rad)
        ve = speed_ms * math.sin(h_rad)
        v_sq = vn * vn + ve * ve

        # 30-second kinematic projected forward position
        dn_30 = dn + vn * 30.0
        de_30 = de + ve * 30.0
        proj_lat = asset.lat + (dn_30 / r_earth) * (180.0 / math.pi)
        proj_lon = asset.lon + (de_30 / (r_earth * math.cos(lat_mid))) * (180.0 / math.pi)
        self.projected_30s_pos = [round(proj_lat, 6), round(proj_lon, 6)]

        if v_sq < 0.25:
            # Target is stationary / hovering (< 1.8 km/h)
            t_cpa = 0.0
            d_cpa = dist_curr
            is_closing = False
            closing_speed_kmh = 0.0
        else:
            # Analytical CPA minimization: t_cpa = - (r . v) / ||v||^2
            dot_product = dn * vn + de * ve
            t_cpa_calc = -dot_product / v_sq
            closing_speed_kmh = float(-(dot_product / max(1.0, dist_curr)) * 3.6)

            if t_cpa_calc < 0.0:
                # Target is moving away from asset
                t_cpa = 0.0
                d_cpa = dist_curr
                is_closing = False
            else:
                t_cpa = float(t_cpa_calc)
                dn_cpa = dn + vn * t_cpa
                de_cpa = de + ve * t_cpa
                d_cpa = float(math.hypot(dn_cpa, de_cpa))
                is_closing = True

        # Military Engagement Envelope Warning Logic
        intercept_warning = False
        warning_level = "CLEAR"

        if self.iff_status == "HOSTILE":
            if dist_curr <= asset.mez_radius_m:
                intercept_warning = True
                warning_level = "CRITICAL_BREACH"
            elif is_closing and d_cpa <= asset.mez_radius_m:
                if t_cpa <= 45.0:
                    intercept_warning = True
                    warning_level = "INTERCEPT_IMMINENT"
                elif t_cpa <= 120.0:
                    intercept_warning = True
                    warning_level = "WARNING"
                else:
                    warning_level = "MONITORING"
            elif is_closing and dist_curr <= asset.ewz_radius_m:
                warning_level = "EARLY_WARNING"
            elif not is_closing:
                warning_level = "CLEAR"

        self.cpa_distance_m = round(d_cpa, 1)
        self.time_to_cpa_sec = round(t_cpa, 1)
        self.distance_to_asset_m = round(dist_curr, 1)
        self.bearing_to_asset_deg = round(bearing_from_asset, 1)
        self.intercept_warning = intercept_warning
        self.warning_level = warning_level

        return {
            "cpa_distance_m": self.cpa_distance_m,
            "time_to_cpa_sec": self.time_to_cpa_sec,
            "distance_to_asset_m": self.distance_to_asset_m,
            "bearing_to_asset_deg": self.bearing_to_asset_deg,
            "intercept_warning": self.intercept_warning,
            "warning_level": self.warning_level,
            "projected_30s_pos": self.projected_30s_pos,
            "closing_speed_kmh": round(closing_speed_kmh, 1),
        }

    @property
    def bbox(self) -> List[float]:
        return self.kalman_tracker.get_state()

    def mark_lost(self) -> None:
        self.state = TrackState.LOST

    def mark_removed(self) -> None:
        self.state = TrackState.REMOVED


def box_iou(boxes1: np.ndarray, boxes2: np.ndarray) -> np.ndarray:
    """
    Computes pairwise Intersection-over-Union (IoU) between two sets of boxes [x1, y1, x2, y2].
    """
    if len(boxes1) == 0 or len(boxes2) == 0:
        return np.zeros((len(boxes1), len(boxes2)), dtype=np.float64)

    area1 = (boxes1[:, 2] - boxes1[:, 0]) * (boxes1[:, 3] - boxes1[:, 1])
    area2 = (boxes2[:, 2] - boxes2[:, 0]) * (boxes2[:, 3] - boxes2[:, 1])

    lt = np.maximum(boxes1[:, None, :2], boxes2[:, :2])  # [N, M, 2]
    rb = np.minimum(boxes1[:, None, 2:], boxes2[:, 2:])  # [N, M, 2]

    wh = np.clip(rb - lt, 0, None)  # [N, M, 2]
    inter = wh[:, :, 0] * wh[:, :, 1]

    union = area1[:, None] + area2 - inter
    return inter / np.clip(union, 1e-6, None)


class ByteTracker:
    """
    ByteTrack Multi-Object Association Engine.
    Leverages two-tier IoU matching to preserve track continuity through smoke & occlusion.
    """

    def __init__(
        self,
        track_high_thresh: float = 0.60,
        track_low_thresh: float = 0.15,
        new_track_thresh: float = 0.70,
        match_thresh: float = 0.80,
        track_buffer: int = 30,
        min_hits: int = 3,
    ) -> None:
        self.track_high_thresh = track_high_thresh
        self.track_low_thresh = track_low_thresh
        self.new_track_thresh = new_track_thresh
        self.match_thresh = match_thresh
        self.track_buffer = track_buffer
        self.min_hits = min_hits

        self.tracked_stracks: List[STrack] = []
        self.lost_stracks: List[STrack] = []
        self.frame_id: int = 0

    def reset(self) -> None:
        """Resets tracker state."""
        self.tracked_stracks.clear()
        self.lost_stracks.clear()
        self.frame_id = 0
        STrack._count = 0

    def update(self, detections: List[Detection]) -> List[STrack]:
        """
        Executes ByteTrack two-stage association for the current frame.
        """
        self.frame_id += 1

        # 1. Kalman prediction for all existing tracks
        for track in self.tracked_stracks:
            track.predict()
        for track in self.lost_stracks:
            track.predict()

        # 2. Partition detections into high and low confidence sets
        dets_high: List[Detection] = []
        dets_low: List[Detection] = []
        for det in detections:
            if det.score >= self.track_high_thresh:
                dets_high.append(det)
            elif det.score >= self.track_low_thresh:
                dets_low.append(det)

        # Candidates pool: active tracked tracks + recently lost tracks
        active_pool = self.tracked_stracks + self.lost_stracks

        # ----------------------------------------------------------------------
        # Stage 1: Associate High-Confidence Detections with Active Tracks
        # ----------------------------------------------------------------------
        matched_tracks_1, unmatched_tracks_1, unmatched_dets_high = self._associate(
            active_pool, dets_high, distance_thresh=self.match_thresh
        )

        for track, det in matched_tracks_1:
            track.update(det)

        # ----------------------------------------------------------------------
        # Stage 2: Associate Low-Confidence Detections with Remaining Tracks
        # (Crucial: recovers targets occluded by smoke, dust, or sensor noise)
        # ----------------------------------------------------------------------
        matched_tracks_2, unmatched_tracks_2, _ = self._associate(
            unmatched_tracks_1, dets_low, distance_thresh=0.50
        )

        for track, det in matched_tracks_2:
            track.update(det)

        # ----------------------------------------------------------------------
        # Stage 3: Handle Unmatched Tracks -> Transition to LOST or REMOVED
        # ----------------------------------------------------------------------
        for track in unmatched_tracks_2:
            if track.state != TrackState.LOST:
                track.mark_lost()

        # ----------------------------------------------------------------------
        # Stage 4: Initialize New Tracks from Unmatched High-Confidence Detections
        # ----------------------------------------------------------------------
        for det in unmatched_dets_high:
            if det.score >= self.new_track_thresh:
                new_track = STrack(det, min_hits=self.min_hits)
                self.tracked_stracks.append(new_track)

        # ----------------------------------------------------------------------
        # Re-organize Track Lists & Purge Expired Tracks
        # ----------------------------------------------------------------------
        new_tracked: List[STrack] = []
        new_lost: List[STrack] = []

        all_tracks = self.tracked_stracks + self.lost_stracks
        # Deduplicate by track_id
        seen_ids = set()
        dedup_tracks = []
        for t in all_tracks:
            if t.track_id not in seen_ids:
                seen_ids.add(t.track_id)
                dedup_tracks.append(t)

        for track in dedup_tracks:
            if track.state in (TrackState.TRACKED, TrackState.NEW):
                new_tracked.append(track)
            elif track.state == TrackState.LOST:
                if track.time_since_update <= self.track_buffer:
                    new_lost.append(track)
                else:
                    track.mark_removed()

        self.tracked_stracks = new_tracked
        self.lost_stracks = new_lost

        # Return all confirmed active or recently lost tracks for display
        return [t for t in self.tracked_stracks if t.state == TrackState.TRACKED or (t.state == TrackState.NEW and t.hits >= 2)]

    def _associate(
        self,
        tracks: List[STrack],
        detections: List[Detection],
        distance_thresh: float,
    ) -> Tuple[List[Tuple[STrack, Detection]], List[STrack], List[Detection]]:
        """Hungarian / Linear Sum Assignment based on IoU cost matrix."""
        if len(tracks) == 0 or len(detections) == 0:
            return [], tracks, detections

        track_boxes = np.array([t.bbox for t in tracks], dtype=np.float64)
        det_boxes = np.array([d.bbox for d in detections], dtype=np.float64)

        iou_matrix = box_iou(track_boxes, det_boxes)
        cost_matrix = 1.0 - iou_matrix

        row_indices, col_indices = linear_sum_assignment(cost_matrix)

        matched: List[Tuple[STrack, Detection]] = []
        unmatched_tracks = set(range(len(tracks)))
        unmatched_dets = set(range(len(detections)))

        for r, c in zip(row_indices, col_indices):
            if cost_matrix[r, c] < distance_thresh:
                matched.append((tracks[r], detections[c]))
                unmatched_tracks.discard(r)
                unmatched_dets.discard(c)

        unmatched_t = [tracks[i] for i in sorted(unmatched_tracks)]
        unmatched_d = [detections[j] for j in sorted(unmatched_dets)]

        return matched, unmatched_t, unmatched_d
