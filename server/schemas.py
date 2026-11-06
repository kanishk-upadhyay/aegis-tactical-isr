"""
AEGIS-TACTICAL-ISR: Telemetry & GeoJSON Pydantic Schemas
========================================================
Implements RFC 7946 GeoJSON and STANAG 4609 compliant data structures for
tactical C2 integration, Cursor-on-Target (CoT), and real-time WebSocket streaming.
"""

from __future__ import annotations
import time
from typing import Any, Dict, List, Literal, Optional
from pydantic import BaseModel, Field


class UAVTelemetry(BaseModel):
    """Real-time UAV platform & gimbal telemetry packet."""
    uav_id: str = Field(default="GARUDA-01", description="UAV Platform Tactical Call Sign")
    lat: float = Field(..., description="Latitude in WGS-84 decimal degrees")
    lon: float = Field(..., description="Longitude in WGS-84 decimal degrees")
    alt_msl_m: float = Field(..., description="Altitude Above Mean Sea Level (m)")
    alt_agl_m: float = Field(..., description="Altitude Above Ground Level (m)")
    groundspeed_kmh: float = Field(default=120.0, description="UAV Groundspeed in km/h")
    heading_deg: float = Field(default=0.0, description="Platform Compass Heading (0-360 deg)")
    roll_deg: float = Field(default=0.0, description="UAV Body Roll angle")
    pitch_deg: float = Field(default=0.0, description="UAV Body Pitch angle")
    yaw_deg: float = Field(default=0.0, description="UAV Body Yaw angle")
    gimbal_roll_deg: float = Field(default=0.0, description="Gimbal Roll stabilization")
    gimbal_pitch_deg: float = Field(default=-45.0, description="Gimbal Tilt Elevation (-90 to +20)")
    gimbal_yaw_deg: float = Field(default=0.0, description="Gimbal Azimuth Pan relative to nose")
    sensor_mode: Literal["RGB", "FLIR"] = Field(default="RGB", description="Active optical sensor mode")
    timestamp: float = Field(default_factory=time.time, description="Epoch timestamp in seconds")


class TacticalTrackPoint(BaseModel):
    """Real-time geolocated tactical track point."""
    track_id: int
    class_name: str
    iff_status: Literal["HOSTILE", "FRIENDLY", "SUSPECT", "UNKNOWN"]
    threat_priority: Literal["CRITICAL", "HIGH", "MEDIUM", "LOW", "NONE"]
    threat_score: int = Field(ge=0, le=100)
    lat: float
    lon: float
    alt_m: float
    speed_kmh: float
    heading_deg: float
    uncertainty_m: float
    bbox: List[float]  # [x1, y1, x2, y2]
    modality: Literal["RGB", "FLIR"]
    status: Literal["TRACKED", "DEAD_RECKONING", "LOST"]
    trajectory: List[List[float]] = Field(default_factory=list)  # [[lat, lon], ...]


class GeoJSONGeometry(BaseModel):
    type: Literal["Point", "Polygon", "LineString"]
    coordinates: Any


class GeoJSONFeature(BaseModel):
    type: Literal["Feature"] = "Feature"
    geometry: GeoJSONGeometry
    properties: Dict[str, Any]


class GeoJSONFeatureCollection(BaseModel):
    type: Literal["FeatureCollection"] = "FeatureCollection"
    features: List[GeoJSONFeature]


class TacticalStreamPayload(BaseModel):
    """Complete multi-spectral tactical picture streamed over WebSocket."""
    timestamp: float = Field(default_factory=time.time)
    uav_telemetry: UAVTelemetry
    active_tracks: List[TacticalTrackPoint]
    camera_footprint_geojson: GeoJSONFeature
    tracks_geojson: GeoJSONFeatureCollection
    threat_summary: Dict[str, int]  # e.g. {"CRITICAL": 1, "HIGH": 2, "MEDIUM": 0}
