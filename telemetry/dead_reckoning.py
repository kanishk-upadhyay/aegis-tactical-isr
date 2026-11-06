"""
AEGIS-TACTICAL-ISR: Inertial Dead Reckoning & Occlusion Predictor
================================================================
Maintains persistent tactical track state estimates during visual and thermal
blackout (smoke screens, terrain defilade, sensor blinding, rapid maneuvers).

Propagates kinematic trajectory using Constant Velocity (CV) / Constant Acceleration (CA)
models and expands spatial uncertainty covariance ellipses.
"""

from __future__ import annotations
import math
import time
from typing import Dict, List, Optional
import numpy as np


class DeadReckonTrack:
    """
    Represents an individual target track undergoing dead reckoning extrapolation.
    """

    def __init__(
        self,
        track_id: int,
        initial_lat: float,
        initial_lon: float,
        initial_alt: float,
        velocity_north_m_s: float,
        velocity_east_m_s: float,
        velocity_down_m_s: float = 0.0,
        initial_uncertainty_m: float = 5.0,
        uncertainty_growth_rate_m_s: float = 2.0,
        damping: float = 0.99,
        max_duration_sec: float = 15.0,
    ) -> None:
        self.track_id = track_id
        self.lat = initial_lat
        self.lon = initial_lon
        self.alt = initial_alt
        self.vn = velocity_north_m_s
        self.ve = velocity_east_m_s
        self.vd = velocity_down_m_s
        self.uncertainty_radius_m = initial_uncertainty_m
        self.growth_rate = uncertainty_growth_rate_m_s
        self.damping = damping
        self.max_duration_sec = max_duration_sec

        self.last_observation_time = time.time()
        self.last_update_time = time.time()
        self.occlusion_duration_sec = 0.0
        self.is_active = True

    def predict(self, dt: float) -> Dict[str, float]:
        """
        Propagates position forward by dt seconds using velocity vector.
        Updates latitude and longitude using geodesic displacement.
        """
        if not self.is_active:
            return {
                "lat": self.lat,
                "lon": self.lon,
                "alt": self.alt,
                "speed_kmh": 0.0,
                "uncertainty_m": self.uncertainty_radius_m,
                "is_active": False,
            }

        # Apply velocity damping (targets often decelerate or maintain cruise)
        self.vn *= (self.damping ** dt)
        self.ve *= (self.damping ** dt)

        delta_north = self.vn * dt
        delta_east = self.ve * dt
        delta_down = self.vd * dt

        # Convert local NED displacement to degrees
        # (Using local spherical approximation for small dt)
        lat_rad = math.radians(self.lat)
        r_earth = 6378137.0
        delta_lat = (delta_north / r_earth) * (180.0 / math.pi)
        delta_lon = (delta_east / (r_earth * math.cos(lat_rad))) * (180.0 / math.pi)

        self.lat += delta_lat
        self.lon += delta_lon
        self.alt += (-delta_down)

        self.occlusion_duration_sec += dt
        self.uncertainty_radius_m += self.growth_rate * dt

        if self.occlusion_duration_sec >= self.max_duration_sec:
            self.is_active = False

        speed_kmh = math.hypot(self.vn, self.ve) * 3.6

        return {
            "lat": self.lat,
            "lon": self.lon,
            "alt": self.alt,
            "vn": self.vn,
            "ve": self.ve,
            "speed_kmh": speed_kmh,
            "uncertainty_m": self.uncertainty_radius_m,
            "occlusion_sec": self.occlusion_duration_sec,
            "is_active": self.is_active,
        }

    def update_observation(
        self,
        new_lat: float,
        new_lon: float,
        new_alt: float,
        dt: float,
        measured_speed_kmh: Optional[float] = None,
    ) -> None:
        """
        Fuses a new visual/thermal detection into the track, resetting uncertainty.
        """
        if dt > 1e-4:
            # Re-estimate velocity
            r_earth = 6378137.0
            lat_rad = math.radians((self.lat + new_lat) / 2.0)
            d_north = math.radians(new_lat - self.lat) * r_earth
            d_east = math.radians(new_lon - self.lon) * (r_earth * math.cos(lat_rad))

            calc_vn = d_north / dt
            calc_ve = d_east / dt

            # Alpha filter velocity update
            alpha = 0.65
            self.vn = alpha * calc_vn + (1.0 - alpha) * self.vn
            self.ve = alpha * calc_ve + (1.0 - alpha) * self.ve

        self.lat = new_lat
        self.lon = new_lon
        self.alt = new_alt
        self.uncertainty_radius_m = 5.0  # Reset to baseline optical measurement uncertainty
        self.occlusion_duration_sec = 0.0
        self.is_active = True
        self.last_observation_time = time.time()


class DeadReckoningEngine:
    """
    Central manager for multi-target dead reckoning during sensor drops.
    """

    def __init__(
        self,
        uncertainty_growth_rate: float = 2.0,
        max_blackout_sec: float = 20.0,
    ) -> None:
        self.tracks: Dict[int, DeadReckonTrack] = {}
        self.growth_rate = uncertainty_growth_rate
        self.max_blackout_sec = max_blackout_sec

    def register_or_update(
        self,
        track_id: int,
        lat: float,
        lon: float,
        alt: float,
        speed_kmh: float,
        heading_deg: float,
        dt: float = 0.1,
    ) -> None:
        """Registers a newly confirmed track or updates an existing one."""
        speed_m_s = speed_kmh / 3.6
        heading_rad = math.radians(heading_deg)
        vn = speed_m_s * math.cos(heading_rad)
        ve = speed_m_s * math.sin(heading_rad)

        if track_id not in self.tracks:
            self.tracks[track_id] = DeadReckonTrack(
                track_id=track_id,
                initial_lat=lat,
                initial_lon=lon,
                initial_alt=alt,
                velocity_north_m_s=vn,
                velocity_east_m_s=ve,
                uncertainty_growth_rate_m_s=self.growth_rate,
                max_duration_sec=self.max_blackout_sec,
            )
        else:
            self.tracks[track_id].update_observation(lat, lon, alt, dt, speed_kmh)

    def step_dead_reckoning(self, occluded_track_ids: List[int], dt: float) -> Dict[int, Dict[str, float]]:
        """
        Extrapolates state for all currently occluded tracks.
        """
        extrapolated: Dict[int, Dict[str, float]] = {}
        for tid in occluded_track_ids:
            if tid in self.tracks:
                res = self.tracks[tid].predict(dt)
                if res["is_active"]:
                    extrapolated[tid] = res
                else:
                    # Drop expired tracks
                    del self.tracks[tid]
        return extrapolated
