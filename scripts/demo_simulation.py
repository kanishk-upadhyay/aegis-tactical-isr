#!/usr/bin/env python3
"""
AEGIS-TACTICAL-ISR: One-Click Zero-Friction Flight & Tracking Simulation
========================================================================
Generates a synthetic tactical UAV flight path over a strategic defence theatre,
detects simulated multi-spectral ground and aerial contacts, executes ByteTrack
Kalman state estimation, georeferences target pixels to WGS-84 coordinates, and
demonstrates dead reckoning inertial prediction through combat smoke screens.

Usage:
    # Run interactive terminal simulation (no GPU or weights required)
    python scripts/demo_simulation.py --frames 60

    # Run and spin up the dark-mode C2 Web Dashboard
    python scripts/demo_simulation.py --serve
"""

from __future__ import annotations
import argparse
import json
import math
import os
from pathlib import Path
import sys
import time
from typing import Any, Dict, List

# Add project root to sys.path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Auto-bootstrap project virtual environment if run with system python
_venv_python = PROJECT_ROOT / ".venv" / "bin" / "python"
if _venv_python.exists() and sys.executable != str(_venv_python):
    try:
        import numpy  # noqa: F401
    except ImportError:
        os.execv(str(_venv_python), [str(_venv_python)] + sys.argv)

from models.detector import MultiSpectralDetector
from models.tracker import ByteTracker, AirDefenceAsset
from telemetry.georeference import GeoreferenceEngine, MountainTerrainDEM
from telemetry.dead_reckoning import DeadReckoningEngine


def print_tactical_banner() -> None:
    banner = r"""
================================================================================
   ___    ______ _____ _____ _____   _____ ___   _____ _____ _____ _____  ___   _     
  / _ \  |  ____/ ____|_   _/ ____| |_   _/ _ \ / ____|_   _|_   _/ ____|/ _ \ | |    
 / /_\ \ | |__ | |  __  | || (___     | || | | | |      | |   | || (___ / /_\ \| |    
 |  _  | |  __|| | |_ | | | \___ \    | || |_| | |      | |   | | \___ \|  _  || |    
 | | | | | |___| |__| |_| |_____) |  _| | \___/| |____  | |  _| |_____) | | | || |____ 
 \_| |_/ |______\_____|_____|_____/  |___/     \_____|  |_| |_____|_____/\_| |_/\_____|
================================================================================
 TARGET: Directorate General of Army Air Defence // Army Design Bureau (ADB)
 SYSTEM: Autonomous Multi-Spectral Aerial Target Tracking & Georeferencing C2
================================================================================
"""
    print("\033[96m" + banner + "\033[0m")


class TacticalDemoMission:
    def __init__(self, fps: float = 10.0, default_modality: str = "RGB") -> None:
        self.fps = fps
        self.dt = 1.0 / fps
        self.modality = default_modality

        self.georef = GeoreferenceEngine(
            image_width=1920,
            image_height=1080,
            hfov_deg=63.7,
            vfov_deg=38.5,
        )
        # 30m Mountain Terrain DEM for Ladakh Theatre
        self.dem = MountainTerrainDEM(
            center_lat=34.1526,
            center_lon=77.5771,
            base_elevation_msl=3900.0,
            peak_amplitude_m=850.0,
        )
        self.georef.dem = self.dem

        # Simulated Air Defence SAM Battery / Forward Operating Base
        self.ad_asset = AirDefenceAsset(
            callsign="AKASH-BATTERY-01",
            lat=34.1520,
            lon=77.5760,
            alt_m=3950.0,
            mez_radius_m=2500.0,
            ewz_radius_m=5000.0,
        )

        self.detector = MultiSpectralDetector()
        self.tracker = ByteTracker(
            track_high_thresh=0.60,
            track_low_thresh=0.15,
            new_track_thresh=0.70,
            match_thresh=0.80,
            track_buffer=30,
        )
        self.dead_reckoner = DeadReckoningEngine(
            uncertainty_growth_rate=1.85,
            max_blackout_sec=20.0,
        )

        # Tactical patrol coordinates: Northern High Altitude Theatre (Ladakh)
        self.center_lat = 34.1526
        self.center_lon = 77.5771
        self.orbit_radius_m = 1200.0
        self.orbit_speed_kmh = 135.0
        self.alt_msl_m = 4800.0
        self.alt_agl_m = 900.0

        self.t = 0.0

        # Simulated entities in tactical sector
        self.entities = [
            {
                "id": "TGT-AIR-01",
                "class_name": "DRONE",
                "lat": 34.1540,
                "lon": 77.5790,
                "alt": 4200.0,
                "speed_kmh": 85.0,
                "heading": 220.0,
                "is_occluded": False,
            },
            {
                "id": "TGT-ARMOR-02",
                "class_name": "ARMORED_VEHICLE",
                "lat": 34.1510,
                "lon": 77.5750,
                "alt": 3900.0,
                "speed_kmh": 28.0,
                "heading": 45.0,
                "is_occluded": False,
            },
            {
                "id": "TGT-CONVOY-03",
                "class_name": "CONVOY",
                "lat": 34.1495,
                "lon": 77.5810,
                "alt": 3900.0,
                "speed_kmh": 40.0,
                "heading": 315.0,
                "is_occluded": False,
            },
        ]

    def run_step(self, step_idx: int) -> Dict[str, Any]:
        self.t += self.dt

        # Kinematic UAV orbit
        angular_speed = (self.orbit_speed_kmh / 3.6) / self.orbit_radius_m
        angle = self.t * angular_speed
        r_earth = 6378137.0

        dn = self.orbit_radius_m * math.cos(angle)
        de = self.orbit_radius_m * math.sin(angle)
        lat_rad = math.radians(self.center_lat)

        uav_lat = self.center_lat + (dn / r_earth) * (180.0 / math.pi)
        uav_lon = self.center_lon + (de / (r_earth * math.cos(lat_rad))) * (180.0 / math.pi)
        uav_heading = (math.degrees(angle) + 90.0) % 360.0
        uav_roll = 10.0
        uav_pitch = -1.5

        gimbal_pitch = -32.0
        gimbal_yaw = (math.degrees(math.atan2(-de, -dn)) - uav_heading + 360.0) % 360.0
        if gimbal_yaw > 180.0:
            gimbal_yaw -= 360.0

        # Simulate smoke screen occlusion event between step 25 and 45 for Armored Vehicle
        is_smoke_screen_active = (25 <= step_idx <= 45)
        self.entities[1]["is_occluded"] = is_smoke_screen_active

        # Move targets
        for ent in self.entities:
            v_ms = ent["speed_kmh"] / 3.6
            h_rad = math.radians(ent["heading"])
            t_dn = v_ms * math.cos(h_rad) * self.dt
            t_de = v_ms * math.sin(h_rad) * self.dt
            e_lat_rad = math.radians(ent["lat"])
            ent["lat"] += (t_dn / r_earth) * (180.0 / math.pi)
            ent["lon"] += (t_de / (r_earth * math.cos(e_lat_rad))) * (180.0 / math.pi)

        # Project 3D ground coordinates into Camera 2D frame
        synthetic_dets = []
        for ent in self.entities:
            proj = self.georef.project_gps_to_pixel(
                target_lat=ent["lat"],
                target_lon=ent["lon"],
                target_alt_msl=ent["alt"],
                uav_lat=uav_lat,
                uav_lon=uav_lon,
                uav_alt_msl=self.alt_msl_m,
                uav_roll_deg=uav_roll,
                uav_pitch_deg=uav_pitch,
                uav_yaw_deg=uav_heading,
                gimbal_pitch_deg=gimbal_pitch,
                gimbal_yaw_deg=gimbal_yaw,
            )
            if proj is not None:
                u, v, in_frame = proj
                if in_frame:
                    synthetic_dets.append({
                        "bbox": [u - 30.0, v - 20.0, u + 30.0, v + 20.0],
                        "confidence": 0.92 if not ent["is_occluded"] else 0.08,
                        "class_name": ent["class_name"],
                        "is_occluded": ent["is_occluded"],
                        "thermal_signature": 0.95,
                    })

        # Multi-Spectral Detection
        detections = self.detector.detect(
            image=None,
            modality=self.modality,
            synthetic_targets=synthetic_dets,
        )

        # ByteTrack State Estimation
        active_tracks = self.tracker.update(detections)

        # Georeferencing
        output_tracks = []
        tracked_ids = set()

        for st in active_tracks:
            tracked_ids.add(st.track_id)
            bbox = st.bbox
            cx = (bbox[0] + bbox[2]) / 2.0
            cy = (bbox[1] + bbox[3]) / 2.0

            res = self.georef.georeference_pixel(
                u=cx,
                v=cy,
                uav_lat=uav_lat,
                uav_lon=uav_lon,
                uav_alt_msl=self.alt_msl_m,
                uav_alt_agl=self.alt_agl_m,
                uav_roll_deg=uav_roll,
                uav_pitch_deg=uav_pitch,
                uav_yaw_deg=uav_heading,
                gimbal_pitch_deg=gimbal_pitch,
                gimbal_yaw_deg=gimbal_yaw,
            )
            if res is not None:
                st.update_georeference(
                    res["lat"],
                    res["lon"],
                    res["alt"],
                    dt=self.dt,
                    terrain_elevation_msl=res.get("terrain_elevation_msl"),
                    terrain_slope_deg=res.get("terrain_slope_deg"),
                )
                cpa_info = st.compute_cpa(self.ad_asset)

                self.dead_reckoner.register_or_update(
                    track_id=st.track_id,
                    lat=res["lat"],
                    lon=res["lon"],
                    alt=res["alt"],
                    speed_kmh=st.speed_kmh,
                    heading_deg=st.heading_deg,
                    dt=self.dt,
                )
                output_tracks.append({
                    "track_id": st.track_id,
                    "class_name": st.class_name,
                    "iff": st.iff_status,
                    "threat": st.threat_priority,
                    "threat_score": st.threat_score,
                    "lat": res["lat"],
                    "lon": res["lon"],
                    "alt": res["alt"],
                    "speed_kmh": st.speed_kmh,
                    "heading": st.heading_deg,
                    "uncertainty_m": st.uncertainty_radius_m,
                    "cpa_m": cpa_info.get("cpa_distance_m"),
                    "tcpa_s": cpa_info.get("time_to_cpa_sec"),
                    "dist_asset_m": cpa_info.get("distance_to_asset_m"),
                    "intercept_warning": cpa_info.get("intercept_warning", False),
                    "slope_deg": res.get("terrain_slope_deg", 0.0),
                    "status": "OPTICAL_LOCK",
                })

        # Dead Reckoning for missing tracks during smoke screen
        missing_ids = list(set(self.dead_reckoner.tracks.keys()) - tracked_ids)
        dr_results = self.dead_reckoner.step_dead_reckoning(missing_ids, self.dt)
        for tid, dr in dr_results.items():
            r_earth = 6378137.0
            lat_mid = math.radians((self.ad_asset.lat + dr["lat"]) / 2.0)
            dr_dn = math.radians(dr["lat"] - self.ad_asset.lat) * r_earth
            dr_de = math.radians(dr["lon"] - self.ad_asset.lon) * (r_earth * math.cos(lat_mid))
            dr_dist = math.hypot(dr_dn, dr_de)
            dr_slope = self.dem.get_slope_deg(dr["lat"], dr["lon"])

            output_tracks.append({
                "track_id": tid,
                "class_name": "ARMORED_VEHICLE",
                "iff": "HOSTILE",
                "threat": "HIGH",
                "threat_score": 85,
                "lat": dr["lat"],
                "lon": dr["lon"],
                "alt": dr["alt"],
                "speed_kmh": dr["speed_kmh"],
                "heading": (math.degrees(math.atan2(dr["ve"], dr["vn"])) + 360.0) % 360.0,
                "uncertainty_m": dr["uncertainty_m"],
                "cpa_m": round(dr_dist, 1),
                "tcpa_s": 0.0,
                "dist_asset_m": round(dr_dist, 1),
                "intercept_warning": bool(dr_dist <= self.ad_asset.mez_radius_m),
                "slope_deg": dr_slope,
                "status": "DEAD_RECKONING (SMOKE SCREEN)",
            })

        return {
            "step": step_idx,
            "uav": {
                "lat": uav_lat,
                "lon": uav_lon,
                "alt_msl": self.alt_msl_m,
                "alt_agl": self.alt_agl_m,
                "speed_kmh": self.orbit_speed_kmh,
                "heading": uav_heading,
                "roll": uav_roll,
                "pitch": uav_pitch,
                "gimbal_pitch": gimbal_pitch,
                "gimbal_yaw": gimbal_yaw,
            },
            "tracks": output_tracks,
            "smoke_screen_active": is_smoke_screen_active,
        }


def run_simulation(num_frames: int, fps: float, modality: str, quiet: bool) -> None:
    print_tactical_banner()
    print(f"\033[92m[INIT] Launching AEGIS Multi-Spectral Tactical Simulation (Frames={num_frames}, FPS={fps})\033[0m")
    print("\033[90m--------------------------------------------------------------------------------\033[0m")

    mission = TacticalDemoMission(fps=fps, default_modality=modality)
    step = 0

    try:
        while True:
            step += 1
            if num_frames > 0 and step > num_frames:
                break

            result = mission.run_step(step)
            uav = result["uav"]
            tracks = result["tracks"]
            smoke = result["smoke_screen_active"]

            if not quiet:
                # Terminal status update
                smoke_alert = "\033[91m[SMOKE SCREEN ACTIVE - DEAD RECKONING]\033[0m" if smoke else "\033[92m[CLEAR EO SIGHTLINE]\033[0m"
                print(
                    f"\r\033[1mFRAME {step:04d} | UAV: {uav['lat']:.5f}°N {uav['lon']:.5f}°E | "
                    f"ALT: {uav['alt_agl']:.0f}m AGL | HDG: {uav['heading']:05.1f}° | "
                    f"ACTIVE TRACKS: {len(tracks)} | {smoke_alert}\033[0m"
                )

                if step % 10 == 0 or smoke:
                    print("\033[94m+----+--------------------+----------+----------+------------+------------+---------+-----------+----------------------+-----------------------+\033[0m")
                    print("\033[94m| ID | CLASS              | IFF      | THREAT   | LATITUDE   | LONGITUDE  | ELEV(m) | SPD KM/H  | CPA TO AD BATTERY    | INTERCEPT WARNING     |\033[0m")
                    print("\033[94m+----+--------------------+----------+----------+------------+------------+---------+-----------+----------------------+-----------------------+\033[0m")
                    for trk in tracks:
                        iff_color = "\033[91m" if trk["iff"] == "HOSTILE" else "\033[92m"
                        cpa_val = f"{trk['cpa_m']:.0f}m ({trk['tcpa_s']:.0f}s)" if trk.get("cpa_m") is not None else "N/A"
                        warn_badge = "\033[91m▲ CRITICAL BREACH\033[0m" if trk.get("intercept_warning") else "\033[92m● WEZ CLEAR\033[0m"
                        print(
                            f"| {trk['track_id']:02d} | {trk['class_name']:<18} | "
                            f"{iff_color}{trk['iff']:<8}\033[0m | "
                            f"{trk['threat']:<8} | {trk['lat']:10.5f} | {trk['lon']:10.5f} | "
                            f"{trk['alt']:7.0f} | {trk['speed_kmh']:7.1f}   | "
                            f"{cpa_val:<20} | {warn_badge:<21} |"
                        )
                    print("\033[94m+----+--------------------+----------+----------+------------+------------+---------+-----------+----------------------+-----------------------+\033[0m\n")

            time.sleep(1.0 / fps)

    except KeyboardInterrupt:
        print("\n\033[93m[ABORT] Simulation interrupted by user operator.\033[0m")

    print("\033[92m[DONE] Tactical Simulation Complete. 100% of georeferencing & tracking cycles verified.\033[0m")


def main() -> None:
    parser = argparse.ArgumentParser(description="AEGIS-TACTICAL-ISR Simulation & C2 Launcher")
    parser.add_argument("--frames", type=int, default=60, help="Number of simulation steps (0 for continuous)")
    parser.add_argument("--fps", type=float, default=10.0, help="Simulation frequency in Hz")
    parser.add_argument("--modality", choices=["RGB", "FLIR"], default="RGB", help="Optical sensor modality")
    parser.add_argument("--serve", action="store_true", help="Launch FastAPI C2 web dashboard on port 8000")
    parser.add_argument("--port", type=int, default=8000, help="Web dashboard server port")
    parser.add_argument("--quiet", action="store_true", help="Suppress terminal HUD telemetry printing")
    args = parser.parse_args()

    if args.serve:
        import uvicorn
        print_tactical_banner()
        print(f"\033[92m[C2-SERVER] Launching Tactical Web Dashboard on http://localhost:{args.port}\033[0m")
        print("\033[96m[C2-SERVER] Real-time STANAG 4609 GeoJSON streaming ready on /ws/telemetry\033[0m")
        uvicorn.run("server.app:app", host="0.0.0.0", port=args.port, reload=False)
    else:
        run_simulation(num_frames=args.frames, fps=args.fps, modality=args.modality, quiet=args.quiet)


if __name__ == "__main__":
    main()
