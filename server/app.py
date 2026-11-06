"""
AEGIS-TACTICAL-ISR: FastAPI Real-Time Tactical C2 Backend
==========================================================
Streams real-time georeferenced multi-spectral aerial target tracks, UAV telemetry,
and sensor ground swath footprints over WebSockets using GeoJSON (RFC 7946).

Serves dark-mode military HUD interface and REST endpoints for command & control.
"""

from __future__ import annotations
import asyncio
from contextlib import asynccontextmanager
import json
import math
import os
from pathlib import Path
import time
from typing import Any, AsyncGenerator, Dict, List, Optional, Set

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse
from fastapi.staticfiles import StaticFiles
import yaml

from models.detector import MultiSpectralDetector, Detection
from models.tracker import ByteTracker, STrack, AirDefenceAsset
from telemetry.georeference import GeoreferenceEngine, MountainTerrainDEM
from telemetry.dead_reckoning import DeadReckoningEngine
from server.schemas import (
    UAVTelemetry,
    TacticalTrackPoint,
    TacticalStreamPayload,
    GeoJSONFeature,
    GeoJSONFeatureCollection,
    GeoJSONGeometry,
)

BASE_DIR = Path(__file__).resolve().parent.parent
CONFIGS_DIR = BASE_DIR / "configs"
DASHBOARD_DIR = BASE_DIR / "dashboard"

@asynccontextmanager
async def lifespan(app: FastAPI) -> AsyncGenerator[None, None]:
    task = asyncio.create_task(simulation_broadcast_loop())
    print("[AEGIS-SERVER] Real-time tactical ISR streaming server initialized.")
    yield
    global simulation_running
    simulation_running = False
    task.cancel()

app = FastAPI(
    title="AEGIS Tactical ISR System",
    description="Autonomous Multi-Spectral Aerial Target Tracking & Georeferencing C2",
    version="1.0.0",
    lifespan=lifespan,
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Load configuration files
def load_config(filename: str) -> Dict[str, Any]:
    cfg_file = CONFIGS_DIR / filename
    if cfg_file.exists():
        with open(cfg_file, "r") as f:
            return yaml.safe_load(f)
    return {}

tracker_config = load_config("tracker_config.yaml")
sensor_specs = load_config("sensor_specs.yaml")

# Core algorithmic components
georef_engine = GeoreferenceEngine(
    image_width=1920,
    image_height=1080,
    hfov_deg=63.7,
    vfov_deg=38.5,
)
detector = MultiSpectralDetector()
tracker = ByteTracker(
    track_high_thresh=0.60,
    track_low_thresh=0.15,
    new_track_thresh=0.70,
    match_thresh=0.80,
    track_buffer=30,
)
dead_reckoner = DeadReckoningEngine(uncertainty_growth_rate=2.0, max_blackout_sec=20.0)

# Global simulation & connection state
class ConnectionManager:
    def __init__(self) -> None:
        self.active_connections: Set[WebSocket] = set()

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        self.active_connections.add(websocket)

    def disconnect(self, websocket: WebSocket) -> None:
        self.active_connections.discard(websocket)

    async def broadcast(self, message: str) -> None:
        to_remove = set()
        for conn in self.active_connections:
            try:
                await conn.send_text(message)
            except Exception:
                to_remove.add(conn)
        for dead in to_remove:
            self.active_connections.discard(dead)

manager = ConnectionManager()

# Tactical State Store
state_lock = asyncio.Lock()
latest_payload: Optional[Dict[str, Any]] = None
simulation_running = True


# Tactical Synthetic Scenario Generator for Real-Time Streaming
class TacticalMissionSimulator:
    """
    Simulates a persistent combat patrol in the Northern Border Sector (Ladakh/Siachen Sector).
    Center: 34.1526 N, 77.5771 E (High Altitude Tactical Theatre).
    """

    def __init__(self) -> None:
        # UAV Patrolling Orbit parameters
        self.center_lat = 34.1526
        self.center_lon = 77.5771
        self.base_alt_msl = 4800.0  # meters MSL
        self.base_alt_agl = 900.0   # meters AGL
        self.orbit_radius_m = 1200.0
        self.orbit_speed_kmh = 135.0

        # Terrain & Air Defence Assets
        self.dem = MountainTerrainDEM(
            center_lat=self.center_lat,
            center_lon=self.center_lon,
            base_elevation_msl=3900.0,
            peak_amplitude_m=850.0,
        )
        georef_engine.dem = self.dem

        self.ad_asset = AirDefenceAsset(
            callsign="AKASH-BATTERY-01",
            lat=34.1520,
            lon=77.5760,
            alt_m=3950.0,
            mez_radius_m=2500.0,
            ewz_radius_m=5000.0,
        )

        self.uav_heading = 0.0
        self.uav_roll = 12.0  # Bank angle in orbit
        self.uav_pitch = -2.0
        self.gimbal_pitch = -32.0  # Angled down toward sector center
        self.gimbal_yaw = -90.0    # Looking inward toward center of orbit
        self.sensor_mode = "RGB"

        self.t = 0.0
        self.dt = 0.1  # 10 Hz streaming rate

        # Ground entities in sector
        self.entities = [
            {
                "id_label": "DRONE_SWARM_LEAD",
                "class_name": "DRONE",
                "lat": 34.1540,
                "lon": 77.5790,
                "alt": 4200.0,
                "speed_kmh": 85.0,
                "heading": 220.0,
                "is_air": True,
                "is_occluded": False,
                "occlusion_timer": 0.0,
            },
            {
                "id_label": "ARMORED_PLATOON_T90",
                "class_name": "ARMORED_VEHICLE",
                "lat": 34.1510,
                "lon": 77.5750,
                "alt": 3900.0,
                "speed_kmh": 28.0,
                "heading": 45.0,
                "is_air": False,
                "is_occluded": False,
                "occlusion_timer": 0.0,
            },
            {
                "id_label": "SUPPLY_CONVOY_COLUMN",
                "class_name": "CONVOY",
                "lat": 34.1495,
                "lon": 77.5810,
                "alt": 3900.0,
                "speed_kmh": 40.0,
                "heading": 315.0,
                "is_air": False,
                "is_occluded": False,
                "occlusion_timer": 0.0,
            },
            {
                "id_label": "INFILTRATION_GROUP",
                "class_name": "PERSONNEL",
                "lat": 34.1530,
                "lon": 77.5740,
                "alt": 3920.0,
                "speed_kmh": 4.5,
                "heading": 90.0,
                "is_air": False,
                "is_occluded": False,
                "occlusion_timer": 0.0,
            },
        ]

    def step(self) -> Dict[str, Any]:
        self.t += self.dt

        # Update UAV circular patrol trajectory
        angular_speed = (self.orbit_speed_kmh / 3.6) / self.orbit_radius_m  # rad/s
        angle = self.t * angular_speed

        # UAV position
        r_earth = 6378137.0
        delta_north = self.orbit_radius_m * math.cos(angle)
        delta_east = self.orbit_radius_m * math.sin(angle)

        lat_rad = math.radians(self.center_lat)
        uav_lat = self.center_lat + (delta_north / r_earth) * (180.0 / math.pi)
        uav_lon = self.center_lon + (delta_east / (r_earth * math.cos(lat_rad))) * (180.0 / math.pi)

        # Heading is tangential to orbit
        self.uav_heading = (math.degrees(angle) + 90.0) % 360.0

        # Gimbal slews to keep camera pointed toward tactical hotspot center
        self.gimbal_yaw = (math.degrees(math.atan2(-delta_east, -delta_north)) - self.uav_heading + 360.0) % 360.0
        if self.gimbal_yaw > 180.0:
            self.gimbal_yaw -= 360.0

        # Periodic smoke screen / thermal occlusion event every 25 seconds
        cycle_phase = int(self.t) % 30
        for ent in self.entities:
            if ent["class_name"] == "ARMORED_VEHICLE":
                if 12 <= cycle_phase <= 18:
                    ent["is_occluded"] = True  # Smoke grenade deployed
                else:
                    ent["is_occluded"] = False

        # Update ground entities kinematic positions
        for ent in self.entities:
            v_ms = ent["speed_kmh"] / 3.6
            h_rad = math.radians(ent["heading"])
            dn = v_ms * math.cos(h_rad) * self.dt
            de = v_ms * math.sin(h_rad) * self.dt

            e_lat_rad = math.radians(ent["lat"])
            ent["lat"] += (dn / r_earth) * (180.0 / math.pi)
            ent["lon"] += (de / (r_earth * math.cos(e_lat_rad))) * (180.0 / math.pi)

        # Project 3D positions into UAV camera 2D pixels
        synthetic_dets: List[Dict[str, Any]] = []
        for ent in self.entities:
            proj = georef_engine.project_gps_to_pixel(
                target_lat=ent["lat"],
                target_lon=ent["lon"],
                target_alt_msl=ent["alt"],
                uav_lat=uav_lat,
                uav_lon=uav_lon,
                uav_alt_msl=self.base_alt_msl,
                uav_roll_deg=self.uav_roll,
                uav_pitch_deg=self.uav_pitch,
                uav_yaw_deg=self.uav_heading,
                gimbal_roll_deg=0.0,
                gimbal_pitch_deg=self.gimbal_pitch,
                gimbal_yaw_deg=self.gimbal_yaw,
            )
            if proj is not None:
                u, v, in_frame = proj
                if in_frame:
                    box_w = 48.0 if ent["class_name"] != "DRONE" else 32.0
                    box_h = 32.0 if ent["class_name"] != "DRONE" else 24.0
                    synthetic_dets.append({
                        "bbox": [u - box_w / 2.0, v - box_h / 2.0, u + box_w / 2.0, v + box_h / 2.0],
                        "confidence": 0.89 if not ent["is_occluded"] else 0.12,
                        "class_name": ent["class_name"],
                        "is_occluded": ent["is_occluded"],
                        "thermal_signature": 0.95,
                    })

        # Run multi-spectral detector
        detections = detector.detect(
            image=None,
            modality=self.sensor_mode,
            synthetic_targets=synthetic_dets,
        )

        # Run ByteTrack multi-object tracker
        active_stracks = tracker.update(detections)

        # Georeference each tracked detection
        tactical_tracks: List[TacticalTrackPoint] = []
        tracked_ids = set()

        for st in active_stracks:
            tracked_ids.add(st.track_id)
            bbox = st.bbox
            cx = (bbox[0] + bbox[2]) / 2.0
            cy = (bbox[1] + bbox[3]) / 2.0

            # Compute ground coordinate via raycasting
            geo_pt = georef_engine.georeference_pixel(
                u=cx,
                v=cy,
                uav_lat=uav_lat,
                uav_lon=uav_lon,
                uav_alt_msl=self.base_alt_msl,
                uav_alt_agl=self.base_alt_agl,
                uav_roll_deg=self.uav_roll,
                uav_pitch_deg=self.uav_pitch,
                uav_yaw_deg=self.uav_heading,
                gimbal_roll_deg=0.0,
                gimbal_pitch_deg=self.gimbal_pitch,
                gimbal_yaw_deg=self.gimbal_yaw,
            )

            if geo_pt is not None:
                st.update_georeference(
                    geo_pt["lat"],
                    geo_pt["lon"],
                    geo_pt["alt"],
                    dt=self.dt,
                    terrain_elevation_msl=geo_pt.get("terrain_elevation_msl"),
                    terrain_slope_deg=geo_pt.get("terrain_slope_deg"),
                )
                cpa_info = st.compute_cpa(self.ad_asset)

                dead_reckoner.register_or_update(
                    track_id=st.track_id,
                    lat=geo_pt["lat"],
                    lon=geo_pt["lon"],
                    alt=geo_pt["alt"],
                    speed_kmh=st.speed_kmh,
                    heading_deg=st.heading_deg,
                    dt=self.dt,
                )

                tactical_tracks.append(
                    TacticalTrackPoint(
                        track_id=st.track_id,
                        class_name=st.class_name,
                        iff_status=st.iff_status,  # type: ignore
                        threat_priority=st.threat_priority,  # type: ignore
                        threat_score=st.threat_score,
                        lat=geo_pt["lat"],
                        lon=geo_pt["lon"],
                        alt_m=geo_pt["alt"],
                        speed_kmh=round(st.speed_kmh, 1),
                        heading_deg=round(st.heading_deg, 1),
                        uncertainty_m=round(st.uncertainty_radius_m, 1),
                        bbox=[round(b, 1) for b in bbox],
                        modality=self.sensor_mode,  # type: ignore
                        status="TRACKED",
                        trajectory=[[round(p[0], 6), round(p[1], 6)] for p in st.trajectory_history[-15:]],
                        cpa_distance_m=cpa_info.get("cpa_distance_m"),
                        time_to_cpa_sec=cpa_info.get("time_to_cpa_sec"),
                        distance_to_asset_m=cpa_info.get("distance_to_asset_m"),
                        bearing_to_asset_deg=cpa_info.get("bearing_to_asset_deg"),
                        intercept_warning=cpa_info.get("intercept_warning", False),
                        warning_level=cpa_info.get("warning_level", "CLEAR"),
                        projected_30s_pos=cpa_info.get("projected_30s_pos"),
                        terrain_elevation_msl=geo_pt.get("terrain_elevation_msl"),
                        terrain_slope_deg=geo_pt.get("terrain_slope_deg"),
                    )
                )

        # Dead reckoning for occluded tracks
        all_known_ids = set(dead_reckoner.tracks.keys())
        missing_ids = list(all_known_ids - tracked_ids)
        dr_results = dead_reckoner.step_dead_reckoning(missing_ids, self.dt)

        for tid, dr in dr_results.items():
            # Estimate CPA for dead reckoning track
            r_earth = 6378137.0
            lat_mid = math.radians((self.ad_asset.lat + dr["lat"]) / 2.0)
            dr_dn = math.radians(dr["lat"] - self.ad_asset.lat) * r_earth
            dr_de = math.radians(dr["lon"] - self.ad_asset.lon) * (r_earth * math.cos(lat_mid))
            dr_dist = math.hypot(dr_dn, dr_de)
            dr_bearing = (math.degrees(math.atan2(dr_de, dr_dn)) + 360.0) % 360.0

            # 30-second projection for dead-reckoned target
            dr_proj_lat = dr["lat"] + (dr["vn"] * 30.0 / r_earth) * (180.0 / math.pi)
            dr_proj_lon = dr["lon"] + (dr["ve"] * 30.0 / (r_earth * math.cos(lat_mid))) * (180.0 / math.pi)

            tactical_tracks.append(
                TacticalTrackPoint(
                    track_id=tid,
                    class_name="ARMORED_VEHICLE (OCCLUDED)",
                    iff_status="HOSTILE",
                    threat_priority="HIGH",
                    threat_score=85,
                    lat=dr["lat"],
                    lon=dr["lon"],
                    alt_m=dr["alt"],
                    speed_kmh=round(dr["speed_kmh"], 1),
                    heading_deg=round((math.degrees(math.atan2(dr["ve"], dr["vn"])) + 360.0) % 360.0, 1),
                    uncertainty_m=round(dr["uncertainty_m"], 1),
                    bbox=[0.0, 0.0, 0.0, 0.0],
                    modality=self.sensor_mode,  # type: ignore
                    status="DEAD_RECKONING",
                    trajectory=[],
                    cpa_distance_m=round(dr_dist, 1),
                    time_to_cpa_sec=0.0,
                    distance_to_asset_m=round(dr_dist, 1),
                    bearing_to_asset_deg=round(dr_bearing, 1),
                    intercept_warning=bool(dr_dist <= self.ad_asset.mez_radius_m),
                    warning_level="CRITICAL_BREACH" if dr_dist <= self.ad_asset.mez_radius_m else "CLEAR",
                    projected_30s_pos=[round(dr_proj_lat, 6), round(dr_proj_lon, 6)],
                    terrain_elevation_msl=self.dem.get_elevation(dr["lat"], dr["lon"]),
                    terrain_slope_deg=self.dem.get_slope_deg(dr["lat"], dr["lon"]),
                )
            )

        # Compute camera ground footprint swath polygon
        footprint_pts = georef_engine.compute_sensor_ground_footprint(
            uav_lat=uav_lat,
            uav_lon=uav_lon,
            uav_alt_msl=self.base_alt_msl,
            uav_alt_agl=self.base_alt_agl,
            uav_roll_deg=self.uav_roll,
            uav_pitch_deg=self.uav_pitch,
            uav_yaw_deg=self.uav_heading,
            gimbal_roll_deg=0.0,
            gimbal_pitch_deg=self.gimbal_pitch,
            gimbal_yaw_deg=self.gimbal_yaw,
        )

        footprint_feature = GeoJSONFeature(
            geometry=GeoJSONGeometry(type="Polygon", coordinates=[footprint_pts]),
            properties={
                "type": "CAMERA_FOV_SWATH",
                "sensor": self.sensor_mode,
                "hfov_deg": georef_engine.hfov_deg,
                "vfov_deg": georef_engine.vfov_deg,
            },
        )

        # Assemble RFC 7946 GeoJSON FeatureCollection for tracks
        track_features: List[GeoJSONFeature] = []
        for trk in tactical_tracks:
            track_features.append(
                GeoJSONFeature(
                    geometry=GeoJSONGeometry(
                        type="Point",
                        coordinates=[trk.lon, trk.lat],
                    ),
                    properties=trk.model_dump(),
                )
            )

        tracks_collection = GeoJSONFeatureCollection(features=track_features)

        # Threat summary breakdown
        threat_summary = {"CRITICAL": 0, "HIGH": 0, "MEDIUM": 0, "LOW": 0}
        for trk in tactical_tracks:
            if trk.threat_priority in threat_summary:
                threat_summary[trk.threat_priority] += 1

        uav_telemetry = UAVTelemetry(
            uav_id="GARUDA-01",
            lat=uav_lat,
            lon=uav_lon,
            alt_msl_m=self.base_alt_msl,
            alt_agl_m=self.base_alt_agl,
            groundspeed_kmh=self.orbit_speed_kmh,
            heading_deg=self.uav_heading,
            roll_deg=self.uav_roll,
            pitch_deg=self.uav_pitch,
            yaw_deg=self.uav_heading,
            gimbal_roll_deg=0.0,
            gimbal_pitch_deg=self.gimbal_pitch,
            gimbal_yaw_deg=self.gimbal_yaw,
            sensor_mode=self.sensor_mode,  # type: ignore
            timestamp=time.time(),
        )

        intercept_count = sum(1 for trk in tactical_tracks if trk.intercept_warning)

        payload = TacticalStreamPayload(
            timestamp=time.time(),
            uav_telemetry=uav_telemetry,
            active_tracks=tactical_tracks,
            camera_footprint_geojson=footprint_feature,
            tracks_geojson=tracks_collection,
            threat_summary=threat_summary,
            air_defence_asset={
                "callsign": self.ad_asset.callsign,
                "name": self.ad_asset.callsign,
                "lat": self.ad_asset.lat,
                "lon": self.ad_asset.lon,
                "alt_m": self.ad_asset.alt_m,
                "mez_radius_m": self.ad_asset.mez_radius_m,
                "engagement_radius_m": self.ad_asset.mez_radius_m,
                "ewz_radius_m": self.ad_asset.ewz_radius_m,
                "warning_radius_m": self.ad_asset.ewz_radius_m,
                "kill_radius_m": self.ad_asset.kill_radius_m,
            },
            intercept_warnings_active=intercept_count,
        )

        return payload.model_dump()


mission_sim = TacticalMissionSimulator()


# Background broadcast loop
async def simulation_broadcast_loop() -> None:
    global latest_payload
    while simulation_running:
        try:
            payload = mission_sim.step()
            async with state_lock:
                latest_payload = payload
            if manager.active_connections:
                message_json = json.dumps(payload)
                await manager.broadcast(message_json)
        except Exception as e:
            print(f"[AEGIS-SERVER] Broadcast loop error: {e}")
        await asyncio.sleep(0.1)  # 10 Hz broadcast rate




# WebSocket Endpoint
@app.websocket("/ws/telemetry")
async def websocket_telemetry(websocket: WebSocket) -> None:
    await manager.connect(websocket)
    try:
        # Immediately send current state if available
        async with state_lock:
            if latest_payload is not None:
                await websocket.send_text(json.dumps(latest_payload))

        while True:
            # Receive client commands (e.g. switch sensor mode or slew gimbal)
            data = await websocket.receive_text()
            try:
                cmd = json.loads(data)
                action = cmd.get("action")
                if action == "SWITCH_SENSOR_MODE":
                    mode = cmd.get("mode", "RGB")
                    if mode in ("RGB", "FLIR"):
                        mission_sim.sensor_mode = mode
                elif action == "SLEW_GIMBAL":
                    if "pitch" in cmd:
                        mission_sim.gimbal_pitch = float(cmd["pitch"])
                    if "yaw" in cmd:
                        mission_sim.gimbal_yaw = float(cmd["yaw"])
            except json.JSONDecodeError:
                pass
    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception:
        manager.disconnect(websocket)


# REST Endpoints
@app.get("/api/v1/health")
async def get_health() -> Dict[str, Any]:
    async with state_lock:
        tracks_count = len(latest_payload["active_tracks"]) if latest_payload else 0
        current_mode = mission_sim.sensor_mode
    return {
        "status": "OPERATIONAL",
        "system": "AEGIS-TACTICAL-ISR",
        "target_organization": "Army Design Bureau / Army Air Defence",
        "active_tracks_count": tracks_count,
        "active_sensor_mode": current_mode,
        "connected_c2_clients": len(manager.active_connections),
        "georeferencing_engine": "WGS-84 Pinhole Raycasting (STANAG 4609)",
        "tracker": "ByteTrack + 8D Extended Kalman Filter",
        "dead_reckoning": "Inertial Covariance Extrapolation",
    }


@app.get("/api/v1/tracks")
async def get_tracks() -> Dict[str, Any]:
    async with state_lock:
        if latest_payload:
            return latest_payload["tracks_geojson"]
        return {"type": "FeatureCollection", "features": []}


@app.get("/api/v1/specs")
async def get_sensor_specs() -> Dict[str, Any]:
    return sensor_specs


@app.post("/api/v1/telemetry/inject")
async def inject_telemetry(telemetry: UAVTelemetry) -> Dict[str, str]:
    """Allows external drone autopilots / MAVLink injectors to feed real flight data."""
    mission_sim.center_lat = telemetry.lat
    mission_sim.center_lon = telemetry.lon
    mission_sim.base_alt_msl = telemetry.alt_msl_m
    mission_sim.base_alt_agl = telemetry.alt_agl_m
    mission_sim.uav_heading = telemetry.heading_deg
    mission_sim.uav_roll = telemetry.roll_deg
    mission_sim.uav_pitch = telemetry.pitch_deg
    mission_sim.gimbal_pitch = telemetry.gimbal_pitch_deg
    mission_sim.gimbal_yaw = telemetry.gimbal_yaw_deg
    mission_sim.sensor_mode = telemetry.sensor_mode
    return {"status": "INJECTED_SUCCESSFULLY"}


# Mount dashboard files
if DASHBOARD_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(DASHBOARD_DIR)), name="static")

@app.get("/", response_class=HTMLResponse)
async def serve_dashboard() -> Any:
    index_file = DASHBOARD_DIR / "index.html"
    if index_file.exists():
        return FileResponse(str(index_file))
    return HTMLResponse("<h3>AEGIS Tactical ISR Server Running. Dashboard files initializing.</h3>")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("server.app:app", host="0.0.0.0", port=8000, reload=False)

