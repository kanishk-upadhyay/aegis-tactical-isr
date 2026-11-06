# AEGIS-TACTICAL-ISR: Autonomous Multi-Spectral Aerial Target Tracking, Georeferencing & C2 Mapping System

[![CI Pipeline](https://github.com/invincible/aegis-tactical-isr/actions/workflows/ci.yml/badge.svg)](https://github.com/invincible/aegis-tactical-isr/actions)
[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Python: 3.10+](https://img.shields.io/badge/Python-3.10%20%7C%203.11%20%7C%203.12-brightgreen.svg)](https://www.python.org/)
[![Target: Indian Army ADB & AAD](https://img.shields.io/badge/Defence%20Target-Army%20Design%20Bureau%20%7C%20DG%20AAD-red.svg)](#military-alignment--operational-context)
[![Standard: STANAG 4609 / WGS-84](https://img.shields.io/badge/Standard-STANAG%204609%20%2F%20WGS--84-orange.svg)](#mathematical-georeferencing-derivation)

> **Industry-grade, production-quality AI-Integrated Defense Project tailored for the Indian Army Internship Programme (AICTE) targeting the Army Design Bureau (ADB) and Directorate General of Army Air Defence (DG AAD).**

---

```
================================================================================
   ___    ______ _____ _____ _____   _____ ___   _____ _____ _____ _____  ___   _     
  / _ \  |  ____/ ____|_   _/ ____| |_   _/ _ \ / ____|_   _|_   _/ ____|/ _ \ | |    
 / /_\ \ | |__ | |  __  | || (___     | || | | | |      | |   | || (___ / /_\ \| |    
 |  _  | |  __|| | |_ | | | \___ \    | || |_| | |      | |   | | \___ \|  _  || |    
 | | | | | |___| |__| |_| |_____) |  _| | \___/| |____  | |  _| |_____) | | | || |____ 
 \_| |_/ |______\_____|_____|_____/  |___/     \_____|  |_| |_____|_____/\_| |_/\_____|
================================================================================
   AUTONOMOUS MULTI-SPECTRAL (EO/FLIR) TARGET TRACKING & GEOREFERENCING C2 SYSTEM
================================================================================
```

---

## 1. Executive Summary & Tactical Overview

Modern aerial surveillance platforms operating in contested border theatres (e.g., Northern High-Altitude Sector / Ladakh or Western Plains) face severe tactical ISR challenges:
1. **Multi-Spectral Signature Variations**: Daylight Electro-Optical (EO) sensors fail in darkness or smoke; uncooled Long-Wave Infrared (LWIR) FLIR sensors have lower spatial resolution but detect thermal emissions from engines, tires, and drone propulsion motors.
2. **Kinematic Track Occlusion**: Targets deploy combat smoke screens, utilize terrain defilade, or execute high-speed erratic turns. Traditional centroid trackers switch IDs or drop locks entirely.
3. **Sensor-to-Shooter Geolocation Lag**: Raw pixels in a camera feed are tactically useless to Air Defence artillery (e.g., Akash Surface-to-Air Missile batteries, L-70 gun units) until transformed with sub-meter mathematical precision into real-world geographic coordinates (Latitude, Longitude, Altitude MSL).

**AEGIS-TACTICAL-ISR** solves these operational bottlenecks by pairing:
- **Multi-Spectral Perception**: Synchronous processing of visible daylight RGB (1920×1080) and Thermal FLIR (640×512, 8–14 µm) with hot-spot signature extraction.
- **ByteTrack & 8D Extended Kalman Filtering**: Two-tier association recovering occluded targets from low-confidence sensor detections without track identity switching.
- **Rigorous 3D Georeferencing**: Optical pinhole raycasting integrated with UAV 3-axis attitude, gimbal kinematics, and WGS-84 ellipsoidal geodesy (STANAG 4609 compliant).
- **Inertial Dead Reckoning**: Constant-velocity trajectory propagation and covariance ellipse estimation during visual blackout.
- **Tactical C2 Military HUD**: Real-time Leaflet.js command interface streaming GeoJSON tracks via WebSockets with NATO MIL-STD-2525 symbology, dynamic sensor ground swath projection, and Air Defence handover triggers.
- **Zero-Friction One-Click Simulation**: Fully executable out of the box without requiring GPU hardware or multi-gigabyte weight downloads.

---

## 2. Military Alignment & Operational Context

| Defence Stakeholder | Operational Focus Area | AEGIS Technical Feature Alignment |
| :--- | :--- | :--- |
| **Army Design Bureau (ADB)** | Indigenisation, High-Altitude Border Reconnaissance, Tactical Edge AI | Lightweight, containerized, CPU-friendly multi-spectral perception pipeline capable of running on ruggedized edge mission computers (e.g., NVIDIA Jetson / x86 tactical payload controllers). |
| **Directorate General of Army Air Defence (DG AAD)** | Counter-UAV Operations, Low-Level Aerial Surveillance, Air Defence Handover | Immediate classification of hostile loitering munitions (`CRITICAL` priority), real-time vector calculation (speed, heading, altitude), and one-click target coordinate transmission to Air Defence fire-control batteries. |
| **Corps of Army Aviation / Artillery** | Tactical UAV Patrolling, Battle Damage Assessment (BDA), Convoy Escort | Continuous ground swath calculation, dead reckoning through smoke screens, and WGS-84 geodetic coordinate generation. |

---

## 3. System Architecture

```mermaid
flowchart TD
    subgraph SENSORS ["UAV Sensor & Avionics Payloads"]
        RGB["Daylight EO Camera<br/>1920x1080 Visible (0.4-0.7 um)"]
        FLIR["Thermal FLIR Camera<br/>640x512 LWIR (8-14 um) NETD <35mK"]
        IMU["Avionics IMU & AHRS<br/>Roll, Pitch, Yaw (100 Hz)"]
        GIMBAL["3-Axis Gyro-Stabilized Gimbal<br/>Pan, Tilt, Roll Encoders"]
        GPS["RTK-GPS / INS Nav Receiver<br/>Lat, Lon, Alt MSL/AGL"]
    end

    subgraph PERCEPTION ["Multi-Spectral Detection Layer"]
        DET["MultiSpectralDetector<br/>(models/detector.py)"]
        RGB --> DET
        FLIR --> DET
        DET --> NMS["Confidence Gating & Non-Maximum Suppression"]
    end

    subgraph TRACKING ["Persistent Association & Tracking Engine"]
        BYTE["ByteTrack 2-Stage Associator<br/>(models/tracker.py)"]
        EKF["8-Dimensional Kalman Filter<br/>State: [cx, cy, a, h, vx, vy, va, vh]"]
        NMS --> BYTE
        BYTE <--> EKF
        BYTE --> IFF["Tactical IFF & Threat Scoring<br/>(CRITICAL / HIGH / MEDIUM / LOW)"]
    end

    subgraph GEO ["Mathematical Georeferencing Engine"]
        GEOREF["Raycast Engine (telemetry/georeference.py)<br/>Pinhole K-1 + DCM Rotations + WGS-84"]
        IMU --> GEOREF
        GIMBAL --> GEOREF
        GPS --> GEOREF
        BYTE --> GEOREF
        GEOREF --> TARGET_GPS["3D World Ground Coordinates<br/>(Lat, Lon, Alt, Range, Azimuth)"]
    end

    subgraph DR ["Combat Occlusion Predictor"]
        DEAD_RECKON["Dead Reckoning Engine (telemetry/dead_reckoning.py)<br/>Inertial Kinematics & Covariance Growth"]
        BYTE -.->|Optical Dropout| DEAD_RECKON
        DEAD_RECKON --> TARGET_GPS
    end

    subgraph C2_SERVER ["FastAPI Real-Time Streaming Server"]
        API["FastAPI C2 Backend (server/app.py)<br/>REST Endpoints + WebSocket Stream"]
        GEOREF --> API
        DEAD_RECKON --> API
        API --> GEOJSON["RFC 7946 GeoJSON FeatureCollection<br/>Tracks + Sensor Swath Polygons"]
    end

    subgraph DASHBOARD ["Military Command & Control (C2) HUD"]
        HUD["Leaflet.js Dark-Mode Tactical HUD (dashboard/)<br/>MIL-STD-2525 Symbology | Reticles | Swath Swath"]
        API ==>|WebSocket /ws/telemetry (10 Hz)| HUD
    end
```

---

## 4. Mathematical Georeferencing Derivation

AEGIS implements a mathematically rigorous coordinate transformation pipeline that maps an image pixel $(u, v)$ to an exact latitude and longitude on the WGS-84 reference ellipsoid.

### Step 1: Camera Optical Frame Raycasting
Using the intrinsic camera calibration matrix $K$:
$$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}, \quad f_x = \frac{W/2}{\tan(\text{HFOV}/2)}, \quad f_y = \frac{H/2}{\tan(\text{VFOV}/2)}$$

A normalized ray vector $\vec{v}_c$ in the Camera Optical Frame ($+X$ Right, $+Y$ Down, $+Z$ Optical Forward) is:
$$\vec{v}_c = K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = \begin{bmatrix} \frac{u - c_x}{f_x} \\ \frac{v - c_y}{f_y} \\ 1 \end{bmatrix}, \quad \hat{r}_c = \frac{\vec{v}_c}{\|\vec{v}_c\|}$$

### Step 2: Direction Cosine Matrices & Attitude Rotation
Let $R_c^g$ align Camera Optical axes with the Gimbal Boresight frame:
$$R_c^g = \begin{bmatrix} 0 & 0 & 1 \\ 1 & 0 & 0 \\ 0 & 1 & 0 \end{bmatrix}$$

The Gimbal to UAV Body rotation $R_g^b$ parametrized by Gimbal Pan $\psi_g$, Tilt $\theta_g$, and Roll $\phi_g$:
$$R_g^b = R_z(\psi_g) R_y(\theta_g) R_x(\phi_g)$$

The UAV Body to Navigation (North-East-Down, NED) rotation $R_b^{NED}$ parametrized by aircraft Heading $\psi_u$, Pitch $\theta_u$, and Roll $\phi_u$:
$$R_b^{NED} = R_z(\psi_u) R_y(\theta_u) R_x(\phi_u)$$

The composite transformation from Camera Optical to NED frame is:
$$R_{cam}^{NED} = R_b^{NED} \cdot R_g^b \cdot R_c^g$$

Transforming the line-of-sight unit vector into NED:
$$\hat{d} = R_{cam}^{NED} \hat{r}_c = \begin{bmatrix} d_N \\ d_E \\ d_D \end{bmatrix}$$

### Step 3: Ground Intersection & Slant Range
For a sensor flying at altitude above ground level $h_{agl}$, ground intersection requires $d_D > 0.005$ (ray pointing toward the terrain). The slant range $s$ along the optical ray is:
$$s = \frac{h_{agl}}{d_D}$$

The target displacement vector in NED coordinates relative to the UAV is:
$$\Delta N = s \cdot d_N, \quad \Delta E = s \cdot d_E, \quad \Delta D = h_{agl}$$

### Step 4: Geodetic WGS-84 Ellipsoidal Projection
Converting local Cartesian NED offsets $(\Delta N, \Delta E)$ into geodetic coordinates on the WGS-84 reference ellipsoid ($a = 6378137.0\text{ m}$, $e^2 \approx 0.00669437999014$):

Radius of curvature in the Prime Vertical:
$$N(\phi) = \frac{a}{\sqrt{1 - e^2 \sin^2 \phi}}$$

Radius of curvature in the Meridian:
$$M(\phi) = \frac{a(1 - e^2)}{(1 - e^2 \sin^2 \phi)^{3/2}}$$

Target latitude $\phi_t$ and longitude $\lambda_t$:
$$\Delta \phi = \frac{\Delta N}{M(\phi) + h_{msl}} \text{ rad}, \quad \Delta \lambda = \frac{\Delta E}{(N(\phi) + h_{msl}) \cos \phi} \text{ rad}$$
$$\phi_t = \phi_0 + \Delta \phi \times \frac{180}{\pi}, \quad \lambda_t = \lambda_0 + \Delta \lambda \times \frac{180}{\pi}$$

### Step 5: Non-Planar Digital Elevation Model (DEM) Raycasting
In high-altitude mountainous theatres (such as the Northern Sector / Ladakh with terrain elevations from 3,500m to 5,500m MSL and steep ridges), a flat-earth assumption introduces unacceptable geolocation errors exceeding 25 to 100 meters.

AEGIS implements an iterative **Ray-Marching & Secant Bisection** line-terrain intersection solver:
1. Define the 3D ray in NED space parametrized by distance $s$:
   $$\vec{P}(s) = \vec{P}_{uav} + s \cdot \hat{d}_{NED}$$
2. Convert $\vec{P}(s)$ to geodetic $(\phi, \lambda)$ and sample the terrain elevation $Z_{dem}(\phi, \lambda)$ from a 30m simulated SRTM elevation grid.
3. Compute the residual elevation function:
   $$f(s) = Z_{ray}(s) - Z_{dem}(\phi(s), \lambda(s))$$
4. Coarse step marching until a bracket is found: $f(s_k) \cdot f(s_{k+1}) \le 0$.
5. Refine with secant bisection down to sub-meter residual convergence ($|f(s)| < 0.2\text{ m}$):
   $$s_{mid} = \frac{s_a \cdot f(s_b) - s_b \cdot f(s_a)}{f(s_b) - f(s_a)}$$
This eliminates terrain elevation-induced displacement and guarantees sub-meter geolocation accuracy across steep slope profiles.

---

## 5. ByteTrack Multi-Object Tracking & Dead Reckoning

### 8D Extended Kalman Filter State
The bounding box kinematic state vector is formulated as:
$$\vec{x} = \begin{bmatrix} c_x & c_y & a & h & \dot{c}_x & \dot{c}_y & \dot{a} & \dot{h} \end{bmatrix}^T$$
where $(c_x, c_y)$ is the 2D bounding box center, $a = w/h$ is aspect ratio, $h$ is box height, and the remaining 4 terms are velocities.

### Two-Stage ByteTrack Association Logic
```mermaid
flowchart TD
    DETS["Current Frame Detections"] --> SPLIT{"Score Thresholding"}
    SPLIT -->|score >= 0.60| HIGH["D_high (High Confidence)"]
    SPLIT -->|0.15 <= score < 0.60| LOW["D_low (Low Confidence / Occluded)"]

    HIGH --> MATCH1["Stage 1: Hungarian IoU Matching with Active Tracks"]
    MATCH1 -->|Matched| UPDATE1["Kalman State Update (Tracked)"]
    MATCH1 -->|Unmatched Tracks| REMAIN_TRACKS["T_remain (Unmatched Active Tracks)"]
    MATCH1 -->|Unmatched High Dets| NEW_TRACKS["Initialize New Candidate Tracklets"]

    LOW --> MATCH2["Stage 2: Match D_low with T_remain"]
    REMAIN_TRACKS --> MATCH2
    MATCH2 -->|Matched| UPDATE2["Recover Occluded Track! (Prevents ID Fragmentation)"]
    MATCH2 -->|Unmatched Tracks| LOST["Transition to LOST (Retain in Buffer for 30 Frames)"]
    LOST -.->|Sensor Dropout| DR["Activate Inertial Dead Reckoning"]
```

### Air Defence Engagement Envelope & Kinematic CPA
To support Directorate General of Army Air Defence (DG AAD) fire control, AEGIS continuously computes kinematic intercept metrics against high-value assets (such as an Akash SAM battery or Forward Operating Base):

Let $\vec{r} = \vec{P}_{tgt} - \vec{P}_{asset}$ denote the relative displacement vector in the local horizontal plane, and $\vec{v} = \vec{v}_{tgt}$ denote the target velocity vector.
The squared distance at future time $t$ is:
$$D(t)^2 = \|\vec{r} + \vec{v} t\|^2 = \|\vec{r}\|^2 + 2 (\vec{r} \cdot \vec{v}) t + \|\vec{v}\|^2 t^2$$

Minimizing $D(t)^2$ with respect to $t$:
$$\frac{d}{dt} D(t)^2 = 2 (\vec{r} \cdot \vec{v}) + 2 \|\vec{v}\|^2 t = 0 \implies t_{cpa} = -\frac{\vec{r} \cdot \vec{v}}{\|\vec{v}\|^2}$$

- If $t_{cpa} < 0$: The target has already passed the point of closest approach and is **diverging**.
- If $t_{cpa} \ge 0$: The target is **closing**, reaching minimum distance at $t_{cpa}$:
  $$d_{cpa} = \|\vec{r} + \vec{v} t_{cpa}\|$$

**Defensive Threat Assessment:**
- If $d_{cpa} \le R_{MEZ}$ (Missile Engagement Zone, 2,500m) and $t_{cpa} \le 120\text{ s}$, the target triggers `INTERCEPT_WARNING` with priority `ENGAGEMENT_CRITICAL`.
- Future 30-second target flight path vectors are projected dynamically on the C2 HUD:
  $$\vec{P}_{30s} = \vec{P}_{tgt} + 30 \cdot \vec{v}$$

---

## 6. Repository Layout

```
aegis-tactical-isr/
├── configs/
│   ├── tracker_config.yaml       # ByteTrack thresholds, Kalman noise, IFF priorities
│   └── sensor_specs.yaml         # EO/IR sensor dimensions, FOVs, gimbal limits
├── models/
│   ├── __init__.py
│   ├── detector.py               # ONNX/YOLOv11 wrapper with zero-friction synthetic fallback
│   └── tracker.py                # 8D Kalman filter & 2-stage ByteTrack association
├── telemetry/
│   ├── __init__.py
│   ├── georeference.py           # Precision pixel-to-WGS84 raycasting engine
│   └── dead_reckoning.py         # Inertial tracking & covariance expansion during smoke defilade
├── server/
│   ├── __init__.py
│   ├── app.py                    # FastAPI real-time C2 WebSocket & REST backend
│   └── schemas.py                # Pydantic telemetry & RFC 7946 GeoJSON schemas
├── dashboard/
│   ├── index.html                # Dark-mode tactical military HUD
│   ├── app.js                    # WebSocket live map track rendering (MIL-STD-2525)
│   └── style.css                 # Military tactical HUD styling & neon threat radar theme
├── data/
│   ├── sample_flir.jpg           # Sample LWIR white-hot thermal infrared frame
│   └── sample_rgb.jpg            # Sample aerial daylight high-resolution frame
├── tests/
│   ├── __init__.py
│   ├── test_georeference.py      # Math precision tests for nadir, tilt, and reprojection
│   ├── test_tracker.py           # Track persistence, occlusion recovery, and dead reckoning tests
│   └── test_server.py            # FastAPI REST & WebSocket integration tests
├── scripts/
│   ├── __init__.py
│   ├── demo_simulation.py       # One-click synthetic flight & tracking simulation
│   └── benchmark_scenarios.py   # Quantitative mission benchmark evaluation suite
├── reports/
│   └── benchmark_report.md      # Auto-generated defense benchmark scorecard
├── Makefile                      # One-touch developer & operator command targets
├── Dockerfile                    # Multi-stage containerized deployment for edge mission computers
├── requirements.txt              # Production dependencies
├── .github/workflows/ci.yml      # GitHub Actions automated CI testing workflow
├── LICENSE                       # Apache License 2.0
├── .gitignore
└── README.md
```

---

## 7. Zero-Friction Quickstart Guide

AEGIS is designed for **immediate evaluation**—no multi-gigabyte neural weight downloads, CUDA installations, or complex toolchains are required.

### Option A: Local Python Environment (Recommended)

```bash
# 1. Clone the repository
git clone https://github.com/invincible/aegis-tactical-isr.git
cd aegis-tactical-isr

# 2. Install dependencies (or use `make install`)
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt

# 3. Run Quantitative Mission Benchmark Suite (or use `make benchmark`)
python scripts/benchmark_scenarios.py

# 4. Run One-Click Terminal Simulation (or use `make demo`)
python scripts/demo_simulation.py --frames 60

# 5. Launch Real-Time Military C2 Web Dashboard (or use `make serve`)
python scripts/demo_simulation.py --serve
```
   Open your browser and navigate to:
   ```
   http://localhost:8000
   ```
   *Experience the real-time dark-mode tactical HUD with Leaflet.js map, live UAV sensor swath footprints, NATO tactical target markers, and simulated multi-spectral PiP video feed.*

---

### Option B: Edge Container Deployment (Docker)

To run in an isolated, security-hardened Linux container:

```bash
# Build the edge container
docker build -t aegis-tactical-isr:latest .

# Run container on port 8000
docker run -d --name aegis-c2 -p 8000:8000 aegis-tactical-isr:latest

# Open http://localhost:8000 in your browser
```

---

## 8. Verification & Unit Test Suite

Run the full pytest suite to verify 100% mathematical precision and tracking logic:

```bash
pytest tests/ -v
```

### Test Coverage Highlights (19 Passing Tests):
- **`test_nadir_projection`**: Confirms optical center projects directly to UAV nadir point with sub-centimeter error.
- **`test_forward_tilt_distance`**: Verifies forward gimbal tilt obeys exact trigonometric ground distance $d = h \tan(\theta)$.
- **`test_altitude_scaling`**: Validates linear ground distance scaling with flight altitude.
- **`test_reprojection_roundtrip`**: Performs 2D $\to$ 3D $\to$ 2D raycast-backprojection roundtrip, verifying sub-pixel ($< 0.1\text{ px}$) mathematical agreement.
- **`test_above_horizon_rejection`**: Confirms rays pointing above the horizon do not trigger false ground intersections.
- **`test_ground_footprint_polygon`**: Validates 5-point closed GeoJSON swath polygon construction.
- **`test_dem_analytical_steep_slope_raycast_precision`**: Verifies sub-meter accuracy ($< 0.2\text{ m}$) across a steep 19.3° mountain slope using iterative DEM ray-marching.
- **`test_flat_earth_versus_dem_elevation_error_demonstration`**: Quantifies and documents the massive 25m–100m error introduced by flat-earth assumptions in mountain warfare.
- **`test_air_defence_cpa_head_on_intercept`**: Validates kinematic Closest Point of Approach (CPA), closing speed, and `INTERCEPT_WARNING` defensive perimeter triggers.
- **`test_air_defence_cpa_diverging_target`**: Verifies negative time-to-CPA for outgoing threats without false alerts.
- **`test_bytetrack_track_persistence`**: Proves zero track ID switching over consecutive frames.
- **`test_bytetrack_low_confidence_occlusion_recovery`**: Proves recovery of degraded/occluded contacts.
- **`test_dead_reckoning_uncertainty_growth`**: Confirms covariance expansion during signal loss and reset upon re-acquisition.
- **`test_api_health_endpoint` & `test_telemetry_injection`**: Verifies full REST & C2 communications contract.

---

## 9. C2 REST API & WebSocket Protocol

### WebSocket Stream (`/ws/telemetry`)
Emits STANAG 4609 compliant JSON packets at 10 Hz:
```json
{
  "timestamp": 1730912400.12,
  "uav_telemetry": {
    "uav_id": "GARUDA-01",
    "lat": 34.1526,
    "lon": 77.5771,
    "alt_msl_m": 4800.0,
    "alt_agl_m": 900.0,
    "groundspeed_kmh": 135.0,
    "heading_deg": 45.0,
    "gimbal_pitch_deg": -45.0,
    "sensor_mode": "RGB"
  },
  "active_tracks": [
    {
      "track_id": 1,
      "class_name": "DRONE",
      "iff_status": "HOSTILE",
      "threat_priority": "CRITICAL",
      "threat_score": 98,
      "lat": 34.1542,
      "lon": 77.5788,
      "alt_m": 4200.0,
      "speed_kmh": 85.0,
      "heading_deg": 220.0,
      "uncertainty_m": 2.4,
      "status": "TRACKED"
    }
  ],
  "camera_footprint_geojson": { "type": "Feature", "geometry": { "type": "Polygon", "coordinates": [...] } },
  "tracks_geojson": { "type": "FeatureCollection", "features": [...] },
  "threat_summary": { "CRITICAL": 1, "HIGH": 1, "MEDIUM": 0, "LOW": 0 }
}
```

### Key REST Endpoints:
- `GET /api/v1/health`: Returns system health, tracking engine state, and active client count.
- `GET /api/v1/tracks`: Returns current tactical track picture as RFC 7946 GeoJSON.
- `GET /api/v1/specs`: Returns detailed optical sensor and gimbal payload parameters.
- `POST /api/v1/telemetry/inject`: Allows real drone autopilot (MAVLink / ArduPilot / PX4) injection.

---

## 10. License & Attribution

This project is licensed under the **Apache License 2.0** - see the [LICENSE](LICENSE) file for details.

Developed for the **Indian Army Internship Programme (AICTE)** in alignment with requirements from the **Army Design Bureau (ADB)** and **Directorate General of Army Air Defence (DG AAD)**.
