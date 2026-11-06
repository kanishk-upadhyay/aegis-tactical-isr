"""
AEGIS-TACTICAL-ISR: Mathematical Georeferencing Engine
=====================================================
Transforms 2D sensor pixel coordinates (u, v) into real-world geographic
coordinates (Latitude, Longitude, Altitude MSL/AGL) using full 3D UAV attitude,
gimbal kinematics, optical pinhole projections, and WGS-84 ellipsoidal geodesy.

Standards: STANAG 4609 / WGS-84 / MIL-STD-2525
Author: Directorate General of Army Air Defence & Army Design Bureau Alignment
"""

from __future__ import annotations
from abc import ABC, abstractmethod
import math
from typing import Any, Callable, Dict, List, Optional, Tuple
import numpy as np


class ElevationModel(ABC):
    """Abstract Base Class for Digital Elevation Models (DEM)."""

    @abstractmethod
    def get_elevation(self, lat: float, lon: float) -> float:
        """Returns terrain elevation in meters MSL at given geographic coordinate."""
        pass

    def get_slope_deg(self, lat: float, lon: float, step_m: float = 30.0) -> float:
        """
        Calculates terrain gradient/slope in degrees using central finite differences.
        """
        r_earth = 6378137.0
        lat_rad = math.radians(lat)
        delta_deg_lat = (step_m / r_earth) * (180.0 / math.pi)
        delta_deg_lon = (step_m / (r_earth * math.cos(lat_rad))) * (180.0 / math.pi)

        h_north = self.get_elevation(lat + delta_deg_lat, lon)
        h_south = self.get_elevation(lat - delta_deg_lat, lon)
        h_east = self.get_elevation(lat, lon + delta_deg_lon)
        h_west = self.get_elevation(lat, lon - delta_deg_lon)

        dh_dn = (h_north - h_south) / (2.0 * step_m)
        dh_de = (h_east - h_west) / (2.0 * step_m)

        grad_mag = math.hypot(dh_dn, dh_de)
        return float(math.degrees(math.atan(grad_mag)))


class MountainTerrainDEM(ElevationModel):
    """
    Simulated 30m SRTM Digital Elevation Model for the Northern Border Sector (Ladakh/Siachen).
    Models realistic high-altitude glacial valleys, rugged ridgelines, and mountain slopes.
    """

    def __init__(
        self,
        center_lat: float = 34.1526,
        center_lon: float = 77.5771,
        base_elevation_msl: float = 3900.0,
        peak_amplitude_m: float = 850.0,
    ) -> None:
        self.center_lat = center_lat
        self.center_lon = center_lon
        self.base_elevation_msl = base_elevation_msl
        self.peak_amplitude_m = peak_amplitude_m
        self.r_earth = 6378137.0

    def get_elevation(self, lat: float, lon: float) -> float:
        """
        Continuous, physically realistic high-altitude terrain elevation with ridges and defiles.
        Combines fundamental ridge harmonics with secondary slope perturbations.
        """
        lat_rad = math.radians((self.center_lat + lat) / 2.0)
        dn = math.radians(lat - self.center_lat) * self.r_earth
        de = math.radians(lon - self.center_lon) * (self.r_earth * math.cos(lat_rad))

        # Primary Karakoram / Ladakh ridgeline orientation (NW-SE strike angle ~ 45 deg)
        strike_rad = math.radians(45.0)
        u_along = dn * math.cos(strike_rad) + de * math.sin(strike_rad)
        v_across = -dn * math.sin(strike_rad) + de * math.cos(strike_rad)

        # Valley floor with flanking mountain slopes
        valley_factor = math.cos(v_across / 800.0)
        ridge_harmonics = 0.65 * math.sin(u_along / 600.0) + 0.35 * math.cos(v_across / 350.0)
        micro_topography = 0.15 * math.sin(dn / 150.0) * math.cos(de / 150.0)

        elevation = (
            self.base_elevation_msl
            + self.peak_amplitude_m * (0.5 * (1.0 - valley_factor) + 0.35 * ridge_harmonics + micro_topography)
        )
        return float(elevation)


class AnalyticalSlopeDEM(ElevationModel):
    """
    Planar slope terrain model for mathematical precision validation and unit testing.
    Elevation equation: h(DN, DE) = h0 + slope_n * DN + slope_e * DE
    """

    def __init__(
        self,
        origin_lat: float = 34.0,
        origin_lon: float = 77.0,
        origin_elevation_msl: float = 3000.0,
        slope_north: float = 0.25,   # ~14.0 deg slope
        slope_east: float = 0.10,    # ~5.7 deg slope
    ) -> None:
        self.origin_lat = origin_lat
        self.origin_lon = origin_lon
        self.origin_elevation_msl = origin_elevation_msl
        self.slope_north = slope_north
        self.slope_east = slope_east
        self.r_earth = 6378137.0

    def get_elevation(self, lat: float, lon: float) -> float:
        lat_rad = math.radians((self.origin_lat + lat) / 2.0)
        dn = math.radians(lat - self.origin_lat) * self.r_earth
        de = math.radians(lon - self.origin_lon) * (self.r_earth * math.cos(lat_rad))
        return float(self.origin_elevation_msl + self.slope_north * dn + self.slope_east * de)


class GeoreferenceEngine:
    """
    Precision raycasting engine transforming image plane detections to WGS-84 ground points.
    
    Coordinate Systems:
    - Camera Optical Frame (C): +X right, +Y down, +Z forward (optical axis)
    - Gimbal Frame (G): +X boresight, +Y right, +Z down
    - UAV Body Frame (B): +X nose, +Y right wing, +Z belly
    - Navigation Frame (NED): +X North, +Y East, +Z Down
    - Geodetic Frame: Latitude, Longitude (WGS-84 degrees), Altitude (meters MSL)
    """

    # WGS-84 Ellipsoid Constants
    WGS84_SEMI_MAJOR_AXIS_A: float = 6378137.0          # meters
    WGS84_FLATTENING_F: float = 1.0 / 298.257223563
    WGS84_ECCENTRICITY_SQ_E2: float = (
        2.0 * WGS84_FLATTENING_F - WGS84_FLATTENING_F ** 2
    )

    def __init__(
        self,
        image_width: int = 1920,
        image_height: int = 1080,
        hfov_deg: float = 63.7,
        vfov_deg: float = 38.5,
        dem: Optional[ElevationModel] = None,
    ) -> None:
        self.image_width = image_width
        self.image_height = image_height
        self.hfov_deg = hfov_deg
        self.vfov_deg = vfov_deg
        self.dem = dem

        # Calculate intrinsic pinhole camera parameters
        hfov_rad = math.radians(hfov_deg)
        vfov_rad = math.radians(vfov_deg)
        self.fx = (image_width / 2.0) / math.tan(hfov_rad / 2.0)
        self.fy = (image_height / 2.0) / math.tan(vfov_rad / 2.0)
        self.cx = image_width / 2.0
        self.cy = image_height / 2.0

        # Intrinsic Camera Matrix K and Inverse K
        self.K = np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)
        self.K_inv = np.linalg.inv(self.K)

        # Base rotation aligning Camera Optical Frame (+Z forward, +X right, +Y down)
        # to Gimbal Boresight Frame (+X forward, +Y right, +Z down)
        self.R_cam_to_gimbal_base = np.array([
            [0.0, 0.0, 1.0],  # Gimbal X (Forward) = Camera Z
            [1.0, 0.0, 0.0],  # Gimbal Y (Right)   = Camera X
            [0.0, 1.0, 0.0],  # Gimbal Z (Down)    = Camera Y
        ], dtype=np.float64)

    # --------------------------------------------------------------------------
    # Rotation Matrix Primitives
    # --------------------------------------------------------------------------
    @staticmethod
    def rot_x(angle_rad: float) -> np.ndarray:
        """Roll rotation matrix about X-axis."""
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        return np.array([
            [1.0, 0.0, 0.0],
            [0.0, c, -s],
            [0.0, s, c]
        ], dtype=np.float64)

    @staticmethod
    def rot_y(angle_rad: float) -> np.ndarray:
        """Pitch rotation matrix about Y-axis."""
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        return np.array([
            [c, 0.0, s],
            [0.0, 1.0, 0.0],
            [-s, 0.0, c]
        ], dtype=np.float64)

    @staticmethod
    def rot_z(angle_rad: float) -> np.ndarray:
        """Yaw / Azimuth rotation matrix about Z-axis."""
        c, s = math.cos(angle_rad), math.sin(angle_rad)
        return np.array([
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0]
        ], dtype=np.float64)

    def compute_camera_to_ned_rotation(
        self,
        uav_roll_deg: float,
        uav_pitch_deg: float,
        uav_yaw_deg: float,
        gimbal_roll_deg: float,
        gimbal_pitch_deg: float,
        gimbal_yaw_deg: float,
    ) -> np.ndarray:
        """
        Computes composite rotation matrix from Camera Optical frame to Navigation NED frame.
        Convention:
        - UAV Body to NED: R_z(yaw) * R_y(pitch) * R_x(roll)
        - Gimbal to Body:  R_z(pan) * R_y(tilt) * R_x(roll)
        - Camera to Gimbal: R_cam_to_gimbal_base
        """
        # UAV Attitude: Body -> NED
        R_body_to_ned = (
            self.rot_z(math.radians(uav_yaw_deg))
            @ self.rot_y(math.radians(uav_pitch_deg))
            @ self.rot_x(math.radians(uav_roll_deg))
        )

        # Gimbal Kinematics: Gimbal -> Body
        R_gimbal_to_body = (
            self.rot_z(math.radians(gimbal_yaw_deg))
            @ self.rot_y(math.radians(gimbal_pitch_deg))
            @ self.rot_x(math.radians(gimbal_roll_deg))
        )

        # Composite transformation: Camera Optical -> NED
        R_cam_to_ned = R_body_to_ned @ R_gimbal_to_body @ self.R_cam_to_gimbal_base
        return R_cam_to_ned

    # --------------------------------------------------------------------------
    # Raycasting & Geodetic Conversions
    # --------------------------------------------------------------------------
    def pixel_to_ray_ned(
        self,
        u: float,
        v: float,
        R_cam_to_ned: np.ndarray,
    ) -> np.ndarray:
        """
        Casts a normalized line-of-sight unit vector from image coordinate (u, v) into NED frame.
        """
        # Normalized camera ray vector
        pixel_homo = np.array([u, v, 1.0], dtype=np.float64)
        ray_cam = self.K_inv @ pixel_homo
        ray_cam_norm = ray_cam / np.linalg.norm(ray_cam)

        # Transform optical ray into NED navigation coordinates
        ray_ned = R_cam_to_ned @ ray_cam_norm
        return ray_ned / np.linalg.norm(ray_ned)

    def ned_to_geodetic(
        self,
        uav_lat: float,
        uav_lon: float,
        uav_alt_msl: float,
        delta_north: float,
        delta_east: float,
    ) -> Tuple[float, float]:
        """Converts local NED displacement (meters) to WGS-84 Geodetic coordinates (degrees)."""
        lat_rad = math.radians(uav_lat)
        sin_lat = math.sin(lat_rad)
        sin_lat_sq = sin_lat * sin_lat

        N = self.WGS84_SEMI_MAJOR_AXIS_A / math.sqrt(
            1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq
        )
        M = (
            self.WGS84_SEMI_MAJOR_AXIS_A
            * (1.0 - self.WGS84_ECCENTRICITY_SQ_E2)
            / ((1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq) ** 1.5)
        )

        delta_lat_rad = delta_north / (M + uav_alt_msl)
        delta_lon_rad = delta_east / ((N + uav_alt_msl) * math.cos(lat_rad))

        target_lat = uav_lat + math.degrees(delta_lat_rad)
        target_lon = uav_lon + math.degrees(delta_lon_rad)
        return float(target_lat), float(target_lon)

    def geodetic_to_ned(
        self,
        uav_lat: float,
        uav_lon: float,
        uav_alt_msl: float,
        target_lat: float,
        target_lon: float,
    ) -> Tuple[float, float]:
        """Converts WGS-84 coordinates to local NED displacement (meters)."""
        lat_rad = math.radians(uav_lat)
        sin_lat = math.sin(lat_rad)
        sin_lat_sq = sin_lat * sin_lat

        N = self.WGS84_SEMI_MAJOR_AXIS_A / math.sqrt(
            1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq
        )
        M = (
            self.WGS84_SEMI_MAJOR_AXIS_A
            * (1.0 - self.WGS84_ECCENTRICITY_SQ_E2)
            / ((1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq) ** 1.5)
        )

        delta_north = math.radians(target_lat - uav_lat) * (M + uav_alt_msl)
        delta_east = math.radians(target_lon - uav_lon) * ((N + uav_alt_msl) * math.cos(lat_rad))
        return float(delta_north), float(delta_east)

    def raycast_terrain_intersection(
        self,
        uav_lat: float,
        uav_lon: float,
        uav_alt_msl: float,
        ray_ned: np.ndarray,
        dem: ElevationModel,
        max_slant_range_m: float = 25000.0,
        step_size_m: float = 20.0,
    ) -> Optional[Dict[str, float]]:
        """
        Iterative 3D non-planar line-terrain intersection solver using hybrid
        coarse ray-marching followed by secant-bisection refinement.
        Eliminates flat-earth elevation error in steep mountainous border terrain.
        """
        d_north = float(ray_ned[0])
        d_east = float(ray_ned[1])
        d_down = float(ray_ned[2])

        if d_down <= 0.005:
            return None  # Ray points at or above horizon

        def height_above_terrain(s: float) -> Tuple[float, float, float, float]:
            dn = s * d_north
            de = s * d_east
            lat, lon = self.ned_to_geodetic(uav_lat, uav_lon, uav_alt_msl, dn, de)
            dem_alt = dem.get_elevation(lat, lon)
            ray_alt = uav_alt_msl - s * d_down
            delta_f = ray_alt - dem_alt
            return delta_f, lat, lon, dem_alt

        # Check UAV altitude above terrain at launch
        f_init, _, _, _ = height_above_terrain(1.0)
        if f_init <= 0.0:
            return None  # Platform is on or below terrain

        # 1. Coarse Ray Marching: identify bracket [s_a, s_b] where f changes sign
        s_prev = 0.0
        f_prev = f_init
        s_max = min(max_slant_range_m, max(200.0, (uav_alt_msl - 500.0) / d_down))
        s_step = max(5.0, step_size_m)
        s = s_step

        found_bracket = False
        s_a = 0.0
        s_b = s_max
        f_a = f_init
        f_b = 0.0

        while s <= s_max:
            f_val, _, _, _ = height_above_terrain(s)
            if f_val <= 0.0:
                s_a = s_prev
                s_b = s
                f_a = f_prev
                f_b = f_val
                found_bracket = True
                break
            s_prev = s
            f_prev = f_val
            s += s_step

        if not found_bracket:
            return None

        # 2. Hybrid Secant & Bisection refinement for sub-centimeter accuracy
        s_best = 0.5 * (s_a + s_b)
        iterations = 0

        for _ in range(25):
            iterations += 1
            if abs(f_b - f_a) > 1e-4:
                # Secant step
                s_sec = s_b - f_b * (s_b - s_a) / (f_b - f_a)
                if s_a < s_sec < s_b:
                    s_mid = s_sec
                else:
                    s_mid = 0.5 * (s_a + s_b)
            else:
                s_mid = 0.5 * (s_a + s_b)

            f_mid, _, _, _ = height_above_terrain(s_mid)
            s_best = s_mid

            if abs(f_mid) < 0.005 or (s_b - s_a) < 0.005:
                break

            if f_mid > 0.0:
                s_a = s_mid
                f_a = f_mid
            else:
                s_b = s_mid
                f_b = f_mid

        # Extract final geodetic target position
        delta_north = s_best * d_north
        delta_east = s_best * d_east
        target_lat, target_lon = self.ned_to_geodetic(uav_lat, uav_lon, uav_alt_msl, delta_north, delta_east)
        target_alt = dem.get_elevation(target_lat, target_lon)
        slope_deg = dem.get_slope_deg(target_lat, target_lon)
        ground_distance = math.hypot(delta_north, delta_east)
        azimuth_deg = (math.degrees(math.atan2(delta_east, delta_north)) + 360.0) % 360.0

        return {
            "lat": target_lat,
            "lon": target_lon,
            "alt": target_alt,
            "slant_range_m": s_best,
            "ground_distance_m": ground_distance,
            "azimuth_deg": azimuth_deg,
            "delta_north_m": delta_north,
            "delta_east_m": delta_east,
            "terrain_elevation_msl": target_alt,
            "terrain_slope_deg": slope_deg,
            "raycast_iterations": iterations,
            "elevation_source": "DEM_3D_NONPLANAR",
        }

    def georeference_pixel(
        self,
        u: float,
        v: float,
        uav_lat: float,
        uav_lon: float,
        uav_alt_msl: float,
        uav_alt_agl: float,
        uav_roll_deg: float = 0.0,
        uav_pitch_deg: float = 0.0,
        uav_yaw_deg: float = 0.0,
        gimbal_roll_deg: float = 0.0,
        gimbal_pitch_deg: float = -90.0,
        gimbal_yaw_deg: float = 0.0,
        target_elevation_msl: Optional[float] = None,
        dem: Optional[ElevationModel] = None,
    ) -> Optional[Dict[str, float]]:
        """
        Calculates real-world WGS-84 geographic coordinates for target pixel (u, v).
        Supports both 3D Digital Elevation Model (DEM) raymarching and flat-earth projections.
        """
        R_cam_to_ned = self.compute_camera_to_ned_rotation(
            uav_roll_deg, uav_pitch_deg, uav_yaw_deg,
            gimbal_roll_deg, gimbal_pitch_deg, gimbal_yaw_deg
        )

        ray_ned = self.pixel_to_ray_ned(u, v, R_cam_to_ned)
        d_north, d_east, d_down = ray_ned[0], ray_ned[1], ray_ned[2]

        if d_down <= 0.005:
            # Ray is pointing at or above horizon (sky or loitering counter-UAV target)
            return None

        active_dem = dem if dem is not None else self.dem
        if active_dem is not None and target_elevation_msl is None:
            # Execute 3D non-planar terrain raycasting
            dem_result = self.raycast_terrain_intersection(
                uav_lat, uav_lon, uav_alt_msl, ray_ned, active_dem
            )
            if dem_result is not None:
                return dem_result

        # Planar / Flat-Earth fallback calculation
        if target_elevation_msl is not None:
            delta_h = max(1.0, uav_alt_msl - target_elevation_msl)
            target_alt = target_elevation_msl
        else:
            delta_h = max(1.0, uav_alt_agl)
            target_alt = uav_alt_msl - delta_h

        slant_range = delta_h / d_down
        delta_north = slant_range * d_north
        delta_east = slant_range * d_east
        ground_distance = math.hypot(delta_north, delta_east)
        azimuth_deg = (math.degrees(math.atan2(delta_east, delta_north)) + 360.0) % 360.0

        target_lat, target_lon = self.ned_to_geodetic(
            uav_lat, uav_lon, uav_alt_msl, delta_north, delta_east
        )

        return {
            "lat": target_lat,
            "lon": target_lon,
            "alt": target_alt,
            "slant_range_m": slant_range,
            "ground_distance_m": ground_distance,
            "azimuth_deg": azimuth_deg,
            "delta_north_m": delta_north,
            "delta_east_m": delta_east,
            "terrain_elevation_msl": target_alt,
            "terrain_slope_deg": 0.0,
            "raycast_iterations": 1,
            "elevation_source": "PLANAR_FALLBACK",
        }

    def project_gps_to_pixel(
        self,
        target_lat: float,
        target_lon: float,
        target_alt_msl: float,
        uav_lat: float,
        uav_lon: float,
        uav_alt_msl: float,
        uav_roll_deg: float = 0.0,
        uav_pitch_deg: float = 0.0,
        uav_yaw_deg: float = 0.0,
        gimbal_roll_deg: float = 0.0,
        gimbal_pitch_deg: float = -90.0,
        gimbal_yaw_deg: float = 0.0,
    ) -> Optional[Tuple[float, float, bool]]:
        """
        Inverse projection: projects a 3D WGS-84 coordinate back into camera pixel plane.
        Returns (u, v, is_in_frame) or None if target is behind the camera sensor.
        """
        lat_rad = math.radians(uav_lat)
        sin_lat = math.sin(lat_rad)
        sin_lat_sq = sin_lat * sin_lat

        N = self.WGS84_SEMI_MAJOR_AXIS_A / math.sqrt(
            1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq
        )
        M = (
            self.WGS84_SEMI_MAJOR_AXIS_A
            * (1.0 - self.WGS84_ECCENTRICITY_SQ_E2)
            / ((1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq) ** 1.5)
        )

        delta_north = math.radians(target_lat - uav_lat) * (M + uav_alt_msl)
        delta_east = math.radians(target_lon - uav_lon) * ((N + uav_alt_msl) * math.cos(lat_rad))
        delta_down = uav_alt_msl - target_alt_msl

        P_ned = np.array([delta_north, delta_east, delta_down], dtype=np.float64)

        R_cam_to_ned = self.compute_camera_to_ned_rotation(
            uav_roll_deg, uav_pitch_deg, uav_yaw_deg,
            gimbal_roll_deg, gimbal_pitch_deg, gimbal_yaw_deg
        )
        # Invert rotation to get NED -> Camera
        R_ned_to_cam = R_cam_to_ned.T
        P_cam = R_ned_to_cam @ P_ned

        # P_cam = [Xc, Yc, Zc]
        if P_cam[2] <= 0.01:
            return None  # Behind optical center

        u = float(self.fx * (P_cam[0] / P_cam[2]) + self.cx)
        v = float(self.fy * (P_cam[1] / P_cam[2]) + self.cy)

        is_in_frame = bool((0 <= u <= self.image_width) and (0 <= v <= self.image_height))
        return (u, v, is_in_frame)

    def compute_sensor_ground_footprint(
        self,
        uav_lat: float,
        uav_lon: float,
        uav_alt_msl: float,
        uav_alt_agl: float,
        uav_roll_deg: float = 0.0,
        uav_pitch_deg: float = 0.0,
        uav_yaw_deg: float = 0.0,
        gimbal_roll_deg: float = 0.0,
        gimbal_pitch_deg: float = -90.0,
        gimbal_yaw_deg: float = 0.0,
        dem: Optional[ElevationModel] = None,
    ) -> List[List[float]]:
        """
        Calculates the 4-corner ground footprint (swath polygon) of the sensor FOV.
        Conforms to non-planar 3D digital elevation models if provided.
        Returns GeoJSON coordinates list: [[lon, lat], [lon, lat], ..., [lon, lat]] (closed loop).
        """
        corners = [
            (0.0, 0.0),                           # Top-Left
            (float(self.image_width), 0.0),       # Top-Right
            (float(self.image_width), float(self.image_height)),  # Bottom-Right
            (0.0, float(self.image_height)),      # Bottom-Left
        ]

        polygon_pts: List[List[float]] = []
        for u, v in corners:
            res = self.georeference_pixel(
                u, v, uav_lat, uav_lon, uav_alt_msl, uav_alt_agl,
                uav_roll_deg, uav_pitch_deg, uav_yaw_deg,
                gimbal_roll_deg, gimbal_pitch_deg, gimbal_yaw_deg,
                dem=dem,
            )
            if res is not None:
                polygon_pts.append([res["lon"], res["lat"]])
            else:
                # Fallback if pointing too close to horizon: project reasonable swath bound
                polygon_pts.append([uav_lon, uav_lat])

        if polygon_pts:
            polygon_pts.append(polygon_pts[0])  # Close the ring for GeoJSON polygon

        return polygon_pts
