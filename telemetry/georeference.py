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
import math
from typing import Dict, List, Optional, Tuple
import numpy as np


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
    ) -> None:
        self.image_width = image_width
        self.image_height = image_height
        self.hfov_deg = hfov_deg
        self.vfov_deg = vfov_deg

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
    ) -> Optional[Dict[str, float]]:
        """
        Calculates real-world WGS-84 geographic coordinates for target pixel (u, v).
        
        Returns:
            Dict containing:
            - lat: Target Latitude (deg)
            - lon: Target Longitude (deg)
            - alt: Target Altitude MSL (m)
            - slant_range_m: Slant range from UAV to target (m)
            - ground_distance_m: Horizontal ground distance (m)
            - azimuth_deg: Compass bearing from UAV to target (deg 0-360)
            Or None if ray points above the horizon (no ground intersection).
        """
        R_cam_to_ned = self.compute_camera_to_ned_rotation(
            uav_roll_deg, uav_pitch_deg, uav_yaw_deg,
            gimbal_roll_deg, gimbal_pitch_deg, gimbal_yaw_deg
        )

        ray_ned = self.pixel_to_ray_ned(u, v, R_cam_to_ned)
        d_north, d_east, d_down = ray_ned[0], ray_ned[1], ray_ned[2]

        # Check ground intersection condition (d_down must point towards earth)
        if d_down <= 0.005:
            # Ray is pointing at or above horizon (sky or loitering counter-UAV target)
            return None

        # Effective vertical height delta from UAV to ground target
        if target_elevation_msl is not None:
            delta_h = max(1.0, uav_alt_msl - target_elevation_msl)
            target_alt = target_elevation_msl
        else:
            delta_h = max(1.0, uav_alt_agl)
            target_alt = uav_alt_msl - delta_h

        # Slant range along ray
        slant_range = delta_h / d_down
        delta_north = slant_range * d_north
        delta_east = slant_range * d_east
        ground_distance = math.hypot(delta_north, delta_east)

        # Azimuth / bearing from UAV to target
        azimuth_deg = (math.degrees(math.atan2(delta_east, delta_north)) + 360.0) % 360.0

        # High-accuracy WGS-84 Local NED to Geodetic conversion
        lat_rad = math.radians(uav_lat)
        sin_lat = math.sin(lat_rad)
        sin_lat_sq = sin_lat * sin_lat

        # Prime vertical radius of curvature N(phi)
        N = self.WGS84_SEMI_MAJOR_AXIS_A / math.sqrt(
            1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq
        )
        # Meridian radius of curvature M(phi)
        M = (
            self.WGS84_SEMI_MAJOR_AXIS_A
            * (1.0 - self.WGS84_ECCENTRICITY_SQ_E2)
            / ((1.0 - self.WGS84_ECCENTRICITY_SQ_E2 * sin_lat_sq) ** 1.5)
        )

        delta_lat_rad = delta_north / (M + uav_alt_msl)
        delta_lon_rad = delta_east / ((N + uav_alt_msl) * math.cos(lat_rad))

        target_lat = uav_lat + math.degrees(delta_lat_rad)
        target_lon = uav_lon + math.degrees(delta_lon_rad)

        return {
            "lat": target_lat,
            "lon": target_lon,
            "alt": target_alt,
            "slant_range_m": slant_range,
            "ground_distance_m": ground_distance,
            "azimuth_deg": azimuth_deg,
            "delta_north_m": delta_north,
            "delta_east_m": delta_east,
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
    ) -> List[List[float]]:
        """
        Calculates the 4-corner ground footprint (swath polygon) of the sensor FOV.
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
                gimbal_roll_deg, gimbal_pitch_deg, gimbal_yaw_deg
            )
            if res is not None:
                polygon_pts.append([res["lon"], res["lat"]])
            else:
                # Fallback if pointing too close to horizon: project reasonable swath bound
                polygon_pts.append([uav_lon, uav_lat])

        if polygon_pts:
            polygon_pts.append(polygon_pts[0])  # Close the ring for GeoJSON polygon

        return polygon_pts
