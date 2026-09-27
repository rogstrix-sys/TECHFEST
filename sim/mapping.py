"""
sim/mapping.py: 3D LiDAR Perception & Occupancy Voxel Grid Mapping Engine.

Simulates onboard multi-channel rotating LiDAR sensors and OctoMap-style 3D
occupancy voxel grid mapping for autonomous UAV swarm perception and SLAM.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any, Dict, List, Optional, Set, Tuple
import numpy as np

from sim.obstacles import ObstacleAABB


@dataclass
class LiDARPoint:
    """A single 3D point return from a LiDAR scan ray."""
    x: float
    y: float
    z: float
    range_m: float
    intensity: float  # [0.0, 1.0] reflection intensity
    obstacle_id: Optional[str] = None


@dataclass
class LiDARScan:
    """A full spherical / cylindrical sweep of LiDAR points at a given timestamp."""
    timestamp: float
    drone_id: str
    sensor_origin: np.ndarray  # [x, y, z] in world frame
    points: List[LiDARPoint] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": round(float(self.timestamp), 3),
            "drone_id": self.drone_id,
            "origin": [round(float(c), 2) for c in self.sensor_origin],
            "num_points": len(self.points),
            # Downsampled/compact point list for fast WebSocket transport [x, y, z, intensity]
            "points": [
                [
                    round(p.x, 2),
                    round(p.y, 2),
                    round(p.z, 2),
                    round(p.intensity, 2)
                ]
                for p in self.points
            ],
        }


class LiDARScanner:
    """
    Simulates a multi-beam rotating 3D LiDAR sensor (e.g., Velodyne VLP-16 / Ouster OS1 equivalent).
    
    Features:
    - Configurable horizontal & vertical FOV with azimuth/elevation steps.
    - Ray-casting against 3D AABB building obstacles and ground terrain.
    - Gaussian range noise model N(0, sigma^2).
    - Ray incident angle reflection intensity modeling.
    """

    def __init__(
        self,
        max_range_m: float = 75.0,
        min_range_m: float = 0.5,
        horizontal_fov_deg: float = 360.0,
        horizontal_resolution_deg: float = 6.0,  # 60 azimuth beams per ring
        vertical_fov_deg: Tuple[float, float] = (-50.0, 15.0),  # -50 deg downward ground look to +15 deg upward
        vertical_channels: int = 16,  # 16 elevation rings (960 rays total)
        range_noise_std_m: float = 0.03,  # 3cm range measurement noise
    ) -> None:
        self.max_range = float(max_range_m)
        self.min_range = float(min_range_m)
        self.horizontal_fov_deg = float(horizontal_fov_deg)
        self.horizontal_resolution_deg = float(horizontal_resolution_deg)
        self.vertical_fov_min_deg, self.vertical_fov_max_deg = vertical_fov_deg
        self.vertical_channels = int(vertical_channels)
        self.range_noise_std = float(range_noise_std_m)

        # Precompute ray unit vectors in sensor body frame
        self._body_ray_dirs = self._precompute_ray_directions()

    def _precompute_ray_directions(self) -> np.ndarray:
        """Precomputes unit ray direction vectors in sensor frame [N_rays, 3]."""
        num_azimuth = int(round(self.horizontal_fov_deg / self.horizontal_resolution_deg))
        azimuths = np.linspace(-np.pi, np.pi, num_azimuth, endpoint=False)
        elevations = np.linspace(
            np.radians(self.vertical_fov_min_deg),
            np.radians(self.vertical_fov_max_deg),
            self.vertical_channels
        )

        dirs = []
        for el in elevations:
            cos_el = np.cos(el)
            sin_el = np.sin(el)
            for az in azimuths:
                dx = cos_el * np.cos(az)
                dy = cos_el * np.sin(az)
                dz = sin_el
                dirs.append([dx, dy, dz])

        return np.asarray(dirs, dtype=np.float64)

    def scan(
        self,
        drone_id: str,
        position: np.ndarray,
        attitude: np.ndarray,  # [roll, pitch, yaw] radians
        obstacles: List[ObstacleAABB],
        sim_time: float = 0.0,
    ) -> LiDARScan:
        """
        Executes a 3D LiDAR scan sweep from position with given attitude.
        Rays are tested against all obstacles and ground plane (z = 0)
        using fast vectorized raycasting.
        """
        pos = np.asarray(position, dtype=np.float64)
        roll, pitch, yaw = attitude[0], attitude[1], attitude[2]

        # Rotation matrix from body to world frame
        # Z-Y-X Euler angle rotation
        cz, sz = np.cos(yaw), np.sin(yaw)
        cy, sy = np.cos(pitch), np.sin(pitch)
        cx, sx = np.cos(roll), np.sin(roll)

        R_z = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
        R_y = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
        R_x = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
        R_body_to_world = R_z @ R_y @ R_x

        # Rotate ray directions into world coordinates
        world_ray_dirs = self._body_ray_dirs @ R_body_to_world.T
        N_rays = len(world_ray_dirs)

        scan = LiDARScan(timestamp=sim_time, drone_id=drone_id, sensor_origin=pos)

        # Broadphase filter: only test obstacles within max_range + bounding radius
        active_obstacles: List[ObstacleAABB] = []
        for obs in obstacles:
            if obs.distance_to_point(pos) <= self.max_range:
                active_obstacles.append(obs)

        closest_dist = np.full(N_rays, self.max_range, dtype=np.float64)
        hit_obstacle_id = np.array([None] * N_rays, dtype=object)
        hit_normals = np.zeros((N_rays, 3), dtype=np.float64)
        hit_normals[:, 2] = 1.0

        # 1. Ground plane intersection (z = 0)
        downward_mask = world_ray_dirs[:, 2] < -1e-5
        if np.any(downward_mask):
            t_ground = -pos[2] / world_ray_dirs[downward_mask, 2]
            valid_g = downward_mask.copy()
            valid_g[downward_mask] = (t_ground >= self.min_range) & (t_ground < closest_dist[downward_mask])
            closest_dist[valid_g] = t_ground[valid_g[downward_mask]]
            hit_obstacle_id[valid_g] = "GROUND"
            hit_normals[valid_g] = [0.0, 0.0, 1.0]

        # 2. Obstacles intersection (Vectorized Ray-AABB slab test)
        if active_obstacles:
            safe_dirs = np.where(np.abs(world_ray_dirs) > 1e-7, world_ray_dirs, 1e-7)
            inv_dirs = 1.0 / safe_dirs
            for obs in active_obstacles:
                min_pt = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
                max_pt = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
                if min_pt is None or max_pt is None:
                    continue
                t1 = (min_pt - pos) * inv_dirs
                t2 = (max_pt - pos) * inv_dirs
                t_min = np.maximum(np.maximum(np.minimum(t1[:, 0], t2[:, 0]), np.minimum(t1[:, 1], t2[:, 1])), np.minimum(t1[:, 2], t2[:, 2]))
                t_max = np.minimum(np.minimum(np.maximum(t1[:, 0], t2[:, 0]), np.maximum(t1[:, 1], t2[:, 1])), np.maximum(t1[:, 2], t2[:, 2]))
                hits = (t_max >= np.maximum(0.0, t_min)) & (t_min < closest_dist) & (t_min >= self.min_range)
                if np.any(hits):
                    closest_dist[hits] = t_min[hits]
                    hit_obstacle_id[hits] = obs.id
                    hit_pts = pos + world_ray_dirs[hits] * t_min[hits, np.newaxis]
                    center = (min_pt + max_pt) * 0.5
                    extent = (max_pt - min_pt) * 0.5
                    norm_diff = (hit_pts - center) / np.maximum(1e-4, extent)
                    dominant_axis = np.argmax(np.abs(norm_diff), axis=1)
                    normals = np.zeros_like(hit_pts)
                    for idx, axis in enumerate(dominant_axis):
                        normals[idx, axis] = np.sign(norm_diff[idx, axis])
                    hit_normals[hits] = normals

        # 3. Assemble point cloud returns
        valid_indices = np.where((closest_dist < self.max_range) & (hit_obstacle_id != None))[0]
        if len(valid_indices) > 0:
            dists = closest_dist[valid_indices]
            dirs = world_ray_dirs[valid_indices]
            normals = hit_normals[valid_indices]
            obs_ids = hit_obstacle_id[valid_indices]

            # Add range measurement noise
            if self.range_noise_std > 0:
                noise = np.random.normal(0.0, self.range_noise_std, size=len(dists))
                dists = np.maximum(self.min_range, dists + noise)

            hit_positions = pos + dirs * dists[:, np.newaxis]

            # Reflection intensity based on Lambertian cosine of incident angle and range decay
            cos_incidence = np.maximum(0.1, np.abs(np.sum(-dirs * normals, axis=1)))
            range_factor = np.maximum(0.2, 1.0 - (dists / self.max_range) * 0.5)
            intensities = np.minimum(1.0, cos_incidence * range_factor)

            for i in range(len(valid_indices)):
                scan.points.append(
                    LiDARPoint(
                        x=float(hit_positions[i, 0]),
                        y=float(hit_positions[i, 1]),
                        z=float(hit_positions[i, 2]),
                        range_m=float(dists[i]),
                        intensity=float(intensities[i]),
                        obstacle_id=str(obs_ids[i]),
                    )
                )

        return scan


@dataclass
class VoxelNode:
    """A single 3D occupancy voxel cell."""
    ix: int
    iy: int
    iz: int
    center: np.ndarray
    log_odds: float = 0.0  # L(m) log-odds of occupancy

    @property
    def occupancy_prob(self) -> float:
        """P(m) = 1 / (1 + exp(-L))"""
        return 1.0 / (1.0 + math.exp(-max(-20.0, min(20.0, self.log_odds))))

    @property
    def is_occupied(self) -> bool:
        """True if probability exceeds standard occupancy threshold (e.g. 0.65)."""
        return self.occupancy_prob >= 0.65

    @property
    def is_free(self) -> bool:
        """True if probability is below free space threshold (e.g. 0.35)."""
        return self.occupancy_prob <= 0.35


class OccupancyGridMap3D:
    """
    3D Occupancy Voxel Grid based on OctoMap log-odds formulation.
    
    Provides:
    - Sparse spatial hashing for infinite/large disaster zones without memory bloat.
    - Ray-marching free-space and hit updates:
        L_new = clamp(L_prev + l_occ, L_min, L_max) for hit voxels
        L_new = clamp(L_prev + l_free, L_min, L_max) for traversed ray voxels
    - Coverage calculation: total volume surveyed and obstacle surface volume.
    """

    def __init__(
        self,
        voxel_size_m: float = 4.0,  # 4m resolution for disaster-scale map
        l_occ: float = 0.85,        # Log-odds increment for hit
        l_free: float = -0.35,      # Log-odds decrement for free ray traversal
        l_min: float = -2.5,        # Clamping lower bound
        l_max: float = 3.5,         # Clamping upper bound
        bounds_x: Tuple[float, float] = (-350.0, 350.0),
        bounds_y: Tuple[float, float] = (-350.0, 350.0),
        bounds_z: Tuple[float, float] = (-5.0, 120.0),
    ) -> None:
        self.voxel_size = float(voxel_size_m)
        self.inv_voxel_size = 1.0 / self.voxel_size
        self.l_occ = float(l_occ)
        self.l_free = float(l_free)
        self.l_min = float(l_min)
        self.l_max = float(l_max)
        self.bounds_x = bounds_x
        self.bounds_y = bounds_y
        self.bounds_z = bounds_z

        # Sparse dictionary of active voxels keyed by (ix, iy, iz)
        self.voxels: Dict[Tuple[int, int, int], VoxelNode] = {}
        self.total_surveyed_points: int = 0
        self.accumulated_hits: List[Tuple[float, float, float, float]] = []

    def world_to_grid(self, pt: np.ndarray) -> Tuple[int, int, int]:
        """Maps 3D world coordinates [x, y, z] to discrete integer grid indices."""
        ix = int(math.floor(pt[0] * self.inv_voxel_size))
        iy = int(math.floor(pt[1] * self.inv_voxel_size))
        iz = int(math.floor(pt[2] * self.inv_voxel_size))
        return (ix, iy, iz)

    def grid_to_world(self, key: Tuple[int, int, int]) -> np.ndarray:
        """Returns the continuous 3D centroid of a voxel cell."""
        ix, iy, iz = key
        return np.array([
            (ix + 0.5) * self.voxel_size,
            (iy + 0.5) * self.voxel_size,
            (iz + 0.5) * self.voxel_size,
        ], dtype=np.float64)

    def is_in_bounds(self, pt: np.ndarray) -> bool:
        """Checks if world point is within operational volume."""
        return (
            self.bounds_x[0] <= pt[0] <= self.bounds_x[1]
            and self.bounds_y[0] <= pt[1] <= self.bounds_y[1]
            and self.bounds_z[0] <= pt[2] <= self.bounds_z[1]
        )

    def insert_scan(self, scan: LiDARScan, max_traversal_steps: int = 6) -> None:
        """
        Integrates a LiDAR scan into the 3D occupancy map using log-odds updates.
        Free space voxels along the ray are decremented; hit endpoints are incremented.
        """
        origin = scan.sensor_origin
        if not self.is_in_bounds(origin):
            return

        self.total_surveyed_points += len(scan.points)

        for pt in scan.points:
            hit_world = np.array([pt.x, pt.y, pt.z], dtype=np.float64)
            if not self.is_in_bounds(hit_world):
                continue

            if len(self.accumulated_hits) < 100000:
                self.accumulated_hits.append((float(pt.x), float(pt.y), float(pt.z), float(pt.intensity)))

            # 1. Update Hit Voxel
            hit_key = self.world_to_grid(hit_world)
            if hit_key not in self.voxels:
                self.voxels[hit_key] = VoxelNode(
                    ix=hit_key[0],
                    iy=hit_key[1],
                    iz=hit_key[2],
                    center=self.grid_to_world(hit_key),
                    log_odds=0.0
                )
            voxel = self.voxels[hit_key]
            voxel.log_odds = min(self.l_max, max(self.l_min, voxel.log_odds + self.l_occ))

            # 2. Sample Free-Space Voxels along ray (Bresenham / coarse ray-march)
            ray_vec = hit_world - origin
            dist = float(np.linalg.norm(ray_vec))
            if dist > self.voxel_size:
                step_size = self.voxel_size * 2.0
                num_steps = min(max_traversal_steps, int(dist / step_size))
                for s in range(1, num_steps):
                    sample_pt = origin + ray_vec * (s * step_size / dist)
                    free_key = self.world_to_grid(sample_pt)
                    if free_key == hit_key:
                        break
                    if free_key not in self.voxels:
                        self.voxels[free_key] = VoxelNode(
                            ix=free_key[0],
                            iy=free_key[1],
                            iz=free_key[2],
                            center=self.grid_to_world(free_key),
                            log_odds=0.0
                        )
                    free_vox = self.voxels[free_key]
                    free_vox.log_odds = min(self.l_max, max(self.l_min, free_vox.log_odds + self.l_free))

    def get_occupied_voxels(self, max_count: int = 1500) -> List[Dict[str, Any]]:
        """
        Returns list of occupied voxels formatted for 3D Three.js client visualization.
        """
        occupied = []
        for key, v in self.voxels.items():
            if v.is_occupied:
                occupied.append({
                    "pos": [round(float(c), 2) for c in v.center],
                    "prob": round(float(v.occupancy_prob), 2),
                    "size": self.voxel_size,
                })
                if len(occupied) >= max_count:
                    break
        return occupied

    def compute_metrics(self) -> Dict[str, float]:
        """Calculates quantitative SLAM mapping metrics for analytics and reporting."""
        occupied_count = sum(1 for v in self.voxels.values() if v.is_occupied)
        free_count = sum(1 for v in self.voxels.values() if v.is_free)
        total_cells = len(self.voxels)

        voxel_vol = self.voxel_size ** 3
        mapped_volume_m3 = total_cells * voxel_vol
        obstacle_volume_m3 = occupied_count * voxel_vol

        # Theater boundary volume
        total_vol = (
            (self.bounds_x[1] - self.bounds_x[0])
            * (self.bounds_y[1] - self.bounds_y[0])
            * (self.bounds_z[1] - self.bounds_z[0])
        )
        coverage_pct = min(100.0, (mapped_volume_m3 / max(1.0, total_vol)) * 100.0 * 20.0)  # scaled explored ratio

        return {
            "occupied_voxels": float(occupied_count),
            "free_voxels": float(free_count),
            "total_mapped_cells": float(total_cells),
            "mapped_volume_m3": round(float(mapped_volume_m3), 1),
            "obstacle_volume_m3": round(float(obstacle_volume_m3), 1),
            "coverage_pct": round(float(coverage_pct), 1),
        }

    def export_point_cloud_ply(self, filename: Optional[str] = None) -> str:
        """
        Exports the 3D LiDAR SLAM Point Cloud in Stanford ASCII PLY format.
        Universally compatible with CloudCompare, Blender (Stanford PLY Importer),
        MeshLab, and Open3D.
        Points are colored by altitude (Z) with reflection intensity values.
        """
        points: List[Tuple[float, float, float, float]] = []
        if hasattr(self, "accumulated_hits") and self.accumulated_hits:
            points = self.accumulated_hits[-65000:]
        else:
            for v in self.voxels.values():
                if v.is_occupied:
                    c = v.center
                    points.append((float(c[0]), float(c[1]), float(c[2]), float(v.occupancy_prob)))

        # Fallback if no scans yet: generate baseline terrain points
        if not points:
            for x in np.linspace(-250.0, 250.0, 40):
                for y in np.linspace(-250.0, 250.0, 40):
                    points.append((float(x), float(y), 0.0, 0.5))

        lines = [
            "ply",
            "format ascii 1.0",
            "comment UAV Swarm Autonomous SLAM LiDAR Point Cloud",
            f"element vertex {len(points)}",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "property float intensity",
            "end_header",
        ]

        z_vals = [p[2] for p in points]
        z_min = min(z_vals) if z_vals else 0.0
        z_max = max(z_vals) if z_vals else 50.0
        z_range = max(1.0, z_max - z_min)

        for x, y, z, intensity in points:
            # Color map based on altitude (altitude rainbow gradient)
            norm_z = (z - z_min) / z_range
            if norm_z < 0.25:
                r, g, b = 0, int(255 * (norm_z / 0.25)), 255
            elif norm_z < 0.5:
                r, g, b = 0, 255, int(255 * (1.0 - (norm_z - 0.25) / 0.25))
            elif norm_z < 0.75:
                r, g, b = int(255 * ((norm_z - 0.5) / 0.25)), 255, 0
            else:
                r, g, b = 255, int(255 * (1.0 - (norm_z - 0.75) / 0.25)), 50
            lines.append(f"{x:.3f} {y:.3f} {z:.3f} {r} {g} {b} {intensity:.2f}")

        ply_data = "\n".join(lines) + "\n"

        if filename:
            with open(filename, "w", encoding="utf-8") as f:
                f.write(ply_data)

        return ply_data

    def export_point_cloud_las(self, filename: Optional[str] = None) -> bytes:
        """
        Exports the 3D LiDAR SLAM Point Cloud in ASPRS LAS 1.2 Binary format.
        Compatible with CloudCompare, PDAL, QGIS, ArcGIS, and civil survey packages.
        Point Data Format 2 (includes RGB color and reflection intensity).
        """
        import struct

        points: List[Tuple[float, float, float, float]] = []
        if hasattr(self, "accumulated_hits") and self.accumulated_hits:
            points = self.accumulated_hits[-65000:]
        else:
            for v in self.voxels.values():
                if v.is_occupied:
                    c = v.center
                    points.append((float(c[0]), float(c[1]), float(c[2]), float(v.occupancy_prob)))

        if not points:
            for x in np.linspace(-250.0, 250.0, 40):
                for y in np.linspace(-250.0, 250.0, 40):
                    points.append((float(x), float(y), 0.0, 0.5))

        num_points = len(points)
        x_vals = [p[0] for p in points]
        y_vals = [p[1] for p in points]
        z_vals = [p[2] for p in points]

        min_x, max_x = min(x_vals), max(x_vals)
        min_y, max_y = min(y_vals), max(y_vals)
        min_z, max_z = min(z_vals), max(z_vals)
        z_range = max(1.0, max_z - min_z)

        scale_x = scale_y = scale_z = 0.001
        offset_x = offset_y = offset_z = 0.0

        header_size = 227
        offset_to_points = 227
        point_data_format = 2  # Format 2 includes RGB color
        point_record_len = 26  # Format 2 byte length

        # ASPRS LAS 1.2 Header (227 bytes)
        header = bytearray()
        header.extend(b"LASF")                                # 4 bytes: Signature
        header.extend(struct.pack("<H", 0))                   # 2 bytes: File Source ID
        header.extend(struct.pack("<H", 0))                   # 2 bytes: Global Encoding
        header.extend(b"\x00" * 16)                           # 16 bytes: Project ID GUID
        header.extend(struct.pack("BB", 1, 2))                # 2 bytes: Version Major 1, Minor 2
        header.extend(b"UAV-X AUTONOMOUS SLAM".ljust(32, b"\x00"))      # 32 bytes: System Identifier
        header.extend(b"UAV Swarm Mapping Engine".ljust(32, b"\x00"))   # 32 bytes: Generating Software
        header.extend(struct.pack("<H", 1))                   # 2 bytes: Day of Year
        header.extend(struct.pack("<H", 2026))                # 2 bytes: Year
        header.extend(struct.pack("<H", header_size))         # 2 bytes: Header Size
        header.extend(struct.pack("<I", offset_to_points))    # 4 bytes: Offset to Point Data
        header.extend(struct.pack("<I", 0))                   # 4 bytes: Number of Variable Length Records
        header.extend(struct.pack("B", point_data_format))    # 1 byte: Point Format
        header.extend(struct.pack("<H", point_record_len))    # 2 bytes: Point Record Length
        header.extend(struct.pack("<I", num_points))          # 4 bytes: Number of Point Records
        header.extend(struct.pack("<5I", num_points, 0, 0, 0, 0)) # 20 bytes: Points by Return
        header.extend(struct.pack("<3d", scale_x, scale_y, scale_z)) # 24 bytes: Scale Factors
        header.extend(struct.pack("<3d", offset_x, offset_y, offset_z)) # 24 bytes: Offsets
        header.extend(struct.pack("<6d", max_x, min_x, max_y, min_y, max_z, min_z)) # 48 bytes: Bounds

        # Point Records
        body = bytearray()
        for x, y, z, intensity in points:
            ix = int(round((x - offset_x) / scale_x))
            iy = int(round((y - offset_y) / scale_y))
            iz = int(round((z - offset_z) / scale_z))
            u_intensity = int(min(65535, max(0, intensity * 65535)))

            # Color calculation (elevation rainbow)
            norm_z = (z - min_z) / z_range
            if norm_z < 0.25:
                r, g, b = 0, int(255 * (norm_z / 0.25)), 255
            elif norm_z < 0.5:
                r, g, b = 0, 255, int(255 * (1.0 - (norm_z - 0.25) / 0.25))
            elif norm_z < 0.75:
                r, g, b = int(255 * ((norm_z - 0.5) / 0.25)), 255, 0
            else:
                r, g, b = 255, int(255 * (1.0 - (norm_z - 0.75) / 0.25)), 50

            red_16 = (r << 8) | r
            green_16 = (g << 8) | g
            blue_16 = (b << 8) | b

            pt_bytes = struct.pack(
                "<iiiHBBbBHHHH",
                ix, iy, iz,
                u_intensity,
                0x09,  # Return 1 of 1
                1,     # Unclassified
                0,     # Scan angle 0
                0,     # User data
                1,     # Point source ID
                red_16,
                green_16,
                blue_16,
            )
            body.extend(pt_bytes)

        las_data = bytes(header + body)

        if filename:
            with open(filename, "wb") as f:
                f.write(las_data)

        return las_data


