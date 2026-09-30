"""
sim/mapping.py: 3D LiDAR Perception & Occupancy Voxel Grid Mapping Engine.

Simulates onboard multi-channel rotating LiDAR sensors and OctoMap-style 3D
occupancy voxel grid mapping for autonomous UAV swarm perception and SLAM.
"""

from __future__ import annotations

from collections import deque
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


class LiDARScan:
    """A full spherical / cylindrical sweep of LiDAR points at a given timestamp."""

    def __init__(
        self,
        timestamp: float,
        drone_id: str,
        sensor_origin: np.ndarray,
        points: Optional[List[LiDARPoint]] = None,
        _pts_array: Optional[np.ndarray] = None,
        _obs_ids: Optional[np.ndarray] = None,
        _dists: Optional[np.ndarray] = None,
    ) -> None:
        self.timestamp = float(timestamp)
        self.drone_id = str(drone_id)
        self.sensor_origin = np.asarray(sensor_origin, dtype=np.float64)
        self._points = points
        self._pts_array = _pts_array
        self._obs_ids = _obs_ids
        self._dists = _dists

        if self._points is not None and self._pts_array is None:
            if self._points:
                self._pts_array = np.array(
                    [[p.x, p.y, p.z, p.intensity] for p in self._points],
                    dtype=np.float64,
                )
                self._obs_ids = np.array([p.obstacle_id or "GROUND" for p in self._points], dtype=object)
                self._dists = np.array([p.range_m for p in self._points], dtype=np.float64)
            else:
                self._pts_array = np.zeros((0, 4), dtype=np.float64)
                self._obs_ids = np.zeros(0, dtype=object)
                self._dists = np.zeros(0, dtype=np.float64)

    @property
    def points(self) -> List[LiDARPoint]:
        """Lazy materialization of LiDARPoint objects for backward compatibility."""
        if self._points is None:
            if self._pts_array is not None and len(self._pts_array) > 0:
                pts = self._pts_array
                dists = self._dists if self._dists is not None else np.linalg.norm(pts[:, :3] - self.sensor_origin, axis=1)
                obs_ids = self._obs_ids if self._obs_ids is not None else ["GROUND"] * len(pts)
                self._points = [
                    LiDARPoint(
                        x=float(pts[i, 0]),
                        y=float(pts[i, 1]),
                        z=float(pts[i, 2]),
                        range_m=float(dists[i]),
                        intensity=float(pts[i, 3]),
                        obstacle_id=str(obs_ids[i]) if obs_ids[i] is not None else None,
                    )
                    for i in range(len(pts))
                ]
            else:
                self._points = []
        return self._points

    @points.setter
    def points(self, val: List[LiDARPoint]) -> None:
        self._points = val
        if val:
            self._pts_array = np.array([[p.x, p.y, p.z, p.intensity] for p in val], dtype=np.float64)
            self._obs_ids = np.array([p.obstacle_id or "GROUND" for p in val], dtype=object)
            self._dists = np.array([p.range_m for p in val], dtype=np.float64)
        else:
            self._pts_array = np.zeros((0, 4), dtype=np.float64)
            self._obs_ids = np.zeros(0, dtype=object)
            self._dists = np.zeros(0, dtype=np.float64)

    def to_dict(self) -> Dict[str, Any]:
        if self._pts_array is not None:
            pts_data = self._pts_array.tolist()
            num_pts = len(self._pts_array)
        else:
            pts_data = [
                [
                    round(p.x, 2),
                    round(p.y, 2),
                    round(p.z, 2),
                    round(p.intensity, 2)
                ]
                for p in self.points
            ]
            num_pts = len(self.points)
        return {
            "timestamp": round(float(self.timestamp), 3),
            "drone_id": self.drone_id,
            "origin": [round(float(c), 2) for c in self.sensor_origin],
            "num_points": num_pts,
            "points": pts_data,
        }

class ObstacleBVHNode:
    """Bounding Volume Hierarchy (BVH) node for accelerated ray-AABB broadphase queries."""
    __slots__ = ("bbox_min", "bbox_max", "obstacle_indices", "left", "right", "is_leaf")

    def __init__(
        self,
        bbox_min: np.ndarray,
        bbox_max: np.ndarray,
        obstacle_indices: List[int],
        left: Optional[ObstacleBVHNode] = None,
        right: Optional[ObstacleBVHNode] = None,
    ) -> None:
        self.bbox_min = bbox_min
        self.bbox_max = bbox_max
        self.obstacle_indices = obstacle_indices
        self.left = left
        self.right = right
        self.is_leaf = left is None and right is None


class ObstacleBVH:
    """
    Bounding Volume Hierarchy (BVH) for accelerated 3D ray-AABB broadphase queries.
    Hierarchically partitions obstacles along primary spatial variance axes,
    reducing broadphase intersection tests from O(N_rays * N_obs) to O(N_rays * log N_obs).
    """

    def __init__(self, obstacles: Sequence[Any], max_leaf_size: int = 4) -> None:
        self.obstacles = list(obstacles)
        self.max_leaf_size = max_leaf_size
        self._valid_indices: List[int] = []
        self._centers: Dict[int, np.ndarray] = {}
        self._mins: Dict[int, np.ndarray] = {}
        self._maxs: Dict[int, np.ndarray] = {}

        for idx, obs in enumerate(self.obstacles):
            min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
            max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
            if min_p is not None and max_p is not None:
                min_arr = np.asarray(min_p, dtype=np.float64)
                max_arr = np.asarray(max_p, dtype=np.float64)
                self._valid_indices.append(idx)
                self._mins[idx] = min_arr
                self._maxs[idx] = max_arr
                self._centers[idx] = (min_arr + max_arr) * 0.5

        self.root = self._build(self._valid_indices)

    def _build(self, indices: List[int]) -> Optional[ObstacleBVHNode]:
        if not indices:
            return None

        node_min = np.min([self._mins[i] for i in indices], axis=0)
        node_max = np.max([self._maxs[i] for i in indices], axis=0)

        if len(indices) <= self.max_leaf_size:
            return ObstacleBVHNode(node_min, node_max, indices)

        centers = [self._centers[i] for i in indices]
        c_arr = np.asarray(centers)
        c_min = np.min(c_arr, axis=0)
        c_max = np.max(c_arr, axis=0)
        split_axis = int(np.argmax(c_max - c_min))

        sorted_indices = sorted(indices, key=lambda i: self._centers[i][split_axis])
        mid = len(sorted_indices) // 2
        left_indices = sorted_indices[:mid]
        right_indices = sorted_indices[mid:]

        if not left_indices or not right_indices:
            return ObstacleBVHNode(node_min, node_max, indices)

        left_node = self._build(left_indices)
        right_node = self._build(right_indices)

        return ObstacleBVHNode(node_min, node_max, indices, left=left_node, right=right_node)

    def intersect_rays(
        self,
        pos: np.ndarray,
        inv_dirs: np.ndarray,
        closest_dist: np.ndarray,
        hit_obs_idx: np.ndarray,
        is_ground: np.ndarray,
        min_range: float,
    ) -> None:
        """Traverse BVH in vectorized chunks to find closest obstacle hits for all rays."""
        if self.root is None:
            return

        all_ray_indices = np.arange(len(inv_dirs), dtype=np.int32)
        stack = [(self.root, all_ray_indices)]

        while stack:
            node, ray_indices = stack.pop()
            if len(ray_indices) == 0:
                continue

            sub_inv = inv_dirs[ray_indices]
            sub_closest = closest_dist[ray_indices]

            t1 = (node.bbox_min - pos) * sub_inv
            t2 = (node.bbox_max - pos) * sub_inv
            t_min = np.maximum(
                np.maximum(np.minimum(t1[:, 0], t2[:, 0]), np.minimum(t1[:, 1], t2[:, 1])),
                np.minimum(t1[:, 2], t2[:, 2]),
            )
            t_max = np.minimum(
                np.minimum(np.maximum(t1[:, 0], t2[:, 0]), np.maximum(t1[:, 1], t2[:, 1])),
                np.maximum(t1[:, 2], t2[:, 2]),
            )

            hits = (t_max >= np.maximum(0.0, t_min)) & (t_min < sub_closest) & (t_max >= min_range)
            if not np.any(hits):
                continue

            active_ray_indices = ray_indices[hits]
            if node.is_leaf:
                leaf_inv = inv_dirs[active_ray_indices]
                for obs_idx in node.obstacle_indices:
                    obs = self.obstacles[obs_idx]
                    min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
                    max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
                    if min_p is None or max_p is None:
                        continue
                    ot1 = (min_p - pos) * leaf_inv
                    ot2 = (max_p - pos) * leaf_inv
                    ot_min = np.maximum(
                        np.maximum(np.minimum(ot1[:, 0], ot2[:, 0]), np.minimum(ot1[:, 1], ot2[:, 1])),
                        np.minimum(ot1[:, 2], ot2[:, 2]),
                    )
                    ot_max = np.minimum(
                        np.minimum(np.maximum(ot1[:, 0], ot2[:, 0]), np.maximum(ot1[:, 1], ot2[:, 1])),
                        np.maximum(ot1[:, 2], ot2[:, 2]),
                    )
                    cur_dists = closest_dist[active_ray_indices]
                    o_hits = (ot_max >= np.maximum(0.0, ot_min)) & (ot_min < cur_dists) & (ot_min >= min_range)
                    if np.any(o_hits):
                        hit_rays = active_ray_indices[o_hits]
                        closest_dist[hit_rays] = ot_min[o_hits]
                        hit_obs_idx[hit_rays] = obs_idx
                        is_ground[hit_rays] = False
            else:
                if node.right is not None:
                    stack.append((node.right, active_ray_indices))
                if node.left is not None:
                    stack.append((node.left, active_ray_indices))


class ObstacleSpatialHash:
    """
    2D/3D Spatial Hash Grid for fast ray-AABB broadphase queries.
    Divides operational space into discrete cells to quickly prune distant obstacles.
    """
    def __init__(self, obstacles: Sequence[Any], cell_size: float = 40.0) -> None:
        self.obstacles = list(obstacles)
        self.cell_size = float(cell_size)
        self.grid: Dict[Tuple[int, int], List[int]] = {}
        for idx, obs in enumerate(self.obstacles):
            min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
            max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
            if min_p is None or max_p is None:
                continue
            ix0 = int(math.floor(min_p[0] / cell_size))
            iy0 = int(math.floor(min_p[1] / cell_size))
            ix1 = int(math.floor(max_p[0] / cell_size))
            iy1 = int(math.floor(max_p[1] / cell_size))
            for ix in range(ix0, ix1 + 1):
                for iy in range(iy0, iy1 + 1):
                    self.grid.setdefault((ix, iy), []).append(idx)

    def get_candidate_indices(self, origin: np.ndarray, max_range: float) -> Set[int]:
        candidates: Set[int] = set()
        r = max_range
        ix0 = int(math.floor((origin[0] - r) / self.cell_size))
        ix1 = int(math.floor((origin[0] + r) / self.cell_size))
        iy0 = int(math.floor((origin[1] - r) / self.cell_size))
        iy1 = int(math.floor((origin[1] + r) / self.cell_size))
        for ix in range(ix0, ix1 + 1):
            for iy in range(iy0, iy1 + 1):
                for idx in self.grid.get((ix, iy), []):
                    candidates.add(idx)
        return candidates


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
        horizontal_resolution_deg: float = 4.0,  # 90 azimuth beams per ring
        vertical_fov_deg: Tuple[float, float] = (-50.0, 15.0),  # -50 deg downward ground look to +15 deg upward
        vertical_channels: int = 24,  # 24 elevation rings (2,160 rays total)
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
        """Precomputes unit ray direction vectors in sensor frame [N_rays, 3] using vectorized broadcasting."""
        num_azimuth = int(round(self.horizontal_fov_deg / self.horizontal_resolution_deg))
        azimuths = np.linspace(-np.pi, np.pi, num_azimuth, endpoint=False, dtype=np.float64)
        elevations = np.linspace(
            np.radians(self.vertical_fov_min_deg),
            np.radians(self.vertical_fov_max_deg),
            self.vertical_channels,
            dtype=np.float64,
        )

        el_grid, az_grid = np.meshgrid(elevations, azimuths, indexing="ij")
        cos_el = np.cos(el_grid)
        dx = cos_el * np.cos(az_grid)
        dy = cos_el * np.sin(az_grid)
        dz = np.sin(el_grid)

        return np.column_stack([dx.ravel(), dy.ravel(), dz.ravel()])

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
        using fast vectorized raycasting with deferred surface normal calculations.
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

        # Broadphase filter: utilize persistent BVH acceleration over obstacle fleet
        active_obstacles: List[ObstacleAABB] = list(obstacles)

        closest_dist = np.full(N_rays, self.max_range, dtype=np.float64)
        hit_obs_idx = np.full(N_rays, -1, dtype=np.int32)
        is_ground = np.zeros(N_rays, dtype=bool)

        # 1. Ground plane intersection (z = 0)
        downward_mask = world_ray_dirs[:, 2] < -1e-5
        if np.any(downward_mask):
            t_ground = -pos[2] / world_ray_dirs[downward_mask, 2]
            valid_g = downward_mask.copy()
            valid_g[downward_mask] = (t_ground >= self.min_range) & (t_ground < closest_dist[downward_mask])
            closest_dist[valid_g] = t_ground[valid_g[downward_mask]]
            is_ground[valid_g] = True

        # 2. Obstacles intersection (Accelerated BVH broadphase & Ray-AABB slab test)
        if active_obstacles:
            safe_dirs = np.where(np.abs(world_ray_dirs) > 1e-7, world_ray_dirs, 1e-7)
            inv_dirs = 1.0 / safe_dirs
            if len(active_obstacles) > 6:
                obs_key = (id(obstacles), len(obstacles))
                if (
                    getattr(self, "_cached_bvh", None) is None
                    or getattr(self, "_cached_bvh_key", None) != obs_key
                ):
                    self._cached_bvh = ObstacleBVH(active_obstacles, max_leaf_size=4)
                    self._cached_bvh_key = obs_key
                self._cached_bvh.intersect_rays(pos, inv_dirs, closest_dist, hit_obs_idx, is_ground, self.min_range)
            else:
                for idx, obs in enumerate(active_obstacles):
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
                        hit_obs_idx[hits] = idx
                        is_ground[hits] = False

        # 3. Assemble point cloud returns with deferred normal computation
        valid_mask = (closest_dist < self.max_range) & (is_ground | (hit_obs_idx >= 0))
        valid_indices = np.where(valid_mask)[0]
        if len(valid_indices) > 0:
            dists = closest_dist[valid_indices]
            dirs = world_ray_dirs[valid_indices]
            obs_indices = hit_obs_idx[valid_indices]
            ground_hits = is_ground[valid_indices]

            normals = np.zeros((len(valid_indices), 3), dtype=np.float64)
            normals[ground_hits, 2] = 1.0

            # Analytical surface normal calculation: exact entry face from ray-AABB intersection
            obs_mask = (obs_indices >= 0)
            if np.any(obs_mask):
                obs_inv = inv_dirs[valid_indices[obs_mask]]
                obs_dirs = dirs[obs_mask]
                for u_idx in np.unique(obs_indices[obs_mask]):
                    u_obs = active_obstacles[u_idx]
                    min_p = getattr(u_obs, "min_pt", getattr(u_obs, "min_bound", None))
                    max_p = getattr(u_obs, "max_pt", getattr(u_obs, "max_bound", None))
                    sub_sel = (obs_indices[obs_mask] == u_idx)
                    t1 = (min_p - pos) * obs_inv[sub_sel]
                    t2 = (max_p - pos) * obs_inv[sub_sel]
                    t_near_sub = np.minimum(t1, t2)
                    axis_sub = np.argmax(t_near_sub, axis=1)
                    sub_normals = np.zeros_like(t_near_sub)
                    sub_normals[np.arange(len(axis_sub)), axis_sub] = -np.sign(obs_dirs[sub_sel, axis_sub])
                    normals[np.where(obs_mask)[0][sub_sel]] = sub_normals

            # Add range measurement noise
            if self.range_noise_std > 0:
                noise = np.random.normal(0.0, self.range_noise_std, size=len(dists))
                dists = np.maximum(self.min_range, dists + noise)

            hit_positions = pos + dirs * dists[:, np.newaxis]

            # Reflection intensity based on Lambertian cosine of incident angle and range decay
            cos_incidence = np.maximum(0.1, np.abs(np.sum(-dirs * normals, axis=1)))
            range_factor = np.maximum(0.2, 1.0 - (dists / self.max_range) * 0.5)
            intensities = np.minimum(1.0, cos_incidence * range_factor)

            if len(active_obstacles) > 0:
                active_obs_ids = np.array([str(getattr(obs, "id", "OBS")) for obs in active_obstacles], dtype=object)
                obs_id_arr = np.where(ground_hits, "GROUND", active_obs_ids[np.maximum(0, obs_indices)])
            else:
                obs_id_arr = np.full(len(valid_indices), "GROUND", dtype=object)

            scan._pts_array = np.column_stack([np.round(hit_positions, 2), np.round(intensities, 2)])
            scan._obs_ids = obs_id_arr
            scan._dists = dists

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
        self._occupied_keys: Set[Tuple[int, int, int]] = set()
        self._free_keys: Set[Tuple[int, int, int]] = set()
        self._previously_occupied_keys: Set[Tuple[int, int, int]] = set()
        self._modified_keys: Set[Tuple[int, int, int]] = set()
        self.total_surveyed_points: int = 0
        self.accumulated_hits: deque = deque(maxlen=100000)
        # Multi-UAV collaborative point storage & spatial mapping
        self.drone_points: Dict[str, deque] = {}
        self.spatial_point_map: Dict[Tuple[int, int, int], Tuple[float, float, float, float, str]] = {}

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

    def insert_scan(self, scan: LiDARScan, max_traversal_steps: int = 4) -> None:
        """
        Integrates an individual UAV LiDAR scan into the 3D occupancy map and collaborative point cloud.
        Records 3D hits from this drone into the fused spatial structure map.
        Free space voxels along rays are cleared; hit endpoints are incremented using vectorized NumPy operations.
        """
        origin = scan.sensor_origin
        if not self.is_in_bounds(origin):
            return

        pts_arr = scan._pts_array
        if pts_arr is None or len(pts_arr) == 0:
            if scan._points and len(scan._points) > 0:
                pts_arr = np.array([[p.x, p.y, p.z, p.intensity] for p in scan._points], dtype=np.float64)
            else:
                return

        inv = self.inv_voxel_size
        vsz = self.voxel_size
        bx, by, bz = self.bounds_x, self.bounds_y, self.bounds_z

        # Vectorized bounds filtering
        in_bounds_mask = (
            (pts_arr[:, 0] >= bx[0]) & (pts_arr[:, 0] <= bx[1]) &
            (pts_arr[:, 1] >= by[0]) & (pts_arr[:, 1] <= by[1]) &
            (pts_arr[:, 2] >= bz[0]) & (pts_arr[:, 2] <= bz[1])
        )
        if not np.any(in_bounds_mask):
            return

        valid_pts = pts_arr[in_bounds_mask]
        self.total_surveyed_points += len(valid_pts)

        drone_id = scan.drone_id or "UNKNOWN"
        if drone_id not in self.drone_points:
            self.drone_points[drone_id] = deque(maxlen=25000)
        d_deque = self.drone_points[drone_id]

        # 1 & 2. Spatial point cloud map & point history tracking
        sp_keys = np.round(valid_pts[:, :3] * 1.25).astype(np.int32)
        _, unique_sp_idx = np.unique(sp_keys, axis=0, return_index=True)
        for idx in unique_sp_idx:
            k = (int(sp_keys[idx, 0]), int(sp_keys[idx, 1]), int(sp_keys[idx, 2]))
            self.spatial_point_map[k] = (float(valid_pts[idx, 0]), float(valid_pts[idx, 1]), float(valid_pts[idx, 2]), float(valid_pts[idx, 3]), drone_id)

        for i in range(len(valid_pts)):
            pt_tuple = (float(valid_pts[i, 0]), float(valid_pts[i, 1]), float(valid_pts[i, 2]), float(valid_pts[i, 3]))
            self.accumulated_hits.append(pt_tuple)
            d_deque.append(pt_tuple)

        # 3. Update Hit Voxels in 3D Occupancy Grid (vectorized grid keys & unique aggregation)
        voxel_indices = np.floor(valid_pts[:, :3] * inv).astype(np.int32)
        unique_vox_keys, hit_counts = np.unique(voxel_indices, axis=0, return_counts=True)

        for k_idx in range(len(unique_vox_keys)):
            ix, iy, iz = int(unique_vox_keys[k_idx, 0]), int(unique_vox_keys[k_idx, 1]), int(unique_vox_keys[k_idx, 2])
            hit_key = (ix, iy, iz)
            count = int(hit_counts[k_idx])
            voxel = self.voxels.get(hit_key)
            if voxel is None:
                c = np.array([(ix + 0.5) * vsz, (iy + 0.5) * vsz, (iz + 0.5) * vsz], dtype=np.float64)
                voxel = VoxelNode(
                    ix=ix,
                    iy=iy,
                    iz=iz,
                    center=c,
                    log_odds=0.0
                )
                self.voxels[hit_key] = voxel
            voxel.log_odds = min(self.l_max, max(self.l_min, voxel.log_odds + count * self.l_occ))
            self._modified_keys.add(hit_key)
            if voxel.log_odds >= 0.619:
                self._occupied_keys.add(hit_key)
                self._free_keys.discard(hit_key)
            elif voxel.log_odds <= -0.619:
                self._free_keys.add(hit_key)
                self._occupied_keys.discard(hit_key)
            else:
                self._occupied_keys.discard(hit_key)
                self._free_keys.discard(hit_key)

        # 4. Fast Free-Space Ray-Marching (sampled along rays for rapid clearance)
        # Sample every 8th ray from valid_pts to clear air corridors with zero CPU stutter
        if len(valid_pts) > 0:
            hit_world_pts = valid_pts[::8, :3]
            if len(hit_world_pts) > 0:
                ray_vecs = hit_world_pts - origin
                dists = np.linalg.norm(ray_vecs, axis=1)
                step_size = vsz * 2.5
                valid_ray_mask = dists > vsz
                if np.any(valid_ray_mask):
                    active_vecs = ray_vecs[valid_ray_mask]
                    active_dists = dists[valid_ray_mask]

                    free_keys_set = set()
                    for idx in range(len(active_vecs)):
                        d = float(active_dists[idx])
                        num_steps = min(max_traversal_steps, int(d / step_size))
                        r_vec = active_vecs[idx]
                        inv_d = 1.0 / d
                        for s in range(1, num_steps):
                            sample_pt = origin + r_vec * (s * step_size * inv_d)
                            free_keys_set.add((
                                int(math.floor(sample_pt[0] * inv)),
                                int(math.floor(sample_pt[1] * inv)),
                                int(math.floor(sample_pt[2] * inv))
                            ))

                    for free_key in free_keys_set:
                        free_vox = self.voxels.get(free_key)
                        if free_vox is None:
                            c = np.array([(free_key[0] + 0.5) * vsz, (free_key[1] + 0.5) * vsz, (free_key[2] + 0.5) * vsz], dtype=np.float64)
                            free_vox = VoxelNode(
                                ix=free_key[0],
                                iy=free_key[1],
                                iz=free_key[2],
                                center=c,
                                log_odds=0.0
                            )
                            self.voxels[free_key] = free_vox
                        free_vox.log_odds = min(self.l_max, max(self.l_min, free_vox.log_odds + self.l_free))
                        self._modified_keys.add(free_key)
                        if free_vox.log_odds <= -0.619:
                            self._free_keys.add(free_key)
                            self._occupied_keys.discard(free_key)
                        elif free_vox.log_odds >= 0.619:
                            self._occupied_keys.add(free_key)
                            self._free_keys.discard(free_key)
                        else:
                            self._occupied_keys.discard(free_key)
                            self._free_keys.discard(free_key)

    def get_occupied_voxels(self, max_count: int = 1500) -> List[Dict[str, Any]]:
        """
        Returns list of occupied voxels formatted for 3D Three.js client visualization.
        O(occupied) iteration time via active occupied key index set.
        """
        occupied = []
        stale_keys = []
        for key in list(self._occupied_keys):
            v = self.voxels.get(key)
            if v and v.log_odds >= 0.619:
                occupied.append({
                    "pos": [round(float(c), 2) for c in v.center],
                    "prob": round(float(v.occupancy_prob), 2),
                    "size": self.voxel_size,
                    "key": key,
                })
                if len(occupied) >= max_count:
                    break
            else:
                stale_keys.append(key)
        for k in stale_keys:
            self._occupied_keys.discard(k)
        return occupied

    def get_voxel_deltas(self, max_count: int = 1500) -> Dict[str, Any]:
        """
        Returns incremental / delta updates (added & removed occupied voxels)
        since the last delta query to minimize WebSocket network payload.
        """
        added = []
        removed = []
        cur_occupied = self._occupied_keys

        # Added: keys now occupied that were not occupied previously
        added_keys = cur_occupied - self._previously_occupied_keys
        # Removed: keys that were occupied previously and are no longer occupied
        removed_keys = self._previously_occupied_keys - cur_occupied

        for key in added_keys:
            v = self.voxels.get(key)
            if v and v.log_odds >= 0.619:
                added.append({
                    "pos": [round(float(c), 2) for c in v.center],
                    "prob": round(float(v.occupancy_prob), 2),
                    "size": self.voxel_size,
                    "key": list(key),
                })
        for key in removed_keys:
            removed.append(list(key))

        self._previously_occupied_keys = set(cur_occupied)
        self._modified_keys.clear()
        return {
            "full": False,
            "added": added[:max_count],
            "removed": removed[:max_count],
            "total_occupied": len(self._occupied_keys),
        }

    def compute_metrics(self) -> Dict[str, float]:
        """Calculates quantitative SLAM mapping metrics for analytics and reporting in O(1) time."""
        total_cells = len(self.voxels)
        occupied_count = len(self._occupied_keys)
        free_count = len(self._free_keys)

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

    def export_point_cloud_ply(
        self,
        filename: Optional[str] = None,
        drones: Optional[Dict[str, Any]] = None,
    ) -> str:
        """
        Exports the 3D LiDAR SLAM Point Cloud in Stanford ASCII PLY format.
        Universally compatible with CloudCompare, Blender (Stanford PLY Importer),
        MeshLab, and Open3D.
        Points are colored by altitude (Z) with reflection intensity values.

        Combines point cloud data from all individual drones to 3D map the city
        and structures.
        """
        points: List[Tuple[float, float, float, float]] = []
        contributing_drones: List[str] = []

        # 1. Combine points directly from individual drones if provided
        if drones:
            for d_id, drone in drones.items():
                d_pts = getattr(drone, "scanned_points", [])
                if d_pts:
                    contributing_drones.append(f"{d_id} ({len(d_pts)} pts)")
                    points.extend(d_pts)

        # 2. If drones not provided or empty, combine from spatial map or accumulated hits
        if not points:
            if hasattr(self, "spatial_point_map") and self.spatial_point_map:
                points = [(p[0], p[1], p[2], p[3]) for p in self.spatial_point_map.values()]
                contributing_drones = [f"{d_id} ({len(pts)} pts)" for d_id, pts in self.drone_points.items() if pts]
            elif hasattr(self, "accumulated_hits") and self.accumulated_hits:
                points = list(self.accumulated_hits)[-65000:]
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

        # Cap points to 100,000 for fast export while preserving complete city coverage
        if len(points) > 100000:
            step = max(1, len(points) // 100000)
            points = points[::step][:100000]

        lines = [
            "ply",
            "format ascii 1.0",
            "comment UAV Swarm Autonomous SLAM LiDAR Point Cloud",
            "comment Net 3D City & Structural Map Combined from All Individual Drone Sensors",
        ]
        if contributing_drones:
            lines.append(f"comment Contributing UAVs: {', '.join(contributing_drones[:16])}")
        lines.extend([
            f"element vertex {len(points)}",
            "property float x",
            "property float y",
            "property float z",
            "property uchar red",
            "property uchar green",
            "property uchar blue",
            "property float intensity",
            "end_header",
        ])

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

    def export_point_cloud_las(
        self,
        filename: Optional[str] = None,
        drones: Optional[Dict[str, Any]] = None,
    ) -> bytes:
        """
        Exports the 3D LiDAR SLAM Point Cloud in ASPRS LAS 1.2 Binary format.
        Compatible with CloudCompare, PDAL, QGIS, ArcGIS, and civil survey packages.
        Point Data Format 2 (includes RGB color and reflection intensity).
        """
        import struct

        points: List[Tuple[float, float, float, float]] = []
        if drones:
            for d_id, drone in drones.items():
                d_pts = getattr(drone, "scanned_points", [])
                if d_pts:
                    points.extend(d_pts)

        if not points:
            if hasattr(self, "spatial_point_map") and self.spatial_point_map:
                points = [(p[0], p[1], p[2], p[3]) for p in self.spatial_point_map.values()]
            elif hasattr(self, "accumulated_hits") and self.accumulated_hits:
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

        if len(points) > 100000:
            step = max(1, len(points) // 100000)
            points = points[::step][:100000]

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


class BayesianThermalGrid:
    """
    2D Bayesian Occupancy & Thermal Belief Density Grid.
    Discretizes the post-disaster operational theater into spatial belief cells.
    Updates probability densities P(Survivor | FLIR) using recursive Bayesian estimation.
    """

    def __init__(
        self,
        bounds_x: Tuple[float, float] = (-350.0, 350.0),
        bounds_y: Tuple[float, float] = (-350.0, 350.0),
        cell_size: float = 20.0,
        prior_p: float = 0.03,
    ) -> None:
        self.bounds_x = bounds_x
        self.bounds_y = bounds_y
        self.cell_size = cell_size
        self.prior_p = prior_p

        self.nx = int(math.ceil((bounds_x[1] - bounds_x[0]) / cell_size))
        self.ny = int(math.ceil((bounds_y[1] - bounds_y[0]) / cell_size))

        # Probability grid: P(Survivor in cell)
        self.grid = np.full((self.nx, self.ny), self.prior_p, dtype=np.float64)
        self.visited = np.zeros((self.nx, self.ny), dtype=bool)

        # Precompute cell centers for fast vectorization
        xs = self.bounds_x[0] + (np.arange(self.nx) + 0.5) * self.cell_size
        ys = self.bounds_y[0] + (np.arange(self.ny) + 0.5) * self.cell_size
        self.cell_xs, self.cell_ys = np.meshgrid(xs, ys, indexing="ij")

        # Caching & Dirty Flag Optimization for ultra-fast serialization
        self._dirty: bool = True
        self._cached_dict: Optional[Dict[str, Any]] = None

    def seed_priors(self, poi_positions: Sequence[Sequence[float]], elevated_p: float = 0.25) -> None:
        """Seed higher prior probabilities in the vicinity of designated disaster sites."""
        self._dirty = True
        if not poi_positions:
            return
        poi_arr = np.asarray(poi_positions, dtype=np.float64)
        dx = self.cell_xs[None, :, :] - poi_arr[:, 0, None, None]
        dy = self.cell_ys[None, :, :] - poi_arr[:, 1, None, None]
        mask = np.any((dx * dx + dy * dy) <= (45.0 ** 2), axis=0)
        self.grid[mask] = np.maximum(self.grid[mask], elevated_p)

    def update_flir_scan(
        self,
        drone_pos: Sequence[float],
        survivor_positions: Sequence[Sequence[float]],
        fov_radius: float = 55.0,
    ) -> None:
        """
        Recursive Bayesian update based on an active FLIR sensor sweep.
        P(S | D) = P(D | S) * P(S) / [P(D | S)*P(S) + P(D | ~S)*(1 - P(S))]
        """
        dx, dy = float(drone_pos[0]), float(drone_pos[1])
        # Fast bounding box culling: drone FOV only interacts with a localized slice
        ix_min = max(0, int(math.floor((dx - fov_radius - self.bounds_x[0]) / self.cell_size)))
        ix_max = min(self.nx, int(math.ceil((dx + fov_radius - self.bounds_x[0]) / self.cell_size)) + 1)
        iy_min = max(0, int(math.floor((dy - fov_radius - self.bounds_y[0]) / self.cell_size)))
        iy_max = min(self.ny, int(math.ceil((dy + fov_radius - self.bounds_y[0]) / self.cell_size)) + 1)

        if ix_min >= ix_max or iy_min >= iy_max:
            return

        sub_xs = self.cell_xs[ix_min:ix_max, iy_min:iy_max]
        sub_ys = self.cell_ys[ix_min:ix_max, iy_min:iy_max]
        sub_dists_sq = (sub_xs - dx) ** 2 + (sub_ys - dy) ** 2
        in_fov = sub_dists_sq <= (fov_radius ** 2)

        if not np.any(in_fov):
            return

        self._dirty = True
        self.visited[ix_min:ix_max, iy_min:iy_max][in_fov] = True

        # Check which cells within FOV contain a survivor signature
        surv_near_cell = np.zeros(sub_xs.shape, dtype=bool)
        if survivor_positions:
            s_arr = np.asarray(survivor_positions, dtype=np.float64)
            s_mask = (
                (s_arr[:, 0] >= dx - fov_radius - self.cell_size) &
                (s_arr[:, 0] <= dx + fov_radius + self.cell_size) &
                (s_arr[:, 1] >= dy - fov_radius - self.cell_size) &
                (s_arr[:, 1] <= dy + fov_radius + self.cell_size)
            )
            if np.any(s_mask):
                active_s = s_arr[s_mask]
                sdx = sub_xs[None, :, :] - active_s[:, 0, None, None]
                sdy = sub_ys[None, :, :] - active_s[:, 1, None, None]
                surv_near_cell = np.any((sdx * sdx + sdy * sdy) <= ((self.cell_size * 0.85) ** 2), axis=0)

        # Cells with a detected heat signature:
        # P(D|S) = 0.92, P(D|~S) = 0.05
        p_d_given_s = 0.92
        p_d_given_not_s = 0.05

        detect_mask = in_fov & surv_near_cell
        sub_grid = self.grid[ix_min:ix_max, iy_min:iy_max]
        if np.any(detect_mask):
            p_prior = sub_grid[detect_mask]
            p_evidence = p_d_given_s * p_prior + p_d_given_not_s * (1.0 - p_prior)
            sub_grid[detect_mask] = np.clip((p_d_given_s * p_prior) / np.maximum(p_evidence, 1e-6), 0.001, 0.999)

        # Cells within FOV with NO heat signature:
        # P(~D|S) = 0.08, P(~D|~S) = 0.95
        p_not_d_given_s = 0.08
        p_not_d_given_not_s = 0.95

        no_detect_mask = in_fov & (~surv_near_cell)
        if np.any(no_detect_mask):
            p_prior = sub_grid[no_detect_mask]
            p_evidence = p_not_d_given_s * p_prior + p_not_d_given_not_s * (1.0 - p_prior)
            sub_grid[no_detect_mask] = np.clip((p_not_d_given_s * p_prior) / np.maximum(p_evidence, 1e-6), 0.001, 0.999)

    def get_highest_entropy_target(self, current_pos: Sequence[float]) -> Optional[np.ndarray]:
        """
        Locates the highest-priority / high-entropy thermal hotspot cell that
        warrants further surveyor inspection.
        """
        # Focus on cells with probability >= 0.15 that haven't reached definitive confidence (p >= 0.85)
        candidate_mask = (self.grid >= 0.15) & (self.grid < 0.85)
        if not np.any(candidate_mask):
            # Fall back to any unvisited elevated prior cell
            candidate_mask = (~self.visited) & (self.grid >= 0.08)

        if not np.any(candidate_mask):
            return None

        cand_xs = self.cell_xs[candidate_mask]
        cand_ys = self.cell_ys[candidate_mask]
        cand_probs = self.grid[candidate_mask]

        cur_x, cur_y = float(current_pos[0]), float(current_pos[1])
        dists = np.hypot(cand_xs - cur_x, cand_ys - cur_y)
        scores = cand_probs / (1.0 + 0.01 * dists)

        best_idx = int(np.argmax(scores))
        return np.array([float(cand_xs[best_idx]), float(cand_ys[best_idx]), 28.0], dtype=np.float64)

    def to_dict(self) -> Dict[str, Any]:
        """Serializes thermal heatmap summary and high-density cells for HUD streaming."""
        if not self._dirty and self._cached_dict is not None:
            return self._cached_dict

        hotspots = []
        hot_mask = self.grid >= 0.12
        if np.any(hot_mask):
            h_xs = self.cell_xs[hot_mask]
            h_ys = self.cell_ys[hot_mask]
            h_ps = self.grid[hot_mask]
            if len(h_ps) > 150:
                top_idx = np.argsort(h_ps)[-150:]
                h_xs = h_xs[top_idx]
                h_ys = h_ys[top_idx]
                h_ps = h_ps[top_idx]
            for x, y, p in zip(h_xs, h_ys, h_ps):
                hotspots.append({
                    "x": round(float(x), 1),
                    "y": round(float(y), 1),
                    "prob": round(float(p), 3),
                })

        total_cells = self.nx * self.ny
        visited_cells = int(np.sum(self.visited))
        self._cached_dict = {
            "cell_size": self.cell_size,
            "bounds": [self.bounds_x[0], self.bounds_x[1], self.bounds_y[0], self.bounds_y[1]],
            "nx": self.nx,
            "ny": self.ny,
            "total_cells": total_cells,
            "visited_cells": visited_cells,
            "coverage_pct": round(float(100.0 * visited_cells / max(1, total_cells)), 1),
            "max_prob": round(float(np.max(self.grid)), 3),
            "avg_prob": round(float(np.mean(self.grid)), 3),
            "hotspots": hotspots[:150],
        }
        self._dirty = False
        return self._cached_dict



