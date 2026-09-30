"""
sim/core.py: Master Swarm Simulation Core Engine.

Orchestrates multi-agent 6-DOF kinematics, deterministic discrete-time physical updates,
obstacle collision avoidance, Virtual Spring Mesh fleet relay positioning,
and compact telemetry snapshot serialization for visualizers and test runners.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import json
import math
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np

from sim.drone import Drone
from sim.environment import DisasterEnvironment
from sim.types import (
    DroneRole,
    FlightMode,
    TelemetrySnapshot,
)
from sim.weather import DrydenTurbulenceModel, WindConfig
from sim.swarm_sim import DroneSpatialGrid


@dataclass
class SimulationConfig:
    """Master simulation configuration parameters."""
    dt: float = 0.05                             # Step time increment (seconds) [20 Hz]
    max_duration: float = 300.0                  # Max mission duration (seconds)
    world_bounds_x: Tuple[float, float] = (-250.0, 250.0)
    world_bounds_y: Tuple[float, float] = (-250.0, 250.0)
    world_bounds_z: Tuple[float, float] = (0.0, 120.0)
    gcs_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    gcs_comm_radius: float = 80.0                # Direct GCS LoS RF range (m)
    enable_downwash: bool = True                 # Enable propeller downwash hazard
    enable_vsm_relays: bool = True               # Enable Virtual Spring Mesh relay positioning
    enable_weather: bool = False                 # Enable Dryden atmospheric wind & turbulence
    wind_config: Optional[WindConfig] = None     # Custom wind/turbulence parameters
    history_buffer_len: int = 600                # Number of telemetry frames to buffer (30s @ 20Hz)
    enable_spatial_grid: bool = True             # Enable accelerated 3D spatial grid for large swarms


class SwarmSimulationCore:
    """
    Master simulation loop coordinator.

    Manages fleet of UAV drones, disaster environment, obstacles,
    physics integration, collision avoidance forces, and telemetry generation.
    """

    def __init__(
        self,
        config: Optional[SimulationConfig] = None,
        environment: Optional[DisasterEnvironment] = None,
    ) -> None:
        """Initialize simulation core with optional config and environment."""
        self.config = config if config is not None else SimulationConfig()
        if environment is not None:
            self.environment = environment
        else:
            self.environment = DisasterEnvironment(
                bounds_x=self.config.world_bounds_x,
                bounds_y=self.config.world_bounds_y,
                bounds_z=self.config.world_bounds_z,
                gcs_position=self.config.gcs_position,
            )

        self.drones: Dict[str, Drone] = {}
        self.obstacles: List[Any] = []
        self.pois: Dict[str, Dict[str, Any]] = {}
        self.weather = DrydenTurbulenceModel(config=self.config.wind_config)

        # Pluggable subsystem references
        self.network_engine: Optional[Any] = None
        self.mission_manager: Optional[Any] = None

        # Simulation clocks and stats
        self.sim_time: float = 0.0
        self.step_count: int = 0
        self.history: deque = deque(maxlen=self.config.history_buffer_len)

        # Swarm tactical formation state
        self.active_formation: str = "AUTONOMOUS"

        # Event logging
        self.collision_events: List[Dict[str, Any]] = []
        self.flight_events: List[Dict[str, Any]] = []

        # 3D Spatial Grid for accelerated proximity / collision avoidance
        self.spatial_grid: DroneSpatialGrid = DroneSpatialGrid(cell_size=25.0)
        self.enable_spatial_grid: bool = getattr(self.config, "enable_spatial_grid", False)

        # Pre-cached vectorized obstacle bounding box arrays
        self._cached_obs_min_pts: Optional[np.ndarray] = None
        self._cached_obs_max_pts: Optional[np.ndarray] = None
        self._cached_obs_count: int = -1

    def _update_cached_obstacles(self) -> None:
        """Synchronize pre-cached NumPy obstacle bounding arrays."""
        if self._cached_obs_count != len(self.obstacles):
            om_list = [getattr(o, "min_pt", getattr(o, "min_bound", None)) for o in self.obstacles]
            ox_list = [getattr(o, "max_pt", getattr(o, "max_bound", None)) for o in self.obstacles]
            if om_list and om_list[0] is not None and ox_list[0] is not None:
                self._cached_obs_min_pts = np.vstack(om_list)
                self._cached_obs_max_pts = np.vstack(ox_list)
            else:
                self._cached_obs_min_pts = None
                self._cached_obs_max_pts = None
            self._cached_obs_count = len(self.obstacles)

    # -------------------------------------------------------------------------
    # Entity Registration
    # -------------------------------------------------------------------------

    def add_drone(self, drone: Drone) -> None:
        """Register a UAV drone instance into the simulation fleet."""
        if drone.id in self.drones:
            raise ValueError(f"Drone with ID '{drone.id}' already registered.")
        self.drones[drone.id] = drone
        drone.obstacles = self.obstacles

    def remove_drone(self, drone_id: str) -> Optional[Drone]:
        """Remove a drone by ID from the fleet."""
        return self.drones.pop(drone_id, None)

    def add_obstacle(self, obstacle: Any) -> None:
        """Add a static 3D obstacle to the simulation."""
        self.obstacles.append(obstacle)
        self.environment.add_obstacle(obstacle)
        if self.mission_manager is not None:
            setattr(self.mission_manager, "obstacles", self.obstacles)
            if hasattr(self.mission_manager, "sector_manager") and self.mission_manager.sector_manager is not None:
                setattr(self.mission_manager.sector_manager, "obstacles", self.obstacles)
        for d in self.drones.values():
            if not hasattr(d, "obstacles") or d.obstacles is None:
                d.obstacles = self.obstacles
            elif obstacle not in d.obstacles:
                d.obstacles.append(obstacle)

    def add_poi(
        self,
        poi_id: str,
        position: Union[np.ndarray, Sequence[float]],
        priority: str = "HIGH",
        required_dwell_time: float = 15.0,
    ) -> None:
        """Register a disaster survey Point of Interest."""
        self.pois[poi_id] = {
            "id": poi_id,
            "position": np.array(position, dtype=np.float64),
            "priority": str(priority),
            "required_dwell_time": float(required_dwell_time),
            "current_dwell_time": 0.0,
            "is_completed": False,
            "assigned_drone_id": None,
            "is_detected": False,
            "is_reported": False,
            "is_spawned": True,
            "detection_time": None,
            "detected_by": None,
        }
        if hasattr(self, "mission_manager") and self.mission_manager is not None:
            if hasattr(self.mission_manager, "sector_manager") and self.mission_manager.sector_manager is not None:
                self.mission_manager.sector_manager.register_abnormality(
                    anomaly_id=poi_id,
                    anomaly_type="TARGET_SITE",
                    position=position,
                    severity=str(priority),
                    description=f"Disaster Target Site: {poi_id}",
                    required_dwell=float(required_dwell_time),
                )

    def set_network_engine(self, network_engine: Any) -> None:
        """Attach a FANET communication and routing subsystem (Milestone 2)."""
        self.network_engine = network_engine

    def set_mission_manager(self, mission_manager: Any) -> None:
        """Attach a high-level disaster survey mission manager (Milestone 3)."""
        self.mission_manager = mission_manager
        if self.mission_manager is not None:
            setattr(self.mission_manager, "obstacles", self.obstacles)
            if hasattr(self.mission_manager, "sector_manager") and self.mission_manager.sector_manager is not None:
                setattr(self.mission_manager.sector_manager, "obstacles", self.obstacles)

    def trigger_obstacle_collapse(
        self,
        obstacle_id: Optional[str] = None,
        collapse_ratio: float = 0.45,
    ) -> Optional[Dict[str, Any]]:
        """
        Simulate a mid-mission dynamic structural failure of a disaster building.
        Reduces building height, updates AABB bounds, expands rubble field,
        and spawns an emergency trapped survivor.
        """
        if not self.obstacles:
            return None

        target_obs = None
        if obstacle_id:
            for obs in self.obstacles:
                if getattr(obs, "id", "") == obstacle_id or getattr(obs, "name", "") == obstacle_id:
                    target_obs = obs
                    break

        if target_obs is None:
            # Default to tall tower (e.g. Damaged Tower Beta or high-rise)
            candidates = [obs for obs in self.obstacles if getattr(obs, "max_pt", [0, 0, 0])[2] >= 35.0]
            target_obs = candidates[0] if candidates else self.obstacles[0]

        # Record original bounds
        old_h = float(target_obs.max_pt[2] - target_obs.min_pt[2])
        old_max_z = float(target_obs.max_pt[2])
        new_max_z = round(float(target_obs.min_pt[2] + old_h * (1.0 - collapse_ratio)), 1)

        # Modify AABB geometry in-place so all drones' APF calculations see the update immediately
        target_obs.max_pt[2] = new_max_z
        target_obs.min_pt[0] -= 4.0
        target_obs.max_pt[0] += 4.0
        target_obs.min_pt[1] -= 4.0
        target_obs.max_pt[1] += 4.0
        setattr(target_obs, "is_collapsed", True)
        self._cached_obs_count = -1
        self._update_cached_obstacles()
        if hasattr(self, "network_engine") and hasattr(self.network_engine, "_los_cache"):
            self.network_engine._los_cache.clear()
        try:
            from sim.planning import invalidate_los_cache
            invalidate_los_cache()
        except ImportError:
            pass

        # Notify mission manager if present
        spawned_survivor = None
        if self.mission_manager is not None:
            if hasattr(self.mission_manager, "emit_tactical_comms"):
                self.mission_manager.emit_tactical_comms(
                    "WARNING",
                    "GCS",
                    f"💥 STRUCTURAL FAILURE: {target_obs.name} collapsed ({int(old_max_z)}m -> {int(new_max_z)}m). Rubble expanded. APF re-routing engaged!",
                    "COLLAPSE",
                )
            if hasattr(self.mission_manager, "survivors"):
                from sim.types import SurvivorRecord
                c_x = float(0.5 * (target_obs.min_pt[0] + target_obs.max_pt[0]))
                c_y = float(0.5 * (target_obs.min_pt[1] + target_obs.max_pt[1]))
                s_id = f"SURVIVOR_COLLAPSE_{len(self.mission_manager.survivors)+1:02d}"
                new_surv = SurvivorRecord(
                    id=s_id,
                    poi_id="POI_COLLAPSE",
                    position=[c_x, c_y, 0.5],
                    heat_c=38.4,
                    confidence=0.96,
                    discovered=False,
                )
                self.mission_manager.survivors[s_id] = new_surv
                spawned_survivor = new_surv.to_dict()

            if hasattr(self.mission_manager, "sector_manager") and self.mission_manager.sector_manager is not None:
                c_x = float(0.5 * (target_obs.min_pt[0] + target_obs.max_pt[0]))
                c_y = float(0.5 * (target_obs.min_pt[1] + target_obs.max_pt[1]))
                self.mission_manager.sector_manager.register_abnormality(
                    anomaly_id=f"ANOMALY_COLLAPSE_{target_obs.id}",
                    anomaly_type="STRUCTURAL_COLLAPSE",
                    position=[c_x, c_y, 0.5],
                    severity="CRITICAL",
                    description=f"Structural Failure Rubble: {target_obs.name}",
                    required_dwell=8.0,
                )

        return {
            "obstacle_id": target_obs.id,
            "obstacle_name": target_obs.name,
            "old_height_m": old_h,
            "new_height_m": round(old_h * (1.0 - collapse_ratio), 1),
            "new_max_pt": target_obs.max_pt.tolist(),
            "new_min_pt": target_obs.min_pt.tolist(),
            "spawned_survivor": spawned_survivor,
        }

    # -------------------------------------------------------------------------
    # Physics & Navigation Steering Calculations
    # -------------------------------------------------------------------------

    def compute_steering_forces(self, drone: Drone) -> np.ndarray:
        """
        Compute net composite steering force acting on a drone.
        Superposes APF attractive, Reynolds separation/alignment,
        obstacle repulsion, downwash avoidance, and boundary containment forces.
        """
        pos_i = drone.position
        vel_i = drone.velocity
        m_i = drone.limits.mass_kg

        # 1. Attractive force to target waypoint
        f_att = np.zeros(3, dtype=np.float64)
        target = drone.get_target_waypoint()
        if drone.flight_mode in (FlightMode.LANDING, FlightMode.EMERGENCY_LAND):
            if target is None:
                target = np.array([pos_i[0], pos_i[1], 0.0], dtype=np.float64)
            err_xy = target[:2] - pos_i[:2]
            f_xy = err_xy * 1.5 - vel_i[:2] * 1.2 * m_i
            v_z_des = -min(1.5, max(0.4, 0.4 * pos_i[2]))
            f_z = 8.0 * (v_z_des - vel_i[2]) * m_i
            f_att = np.array([f_xy[0], f_xy[1], f_z], dtype=np.float64)
        elif target is not None:
            err = target - pos_i
            dist = math.sqrt(float(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]))
            k_att = 1.5
            if dist > 15.0:
                f_att = (err / dist) * 15.0 * k_att
            elif dist > 1e-4:
                f_att = err * k_att

        # Prioritized Safety Attenuation: Attenuate attractive force along blocked paths
        r_sep_limit = getattr(drone.limits, "separation_radius", 6.0)
        atten_horizon = max(8.0, r_sep_limit * 1.5)
        atten_barrier = max(2.2, r_sep_limit * 1.1)

        # Determine candidate peer drones for interactions
        if self.enable_spatial_grid and self.spatial_grid is not None and len(self.drones) >= 8:
            if getattr(self.spatial_grid, "total_drones", 0) != len(self.drones):
                self.spatial_grid.build(list(self.drones.values()))
            peer_candidates = self.spatial_grid.query_nearby_peers(drone, search_radius=max(atten_horizon, 40.0))
            candidate_ids = sorted([p.id for p in peer_candidates if getattr(p, "id", None) != drone.id])
        else:
            candidate_ids = [k for k in sorted(self.drones.keys()) if k != drone.id]

        if candidate_ids:
            cached_idx_map = getattr(self, "_cached_drone_idx_map", None)
            cached_positions = getattr(self, "_cached_drone_positions", None)
            cached_velocities = getattr(self, "_cached_drone_velocities", None)
            cached_roles = getattr(self, "_cached_drone_roles", None)

            if cached_idx_map is not None and cached_positions is not None and len(cached_positions) == len(self.drones) and all(cid in cached_idx_map for cid in candidate_ids):
                indices = [cached_idx_map[cid] for cid in candidate_ids]
                cand_positions = cached_positions[indices]
                cand_velocities = cached_velocities[indices]
                cand_roles = [cached_roles[idx] for idx in indices]
            else:
                cand_positions = np.array([self.drones[cid].position for cid in candidate_ids], dtype=np.float64)
                cand_velocities = np.array([self.drones[cid].velocity for cid in candidate_ids], dtype=np.float64)
                cand_roles = [self.drones[cid].role for cid in candidate_ids]

            deltas = pos_i - cand_positions
            dists = np.linalg.norm(deltas, axis=1)

            # Prioritized Safety Attenuation: Attenuate attractive force along blocked paths
            atten_mask = (dists > 0.0) & (dists < atten_horizon)
            if np.any(atten_mask):
                for idx in np.nonzero(atten_mask)[0]:
                    d_peer = dists[idx]
                    delta = deltas[idx]
                    r_hat = delta / d_peer
                    gamma = 0.0 if d_peer <= atten_barrier else ((d_peer - atten_barrier) / (atten_horizon - atten_barrier)) ** 2
                    proj = max(0.0, float(np.dot(f_att, -r_hat)))
                    f_att -= proj * (1.0 - gamma) * (-r_hat)
        else:
            cand_positions = None
            cand_velocities = None
            cand_roles = None
            deltas = None
            dists = None

        self._update_cached_obstacles()

        if self._cached_obs_min_pts is not None:
            closest_pts_all = np.clip(pos_i, self._cached_obs_min_pts, self._cached_obs_max_pts)
            push_all = pos_i - closest_pts_all
            dists_all = np.linalg.norm(push_all, axis=1)

            # Vectorized obstacle attenuation along blocked attractive paths
            near_atten_indices = np.nonzero(dists_all < 10.0)[0]
            for obs_idx in near_atten_indices:
                obs = self.obstacles[obs_idx]
                dist_o = float(dists_all[obs_idx])
                closest_pt = closest_pts_all[obs_idx]
                push = push_all[obs_idx]
                push_norm = dist_o
                unit_push = push / push_norm if push_norm > 1e-4 else (
                    obs.surface_normal(pos_i) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                )
                gamma_obs = 0.0 if dist_o <= 2.5 else ((dist_o - 2.5) / (10.0 - 2.5)) ** 2
                proj_o = max(0.0, float(np.dot(f_att, -unit_push)))
                f_att -= proj_o * (1.0 - gamma_obs) * (-unit_push)
        else:
            closest_pts_all = None
            push_all = None
            dists_all = None

        # 2. Inter-drone separation & alignment (Reynolds) & Downwash (Vectorized via NumPy)
        f_sep = np.zeros(3, dtype=np.float64)
        f_align = np.zeros(3, dtype=np.float64)
        f_downwash = np.zeros(3, dtype=np.float64)

        if candidate_ids and dists is not None:
            # Fast bounding rejection: candidate drones within 25.0m
            valid_mask = dists <= 25.0
            zero_mask = valid_mask & (dists <= 1e-4)
            if np.any(zero_mask):
                f_sep += np.array([1.0, 0.0, 0.0], dtype=np.float64) * (80.0 * float(np.sum(zero_mask)))

            calc_mask = valid_mask & (dists > 1e-4)
            if np.any(calc_mask):
                sub_d = dists[calc_mask]
                sub_delta = deltas[calc_mask]
                sub_vel = cand_velocities[calc_mask]

                r_hats = sub_delta / sub_d[:, None]
                v_rels = vel_i - sub_vel
                v_close = np.maximum(0.0, -np.sum(v_rels * r_hats, axis=1))

                if r_sep_limit > 6.0:
                    r_sep_dyn = np.maximum(r_sep_limit * 1.35, (v_close ** 2) / 4.0 + 1.2 * v_close + r_sep_limit * 1.1)
                    active = sub_d < r_sep_dyn
                    if np.any(active):
                        act_d = sub_d[active]
                        act_dyn = r_sep_dyn[active]
                        act_close = v_close[active]
                        act_rhat = r_hats[active]

                        d_eff = np.maximum(act_d - (r_sep_limit * 0.4), 0.1)
                        r_eff = np.maximum(act_dyn - (r_sep_limit * 0.4), 0.2)
                        scale = np.maximum(1.0, (act_dyn / 6.0) ** 2)
                        mag_apf = 60.0 * scale * (1.0 / d_eff - 1.0 / r_eff) / (d_eff ** 2)
                        mag_damp = 18.0 * act_close * ((act_dyn - act_d) / act_dyn) ** 2 * m_i
                        barrier_dist = r_sep_limit * 1.15
                        mag_barrier = np.where(act_d < barrier_dist, 150.0 * ((barrier_dist / np.maximum(act_d, 0.1)) ** 3), 0.0)
                        mag_total = np.minimum(mag_apf + mag_damp + mag_barrier, 400.0)
                        f_sep += np.sum(mag_total[:, None] * act_rhat, axis=0)
                else:
                    r_sep_dyn = np.maximum(6.0, (v_close ** 2) / 6.0 + 0.8 * v_close + 2.5)
                    active = sub_d < r_sep_dyn
                    if np.any(active):
                        act_d = sub_d[active]
                        act_dyn = r_sep_dyn[active]
                        act_close = v_close[active]
                        act_rhat = r_hats[active]

                        d_eff = np.maximum(act_d - 1.8, 0.1)
                        r_eff = np.maximum(act_dyn - 1.8, 0.2)
                        mag_apf = 45.0 * (1.0 / d_eff - 1.0 / r_eff) / (d_eff ** 2)
                        mag_damp = 12.0 * act_close * ((act_dyn - act_d) / act_dyn) ** 2 * m_i
                        mag_barrier = np.where(act_d < 2.5, 80.0 * ((2.2 / np.maximum(act_d, 0.1)) ** 3), 0.0)
                        mag_total = np.minimum(mag_apf + mag_damp + mag_barrier, 250.0)
                        f_sep += np.sum(mag_total[:, None] * act_rhat, axis=0)

            # Alignment force (within 12.0m, same role)
            if drone.flight_mode == FlightMode.TRANSIT:
                same_role_mask = np.array([r == drone.role for r in cand_roles], dtype=bool)
                align_mask = (dists > 0.0) & (dists < 12.0) & same_role_mask
                if np.any(align_mask):
                    f_align += np.sum(0.5 * (cand_velocities[align_mask] - vel_i), axis=0)

            # Aerodynamic downwash cone avoidance
            if self.config.enable_downwash:
                dz = pos_i[2] - cand_positions[:, 2]
                d_xy = np.linalg.norm(deltas[:, :2], axis=1)
                dw_mask = (dz >= -8.0) & (dz <= -0.5) & (d_xy <= (np.abs(dz) * 0.4663 + 1.0))
                if np.any(dw_mask):
                    sub_dxy = d_xy[dw_mask]
                    sub_delta = deltas[dw_mask, :2]
                    lat_dir = np.where(sub_dxy[:, None] > 1e-3, sub_delta / np.maximum(sub_dxy[:, None], 1e-6), np.array([1.0, 0.0]))
                    mag_dw = 35.0 * np.exp(-(sub_dxy ** 2) / 8.0)
                    f_downwash[:2] += np.sum(lat_dir * mag_dw[:, None], axis=0)
                    f_downwash[2] -= np.sum(4.0 * np.exp(-(sub_dxy ** 2) / 2.0))

        # 3. Obstacle repulsion forces
        f_obs = np.zeros(3, dtype=np.float64)
        if self._cached_obs_min_pts is not None and dists_all is not None:
            rep_indices = np.nonzero(dists_all < 35.0)[0]
            for obs_idx in rep_indices:
                obs = self.obstacles[obs_idx]
                dist_obs = float(dists_all[obs_idx])
                closest_pt = closest_pts_all[obs_idx]
                push_dir = push_all[obs_idx]
                norm_push = dist_obs
                if norm_push > 1e-4:
                    unit_push = push_dir / norm_push
                elif hasattr(obs, "surface_normal"):
                    unit_push = obs.surface_normal(pos_i)
                else:
                    unit_push = np.array([0.0, 0.0, 1.0], dtype=np.float64)

                # Dynamic sensing horizon based on approach speed
                v_approach = max(0.0, float(-np.dot(vel_i, unit_push)))
                rho_0_dyn = max(8.0, (v_approach ** 2) / (2.0 * drone.limits.max_accel) + 0.6 * v_approach + 2.5)

                if dist_obs < rho_0_dyn:
                    d_eff = max(dist_obs - 2.0, 0.1)
                    rho_eff = max(rho_0_dyn - 2.0, 0.2)
                    mag_apf = 60.0 * (1.0 / d_eff - 1.0 / rho_eff) / (d_eff ** 2)
                    mag_damp = 15.0 * v_approach * ((rho_0_dyn - dist_obs) / rho_0_dyn) ** 2 * m_i
                    mag_barrier = 100.0 * ((2.5 / max(dist_obs, 0.1)) ** 3) if dist_obs < 3.0 else 0.0
                    mag = min(mag_apf + mag_damp + mag_barrier, 300.0)
                    f_obs += mag * unit_push

                    # Tangential vortex force to prevent saddle-point stagnation
                    vortex = np.cross(unit_push, np.array([0.0, 0.0, 1.0]))
                    if np.linalg.norm(vortex) > 1e-4:
                        f_obs += (vortex / np.linalg.norm(vortex)) * 20.0
        else:
            for obs in self.obstacles:
                if hasattr(obs, "distance_and_closest_point"):
                    dist_obs, closest_pt = obs.distance_and_closest_point(pos_i)
                elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                    closest_pt = np.clip(pos_i, obs.min_pt, obs.max_pt)
                    dist_obs = float(np.linalg.norm(pos_i - closest_pt))
                elif hasattr(obs, "min_bound") and hasattr(obs, "max_bound"):
                    closest_pt = np.clip(pos_i, obs.min_bound, obs.max_bound)
                    dist_obs = float(np.linalg.norm(pos_i - closest_pt))
                else:
                    continue

                push_dir = pos_i - closest_pt
                norm_push = float(np.linalg.norm(push_dir))
                if norm_push > 1e-4:
                    unit_push = push_dir / norm_push
                elif hasattr(obs, "surface_normal"):
                    unit_push = obs.surface_normal(pos_i)
                else:
                    unit_push = np.array([0.0, 0.0, 1.0], dtype=np.float64)

                v_approach = max(0.0, float(-np.dot(vel_i, unit_push)))
                rho_0_dyn = max(8.0, (v_approach ** 2) / (2.0 * drone.limits.max_accel) + 0.6 * v_approach + 2.5)

                if dist_obs < rho_0_dyn:
                    d_eff = max(dist_obs - 2.0, 0.1)
                    rho_eff = max(rho_0_dyn - 2.0, 0.2)
                    mag_apf = 60.0 * (1.0 / d_eff - 1.0 / rho_eff) / (d_eff ** 2)
                    mag_damp = 15.0 * v_approach * ((rho_0_dyn - dist_obs) / rho_0_dyn) ** 2 * m_i
                    mag_barrier = 100.0 * ((2.5 / max(dist_obs, 0.1)) ** 3) if dist_obs < 3.0 else 0.0
                    mag = min(mag_apf + mag_damp + mag_barrier, 300.0)
                    f_obs += mag * unit_push

                    vortex = np.cross(unit_push, np.array([0.0, 0.0, 1.0]))
                    if np.linalg.norm(vortex) > 1e-4:
                        f_obs += (vortex / np.linalg.norm(vortex)) * 20.0

        # 4. Soft boundary containment force
        f_bound = np.zeros(3, dtype=np.float64)
        margin = 15.0
        bx_min, bx_max = self.config.world_bounds_x
        by_min, by_max = self.config.world_bounds_y
        bz_min, bz_max = self.config.world_bounds_z

        if pos_i[0] < bx_min + margin:
            f_bound[0] += 20.0 * (bx_min + margin - pos_i[0])
        elif pos_i[0] > bx_max - margin:
            f_bound[0] -= 20.0 * (pos_i[0] - (bx_max - margin))

        if pos_i[1] < by_min + margin:
            f_bound[1] += 20.0 * (by_min + margin - pos_i[1])
        elif pos_i[1] > by_max - margin:
            f_bound[1] -= 20.0 * (pos_i[1] - (by_max - margin))

        if pos_i[2] > bz_max - margin:
            f_bound[2] -= 25.0 * (pos_i[2] - (bz_max - margin))

        return f_att + f_sep + f_align + f_obs + f_downwash + f_bound

    def update_vsm_relay_setpoints(self) -> None:
        """
        Virtual Spring Mesh (VSM) positioning algorithm.
        Positions Relay UAVs along the line of sight between GCS and Survey UAVs
        at elevated altitudes to guarantee RF connectivity over obstacles.
        """
        if not self.config.enable_vsm_relays:
            return

        survey_drones = [d for d in self.drones.values() if d.role == DroneRole.SURVEY]
        relay_drones = [d for d in self.drones.values() if d.role == DroneRole.RELAY]

        if not survey_drones or not relay_drones:
            return

        # Compute centroid of survey drones
        survey_positions = np.array([d.position for d in survey_drones], dtype=np.float64)
        centroid_xy = np.mean(survey_positions[:, :2], axis=0)
        gcs_xy = np.array(self.config.gcs_position[:2], dtype=np.float64)

        num_relays = len(relay_drones)

        # Check if fleet is dispersed across both West (x < 0) and East (x >= 0) sectors
        west_surveys = [d for d in survey_drones if d.position[0] < 0]
        east_surveys = [d for d in survey_drones if d.position[0] >= 0]

        if west_surveys and east_surveys and num_relays >= 2:
            west_centroid = np.mean([d.position[:2] for d in west_surveys], axis=0)
            east_centroid = np.mean([d.position[:2] for d in east_surveys], axis=0)
            num_west = num_relays // 2
            for idx, relay in enumerate(relay_drones):
                if relay.flight_mode in (FlightMode.IDLE, FlightMode.TAKEOFF, FlightMode.RTL, FlightMode.LANDING, FlightMode.LANDED, FlightMode.EMERGENCY_LAND, FlightMode.COMPLETED):
                    continue
                if idx < num_west:
                    frac = (idx + 1.0) / (num_west + 1.0)
                    target_xy = gcs_xy + frac * (west_centroid - gcs_xy)
                else:
                    e_idx = idx - num_west
                    frac = (e_idx + 1.0) / (num_relays - num_west + 1.0)
                    target_xy = gcs_xy + frac * (east_centroid - gcs_xy)
                target_z = 70.0 + idx * (20.0 / max(num_relays - 1, 1))
                relay.set_target_waypoint(np.array([target_xy[0], target_xy[1], target_z], dtype=np.float64))
        else:
            for idx, relay in enumerate(relay_drones):
                if relay.flight_mode in (FlightMode.IDLE, FlightMode.TAKEOFF, FlightMode.RTL, FlightMode.LANDING, FlightMode.LANDED, FlightMode.EMERGENCY_LAND, FlightMode.COMPLETED):
                    continue
                fraction = (idx + 1.0) / (num_relays + 1.0)
                target_xy = gcs_xy + fraction * (centroid_xy - gcs_xy)
                # Partition altitude in Layer 4 ([70, 90]m)
                target_z = 70.0 + idx * (20.0 / max(num_relays - 1, 1))
                target_pos = np.array([target_xy[0], target_xy[1], target_z], dtype=np.float64)
                relay.set_target_waypoint(target_pos)

    def set_swarm_formation(self, formation_name: str) -> bool:
        """
        Switch swarm tactical formation geometry:
        - 'AUTONOMOUS': Standard distributed autonomous exploration & relay mesh.
        - 'V_FORMATION': Aerodynamic wedge / chevron with leader at apex and staggered wingmen.
        - 'LINE_SWEEP': Lateral line-abreast wall sweep across the operational area.
        - 'PERIMETER_ORBIT': Orbital surveillance ring encircling the disaster core.
        """
        valid_formations = {"AUTONOMOUS", "V_FORMATION", "LINE_SWEEP", "PERIMETER_ORBIT"}
        norm = str(formation_name).strip().upper().replace("-", "_").replace(" ", "_")
        aliases = {
            "V": "V_FORMATION",
            "V_SHAPE": "V_FORMATION",
            "CHEVRON": "V_FORMATION",
            "WEDGE": "V_FORMATION",
            "LINE": "LINE_SWEEP",
            "SWEEP": "LINE_SWEEP",
            "WALL": "LINE_SWEEP",
            "ORBIT": "PERIMETER_ORBIT",
            "PERIMETER": "PERIMETER_ORBIT",
            "RING": "PERIMETER_ORBIT",
            "AUTO": "AUTONOMOUS",
        }
        if norm in aliases:
            norm = aliases[norm]
        if norm not in valid_formations:
            return False
        self.active_formation = norm
        if self.mission_manager is not None and hasattr(self.mission_manager, "emit_tactical_comms"):
            self.mission_manager.emit_tactical_comms(
                "INFO",
                "SWARM",
                f"🛡️ FORMATION RECONFIGURED: Swarm engaged in [{self.active_formation}]",
                "OPERATIONS"
            )
        return True

    def update_formation_setpoints(self) -> None:
        """
        Compute coordinated formation setpoints when active_formation != 'AUTONOMOUS'.
        Applies kinematic offsets relative to the fleet leader or mission center,
        dynamically feeding attractive APF setpoints while Reynolds separation
        and obstacle repulsion guarantee collision safety.
        """
        if self.active_formation == "AUTONOMOUS":
            return

        # Select airborne participating drones
        airborne_drones = [
            d for d in sorted(self.drones.values(), key=lambda d: d.id)
            if d.flight_mode in (FlightMode.TAKEOFF, FlightMode.TRANSIT, FlightMode.SURVEYING, FlightMode.RELAY, FlightMode.DATA_TX)
            and not getattr(d, "is_manual_override", False)
            and not getattr(d, "is_fault_injected", False)
            and not getattr(d, "is_low_battery_rtb", False)
        ]
        if not airborne_drones:
            return

        leader = airborne_drones[0]
        pos_lead = leader.position
        tgt_lead = leader.get_target_waypoint()
        if tgt_lead is not None and np.linalg.norm(tgt_lead[:2] - pos_lead[:2]) > 2.0:
            h_dir = tgt_lead[:2] - pos_lead[:2]
            h_dir = h_dir / np.linalg.norm(h_dir)
        elif np.linalg.norm(leader.velocity[:2]) > 0.5:
            h_dir = leader.velocity[:2] / np.linalg.norm(leader.velocity[:2])
        else:
            h_dir = np.array([0.0, 1.0], dtype=np.float64)

        # Lateral right vector perpendicular to heading
        right_dir = np.array([h_dir[1], -h_dir[0]], dtype=np.float64)

        if self.active_formation == "V_FORMATION":
            # Apex leader stays on course; wingmen stagger in V-shape
            for idx, drone in enumerate(airborne_drones):
                if idx == 0:
                    continue
                rank = (idx + 1) // 2
                side = -1.0 if (idx % 2 == 1) else 1.0
                lateral_dist = side * rank * 16.0
                long_dist = -rank * 14.0
                target_xy = pos_lead[:2] + h_dir * long_dist + right_dir * lateral_dist
                target_z = max(24.0, pos_lead[2] + ((rank - 1) % 3) * 2.5)
                drone.set_target_waypoint(np.array([target_xy[0], target_xy[1], target_z], dtype=np.float64))
                if drone.flight_mode in (FlightMode.SURVEYING, FlightMode.RELAY, FlightMode.DATA_TX):
                    drone.flight_mode = FlightMode.TRANSIT
                # V-formation wingmen draft in the upwash vortices
                drone.is_drafting = True
                drone.drafting_leader_id = leader.id
                drone.drafting_saving_pct = round(12.0 + (rank % 3) * 4.0, 1)

        elif self.active_formation == "LINE_SWEEP":
            # Lateral wall sweep perpendicular to heading
            n_drones = len(airborne_drones)
            spacing = 18.0
            mid_pt = pos_lead[:2]
            for idx, drone in enumerate(airborne_drones):
                offset_lat = (idx - (n_drones - 1) / 2.0) * spacing
                target_xy = mid_pt + right_dir * offset_lat
                target_z = max(25.0, 32.0 + (idx % 3) * 3.0)
                drone.set_target_waypoint(np.array([target_xy[0], target_xy[1], target_z], dtype=np.float64))
                if drone.flight_mode in (FlightMode.SURVEYING, FlightMode.RELAY, FlightMode.DATA_TX):
                    drone.flight_mode = FlightMode.TRANSIT
                drone.is_drafting = False

        elif self.active_formation == "PERIMETER_ORBIT":
            # Orbital ring encircling the disaster core at (0, 0)
            n_drones = len(airborne_drones)
            radius = 80.0
            omega = 0.08  # rad/s slow rotation
            base_angle = self.sim_time * omega
            for idx, drone in enumerate(airborne_drones):
                theta = base_angle + (2.0 * math.pi * idx / max(1, n_drones))
                target_x = radius * math.cos(theta)
                target_y = radius * math.sin(theta)
                target_z = 35.0 + (idx % 4) * 4.0
                drone.set_target_waypoint(np.array([target_x, target_y, target_z], dtype=np.float64))
                if drone.flight_mode in (FlightMode.SURVEYING, FlightMode.RELAY, FlightMode.DATA_TX):
                    drone.flight_mode = FlightMode.TRANSIT
                drone.is_drafting = False

    # -------------------------------------------------------------------------
    # Master Step Execution Loop
    # -------------------------------------------------------------------------

    def step(self, dt: Optional[float] = None) -> TelemetrySnapshot:
        """
        Advance simulation by dt seconds.
        Executes perception, VSM relay positioning, force calculations,
        kinematic integration, subsystem updates, and telemetry serialization.
        """
        step_dt = dt if dt is not None else self.config.dt
        if step_dt <= 0.0:
            raise ValueError(f"Step dt must be strictly positive, got {step_dt}")

        self.sim_time += step_dt
        self.step_count += 1

        # Phase 1: Dynamic Relay Positioning (VSM) or Tactical Swarm Formations
        if self.active_formation == "AUTONOMOUS":
            self.update_vsm_relay_setpoints()
        else:
            self.update_formation_setpoints()

        # Update spatial grid for accelerated proximity queries
        if self.spatial_grid is not None:
            self.spatial_grid.build(list(self.drones.values()))

        drone_ids = sorted(self.drones.keys())
        self._cached_drone_ids = drone_ids
        if drone_ids:
            self._cached_drone_positions = np.array([self.drones[did].position for did in drone_ids], dtype=np.float64)
            self._cached_drone_velocities = np.array([self.drones[did].velocity for did in drone_ids], dtype=np.float64)
            self._cached_drone_roles = [self.drones[did].role for did in drone_ids]
            self._cached_drone_idx_map = {did: i for i, did in enumerate(drone_ids)}

        # Phase 2: Compute steering forces (double-buffered) and advance physics
        self._update_cached_obstacles()
        forces = {}
        for drone_id in drone_ids:
            drone = self.drones[drone_id]
            if drone.flight_mode not in (FlightMode.IDLE, FlightMode.LANDED, FlightMode.COMPLETED):
                forces[drone_id] = self.compute_steering_forces(drone)
            else:
                forces[drone_id] = np.zeros(3, dtype=np.float64)

        for drone_id in drone_ids:
            drone = self.drones[drone_id]
            accel = forces[drone_id] / drone.limits.mass_kg
            if self.config.enable_weather:
                w_vec = self.weather.sample(drone.position, drone.velocity, step_dt)
            else:
                w_vec = None
            drone.step(step_dt, desired_accel=accel, obstacles=self.obstacles, ambient_wind=w_vec)

        # Phase 3: Enforce hard ground collision, world boundary & obstacle clamping
        for drone_id in drone_ids:
            drone = self.drones[drone_id]
            self.environment.enforce_bounds(drone)
            if self._cached_obs_min_pts is not None and self._cached_obs_max_pts is not None:
                inside_mask = np.all((drone.position >= self._cached_obs_min_pts) & (drone.position <= self._cached_obs_max_pts), axis=1)
                if np.any(inside_mask):
                    for obs_idx in np.nonzero(inside_mask)[0]:
                        obs = self.obstacles[obs_idx]
                        normal = obs.surface_normal(drone.position) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                        min_p = self._cached_obs_min_pts[obs_idx]
                        max_p = self._cached_obs_max_pts[obs_idx]
                        for axis in range(3):
                            if normal[axis] > 0.5:
                                drone.position[axis] = max_p[axis] + 0.02
                            elif normal[axis] < -0.5:
                                drone.position[axis] = min_p[axis] - 0.02
                        v_dot_n = float(np.dot(drone.velocity, normal))
                        if v_dot_n < 0.0:
                            drone.velocity -= v_dot_n * normal
                        a_dot_n = float(np.dot(drone.acceleration, normal))
                        if a_dot_n < 0.0:
                            drone.acceleration -= a_dot_n * normal
            else:
                for obs in self.obstacles:
                    if hasattr(obs, "contains_point") and obs.contains_point(drone.position, margin=0.0):
                        normal = obs.surface_normal(drone.position) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                        min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
                        max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
                        if min_p is not None and max_p is not None:
                            for axis in range(3):
                                if normal[axis] > 0.5:
                                    drone.position[axis] = max_p[axis] + 0.02
                                elif normal[axis] < -0.5:
                                    drone.position[axis] = min_p[axis] - 0.02
                        v_dot_n = float(np.dot(drone.velocity, normal))
                        if v_dot_n < 0.0:
                            drone.velocity -= v_dot_n * normal
                        a_dot_n = float(np.dot(drone.acceleration, normal))
                        if a_dot_n < 0.0:
                            drone.acceleration -= a_dot_n * normal

            # Touchdown detection for descending drones
            if drone.flight_mode in (FlightMode.LANDING, FlightMode.EMERGENCY_LAND):
                if drone.position[2] <= 0.25 and abs(float(drone.velocity[2])) <= 1.0:
                    drone.position[2] = 0.0
                    drone.velocity[:] = 0.0
                    drone.acceleration[:] = 0.0
                    drone.rotor_speeds[:] = 0.0
                    drone.set_flight_mode(FlightMode.LANDED)
                    drone._landed_for_recharge = True
                    drone.target_position = None
                    drone.assigned_poi_id = None
                    drone.is_transmitting = False

            # GCS Automated Rapid Battery Charging & Continuous Mission Persistence
            if drone.flight_mode == FlightMode.LANDED and drone.position[2] <= 0.4:
                if hasattr(drone, "battery") and drone.battery is not None:
                    if drone.battery.soc < 0.999:
                        drone.battery.soc = min(1.0, drone.battery.soc + 0.05 * step_dt)
                        drone._is_charging = True
                    if drone.battery.soc >= 0.999:
                        drone._is_charging = False
                        if getattr(drone, "_landed_for_recharge", False) or getattr(drone, "is_low_battery_rtb", False):
                            is_retreat = False
                            if self.mission_manager and getattr(self.mission_manager, "retreat_all_requested", False):
                                is_retreat = True
                            if not is_retreat and getattr(drone, "auto_relaunch", True):
                                drone._landed_for_recharge = False
                                drone.is_low_battery_rtb = False
                                drone.is_comms_loss_rtl = False
                                drone.set_flight_mode(FlightMode.TAKEOFF)
                                relaunch_target = np.array([drone.position[0], drone.position[1], 35.0], dtype=np.float64)
                                drone.set_target_waypoint(relaunch_target)
                                if self.mission_manager and hasattr(self.mission_manager, "emit_tactical_comms"):
                                    self.mission_manager.emit_tactical_comms(
                                        "SUCCESS",
                                        drone.id,
                                        f"🔋 RECHARGED (100% SoC) — Auto-relaunching to continuous disaster patrol",
                                        "GCS_CHARGER"
                                    )

        # Phase 4: Subsystem updates (Network, Mission & Dynamic POI Spawner)
        if hasattr(self, "poi_spawner") and self.poi_spawner is not None:
            self.poi_spawner.update(self.sim_time, self.pois)

        if self.network_engine is not None:
            if hasattr(self.network_engine, "update"):
                self.network_engine.update(
                    drones=self.drones,
                    gcs_pos=np.array(self.config.gcs_position),
                    obstacles=self.obstacles,
                    dt=step_dt,
                )
            elif hasattr(self.network_engine, "update_topology"):
                node_pos = {d.id: d.position for d in self.drones.values()}
                node_pos["GCS"] = np.array(self.config.gcs_position)
                self.network_engine.update_topology(node_pos, self.obstacles)

        if self.mission_manager is not None and hasattr(self.mission_manager, "update"):
            self.mission_manager.update(
                drones=self.drones,
                pois=self.pois,
                dt=step_dt,
                network_engine=self.network_engine,
                obstacles=self.obstacles,
            )

        # Phase 5: Generate and buffer telemetry snapshot
        snapshot = self.get_telemetry_snapshot()
        self.history.append(snapshot)
        return snapshot

    # -------------------------------------------------------------------------
    # Telemetry Snapshot Generation & Serialization
    # -------------------------------------------------------------------------

    def get_telemetry_snapshot(self) -> TelemetrySnapshot:
        """
        Assemble comprehensive, immutable TelemetrySnapshot dataclass.
        Guarantees strict schema adherence for both visualization and testing.
        """
        drones_list = []
        drone_ids = getattr(self, "_cached_drone_ids", None)
        if drone_ids is None or len(drone_ids) != len(self.drones):
            drone_ids = sorted(self.drones.keys())
            self._cached_drone_ids = drone_ids

        for drone_id in drone_ids:
            d = self.drones[drone_id]
            st = d.get_state()
            role_str = st.role.name if hasattr(st.role, "name") else str(st.role)
            mode_str = st.flight_mode.name if hasattr(st.flight_mode, "name") else str(st.flight_mode)
            drone_entry: Dict[str, Any] = {
                "id": st.id,
                "role": role_str,
                "position": [round(float(c), 3) for c in st.position],
                "velocity": [round(float(v), 3) for v in st.velocity],
                "attitude": [round(float(a), 4) for a in st.attitude],
                "rotor_speeds": [round(float(r), 1) for r in st.rotor_speeds],
                "battery_soc": round(float(st.battery_soc), 4),
                "battery_pct": round(float(st.battery_soc * 100.0), 1),
                "flight_mode": mode_str,
                "assigned_poi_id": st.assigned_poi_id,
                "target_position": [round(float(c), 3) for c in st.target_position] if st.target_position is not None else None,
                "power_w": round(float(getattr(d.battery, "current_power_w", 0.0)), 1) if hasattr(d, "battery") else 0.0,
                "est_endurance_min": round(float(d.battery.remaining_flight_time_s() / 60.0), 1) if hasattr(d, "battery") else 30.0,
                "comms_loss": bool(getattr(d, "is_comms_loss_rtl", False)),
                "is_manual_override": bool(st.is_manual_override),
                "is_fault_injected": bool(st.is_fault_injected),
                "is_drafting": bool(st.is_drafting),
                "drafting_leader_id": st.drafting_leader_id,
                "drafting_saving_pct": float(st.drafting_saving_pct),
            }
            if getattr(d, "assigned_sector_id", None) is not None:
                drone_entry["sector"] = d.assigned_sector_id
            if d.flight_mode == FlightMode.LANDED and d.position[2] <= 0.4 and (getattr(d, "_is_charging", False) or d.battery.soc < 0.99):
                drone_entry["is_charging"] = True
            if getattr(self.config, "include_estimates", False) and st.estimated_position is not None:
                drone_entry["estimated_position"] = [round(float(c), 3) for c in st.estimated_position]
                drone_entry["estimated_velocity"] = [round(float(v), 3) for v in st.estimated_velocity] if st.estimated_velocity is not None else None
            drones_list.append(drone_entry)

        # GCS telemetry
        gcs_data = {
            "position": list(self.config.gcs_position),
            "comm_radius": self.config.gcs_comm_radius,
            "packets_received": getattr(self.network_engine, "gcs_packet_count", 0),
        }

        # PoI status telemetry
        pois_list = []
        for poi_id in sorted(self.pois.keys()):
            poi = self.pois[poi_id]
            req_time = max(poi["required_dwell_time"], 1e-4)
            progress = round(float(min(1.0, poi["current_dwell_time"] / req_time) * 100.0), 1)
            p_dict = {
                "id": poi["id"],
                "position": [round(float(c), 2) for c in poi["position"]],
                "priority": poi["priority"],
                "progress": progress,
                "is_completed": poi["is_completed"],
                "assigned_drone": poi["assigned_drone_id"],
                "is_spawned": poi.get("is_spawned", True),
                "is_detected": poi.get("is_detected", False),
                "is_reported": poi.get("is_reported", False),
            }
            if "reporting_latency_s" in poi:
                p_dict["reporting_latency_s"] = poi["reporting_latency_s"]
            if "is_sla_compliant" in poi:
                p_dict["is_sla_compliant"] = poi["is_sla_compliant"]
            pois_list.append(p_dict)

        # Network links and routes
        active_routes: List[List[str]] = []
        links_list: List[Dict[str, Any]] = []
        packets_list: List[Dict[str, Any]] = []
        metrics = {
            "pdr": 1.0,
            "avg_latency_ms": 0.0,
            "completed_pois": sum(1 for p in self.pois.values() if p["is_completed"]),
        }

        if self.network_engine is not None:
            if hasattr(self.network_engine, "get_active_routes"):
                active_routes = self.network_engine.get_active_routes()
            if hasattr(self.network_engine, "get_active_links"):
                links_list = self.network_engine.get_active_links()
            if hasattr(self.network_engine, "get_active_packets"):
                packets_list = self.network_engine.get_active_packets()
            if hasattr(self.network_engine, "get_metrics"):
                metrics.update(self.network_engine.get_metrics())

        weather_data = None
        if self.config.enable_weather:
            ref_wind = self.weather.get_mean_wind(10.0)
            sample_wind = self.weather.sample(np.array([0.0, 0.0, 20.0]), np.zeros(3), 0.05)
            sample_speed = float(np.linalg.norm(sample_wind))
            weather_data = {
                "enabled": True,
                "mean_speed_mps": round(float(self.weather.config.mean_speed_mps), 2),
                "current_speed_mps": round(sample_speed, 2),
                "current_speed_kts": round(sample_speed * 1.94384, 1),
                "direction_deg": round(float(self.weather.config.direction_deg), 1),
                "ref_wind_vector": [round(float(w), 2) for w in ref_wind],
                "sample_wind_vector": [round(float(w), 2) for w in sample_wind],
                "gust_active": bool(getattr(self.weather, "_gust_active", False)),
                "turbulence_intensity": self.weather.config.turbulence_intensity,
            }

        # Mission Time Budget & Priority Queue telemetry
        rem_s = round(float(self.mission_manager.get_time_remaining()), 1) if (self.mission_manager and hasattr(self.mission_manager, "get_time_remaining")) else round(max(0.0, 300.0 - float(self.sim_time)), 1)
        b_status = self.mission_manager.get_budget_status() if (self.mission_manager and hasattr(self.mission_manager, "get_budget_status")) else "ON_SCHEDULE"
        mission_budget_data = {
            "total_budget_s": float(getattr(self.mission_manager, "mission_time_budget", 300.0)),
            "elapsed_s": round(float(self.sim_time), 1),
            "remaining_s": rem_s,
            "time_remaining_s": rem_s,
            "status": b_status,
            "is_all_completed": b_status == "COMPLETED" or (len(self.pois) > 0 and sum(1 for p in self.pois.values() if p["is_completed"]) == len(self.pois)),
        }
        priority_queue_data = []
        if self.mission_manager is not None and hasattr(self.mission_manager, "get_priority_queue_telemetry"):
            priority_queue_data = self.mission_manager.get_priority_queue_telemetry()

        # Synthetic Thermal AI Survivors, Tactical Visual Comms, and Charging Pads
        survivors_data = self.mission_manager.get_survivors_telemetry() if (self.mission_manager and hasattr(self.mission_manager, "get_survivors_telemetry")) else None
        tactical_comms_data = self.mission_manager.get_tactical_comms_telemetry() if (self.mission_manager and hasattr(self.mission_manager, "get_tactical_comms_telemetry")) else None
        charging_pads_data = self.mission_manager.get_charging_pads_telemetry() if (self.mission_manager and hasattr(self.mission_manager, "get_charging_pads_telemetry")) else None
        sectors_data = self.mission_manager.get_sectors_telemetry() if (self.mission_manager and hasattr(self.mission_manager, "get_sectors_telemetry")) else None
        abnormalities_data = self.mission_manager.get_abnormalities_telemetry() if (self.mission_manager and hasattr(self.mission_manager, "get_abnormalities_telemetry")) else None

        challenge_data = None
        if hasattr(self, "challenge_monitor") and self.challenge_monitor is not None:
            challenge_data = self.challenge_monitor.evaluate_step(
                self.sim_time,
                self.drones,
                getattr(self, "poi_spawner", None),
                links_list,
                np.array(self.config.gcs_position),
            )

        return TelemetrySnapshot(
            sim_time=round(self.sim_time, 3),
            drones=drones_list,
            gcs=gcs_data,
            pois=pois_list,
            active_routes=active_routes,
            links=links_list,
            packets=packets_list,
            metrics=metrics,
            weather=weather_data,
            priority_queue=priority_queue_data,
            mission_budget=mission_budget_data,
            survivors=survivors_data,
            tactical_comms=tactical_comms_data,
            charging_pads=charging_pads_data,
            challenge_constraints=challenge_data,
            active_formation=self.active_formation,
            sectors=sectors_data,
            abnormalities=abnormalities_data,
        )

    def to_dict(self) -> Dict[str, Any]:
        """
        Convert current snapshot to dictionary matching vis/server.py and test expectations.
        Provides backward/forward compatible aliases ('swarm' and 'drones').
        """
        snapshot = self.get_telemetry_snapshot()
        return snapshot.to_dict()

    def to_json(self, indent: Optional[int] = None) -> str:
        """Serialize current simulation frame to JSON string."""
        return json.dumps(self.to_dict(), indent=indent)

    def reset(self) -> None:
        """Reset simulation clock and drone states to initial configurations."""
        self.sim_time = 0.0
        self.step_count = 0
        self.active_formation = "AUTONOMOUS"
        self.history.clear()
        self.collision_events.clear()
        self.flight_events.clear()
        for drone in self.drones.values():
            drone.reset()

    def trigger_fleet_retreat(self) -> None:
        """Command all active drones in the swarm to autonomously retreat (RTL) and land safely."""
        if self.mission_manager is not None and hasattr(self.mission_manager, "trigger_fleet_retreat"):
            self.mission_manager.trigger_fleet_retreat()
        for drone in self.drones.values():
            if drone.flight_mode not in (FlightMode.LANDED, FlightMode.IDLE):
                drone.trigger_retreat()

    def trigger_drone_retreat(self, drone_id: str) -> None:
        """Command an individual drone to autonomously retreat (RTL) and land safely."""
        if self.mission_manager is not None and hasattr(self.mission_manager, "trigger_drone_retreat"):
            self.mission_manager.trigger_drone_retreat(drone_id, self.drones)
        elif drone_id in self.drones:
            self.drones[drone_id].trigger_retreat()

    def trigger_chaos_fault(self, target_drone_id: Optional[str] = None) -> Optional[str]:
        """
        Swarm Self-Healing Chaos Fault Injection:
        Simulate sudden motor flameout on target or random airborne drone.
        """
        if self.mission_manager is not None and hasattr(self.mission_manager, "trigger_chaos_fault"):
            victim_id = self.mission_manager.trigger_chaos_fault(self.drones, target_drone_id)
            return victim_id
        elif target_drone_id and target_drone_id in self.drones:
            self.drones[target_drone_id].inject_fault()
            return target_drone_id
        return None

    def set_drone_manual_control(
        self,
        drone_id: str,
        vx: float = 0.0,
        vy: float = 0.0,
        vz: float = 0.0,
        yaw_rate: float = 0.0,
        enabled: bool = True
    ) -> bool:
        """
        Manual FPV Controller Mode:
        Direct manual velocity vector control override for operator takeover.
        """
        if drone_id not in self.drones:
            return False
        drone = self.drones[drone_id]
        drone.set_manual_control(vx, vy, vz, yaw_rate, enabled=enabled)
        return True

    def dispatch_drone_waypoint(self, drone_id: str, target_pos: Sequence[float]) -> bool:
        """
        Interactive 3D Click-to-Dispatch:
        Directly assign an operational 3D waypoint to any UAV in the fleet.
        """
        if drone_id not in self.drones:
            return False
        drone = self.drones[drone_id]
        tgt = np.asarray(target_pos, dtype=np.float64).flatten()
        if tgt.shape[0] == 2:
            target = np.array([tgt[0], tgt[1], 35.0], dtype=np.float64)
        elif tgt.shape[0] >= 3:
            target = np.array([tgt[0], tgt[1], max(15.0, float(tgt[2]))], dtype=np.float64)
        else:
            return False

        if hasattr(drone, "set_waypoint_path"):
            curr_pos = drone.position
            mid_pt = 0.5 * (curr_pos + target) + np.array([0.0, 0.0, 2.0], dtype=np.float64)
            drone.set_waypoint_path([curr_pos, mid_pt, target], smooth=True)
        else:
            drone.set_target_waypoint(target)
        if drone.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            drone.flight_mode = FlightMode.TAKEOFF
        elif drone.flight_mode != FlightMode.TAKEOFF:
            drone.flight_mode = FlightMode.TRANSIT
        drone.is_manual_override = False
        drone.manual_vel_cmd = None

        if self.mission_manager is not None and hasattr(self.mission_manager, "emit_tactical_comms"):
            self.mission_manager.emit_tactical_comms(
                "INFO",
                drone.id,
                f"🎯 TACTICAL DISPATCH: {drone.id} routed to ({target[0]:.0f}, {target[1]:.0f}, {target[2]:.0f}m)",
                "OPERATOR"
            )
        return True


