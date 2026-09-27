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

        # Event logging
        self.collision_events: List[Dict[str, Any]] = []
        self.flight_events: List[Dict[str, Any]] = []

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
        }

    def set_network_engine(self, network_engine: Any) -> None:
        """Attach a FANET communication and routing subsystem (Milestone 2)."""
        self.network_engine = network_engine

    def set_mission_manager(self, mission_manager: Any) -> None:
        """Attach a high-level disaster survey mission manager (Milestone 3)."""
        self.mission_manager = mission_manager

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
            dist = float(np.linalg.norm(err))
            k_att = 1.5
            if dist > 15.0:
                f_att = (err / dist) * 15.0 * k_att
            elif dist > 1e-4:
                f_att = err * k_att

        # Prioritized Safety Attenuation: Attenuate attractive force along blocked paths
        for other_id in sorted(self.drones.keys()):
            if other_id == drone.id:
                continue
            other = self.drones[other_id]
            delta = pos_i - other.position
            d_peer = float(np.linalg.norm(delta))
            if 0.0 < d_peer < 8.0:
                r_hat = delta / d_peer
                gamma = 0.0 if d_peer <= 2.2 else ((d_peer - 2.2) / (8.0 - 2.2)) ** 2
                proj = max(0.0, float(np.dot(f_att, -r_hat)))
                f_att -= proj * (1.0 - gamma) * (-r_hat)

        for obs in self.obstacles:
            if hasattr(obs, "distance_and_closest_point"):
                dist_o, closest_pt = obs.distance_and_closest_point(pos_i)
            elif hasattr(obs, "min_bound") and hasattr(obs, "max_bound"):
                closest_pt = np.clip(pos_i, obs.min_bound, obs.max_bound)
                dist_o = float(np.linalg.norm(pos_i - closest_pt))
            elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                closest_pt = np.clip(pos_i, obs.min_pt, obs.max_pt)
                dist_o = float(np.linalg.norm(pos_i - closest_pt))
            else:
                continue

            if dist_o < 10.0:
                push = pos_i - closest_pt
                push_norm = float(np.linalg.norm(push))
                unit_push = push / push_norm if push_norm > 1e-4 else (
                    obs.surface_normal(pos_i) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                )
                gamma_obs = 0.0 if dist_o <= 2.5 else ((dist_o - 2.5) / (10.0 - 2.5)) ** 2
                proj_o = max(0.0, float(np.dot(f_att, -unit_push)))
                f_att -= proj_o * (1.0 - gamma_obs) * (-unit_push)

        # 2. Inter-drone separation & alignment (Reynolds) & Downwash
        f_sep = np.zeros(3, dtype=np.float64)
        f_align = np.zeros(3, dtype=np.float64)
        f_downwash = np.zeros(3, dtype=np.float64)

        for other_id in sorted(self.drones.keys()):
            if other_id == drone.id:
                continue
            other = self.drones[other_id]
            delta = pos_i - other.position
            dist = float(np.linalg.norm(delta))

            # Dynamic closing velocity repulsive horizon
            if dist > 1e-4:
                r_hat = delta / dist
                v_rel = vel_i - other.velocity
                v_close = max(0.0, float(-np.dot(v_rel, r_hat)))
                r_sep_dyn = max(6.0, (v_close ** 2) / 6.0 + 0.8 * v_close + 2.5)

                if dist < r_sep_dyn:
                    d_eff = max(dist - 1.8, 0.1)
                    r_eff = max(r_sep_dyn - 1.8, 0.2)
                    mag_apf = 45.0 * (1.0 / d_eff - 1.0 / r_eff) / (d_eff ** 2)
                    mag_damp = 12.0 * v_close * ((r_sep_dyn - dist) / r_sep_dyn) ** 2 * m_i
                    mag_barrier = 80.0 * ((2.2 / max(dist, 0.1)) ** 3) if dist < 2.5 else 0.0
                    mag_total = min(mag_apf + mag_damp + mag_barrier, 250.0)
                    f_sep += mag_total * r_hat
            elif dist <= 1e-4:
                f_sep += np.array([1.0, 0.0, 0.0], dtype=np.float64) * 80.0

            # Alignment force (within 12.0m, same role)
            if 0.0 < dist < 12.0 and other.role == drone.role and drone.flight_mode == FlightMode.TRANSIT:
                f_align += 0.5 * (other.velocity - vel_i)

            # Aerodynamic downwash cone avoidance
            if self.config.enable_downwash:
                dz = pos_i[2] - other.position[2]
                d_xy = float(np.linalg.norm(delta[:2]))
                # If drone_i is below other drone within 25-degree opening cone
                if -8.0 <= dz <= -0.5 and d_xy <= (abs(dz) * 0.4663 + 1.0):
                    if d_xy > 1e-3:
                        lateral_dir = delta[:2] / d_xy
                    else:
                        lateral_dir = np.array([1.0, 0.0], dtype=np.float64)
                    mag_dw = 35.0 * math.exp(-(d_xy ** 2) / 8.0)
                    f_downwash[0] += lateral_dir[0] * mag_dw
                    f_downwash[1] += lateral_dir[1] * mag_dw
                    f_downwash[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)

        # 3. Obstacle repulsion forces
        f_obs = np.zeros(3, dtype=np.float64)
        for obs in self.obstacles:
            if hasattr(obs, "distance_and_closest_point"):
                dist_obs, closest_pt = obs.distance_and_closest_point(pos_i)
            elif hasattr(obs, "min_bound") and hasattr(obs, "max_bound"):
                closest_pt = np.clip(pos_i, obs.min_bound, obs.max_bound)
                dist_obs = float(np.linalg.norm(pos_i - closest_pt))
            elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                closest_pt = np.clip(pos_i, obs.min_pt, obs.max_pt)
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
                if relay.flight_mode in (FlightMode.RTL, FlightMode.LANDING, FlightMode.LANDED, FlightMode.EMERGENCY_LAND, FlightMode.COMPLETED):
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
                if relay.flight_mode in (FlightMode.RTL, FlightMode.LANDING, FlightMode.LANDED, FlightMode.EMERGENCY_LAND, FlightMode.COMPLETED):
                    continue
                fraction = (idx + 1.0) / (num_relays + 1.0)
                target_xy = gcs_xy + fraction * (centroid_xy - gcs_xy)
                # Partition altitude in Layer 4 ([70, 90]m)
                target_z = 70.0 + idx * (20.0 / max(num_relays - 1, 1))
                target_pos = np.array([target_xy[0], target_xy[1], target_z], dtype=np.float64)
                relay.set_target_waypoint(target_pos)

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

        # Phase 1: Dynamic Relay Positioning (VSM)
        self.update_vsm_relay_setpoints()

        # Phase 2: Compute steering forces (double-buffered) and advance physics
        forces = {}
        for drone_id in sorted(self.drones.keys()):
            drone = self.drones[drone_id]
            if drone.flight_mode not in (FlightMode.IDLE, FlightMode.LANDED, FlightMode.COMPLETED):
                forces[drone_id] = self.compute_steering_forces(drone)
            else:
                forces[drone_id] = np.zeros(3, dtype=np.float64)

        for drone_id in sorted(self.drones.keys()):
            drone = self.drones[drone_id]
            accel = forces[drone_id] / drone.limits.mass_kg
            if self.config.enable_weather:
                w_vec = self.weather.sample(drone.position, drone.velocity, step_dt)
            else:
                w_vec = None
            drone.step(step_dt, desired_accel=accel, obstacles=self.obstacles, ambient_wind=w_vec)

        # Phase 3: Enforce hard ground collision, world boundary & obstacle clamping
        for drone_id in sorted(self.drones.keys()):
            drone = self.drones[drone_id]
            self.environment.enforce_bounds(drone)
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
                    drone.target_position = None
                    drone.assigned_poi_id = None
                    drone.is_transmitting = False

        # Phase 4: Subsystem updates (Network & Mission)
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
        for drone_id in sorted(self.drones.keys()):
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
            pois_list.append({
                "id": poi["id"],
                "position": [round(float(c), 2) for c in poi["position"]],
                "priority": poi["priority"],
                "progress": progress,
                "is_completed": poi["is_completed"],
                "assigned_drone": poi["assigned_drone_id"],
            })

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

