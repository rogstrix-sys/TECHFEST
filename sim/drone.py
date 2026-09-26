"""
sim/drone.py: 6-DOF Quadcopter Kinematics, Dynamics, Flocking & Vector Steering Engine.

Implements Newton-Euler translational and rotational kinematics,
semi-implicit Euler state integration, Khatib Artificial Potential Fields,
Reynolds boids flocking forces, aerodynamic downwash cone repulsion,
and 4-tier altitude corridor deconfliction.
"""

from __future__ import annotations

import math
from typing import Any, List, Optional, Sequence, Tuple, Union
import numpy as np

from sim.sensors import DroneEKF, SensorSuite
from sim.types import (
    BatteryModel,
    DroneLimits,
    DroneRole,
    DroneState,
    FlightMode,
)


class Drone:
    """
    Autonomous 6-DOF Quadcopter Agent.
    Implements Newton-Euler translational and rotational kinematics,
    Khatib APF, Reynolds flocking, downwash repulsion, and altitude corridor clamping.
    """

    def __init__(
        self,
        drone_id: str,
        role: Union[DroneRole, str] = DroneRole.SURVEY,
        initial_position: Optional[np.ndarray] = None,
        initial_pos: Optional[np.ndarray] = None,
        limits: Optional[DroneLimits] = None,
        battery: Optional[BatteryModel] = None,
    ) -> None:
        self.id = str(drone_id)
        if isinstance(role, DroneRole):
            self.role = role
        else:
            try:
                self.role = DroneRole(str(role).upper())
            except ValueError:
                self.role = DroneRole.SURVEY

        self.limits = limits if limits is not None else DroneLimits()
        self.battery = battery if battery is not None else BatteryModel()

        # 3D Physical States (Inertial ENU Frame)
        init_p = initial_position if initial_position is not None else initial_pos
        init_pos = init_p if init_p is not None else np.zeros(3, dtype=np.float64)
        self._initial_pos = np.array(init_pos, dtype=np.float64)
        self.position = self._initial_pos.copy()
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)

        # Attitude: Euler [roll, pitch, yaw] in radians and unit Quaternion [qw, qx, qy, qz]
        self.attitude = np.zeros(3, dtype=np.float64)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.angular_velocity = np.zeros(3, dtype=np.float64)
        self.rotor_speeds = np.full(4, 993.4, dtype=np.float64)

        # Mission & Navigation
        self.flight_mode = FlightMode.IDLE
        self.target_position: Optional[np.ndarray] = None
        self.assigned_poi_id: Optional[str] = None
        self.is_transmitting: bool = False
        self.dwell_time: float = 0.0
        self.obstacles: List[Any] = []
        self.home_position: np.ndarray = self._initial_pos.copy()
        self.recovery_pad: Optional[np.ndarray] = None

        # Physical constants
        self.g = 9.80665

        # Avionics Sensor Suite & 9-State Extended Kalman Filter (EKF)
        self.sensor_suite = SensorSuite()
        self.ekf = DroneEKF(initial_position=self.position)
        self.ambient_wind = np.zeros(3, dtype=np.float64)

    # -------------------------------------------------------------------------
    # Properties & Aliases
    # -------------------------------------------------------------------------

    @property
    def drone_id(self) -> str:
        """Alias for self.id."""
        return self.id

    @property
    def state(self) -> DroneState:
        """Dynamic state snapshot."""
        return self.get_state()

    def set_target_waypoint(self, pos: Union[np.ndarray, Sequence[float]]) -> None:
        """Sets active 3D waypoint navigation setpoint."""
        self.target_position = np.array(pos, dtype=np.float64)

    def set_target(self, pos: Union[np.ndarray, Sequence[float]]) -> None:
        """Alias for set_target_waypoint."""
        self.set_target_waypoint(pos)

    def get_target_waypoint(self) -> Optional[np.ndarray]:
        """Returns current target waypoint or None."""
        return self.target_position.copy() if self.target_position is not None else None

    def set_home_position(self, pos: Union[np.ndarray, Sequence[float]]) -> None:
        """Sets designated Return-to-Launch (RTL) home position."""
        self.home_position = np.array(pos, dtype=np.float64)

    def set_recovery_pad(self, pos: Union[np.ndarray, Sequence[float]]) -> None:
        """Sets assigned recovery / landing pad setpoint."""
        self.recovery_pad = np.array(pos, dtype=np.float64)

    def get_home_position(self) -> np.ndarray:
        """Returns active recovery pad or home position."""
        if self.recovery_pad is not None:
            return self.recovery_pad.copy()
        return self.home_position.copy()

    def trigger_retreat(
        self,
        recovery_pad: Optional[Union[np.ndarray, Sequence[float]]] = None,
        cruise_altitude: float = 55.0,
    ) -> None:
        """
        Commands autonomous retreat (RTH/RTL) towards recovery pad.
        Transitions mode to RTL and establishes safe Tier 3 obstacle clearance altitude.
        """
        if recovery_pad is not None:
            self.set_recovery_pad(recovery_pad)
        pad = self.get_home_position()
        self.set_flight_mode(FlightMode.RTL)
        self.set_target_waypoint(np.array([pad[0], pad[1], float(cruise_altitude)], dtype=np.float64))
        self.assigned_poi_id = None
        self.is_transmitting = False
        self.dwell_time = 0.0

    def set_flight_mode(self, mode: Union[FlightMode, str]) -> None:
        """Updates autonomous flight mode."""
        if isinstance(mode, FlightMode):
            self.flight_mode = mode
        else:
            self.flight_mode = FlightMode(str(mode).upper())

    def reset(self) -> None:
        """Resets drone kinematic state and recharges battery."""
        self.position = self._initial_pos.copy()
        self.velocity = np.zeros(3, dtype=np.float64)
        self.acceleration = np.zeros(3, dtype=np.float64)
        self.attitude = np.zeros(3, dtype=np.float64)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        self.angular_velocity = np.zeros(3, dtype=np.float64)
        self.rotor_speeds = np.full(4, 993.4, dtype=np.float64)
        self.flight_mode = FlightMode.IDLE
        self.target_position = None
        self.assigned_poi_id = None
        self.is_transmitting = False
        self.dwell_time = 0.0
        self.recovery_pad = None
        self.home_position = self._initial_pos.copy()
        self.battery.reset()
        self.sensor_suite = SensorSuite()
        self.ekf = DroneEKF(initial_position=self.position)
        self.ambient_wind = np.zeros(3, dtype=np.float64)

    # -------------------------------------------------------------------------
    # Attitude Conversion Helpers
    # -------------------------------------------------------------------------

    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Converts Euler roll, pitch, yaw (radians) into a normalized unit quaternion [qw, qx, qy, qz]."""
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        q = np.array([qw, qx, qy, qz], dtype=np.float64)
        norm = float(np.linalg.norm(q))
        return q / norm if norm > 1e-9 else np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)

    @staticmethod
    def quaternion_to_euler(q: np.ndarray) -> np.ndarray:
        """Converts quaternion [qw, qx, qy, qz] into Euler roll, pitch, yaw (radians)."""
        qw, qx, qy, qz = float(q[0]), float(q[1]), float(q[2]), float(q[3])

        # Roll (x-axis rotation)
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2.0 * (qw * qy - qz * qx)
        pitch = math.asin(max(-1.0, min(1.0, sinp)))

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=np.float64)

    # -------------------------------------------------------------------------
    # Force Synthesis: APF, Flocking, Downwash, Corridors
    # -------------------------------------------------------------------------

    def compute_attractive_force(self) -> np.ndarray:
        """Calculates conic-parabolic APF attractive force towards target_position."""
        if self.target_position is None:
            return np.zeros(3, dtype=np.float64)

        diff = self.target_position - self.position
        dist = float(np.linalg.norm(diff))
        if dist < 1e-4:
            return np.zeros(3, dtype=np.float64)

        d_thresh = 15.0
        k_att = 1.2
        if dist <= d_thresh:
            return k_att * diff
        else:
            return d_thresh * k_att * (diff / dist)

    def compute_obstacle_repulsion(self, obstacles: List[Any]) -> np.ndarray:
        """
        Calculates APF obstacle repulsion forces from 3D AABBs with dynamic velocity-dependent
        sensing horizon, approach damping, and singularity barrier.
        """
        f_obs = np.zeros(3, dtype=np.float64)
        m = self.limits.mass_kg
        a_max = self.limits.max_accel

        for obs in obstacles:
            if hasattr(obs, "distance_and_closest_point"):
                dist, closest_pt = obs.distance_and_closest_point(self.position)
            elif hasattr(obs, "min_bound") and hasattr(obs, "max_bound"):
                closest_pt = np.clip(self.position, obs.min_bound, obs.max_bound)
                dist = float(np.linalg.norm(self.position - closest_pt))
            elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                closest_pt = np.clip(self.position, obs.min_pt, obs.max_pt)
                dist = float(np.linalg.norm(self.position - closest_pt))
            else:
                continue

            push = self.position - closest_pt
            push_norm = float(np.linalg.norm(push))
            if push_norm > 1e-4:
                n_hat = push / push_norm
            elif hasattr(obs, "surface_normal"):
                n_hat = obs.surface_normal(self.position)
            else:
                n_hat = np.array([0.0, 0.0, 1.0], dtype=np.float64)

            # Dynamic sensing horizon based on approach speed
            v_approach = max(0.0, float(-np.dot(self.velocity, n_hat)))
            rho_0_dyn = max(8.0, (v_approach ** 2) / (2.0 * a_max) + 0.6 * v_approach + 2.5)

            if dist < rho_0_dyn:
                d_eff = max(dist - 2.0, 0.1)
                rho_eff = max(rho_0_dyn - 2.0, 0.2)
                mag_apf = 60.0 * (1.0 / d_eff - 1.0 / rho_eff) / (d_eff ** 2)
                mag_damp = 15.0 * v_approach * ((rho_0_dyn - dist) / rho_0_dyn) ** 2 * m
                mag_barrier = 100.0 * ((2.5 / max(dist, 0.1)) ** 3) if dist < 3.0 else 0.0
                mag = min(mag_apf + mag_damp + mag_barrier, 300.0)
                f_obs += mag * n_hat

                # Lateral vortex circulatory force when approaching obstacle at speed to avoid head-on stagnation
                if v_approach > 1.0:
                    vortex = np.cross(n_hat, np.array([0.0, 0.0, 1.0]))
                    if np.linalg.norm(vortex) > 1e-4:
                        f_obs += (vortex / np.linalg.norm(vortex)) * 20.0

        return f_obs

    def compute_flocking_forces(self, peers: List[Drone]) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Calculates Reynolds separation, alignment, and cohesion forces with dynamic
        relative closing velocity horizon, approach damping, and singularity barrier.
        """
        f_sep = np.zeros(3, dtype=np.float64)
        f_align = np.zeros(3, dtype=np.float64)
        f_coh = np.zeros(3, dtype=np.float64)

        neighbors: List[Drone] = []
        r_percept = 12.0
        r_sep_static = self.limits.separation_radius
        m = self.limits.mass_kg

        for peer in peers:
            if peer.id == self.id or peer.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                continue

            diff = self.position - peer.position
            dist = float(np.linalg.norm(diff))
            if dist < 1e-4:
                continue

            r_hat = diff / dist
            v_rel = self.velocity - peer.velocity
            v_close = max(0.0, float(-np.dot(v_rel, r_hat)))

            # Dynamic velocity-dependent separation horizon
            r_sep_dyn = max(r_sep_static, (v_close ** 2) / 6.0 + 0.8 * v_close + 2.5)

            if dist < r_sep_dyn:
                d_eff = max(dist - 1.8, 0.1)
                r_eff = max(r_sep_dyn - 1.8, 0.2)
                mag_apf = 45.0 * (1.0 / d_eff - 1.0 / r_eff) / (d_eff ** 2)
                mag_damp = 12.0 * v_close * ((r_sep_dyn - dist) / r_sep_dyn) ** 2 * m
                mag_barrier = 80.0 * ((2.2 / max(dist, 0.1)) ** 3) if dist < 2.5 else 0.0

                mag_total = min(mag_apf + mag_damp + mag_barrier, 250.0)
                f_sep += mag_total * r_hat

            if dist < r_percept:
                neighbors.append(peer)

        if neighbors:
            # Alignment (match velocity of active transit flock)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_vel = np.mean([p.velocity for p in neighbors], axis=0)
                f_align = 0.8 * (avg_vel - self.velocity)

            # Cohesion (pull towards local center of mass)
            if self.flight_mode == FlightMode.TRANSIT:
                avg_pos = np.mean([p.position for p in neighbors], axis=0)
                f_coh = 0.2 * (avg_pos - self.position)

        return f_sep, f_align, f_coh

    def compute_downwash_repulsion(self, peers: List[Drone]) -> np.ndarray:
        """
        Calculates asymmetric vertical downwash jet repulsion.
        Pushes this drone horizontally outward if it is located inside the
        turbulent propeller wash cone of a higher drone.
        """
        f_dw = np.zeros(3, dtype=np.float64)
        tan_theta = 0.4663  # tan(25 deg)
        k_dw = 30.0
        sigma_dw = 1.8

        for peer in peers:
            if peer.id == self.id or peer.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                continue

            # Delta z: positive if peer is ABOVE this drone
            dz = peer.position[2] - self.position[2]
            if 0.5 <= dz <= 10.0:
                diff_xy = self.position[:2] - peer.position[:2]
                d_xy = float(np.linalg.norm(diff_xy))
                r_cone = dz * tan_theta

                if d_xy < r_cone:
                    # Inside the downwash cone! Outward lateral push
                    if d_xy > 1e-3:
                        n_xy = diff_xy / d_xy
                    else:
                        n_xy = np.array([1.0, 0.0], dtype=np.float64)  # Deterministic escape vector

                    mag = k_dw * math.exp(-(d_xy ** 2) / (2.0 * (sigma_dw ** 2)))
                    f_dw[0] += mag * n_xy[0]
                    f_dw[1] += mag * n_xy[1]
                    f_dw[2] -= 4.0 * math.exp(-(d_xy ** 2) / 2.0)  # Downward turbulence sink

        return f_dw

    def compute_corridor_force(self) -> np.ndarray:
        """Calculates restoring vertical force to enforce 4-tier airspace corridor."""
        z_min, z_max = self.get_assigned_altitude_band()
        pz = self.position[2]
        k_corr = 15.0
        d_corr = 4.0

        fz = 0.0
        if pz < z_min:
            fz = k_corr * (z_min - pz) - d_corr * self.velocity[2]
        elif pz > z_max:
            fz = k_corr * (z_max - pz) - d_corr * self.velocity[2]

        return np.array([0.0, 0.0, fz], dtype=np.float64)

    def get_assigned_altitude_band(self) -> Tuple[float, float]:
        """Maps drone role and flight mode to airspace altitude envelope."""
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            return 0.0, 1.0
        elif self.flight_mode in (FlightMode.TAKEOFF, FlightMode.LANDING, FlightMode.EMERGENCY_LAND):
            return 0.0, 25.0
        elif self.flight_mode == FlightMode.SURVEYING:
            return 25.0, 45.0  # Tier 2: PoI Inspection
        elif self.flight_mode in (FlightMode.TRANSIT, FlightMode.RTL):
            return 50.0, 65.0  # Tier 3: High-speed Transit
        elif self.flight_mode == FlightMode.RELAY:
            return 70.0, 90.0  # Tier 4: Elevated Relay Mesh
        return 0.0, 120.0

    def compute_total_force(
        self,
        obstacles: Optional[List[Any]] = None,
        peers: Optional[List[Drone]] = None,
    ) -> np.ndarray:
        """
        Synthesizes all vector steering forces into a composite commanded force,
        applying Prioritized Safety Attenuation of waypoint attraction when blocked
        by peers or obstacles.
        """
        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            return np.zeros(3, dtype=np.float64)

        obs_list = obstacles if obstacles is not None else getattr(self, "obstacles", [])
        peer_list = peers if peers is not None else []

        if self.flight_mode in (FlightMode.LANDING, FlightMode.EMERGENCY_LAND):
            target = self.target_position if self.target_position is not None else np.array([self.position[0], self.position[1], 0.0], dtype=np.float64)
            err_xy = target[:2] - self.position[:2]
            f_xy = err_xy * 1.5 - self.velocity[:2] * 1.2 * self.limits.mass_kg
            v_z_des = -min(1.5, max(0.4, 0.4 * self.position[2]))
            f_z = 8.0 * (v_z_des - self.velocity[2]) * self.limits.mass_kg
            f_att = np.array([f_xy[0], f_xy[1], f_z], dtype=np.float64)
            f_obs = self.compute_obstacle_repulsion(obs_list)
            f_sep, f_align, f_coh = self.compute_flocking_forces(peer_list)
            return f_att + f_obs + f_sep

        f_att = self.compute_attractive_force()

        # Prioritized Safety Attenuation: Attenuate attractive force along blocked paths
        for peer in peer_list:
            if peer.id == self.id or peer.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                continue
            delta = self.position - peer.position
            dist = float(np.linalg.norm(delta))
            if 0.0 < dist < 8.0:
                r_hat = delta / dist
                gamma = 0.0 if dist <= 2.2 else ((dist - 2.2) / (8.0 - 2.2)) ** 2
                proj = max(0.0, float(np.dot(f_att, -r_hat)))
                f_att -= proj * (1.0 - gamma) * (-r_hat)

        for obs in obs_list:
            if hasattr(obs, "distance_and_closest_point"):
                dist_o, closest_pt = obs.distance_and_closest_point(self.position)
            elif hasattr(obs, "min_pt") and hasattr(obs, "max_pt"):
                closest_pt = np.clip(self.position, obs.min_pt, obs.max_pt)
                dist_o = float(np.linalg.norm(self.position - closest_pt))
            elif hasattr(obs, "min_bound") and hasattr(obs, "max_bound"):
                closest_pt = np.clip(self.position, obs.min_bound, obs.max_bound)
                dist_o = float(np.linalg.norm(self.position - closest_pt))
            else:
                continue

            if dist_o < 10.0:
                push = self.position - closest_pt
                push_norm = float(np.linalg.norm(push))
                unit_push = push / push_norm if push_norm > 1e-4 else (
                    obs.surface_normal(self.position) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                )
                gamma_obs = 0.0 if dist_o <= 2.5 else ((dist_o - 2.5) / (10.0 - 2.5)) ** 2
                proj_o = max(0.0, float(np.dot(f_att, -unit_push)))
                f_att -= proj_o * (1.0 - gamma_obs) * (-unit_push)

        f_obs = self.compute_obstacle_repulsion(obs_list)
        f_sep, f_align, f_coh = self.compute_flocking_forces(peer_list)
        f_dw = self.compute_downwash_repulsion(peer_list)
        f_corr = self.compute_corridor_force()

        # Local minimum escape check
        net_horizontal = f_att[:2] + f_obs[:2]
        if np.linalg.norm(net_horizontal) < 0.5 and np.linalg.norm(f_obs[:2]) > 1.0:
            n_obs = f_obs[:2] / np.linalg.norm(f_obs[:2])
            f_tan = np.array([-n_obs[1], n_obs[0], 0.0], dtype=np.float64) * 20.0
            return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr + f_tan

        return f_att + f_obs + f_sep + f_align + f_coh + f_dw + f_corr

    # -------------------------------------------------------------------------
    # Physics Integration & State Update
    # -------------------------------------------------------------------------

    def step(
        self,
        dt: float,
        desired_accel: Optional[np.ndarray] = None,
        obstacles: Optional[Sequence[Any]] = None,
        ambient_wind: Optional[np.ndarray] = None,
    ) -> None:
        """
        Advance drone physics by dt seconds.
        If desired_accel is supplied, commanded force is desired_accel * mass.
        Otherwise, commanded force is synthesized from active fields.
        """
        obs_list = obstacles if obstacles is not None else getattr(self, "obstacles", [])
        if desired_accel is not None:
            cmd_force = np.asarray(desired_accel, dtype=np.float64) * self.limits.mass_kg
        else:
            cmd_force = self.compute_total_force(obstacles=obs_list)
        self.step_physics(dt, cmd_force, obstacles=obs_list, ambient_wind=ambient_wind)

    def step_physics(
        self,
        dt: float,
        commanded_force: np.ndarray,
        obstacles: Optional[Sequence[Any]] = None,
        ambient_wind: Optional[np.ndarray] = None,
    ) -> None:
        """
        Executes semi-implicit Euler integration of translational and rotational kinematics.
        Enforces physical acceleration, speed, vertical rate, ground contact, and hard
        obstacle boundary collision constraints.
        """
        if ambient_wind is not None:
            self.ambient_wind = np.asarray(ambient_wind, dtype=np.float64)

        if self.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
            self.velocity[:] = 0.0
            self.acceleration[:] = 0.0
            if self.flight_mode == FlightMode.LANDED:
                self.rotor_speeds[:] = 0.0
            self.battery.step(dt, speed=0.0, accel=0.0, is_transmitting=False, is_surveying=False)
            return

        m = self.limits.mass_kg
        obs_list = obstacles if obstacles is not None else getattr(self, "obstacles", [])

        # 1. Commanded Acceleration & Saturation
        a_cmd = commanded_force / m
        a_mag = float(np.linalg.norm(a_cmd))
        if a_mag > self.limits.max_accel:
            self.acceleration = (a_cmd / a_mag) * self.limits.max_accel
        else:
            self.acceleration = a_cmd.copy()

        # 2. Linear Drag Integration with ambient wind
        v_air = self.velocity - self.ambient_wind
        drag_accel = np.array([
            (self.limits.drag_coeff_xy / m) * v_air[0],
            (self.limits.drag_coeff_xy / m) * v_air[1],
            (self.limits.drag_coeff_z / m) * v_air[2],
        ], dtype=np.float64)
        a_net = self.acceleration - drag_accel

        # 3. Velocity Integration & Hard Speed Clamping
        self.velocity += a_net * dt

        # Horizontal speed clamp
        v_xy = float(np.linalg.norm(self.velocity[:2]))
        if v_xy > self.limits.max_speed_xy:
            self.velocity[:2] = (self.velocity[:2] / v_xy) * self.limits.max_speed_xy

        # Vertical climb / descent clamp
        self.velocity[2] = max(-self.limits.max_speed_z_down, min(self.limits.max_speed_z_up, self.velocity[2]))

        # 4. Position Integration & Ground Surface Constraint
        self.position += self.velocity * dt
        if self.position[2] <= 0.0:
            self.position[2] = 0.0
            self.velocity[2] = max(0.0, float(self.velocity[2]))
            self.acceleration[2] = max(0.0, float(self.acceleration[2]))

        # Touchdown detection for landing flight modes
        if self.flight_mode in (FlightMode.LANDING, FlightMode.EMERGENCY_LAND):
            if self.position[2] <= 0.25 and abs(float(self.velocity[2])) <= 1.0:
                self.position[2] = 0.0
                self.velocity[:] = 0.0
                self.acceleration[:] = 0.0
                self.rotor_speeds[:] = 0.0
                self.set_flight_mode(FlightMode.LANDED)
                self.target_position = None
                self.assigned_poi_id = None
                self.is_transmitting = False
                return

        # 5. Hard Obstacle Surface Collision Clamping
        for obs in obs_list:
            if hasattr(obs, "contains_point") and obs.contains_point(self.position, margin=0.0):
                normal = obs.surface_normal(self.position) if hasattr(obs, "surface_normal") else np.array([0.0, 0.0, 1.0])
                min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
                max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
                if min_p is not None and max_p is not None:
                    for axis in range(3):
                        if normal[axis] > 0.5:
                            self.position[axis] = max_p[axis] + 0.02
                        elif normal[axis] < -0.5:
                            self.position[axis] = min_p[axis] - 0.02
                v_dot_n = float(np.dot(self.velocity, normal))
                if v_dot_n < 0.0:
                    self.velocity -= v_dot_n * normal
                a_dot_n = float(np.dot(self.acceleration, normal))
                if a_dot_n < 0.0:
                    self.acceleration -= a_dot_n * normal

        # 6. Attitude Generation (Euler & Quaternion)
        self._update_attitude(dt)

        # 7. Rotor Speeds Update
        hover_speed = 993.4
        a_norm = float(np.linalg.norm(self.acceleration))
        delta_rotor = 50.0 * (a_norm / self.limits.max_accel)
        self.rotor_speeds = np.clip(
            np.array([
                hover_speed + delta_rotor,
                hover_speed - delta_rotor,
                hover_speed + delta_rotor,
                hover_speed - delta_rotor,
            ], dtype=np.float64),
            400.0,
            1500.0,
        )

        # 8. Avionics Sensor Suite Sampling & EKF State Estimation Update
        hdop = 1.0
        for obs in obs_list:
            if hasattr(obs, "distance_and_closest_point"):
                d_o, _ = obs.distance_and_closest_point(self.position)
                if d_o < 15.0:
                    hdop = max(hdop, 1.0 + (15.0 - d_o) / 5.0)

        meas = self.sensor_suite.sample(
            true_pos=self.position,
            true_vel=self.velocity,
            true_accel=self.acceleration,
            true_angular_vel=self.angular_velocity,
            dt=dt,
            gps_hdop_multiplier=hdop,
        )
        self.ekf.predict(meas["imu"]["accel"], dt)
        if meas["gps"] is not None:
            self.ekf.update_gps(meas["gps"]["position"], meas["gps"]["velocity"], hdop=meas["gps"]["hdop"])
        if meas["baro"] is not None:
            self.ekf.update_baro(meas["baro"])

        # 9. Battery State of Charge Depletion
        speed = float(np.linalg.norm(self.velocity))
        self.battery.step(
            dt=dt,
            speed=speed,
            accel=a_norm,
            is_transmitting=self.is_transmitting,
            is_surveying=(self.flight_mode == FlightMode.SURVEYING),
        )

    def _update_attitude(self, dt: float) -> None:
        """Calculates desired tilt from acceleration and smoothly tracks it."""
        g = self.g
        # Desired roll (bank angle proportional to lateral acceleration)
        desired_roll = float(np.clip(self.acceleration[1] / g, -self.limits.max_tilt_rad, self.limits.max_tilt_rad))
        # Desired pitch (pitch angle proportional to longitudinal acceleration)
        desired_pitch = float(np.clip(-self.acceleration[0] / g, -self.limits.max_tilt_rad, self.limits.max_tilt_rad))

        # Desired yaw: track horizontal velocity vector or target
        v_xy = float(np.linalg.norm(self.velocity[:2]))
        if v_xy > 0.5:
            target_yaw = math.atan2(self.velocity[1], self.velocity[0])
        elif self.target_position is not None:
            target_yaw = math.atan2(self.target_position[1] - self.position[1], self.target_position[0] - self.position[0])
        else:
            target_yaw = float(self.attitude[2])

        # Slew rate filtering (exponential decay filter for unconditional numerical stability)
        tau_att = 0.12
        alpha = 1.0 - math.exp(-dt / max(tau_att, 1e-4))
        roll_diff = desired_roll - self.attitude[0]
        pitch_diff = desired_pitch - self.attitude[1]
        self.attitude[0] += roll_diff * alpha
        self.attitude[1] += pitch_diff * alpha
        self.attitude[0] = float(np.clip(self.attitude[0], -self.limits.max_tilt_rad, self.limits.max_tilt_rad))
        self.attitude[1] = float(np.clip(self.attitude[1], -self.limits.max_tilt_rad, self.limits.max_tilt_rad))


        # Yaw wrap-around with rate limit
        yaw_err = math.atan2(math.sin(target_yaw - self.attitude[2]), math.cos(target_yaw - self.attitude[2]))
        max_dyaw = self.limits.max_yaw_rate * dt
        self.attitude[2] += max(-max_dyaw, min(max_dyaw, yaw_err))
        self.attitude[2] = math.atan2(math.sin(self.attitude[2]), math.cos(self.attitude[2]))

        # Normalize and update quaternion
        self.quaternion = self.euler_to_quaternion(self.attitude[0], self.attitude[1], self.attitude[2])

    def get_state(self) -> DroneState:
        """Produces a clean DroneState snapshot matching PROJECT.md interface."""
        role_str = self.role.value if hasattr(self.role, "value") else str(self.role)
        mode_str = self.flight_mode.value if hasattr(self.flight_mode, "value") else str(self.flight_mode)
        return DroneState(
            id=self.id,
            role=role_str,
            position=self.position.copy(),
            velocity=self.velocity.copy(),
            attitude=self.attitude.copy(),
            rotor_speeds=self.rotor_speeds.copy(),
            battery_soc=float(self.battery.soc),
            flight_mode=mode_str,
            assigned_poi_id=self.assigned_poi_id,
            target_position=self.target_position.copy() if self.target_position is not None else None,
            quaternion=self.quaternion.copy(),
            acceleration=self.acceleration.copy(),
            estimated_position=self.ekf.estimated_position,
            estimated_velocity=self.ekf.estimated_velocity,
        )
