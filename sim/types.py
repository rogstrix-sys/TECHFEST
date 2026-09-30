"""
sim/types.py: Core Data Models, Enums, and Telemetry Structures for UAV Swarm Simulation.

Strictly implements interface contracts defined in PROJECT.md and explorer specifications.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional
import numpy as np


class DroneRole(str, Enum):
    """Operational role assigned to a UAV in the FANET mission."""
    SURVEY = "SURVEY"    # Navigates to PoIs, collects sensor data, initiates transmissions
    RELAY = "RELAY"      # Positioned via Virtual Spring Mesh to maintain multi-hop RF connectivity
    RESERVE = "RESERVE"  # Standby unit ready to replace low-battery or compromised nodes

    def __str__(self) -> str:
        return self.value


class FlightMode(str, Enum):
    """MAVSDK / PX4 compliant autonomous flight modes."""
    IDLE = "IDLE"                      # Disarmed / waiting on ground pad
    TAKEOFF = "TAKEOFF"                # Ascending vertically to assigned corridor
    TRANSIT = "TRANSIT"                # Cruising horizontally between waypoints
    SURVEYING = "SURVEYING"            # Loitering / orbiting active PoI for sensor collection
    RELAY = "RELAY"                    # Holding dynamic network mesh position
    DATA_TX = "DATA_TX"                # Transmitting high-bandwidth sensor payload
    RTL = "RTL"                        # Returning to Ground Control Station (Return-to-Launch)
    LANDING = "LANDING"                # Final vertical descent phase
    LANDED = "LANDED"                  # Safely landed and disarmed at base
    DOCKED_SWAPPING = "DOCKED_SWAPPING"# Landed at charging pad undergoing automated battery hot-swap
    EMERGENCY_LAND = "EMERGENCY_LAND"  # Critical failsafe descent due to low battery or fault
    MANUAL = "MANUAL"                  # Direct human pilot manual override via gamepad / keyboard
    COMPLETED = "COMPLETED"            # Mission objectives completed

    def __str__(self) -> str:
        return self.value


class PoIPriority(str, Enum):
    """Priority classifications for disaster Points of Interest."""
    LOW = "LOW"
    MEDIUM = "MEDIUM"
    HIGH = "HIGH"
    CRITICAL = "CRITICAL"

    def __str__(self) -> str:
        return self.value


class TelemetryDict(dict):
    """
    Dictionary container providing backward/forward compatibility aliases
    ('swarm' -> 'drones', 'routes' -> 'active_routes', 'timestamp' -> 'sim_time')
    without inflating JSON serialization size.
    """
    def __getitem__(self, key: Any) -> Any:
        if key == "swarm":
            return super().__getitem__("drones")
        if key == "routes":
            return super().__getitem__("active_routes")
        if key == "timestamp":
            return super().__getitem__("sim_time")
        if key == "wind":
            return super().get("weather")
        if key in ("formation", "active_formation"):
            return super().get("active_formation", "AUTONOMOUS")
        return super().__getitem__(key)

    def __contains__(self, key: Any) -> bool:
        if key in ("swarm", "routes", "timestamp", "wind", "formation", "active_formation"):
            return True
        return super().__contains__(key)

    def get(self, key: Any, default: Any = None) -> Any:
        if key in ("swarm", "routes", "timestamp", "wind", "formation", "active_formation"):
            return self[key]
        return super().get(key, default)



@dataclass
class DroneLimits:
    """Kinematic, dynamic, and physical limits for quadcopter airframe."""
    max_speed_xy: float = 10.0          # Maximum horizontal speed (m/s)
    max_speed_z_up: float = 3.5         # Maximum vertical climb rate (m/s)
    max_speed_z_down: float = 2.5       # Maximum vertical descent rate to avoid VRS (m/s)
    max_accel: float = 4.0              # Maximum linear acceleration magnitude (m/s^2)
    max_tilt_rad: float = 0.5236        # Maximum bank/tilt angle (30 degrees in radians)
    max_yaw_rate: float = 1.5708        # Maximum rotational yaw speed (90 deg/s in radians)
    mass_kg: float = 1.20               # Quadcopter dry mass including battery (kg)
    hover_thrust_n: float = 11.768      # Hover thrust: m * g = 1.20 * 9.80665 (N)
    thrust_to_weight: float = 2.2       # Max thrust-to-weight ratio (TWR)
    rotor_thrust_coeff: float = 2.98e-6 # k_f in N / (rad/s)^2
    rotor_torque_coeff: float = 1.14e-7 # k_m in N*m / (rad/s)^2
    arm_length_m: float = 0.225         # Frame arm length from center to motor (m)
    separation_radius: float = 6.0      # Reynolds boid perception separation radius (m)
    collision_radius: float = 1.0       # Physical airframe safety radius for collision test (m)
    drag_coeff_xy: float = 0.15         # Linear drag coefficient in horizontal plane (N*s/m)
    drag_coeff_z: float = 0.25          # Linear drag coefficient in vertical direction (N*s/m)
    clamp_3d_speed: bool = False        # When True, clamps total 3D speed to max_speed_xy


@dataclass
class BatteryModel:
    """Electro-mechanical LiPo battery model with realistic power dissipation."""
    capacity_mah: float = 5000.0          # Total battery capacity in milliampere-hours (mAh)
    nominal_voltage: float = 14.8         # 4S LiPo nominal voltage (Volts)
    soc: float = 1.0                      # State of Charge fraction in [0.0, 1.0]
    p_base_w: float = 12.0                # Flight computer, IMU, GPS base electronics power (W)
    p_hover_w: float = 180.0              # Baseline aerodynamic hover propulsion power (W)
    p_payload_w: float = 8.0              # Optical camera / LiDAR sensor package power (W)
    p_tx_w: float = 15.0                  # Active high-power RF relay transmission power (W)
    p_rx_w: float = 3.0                   # RF idle receiving/listening power (W)
    rtb_soc_threshold: float = 0.25       # State of Charge threshold to trigger RTL (25%)
    emergency_soc_threshold: float = 0.10 # State of Charge threshold for emergency landing (10%)
    current_power_w: float = 0.0          # Instantaneous total power draw in Watts
    last_p_prop: float = 0.0              # Instantaneous propulsion power
    last_p_climb: float = 0.0             # Gravitational climb power component
    last_p_parasite: float = 0.0          # Aerodynamic parasite drag power component
    last_altitude_factor: float = 1.0     # Air density altitude scaling factor

    @property
    def total_energy_joules(self) -> float:
        """Total usable energy capacity in Joules: V * Ah * 3600."""
        return (self.capacity_mah * 1e-3) * self.nominal_voltage * 3600.0

    def step(
        self,
        dt: float,
        speed: float = 0.0,
        accel: float = 0.0,
        is_transmitting: bool = False,
        is_surveying: bool = False,
        altitude: float = 0.0,
        climb_rate: float = 0.0,
        airspeed: Optional[float] = None,
        drafting_factor: float = 0.0,
    ) -> float:
        if airspeed is not None:
            speed = airspeed
        """
        Integrates power consumption over dt seconds and updates SoC.
        Accounts for:
        1. Altitude / air density: Thinner air requires higher rotor RPM to generate lift,
           increasing induced hover power by ~1 / sqrt(rho_ratio).
        2. Airspeed: Induced power drops slightly with translational lift, but parasite drag
           grows cubically with airspeed (0.5 * rho * v^3 * Cd * A).
        3. Climb rate: Climbing against gravity consumes potential power (m * g * vz / eta),
           while descent lowers power draw.
        4. Formation drafting upwash: Reduces induced hover power by drafting_factor (up to 20%).
        """
        # 1. Barometric air density ratio relative to sea-level (rho0 = 1.225 kg/m^3)
        alt = max(0.0, float(altitude))
        rho_ratio = float(np.exp(-alt / 8400.0))
        alt_factor = 1.0 / float(np.sqrt(max(0.4, rho_ratio)))
        self.last_altitude_factor = alt_factor

        # 2. Aerodynamic propulsion power scaling with airspeed
        v = max(0.0, float(speed))
        # Translational lift reduces induced hover power (minimum power cruise speed bucket ~ 6.5 m/s)
        trans_lift_factor = max(0.72, 1.0 / float(np.sqrt(1.0 + (v / 8.0) ** 2)))
        # Upwash drafting attenuation (V-formation aerodynamic benefit)
        draft_benefit = max(0.0, min(0.20, float(drafting_factor)))
        # Parasite drag scales cubically with airspeed: 0.5 * rho * CdA * v^3
        # Frontal drag area CdA ~ 0.075 m^2 for tilted quadcopter body and frame
        p_parasite = 0.5 * 1.225 * rho_ratio * 0.075 * (v ** 3)
        self.last_p_parasite = p_parasite

        # 3. Gravitational potential work rate for vertical climb / descent
        mass_kg = 1.8  # Nominal operational quadcopter mass (kg)
        vz = float(climb_rate)
        if vz > 0.05:
            p_climb = (mass_kg * 9.81 * vz) / 0.75  # 75% powertrain electrical-to-mechanical efficiency
        elif vz < -0.05:
            # Gentle descent reduces rotor thrust demand; bounded to prevent negative propulsion
            p_climb = max(-0.35 * self.p_hover_w, (mass_kg * 9.81 * vz) * 0.45)
        else:
            p_climb = 0.0
        self.last_p_climb = p_climb

        # Total aerodynamic & dynamic propulsion power
        p_prop = (self.p_hover_w * (1.0 - draft_benefit) * trans_lift_factor * alt_factor) + p_parasite + p_climb + (self.p_hover_w * 0.08 * abs(accel))
        p_prop = max(25.0, p_prop)
        self.last_p_prop = p_prop

        # Sensor and RF power
        p_sensor = self.p_payload_w if is_surveying else 0.0
        p_rf = self.p_tx_w if is_transmitting else self.p_rx_w
        p_total = self.p_base_w + p_prop + p_sensor + p_rf
        self.current_power_w = p_total

        delta_energy = p_total * dt
        delta_soc = delta_energy / self.total_energy_joules
        self.soc = max(0.0, min(1.0, self.soc - delta_soc))
        return p_total

    def reset(self) -> None:
        """Reset battery state to full charge."""
        self.soc = 1.0
        self.current_power_w = 0.0
        self.last_p_prop = 0.0
        self.last_p_climb = 0.0
        self.last_p_parasite = 0.0
        self.last_altitude_factor = 1.0

    def is_low(self) -> bool:
        """Returns True if battery is below Return-to-Base threshold (25%)."""
        return self.soc <= self.rtb_soc_threshold

    def is_critical(self) -> bool:
        """Returns True if battery is below Emergency Landing threshold (10%)."""
        return self.soc <= self.emergency_soc_threshold

    def remaining_flight_time_s(self, average_power_w: Optional[float] = None) -> float:
        """Estimates remaining flight endurance in seconds under current or nominal power draw."""
        p_draw = average_power_w if average_power_w is not None else (self.current_power_w if self.current_power_w > 10.0 else 195.0)
        remaining_joules = self.soc * self.total_energy_joules
        return remaining_joules / max(1.0, p_draw)

    def reset(self) -> None:
        """Recharges battery to 100% SoC."""
        self.soc = 1.0


@dataclass
class AltitudeCorridor:
    """Definition of an airspace altitude layer for traffic deconfliction."""
    name: str
    z_min: float
    z_max: float
    z_nominal: float

    def contains(self, z: float) -> bool:
        """Returns True if altitude z is within this corridor [z_min, z_max]."""
        return self.z_min <= z <= self.z_max


# Predefined 4-tier airspace corridors
AIRSPACE_CORRIDORS = {
    "TIER_1_LAUNCH": AltitudeCorridor("Tier 1: Launch & Recovery", 0.0, 20.0, 10.0),
    "TIER_2_SURVEY": AltitudeCorridor("Tier 2: PoI Inspection", 25.0, 45.0, 35.0),
    "TIER_3_TRANSIT": AltitudeCorridor("Tier 3: Transit & RTL", 50.0, 65.0, 55.0),
    "TIER_4_RELAY": AltitudeCorridor("Tier 4: High-Altitude Relay", 70.0, 90.0, 80.0),
}


@dataclass
class DroneState:
    """
    Public telemetry and kinematic state container conforming to PROJECT.md Contract #1.
    """
    id: str
    role: str                                    # 'SURVEY' | 'RELAY' | 'RESERVE'
    position: np.ndarray                         # shape (3,), [x, y, z] in meters
    velocity: np.ndarray                         # shape (3,), [vx, vy, vz] in m/s
    attitude: np.ndarray                         # shape (3,), [roll, pitch, yaw] in radians
    rotor_speeds: np.ndarray                     # shape (4,), [w1, w2, w3, w4] in rad/s
    battery_soc: float                           # [0.0, 1.0]
    flight_mode: str                             # 'IDLE', 'TAKEOFF', 'TRANSIT', etc.
    assigned_poi_id: Optional[str] = None        # ID of active PoI being surveyed
    target_position: Optional[np.ndarray] = None # Current 3D waypoint setpoint
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    estimated_position: Optional[np.ndarray] = None
    estimated_velocity: Optional[np.ndarray] = None
    is_manual_override: bool = False
    is_fault_injected: bool = False
    is_drafting: bool = False
    drafting_leader_id: Optional[str] = None
    drafting_saving_pct: float = 0.0

    def __post_init__(self) -> None:
        self.position = np.asarray(self.position, dtype=np.float64)
        self.velocity = np.asarray(self.velocity, dtype=np.float64)
        self.attitude = np.asarray(self.attitude, dtype=np.float64)
        self.rotor_speeds = np.asarray(self.rotor_speeds, dtype=np.float64)
        self.quaternion = np.asarray(self.quaternion, dtype=np.float64)
        self.acceleration = np.asarray(self.acceleration, dtype=np.float64)
        if self.target_position is not None:
            self.target_position = np.asarray(self.target_position, dtype=np.float64)

    def to_dict(self) -> Dict[str, Any]:
        """Serializes state into a compact dictionary for WebSocket telemetry frames."""
        role_str = self.role.value if hasattr(self.role, "value") else str(self.role)
        mode_str = self.flight_mode.value if hasattr(self.flight_mode, "value") else str(self.flight_mode)
        d = {
            "id": self.id,
            "role": role_str,
            "flight_mode": mode_str,
            "position": [round(float(v), 3) for v in self.position],
            "velocity": [round(float(v), 3) for v in self.velocity],
            "attitude": [round(float(v), 4) for v in self.attitude],
            "quaternion": [round(float(v), 4) for v in self.quaternion],
            "rotor_speeds": [round(float(w), 1) for w in self.rotor_speeds],
            "battery_soc": round(float(self.battery_soc), 4),
            "battery_pct": round(float(self.battery_soc * 100.0), 1),
            "assigned_poi_id": self.assigned_poi_id,
            "target_position": [round(float(v), 3) for v in self.target_position] if self.target_position is not None else None,
            "estimated_position": [round(float(v), 3) for v in self.estimated_position] if self.estimated_position is not None else None,
            "estimated_velocity": [round(float(v), 3) for v in self.estimated_velocity] if self.estimated_velocity is not None else None,
            "is_manual_override": bool(self.is_manual_override),
            "is_fault_injected": bool(self.is_fault_injected),
            "is_drafting": bool(self.is_drafting),
        }
        if self.is_drafting:
            d["drafting_leader_id"] = self.drafting_leader_id
            d["drafting_saving_pct"] = round(float(self.drafting_saving_pct), 1)
        return d


@dataclass
class TacticalCommsEvent:
    """Tactical radio comms chatter event displayed on visual HUD ticker and log."""
    timestamp: float
    level: str             # 'INFO' | 'WARN' | 'CRITICAL' | 'SUCCESS'
    callsign: str          # e.g. 'GCS', 'SCOUT_1', 'RELAY_3'
    message: str
    category: str = "MISSION"  # 'COMMS' | 'BATTERY' | 'SAR' | 'MISSION' | 'SWAP'

    def to_dict(self) -> Dict[str, Any]:
        return {
            "timestamp": round(float(self.timestamp), 2),
            "level": self.level,
            "callsign": self.callsign,
            "message": self.message,
            "category": self.category,
        }


@dataclass
class SurvivorRecord:
    """Simulated trapped survivor heat signature located at disaster site."""
    id: str
    poi_id: str
    position: List[float]
    heat_c: float
    confidence: float
    discovered: bool = False
    discovery_time: Optional[float] = None
    discovered_by: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "poi_id": self.poi_id,
            "position": [round(float(v), 2) for v in self.position],
            "heat_c": round(float(self.heat_c), 1),
            "confidence": round(float(self.confidence), 2),
            "discovered": bool(self.discovered),
            "discovery_time": round(float(self.discovery_time), 2) if self.discovery_time is not None else None,
            "discovered_by": self.discovered_by,
        }


@dataclass
class ScanningSector:
    """
    Geographic operational search sector allocated to a specific drone for systematic scanning.
    """
    id: str                               # e.g. "SECTOR_1_NW"
    name: str                             # e.g. "Northwest Research & Bio Sector"
    bounds: List[float]                   # [x_min, x_max, y_min, y_max, z_min, z_max]
    center: List[float]                   # [cx, cy, cz]
    assigned_drone_id: Optional[str] = None
    patrol_waypoints: List[List[float]] = field(default_factory=list)
    scan_progress: float = 0.0            # 0.0 to 100.0%
    detected_abnormalities: List[str] = field(default_factory=list)
    status: str = "PENDING"               # "PENDING" | "SCANNING" | "INSPECTING" | "CLEARED"

    def to_dict(self) -> Dict[str, Any]:
        d = {
            "id": self.id,
            "b": [int(b) for b in self.bounds[:4]],
            "s": self.status,
            "p": round(float(self.scan_progress), 1),
        }
        if self.assigned_drone_id:
            d["d"] = self.assigned_drone_id
        return d


@dataclass
class AbnormalityRecord:
    """
    Abnormality or target site identified within a drone's scanning sector.
    """
    id: str                               # e.g. "ANOMALY_COLLAPSE_01", "POI_HOSPITAL"
    sector_id: str                        # e.g. "SECTOR_5_WEST"
    type: str                             # "TARGET_SITE" | "SURVIVOR" | "STRUCTURAL_COLLAPSE" | "HAZARD_LEAK"
    position: List[float]                 # [x, y, z]
    severity: str = "HIGH"                # "CRITICAL" | "HIGH" | "MEDIUM" | "LOW"
    description: str = ""
    detected: bool = False
    detection_time: Optional[float] = None
    detected_by: Optional[str] = None
    confidence: float = 0.95
    inspected: bool = False
    inspection_dwell: float = 0.0
    required_dwell: float = 6.0

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "sector": self.sector_id,
            "type": self.type,
            "pos": [round(float(c), 1) for c in self.position],
            "severity": self.severity,
            "detected": bool(self.detected),
            "inspected": bool(self.inspected),
            "progress": round(min(100.0, (self.inspection_dwell / max(0.1, self.required_dwell)) * 100.0), 1),
        }


@dataclass
class TelemetrySnapshot:
    """
    Complete simulation state snapshot conforming to PROJECT.md Contract #4.
    Compact frame size (< 2.5 KB @ 30 Hz).
    """
    sim_time: float
    drones: List[Dict[str, Any]]
    gcs: Dict[str, Any]
    pois: List[Dict[str, Any]]
    active_routes: List[List[str]]  # e.g. [['UAV_3', 'UAV_1', 'GCS']]
    links: List[Dict[str, Any]]     # [{'source': 'UAV_3', 'target': 'UAV_1', 'snr': 18.2, 'status': 'ACTIVE'}]
    packets: List[Dict[str, Any]]   # in-flight packet traces
    metrics: Dict[str, float]       # {'pdr': 0.98, 'avg_latency_ms': 14.2, 'completed_pois': 3}
    weather: Optional[Dict[str, Any]] = None
    priority_queue: Optional[List[Dict[str, Any]]] = None
    mission_budget: Optional[Dict[str, Any]] = None
    survivors: Optional[Dict[str, Any]] = None
    tactical_comms: Optional[List[Dict[str, Any]]] = None
    charging_pads: Optional[List[Dict[str, Any]]] = None
    challenge_constraints: Optional[Dict[str, Any]] = None
    active_formation: Optional[str] = "AUTONOMOUS"
    sectors: Optional[List[Dict[str, Any]]] = None
    abnormalities: Optional[List[Dict[str, Any]]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Serializes snapshot to dictionary with Three.js cockpit compatible aliases."""
        d: Dict[str, Any] = {
            "sim_time": round(float(self.sim_time), 3),
            "drones": self.drones,
            "gcs": self.gcs,
            "pois": self.pois,
            "active_routes": self.active_routes,
            "links": self.links,
            "packets": self.packets,
            "metrics": self.metrics,
        }
        if self.weather is not None:
            d["weather"] = self.weather
        if self.priority_queue is not None:
            d["priority_queue"] = self.priority_queue
        if self.mission_budget is not None:
            d["mission_budget"] = self.mission_budget
        if self.survivors is not None:
            d["survivors"] = self.survivors
        if self.tactical_comms is not None:
            d["tactical_comms"] = self.tactical_comms
        if self.charging_pads is not None:
            d["charging_pads"] = self.charging_pads
        if self.challenge_constraints is not None:
            d["challenge_constraints"] = self.challenge_constraints
        if self.active_formation is not None and self.active_formation != "AUTONOMOUS":
            d["active_formation"] = self.active_formation
        if self.sectors is not None:
            d["sectors"] = self.sectors
        if self.abnormalities:
            d["abnormalities"] = self.abnormalities
        return TelemetryDict(d)


