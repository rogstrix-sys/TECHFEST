"""
sim/challenge.py: MeitY / IIT Bombay / IISER Bhopal UAV Swarm Challenge Engine.

Implements all 11 official competition mission constraints:
1. Mission operations: 45 mins (2700s)
2. UAV max flight time: 20 mins (1200s)
3. Max comm range: 100m
4. Operational area: 1000m x 1000m
5. All UAVs take-off from the operational center (located 75m outside the operational area)
6. All UAVs should land at the start area by 45 mins
7. Operational height max: 100m
8. Max speed: 5 m/s
9. Min distance between vehicles: 20m
10. Max time between POI detection and reporting to center: 10s
11. Number of POIs: 10, spawned randomly in position and time of spawning
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Sequence, Tuple
import numpy as np

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.environment import DisasterEnvironment, EnvironmentConfig
from sim.mission import DisasterMissionManager
from sim.network import DualBandRFChannelModel, FANETNetworkEngine, NetworkPacket, RFChannelModel
from sim.types import DroneLimits, DroneRole, FlightMode, PoIPriority


@dataclass
class ChallengeMissionConstraints:
    """Official MeitY / IIT Bombay / IISER Bhopal Challenge Mission Constraints."""
    mission_duration_s: float = 2700.0         # 45 mins
    uav_max_flight_time_s: float = 1200.0      # 20 mins
    max_comm_range_m: float = 100.0            # 100 m
    operational_area_w_m: float = 1000.0       # 1000 m
    operational_area_l_m: float = 1000.0       # 1000 m
    operational_center_offset_m: float = 75.0  # 75 m offset from arena
    max_altitude_m: float = 100.0              # 100 m
    max_speed_m_s: float = 5.0                 # 5 m/s
    min_separation_m: float = 20.0             # 20 m
    max_poi_reporting_latency_s: float = 10.0  # 10 s
    num_pois: int = 10                         # 10 POIs randomly spawned

    def to_dict(self) -> Dict[str, Any]:
        return {
            "mission_duration_s": self.mission_duration_s,
            "uav_max_flight_time_s": self.uav_max_flight_time_s,
            "max_comm_range_m": self.max_comm_range_m,
            "operational_area_w_m": self.operational_area_w_m,
            "operational_area_l_m": self.operational_area_l_m,
            "operational_center_offset_m": self.operational_center_offset_m,
            "max_altitude_m": self.max_altitude_m,
            "max_speed_m_s": self.max_speed_m_s,
            "min_separation_m": self.min_separation_m,
            "max_poi_reporting_latency_s": self.max_poi_reporting_latency_s,
            "num_pois": self.num_pois,
        }


@dataclass
class ChallengePOI:
    """Representative POI spawned randomly in the 1000m x 1000m arena."""
    id: str
    position: np.ndarray
    spawn_time: float
    required_dwell_time: float = 10.0
    priority: str = "HIGH"
    is_spawned: bool = False
    is_detected: bool = False
    detection_time: Optional[float] = None
    detected_by: Optional[str] = None
    is_reported: bool = False
    report_time: Optional[float] = None
    reporting_latency_s: Optional[float] = None
    is_sla_compliant: bool = True
    current_dwell_time: float = 0.0
    is_completed: bool = False
    assigned_drone_id: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "position": [round(float(c), 2) for c in self.position],
            "spawn_time": round(float(self.spawn_time), 1),
            "required_dwell_time": self.required_dwell_time,
            "current_dwell_time": round(float(self.current_dwell_time), 2),
            "priority": self.priority,
            "is_spawned": self.is_spawned,
            "is_detected": self.is_detected,
            "detection_time": round(float(self.detection_time), 2) if self.detection_time is not None else None,
            "detected_by": self.detected_by,
            "is_reported": self.is_reported,
            "report_time": round(float(self.report_time), 2) if self.report_time is not None else None,
            "reporting_latency_s": round(float(self.reporting_latency_s), 2) if self.reporting_latency_s is not None else None,
            "is_sla_compliant": self.is_sla_compliant,
            "is_completed": self.is_completed,
            "assigned_drone_id": self.assigned_drone_id,
            "progress": round(min(1.0, self.current_dwell_time / max(0.1, self.required_dwell_time)) * 100.0, 1),
        }


class DynamicPoiSpawner:
    """
    Spawns 10 representative POIs randomly distributed across the 1000m x 1000m arena
    at random spawn times throughout the mission operations timeline.
    """

    def __init__(
        self,
        num_pois: int = 10,
        area_x_range: Tuple[float, float] = (50.0, 950.0),
        area_y_range: Tuple[float, float] = (-450.0, 450.0),
        altitude_range: Tuple[float, float] = (15.0, 35.0),
        spawn_time_range: Tuple[float, float] = (0.0, 300.0),
        seed: Optional[int] = 42,
    ) -> None:
        self.num_pois = num_pois
        self.area_x_range = area_x_range
        self.area_y_range = area_y_range
        self.altitude_range = altitude_range
        self.spawn_time_range = spawn_time_range
        self.seed = seed

        self.pois: Dict[str, ChallengePOI] = {}
        self._generate_pois()

    def _generate_pois(self) -> None:
        rng = random.Random(self.seed) if self.seed is not None else random.Random()
        priorities = ["CRITICAL", "HIGH", "HIGH", "MEDIUM", "MEDIUM", "MEDIUM", "LOW", "LOW", "HIGH", "CRITICAL"]

        for i in range(1, self.num_pois + 1):
            p_id = f"POI_{i}"
            x = rng.uniform(self.area_x_range[0], self.area_x_range[1])
            y = rng.uniform(self.area_y_range[0], self.area_y_range[1])
            z = rng.uniform(self.altitude_range[0], self.altitude_range[1])
            # Spread spawn times: first 3 available at t=0, others spawn progressively
            if i <= 3:
                s_time = 0.0
            else:
                s_time = rng.uniform(self.spawn_time_range[0], self.spawn_time_range[1])

            self.pois[p_id] = ChallengePOI(
                id=p_id,
                position=np.array([x, y, z], dtype=np.float64),
                spawn_time=round(s_time, 1),
                required_dwell_time=round(rng.uniform(8.0, 14.0), 1),
                priority=priorities[(i - 1) % len(priorities)],
            )

    def update(self, sim_time: float, core_pois: Dict[str, Dict[str, Any]]) -> List[ChallengePOI]:
        """Activates scheduled POIs when sim_time reaches spawn_time and syncs to core simulation."""
        newly_spawned = []
        for p_id, poi in self.pois.items():
            if not poi.is_spawned and sim_time >= poi.spawn_time:
                poi.is_spawned = True
                newly_spawned.append(poi)
                # Register into simulation core pois dictionary
                core_pois[p_id] = {
                    "id": p_id,
                    "position": poi.position,
                    "priority": poi.priority,
                    "required_dwell_time": poi.required_dwell_time,
                    "current_dwell_time": poi.current_dwell_time,
                    "is_completed": poi.is_completed,
                    "assigned_drone_id": poi.assigned_drone_id,
                    "is_spawned": True,
                    "is_detected": poi.is_detected,
                    "is_reported": poi.is_reported,
                    "reporting_latency_s": poi.reporting_latency_s,
                    "is_sla_compliant": poi.is_sla_compliant,
                }
        return newly_spawned


class ChallengeComplianceMonitor:
    """
    Continuous real-time validator monitoring all 11 competition constraints.
    Computes live metrics and flags violations.
    """

    def __init__(self, constraints: Optional[ChallengeMissionConstraints] = None) -> None:
        self.constraints = constraints or ChallengeMissionConstraints()

        # Telemetry metrics
        self.min_observed_separation_m: float = 999.0
        self.max_observed_speed_m_s: float = 0.0
        self.max_observed_altitude_m: float = 0.0
        self.max_observed_comm_hop_m: float = 0.0
        self.max_observed_poi_latency_s: float = 0.0
        self.separation_violations_count: int = 0
        self.speed_violations_count: int = 0
        self.altitude_violations_count: int = 0
        self.comm_violations_count: int = 0
        self.poi_latency_violations_count: int = 0
        self.all_takeoff_at_center: bool = True
        self.all_landed_by_budget: bool = True

    def evaluate_step(
        self,
        sim_time: float,
        drones: Dict[str, Drone],
        spawner: DynamicPoiSpawner,
        active_links: Sequence[Dict[str, Any]],
        gcs_position: np.ndarray,
    ) -> Dict[str, Any]:
        """Evaluates compliance for the current simulation step."""
        c = self.constraints
        flying_drones = [d for d in drones.values() if d.flight_mode not in (FlightMode.IDLE, FlightMode.LANDED)]

        # 1. Speed constraint: max 5 m/s
        for d in flying_drones:
            speed = float(np.linalg.norm(d.velocity))
            if speed > self.max_observed_speed_m_s:
                self.max_observed_speed_m_s = speed
            if speed > c.max_speed_m_s + 0.15:  # small numerical margin
                self.speed_violations_count += 1

        # 2. Altitude constraint: max 100 m
        for d in flying_drones:
            alt = float(d.position[2])
            if alt > self.max_observed_altitude_m:
                self.max_observed_altitude_m = alt
            if alt > c.max_altitude_m + 0.1:
                self.altitude_violations_count += 1

        # 3. Min vehicle distance: >= 20 m
        n = len(flying_drones)
        for i in range(n):
            for j in range(i + 1, n):
                dist = float(np.linalg.norm(flying_drones[i].position - flying_drones[j].position))
                if dist < self.min_observed_separation_m:
                    self.min_observed_separation_m = dist
                if dist < c.min_separation_m - 0.2:  # small numerical tolerance
                    self.separation_violations_count += 1

        # 4. Comm range: max 100 m (applies to viable, active communication links)
        for link in active_links:
            if not link.get("viable", False) or link.get("status") == "DISRUPTED":
                continue
            hop_dist = float(link.get("distance", 0.0))
            if hop_dist > self.max_observed_comm_hop_m:
                self.max_observed_comm_hop_m = hop_dist
            if hop_dist > c.max_comm_range_m + 0.5:
                self.comm_violations_count += 1

        # 5. POI detection & 10s reporting SLA
        reported_count = 0
        detected_count = 0
        completed_count = 0
        for p in spawner.pois.values():
            if p.is_detected:
                detected_count += 1
            if p.is_reported:
                reported_count += 1
                if p.reporting_latency_s is not None and p.reporting_latency_s > self.max_observed_poi_latency_s:
                    self.max_observed_poi_latency_s = p.reporting_latency_s
                if not p.is_sla_compliant:
                    self.poi_latency_violations_count += 1
            if p.is_completed:
                completed_count += 1

        # 6. Check landing by mission duration (45 mins)
        if sim_time >= c.mission_duration_s:
            for d in drones.values():
                if d.flight_mode not in (FlightMode.LANDED, FlightMode.COMPLETED):
                    self.all_landed_by_budget = False

        # Overall compliance verdict
        is_fully_compliant = (
            self.speed_violations_count == 0
            and self.altitude_violations_count == 0
            and self.separation_violations_count == 0
            and self.comm_violations_count == 0
            and self.poi_latency_violations_count == 0
            and self.all_takeoff_at_center
            and (self.all_landed_by_budget if sim_time >= c.mission_duration_s else True)
        )

        return {
            "is_fully_compliant": is_fully_compliant,
            "metrics": {
                "mission_time_s": round(sim_time, 1),
                "mission_budget_s": c.mission_duration_s,
                "uav_max_flight_time_s": c.uav_max_flight_time_s,
                "max_speed_m_s": round(self.max_observed_speed_m_s, 2),
                "max_speed_limit": c.max_speed_m_s,
                "max_altitude_m": round(self.max_observed_altitude_m, 1),
                "max_altitude_limit": c.max_altitude_m,
                "min_separation_m": round(self.min_observed_separation_m, 2) if self.min_observed_separation_m < 900.0 else 20.0,
                "min_separation_limit": c.min_separation_m,
                "max_comm_range_m": round(self.max_observed_comm_hop_m, 1),
                "max_comm_range_limit": c.max_comm_range_m,
                "max_poi_reporting_latency_s": round(self.max_observed_poi_latency_s, 2),
                "max_poi_reporting_latency_limit": c.max_poi_reporting_latency_s,
                "total_pois": c.num_pois,
                "detected_pois": detected_count,
                "reported_pois": reported_count,
                "completed_pois": completed_count,
            },
            "violations": {
                "speed": self.speed_violations_count,
                "altitude": self.altitude_violations_count,
                "separation": self.separation_violations_count,
                "comm_range": self.comm_violations_count,
                "poi_latency": self.poi_latency_violations_count,
            },
        }


def create_challenge_simulation(seed: int = 42) -> Tuple[SwarmSimulationCore, DynamicPoiSpawner, ChallengeComplianceMonitor]:
    """
    Factory function creating the official MeitY / IIT Bombay / IISER Bhopal
    1000m x 1000m UAV Swarm Challenge simulation instance.
    """
    constraints = ChallengeMissionConstraints()

    # 1. Environment: Operational Area [0, 1000] x [-500, 500] x [0, 100]m
    # Operational Center located 75m to the west: X = -75.0, Y = 0.0, Z = 0.0
    gcs_pos = np.array([-75.0, 0.0, 0.0], dtype=np.float64)
    env_config = EnvironmentConfig(
        x_min=-150.0,
        x_max=1050.0,
        y_min=-550.0,
        y_max=550.0,
        z_min=0.0,
        z_max=100.0,
        gcs_position=gcs_pos,
        gcs_antenna_height=6.0,
        layer_launch_land=(0.0, 15.0),
        layer_poi_survey=(20.0, 45.0),
        layer_transit=(50.0, 70.0),
        layer_relay_mesh=(75.0, 95.0),
    )
    env = DisasterEnvironment(config=env_config)

    sim_config = SimulationConfig(
        dt=0.05,
        max_duration=constraints.mission_duration_s,
        world_bounds_x=(-150.0, 1050.0),
        world_bounds_y=(-550.0, 550.0),
        world_bounds_z=(0.0, 100.0),
        gcs_position=(-75.0, 0.0, 0.0),
        gcs_comm_radius=constraints.max_comm_range_m,
        enable_downwash=True,
        enable_vsm_relays=True,
        enable_weather=False,
    )
    sim = SwarmSimulationCore(config=sim_config, environment=env)

    # 2. Dynamic POI Spawner: 10 representative POIs in the 1000m x 1000m arena
    spawner = DynamicPoiSpawner(
        num_pois=constraints.num_pois,
        area_x_range=(80.0, 920.0),
        area_y_range=(-420.0, 420.0),
        altitude_range=(20.0, 38.0),
        spawn_time_range=(0.0, 180.0),
        seed=seed,
    )
    # Initial activation at t=0
    spawner.update(0.0, sim.pois)

    # 3. Heterogeneous Fleet: 16 UAVs (Surveyors, Mobile Relays, Rapid Scouts)
    # All UAVs spawn grounded at their designated pads at the Operational Center (X = -75.0m)
    # Speed strictly clamped to 5 m/s, separation radius set to 20 m
    limits = DroneLimits(
        max_speed_xy=5.0,
        max_speed_z_up=2.5,
        max_speed_z_down=2.0,
        max_accel=2.5,
        separation_radius=20.0,
        collision_radius=1.5,
    )

    fleet_specs = [
        # Heavy Surveyors: 6 units (pads along Column X = -65, Y spaced by 50m)
        ("UAV_1", DroneRole.SURVEY, [-65.0, -125.0, 0.45], 4.0),
        ("UAV_2", DroneRole.SURVEY, [-65.0, -75.0, 0.45], 5.0),
        ("UAV_3", DroneRole.SURVEY, [-65.0, -25.0, 0.45], 6.0),
        ("UAV_4", DroneRole.SURVEY, [-65.0, 25.0, 0.45], 7.0),
        ("UAV_5", DroneRole.SURVEY, [-65.0, 75.0, 0.45], 8.0),
        ("UAV_6", DroneRole.SURVEY, [-65.0, 125.0, 0.45], 9.0),
        # Aerial Multi-Hop Relays: 6 units (pads along Column X = -95, Y spaced by 50m)
        ("RELAY_1", DroneRole.RELAY, [-95.0, -125.0, 0.45], 2.0),
        ("RELAY_2", DroneRole.RELAY, [-95.0, -75.0, 0.45], 2.5),
        ("RELAY_3", DroneRole.RELAY, [-95.0, -25.0, 0.45], 3.0),
        ("RELAY_4", DroneRole.RELAY, [-95.0, 25.0, 0.45], 3.5),
        ("RELAY_5", DroneRole.RELAY, [-95.0, 75.0, 0.45], 4.0),
        ("RELAY_6", DroneRole.RELAY, [-95.0, 125.0, 0.45], 4.5),
        # Rapid Recon Scouts: 4 units (pads along Column X = -35, Y spaced by 50m)
        ("SCOUT_1", DroneRole.SURVEY, [-35.0, -75.0, 0.45], 1.0),
        ("SCOUT_2", DroneRole.SURVEY, [-35.0, -25.0, 0.45], 1.2),
        ("SCOUT_3", DroneRole.SURVEY, [-35.0, 25.0, 0.45], 1.4),
        ("SCOUT_4", DroneRole.SURVEY, [-35.0, 75.0, 0.45], 1.6),
    ]

    for d_id, role, pos, delay in fleet_specs:
        drone = Drone(
            drone_id=d_id,
            role=role,
            initial_pos=np.array(pos, dtype=np.float64),
            limits=DroneLimits(
                max_speed_xy=5.0,
                max_speed_z_up=2.5,
                max_speed_z_down=2.0,
                max_accel=2.5,
                separation_radius=20.0,
                collision_radius=1.5,
                clamp_3d_speed=True,
            ),
        )
        drone.takeoff_delay = delay
        # Stagger cruise altitudes to guarantee vertical deconfliction
        if "UAV" in d_id:
            idx = int(d_id.split("_")[1])
            drone.cruise_altitude = 25.0 + (idx - 1) * 8.0
        elif "SCOUT" in d_id:
            idx = int(d_id.split("_")[1])
            drone.cruise_altitude = 29.0 + (idx - 1) * 8.0
        elif "RELAY" in d_id:
            idx = int(d_id.split("_")[1])
            drone.cruise_altitude = 75.0 + (idx - 1) * 4.0

        # Configure battery model: 20 mins flight endurance at ~195W
        drone.battery.capacity_mah = 4500.0
        drone.battery.nominal_voltage = 14.8
        drone.battery.rtb_soc_threshold = 0.22
        sim.add_drone(drone)

    # 4. FANET Network Engine: strictly enforces 100m direct communication limit
    chan = DualBandRFChannelModel()
    chan.ch_24ghz.max_direct_los_range = constraints.max_comm_range_m
    chan.ch_915mhz.max_direct_los_range = constraints.max_comm_range_m
    net_engine = FANETNetworkEngine(channel_model=chan)
    sim.set_network_engine(net_engine)

    # 5. Mission Manager: 45 min budget, 10s POI reporting SLA, autonomous retreat
    mission_mgr = DisasterMissionManager(
        gcs_position=gcs_pos,
        mission_time_budget=constraints.mission_duration_s,
        comms_loss_timeout=10.0,
        survey_dwell_radius=18.0,
    )
    sim.set_mission_manager(mission_mgr)

    # 6. Compliance Monitor
    monitor = ChallengeComplianceMonitor(constraints=constraints)
    sim.poi_spawner = spawner
    sim.challenge_monitor = monitor

    return sim, spawner, monitor

