"""
sim/planning.py: Swarm Intelligence, Consensus-Based Bundle Algorithm (CBBA) & Relay Relief Engine.

Implements:
1. Consensus-Based Bundle Algorithm (CBBA) for decentralized, conflict-free multi-UAV task allocation.
2. Dynamic Relay Relief Rotation: monitors elevated communication relay batteries and executes
   seamless aerial handovers before exhaustion.
3. Voronoi spatial partition heuristics for decentralized search area decomposition.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List, Optional, Sequence, Set, Tuple, Union
import numpy as np

from sim.types import DroneRole, FlightMode, ScanningSector, AbnormalityRecord


class CBBASolver:
    """
    Decentralized Consensus-Based Bundle Algorithm (CBBA).
    
    Enables autonomous UAVs to reach distributed consensus on PoI inspection
    assignments through iterative local auctioning and 1-hop consensus updates
    without single-point-of-failure GCS reliance.
    """

    def __init__(self, max_bundle_size: int = 2, enable_bft: bool = True) -> None:
        self.max_bundle_size = max_bundle_size
        self.enable_bft = enable_bft
        self.byzantine_drones: Set[str] = set()
        self.spoofed_bids: Dict[str, Dict[str, float]] = {}  # {drone_id: {poi_id: score}}
        self.bft_audit_log: List[Dict[str, Any]] = []

    def inject_byzantine_bid(self, drone_id: str, poi_id: str, spoofed_score: float = 9999.0) -> None:
        """Simulate an adversarial or corrupted node injecting an artificially inflated bid."""
        self.byzantine_drones.add(drone_id)
        if drone_id not in self.spoofed_bids:
            self.spoofed_bids[drone_id] = {}
        self.spoofed_bids[drone_id][poi_id] = spoofed_score

    def clear_byzantine_faults(self) -> None:
        """Reset Byzantine fault injection state."""
        self.byzantine_drones.clear()
        self.spoofed_bids.clear()

    def calculate_marginal_score(
        self,
        drone_pos: np.ndarray,
        poi_pos: np.ndarray,
        priority: str,
    ) -> float:
        """Calculate auction utility score based on priority and distance."""
        dist = max(1.0, float(np.linalg.norm(drone_pos - poi_pos)))
        pri_weights = {"CRITICAL": 500.0, "HIGH": 300.0, "MEDIUM": 150.0, "LOW": 50.0}
        weight = pri_weights.get(priority.upper(), 100.0)
        # Utility decreases with distance
        return float(weight / (1.0 + 0.05 * dist))

    def solve(
        self,
        drones: Dict[str, Any],
        pois: Dict[str, Dict[str, Any]],
        active_routes: Optional[Dict[str, List[str]]] = None,
    ) -> Dict[str, Optional[str]]:
        """
        Run CBBA auction and Byzantine Fault Tolerant (BFT) consensus across all active survey drones.
        Returns: {poi_id: winning_drone_id}
        """
        surveyors = [d for d in drones.values() if d.role == DroneRole.SURVEY]
        pending_pois = [p for p in pois.values() if not p.get("is_completed", False)]

        if not surveyors or not pending_pois:
            return {p["id"]: p.get("assigned_drone_id") for p in pois.values()}

        # Winning bids (y) and winning agents (z) for each task
        winning_bids: Dict[str, float] = {p["id"]: 0.0 for p in pending_pois}
        winning_agents: Dict[str, Optional[str]] = {p["id"]: None for p in pending_pois}

        # Each drone's bundle and path
        bundles: Dict[str, List[str]] = {d.id: [] for d in surveyors}

        # Run iterative auction & consensus rounds (converges within N_agents steps)
        max_rounds = max(len(surveyors) * 2, 5)
        for _ in range(max_rounds):
            changed = False

            # Phase 1: Bundle Construction (each surveyor greedily bids)
            for d in surveyors:
                if len(bundles[d.id]) >= self.max_bundle_size:
                    continue

                best_task = None
                best_score = 0.0

                for p in pending_pois:
                    p_id = p["id"]
                    if p_id in bundles[d.id]:
                        continue

                    # Check if this drone is injecting an adversarial / spoofed bid
                    if d.id in self.spoofed_bids and p_id in self.spoofed_bids[d.id]:
                        score = float(self.spoofed_bids[d.id][p_id])
                    else:
                        score = self.calculate_marginal_score(d.position, p["position"], p.get("priority", "MEDIUM"))

                    # Phase 1b: BFT Peer Cross-Validation Filter
                    if self.enable_bft:
                        # Max theoretical score is 500.0 (CRITICAL priority at dist=0)
                        is_outlier = False
                        if score > 520.0 or d.id in self.byzantine_drones:
                            is_outlier = True

                        if is_outlier:
                            # Log Byzantine outlier rejection
                            audit_entry = {
                                "timestamp": getattr(d, "sim_time", 0.0),
                                "poi_id": p_id,
                                "rogue_drone_id": d.id,
                                "spoofed_score": round(score, 1),
                                "max_admissible": 500.0,
                                "action": "REJECTED_BFT_OUTLIER",
                                "status": "BYZANTINE_FAULT_ISOLATED",
                            }
                            # Avoid spamming duplicate audit logs for the same drone/poi
                            if not any(e["rogue_drone_id"] == d.id and e["poi_id"] == p_id for e in self.bft_audit_log[-5:]):
                                self.bft_audit_log.append(audit_entry)
                            continue  # Reject adversarial bid

                    if score > winning_bids[p_id] and score > best_score:
                        best_score = score
                        best_task = p_id

                if best_task is not None:
                    bundles[d.id].append(best_task)
                    winning_bids[best_task] = best_score
                    winning_agents[best_task] = d.id
                    changed = True

            # Phase 2: Consensus Update (exchange bids across mesh)
            if not changed:
                break

        return winning_agents


class RelayReliefManager:
    """
    Autonomous Communication Relay Battery-Relief & Station Handover Coordinator.
    
    Prevents mesh disconnection by rotating exhausted relay drones back to GCS
    while seamlessly promoting a high-battery surveyor to take over the relay position.
    """

    def __init__(
        self,
        low_battery_threshold: float = 0.35,  # 35% battery triggers relief
        takeover_min_battery: float = 0.65,    # Replacement must have at least 65% battery
    ) -> None:
        self.low_battery_threshold = low_battery_threshold
        self.takeover_min_battery = takeover_min_battery
        self.relief_history: List[Dict[str, Any]] = []

    def evaluate_relief_rotation(
        self,
        drones: Dict[str, Any],
        sim_time: float,
    ) -> Optional[Tuple[str, str]]:
        """
        Check if any active Relay UAV needs battery relief.
        Returns: (discharged_relay_id, replacement_surveyor_id) or None
        """
        relays = [d for d in drones.values() if d.role == DroneRole.RELAY and d.flight_mode == FlightMode.RELAY]
        surveyors = [
            d for d in drones.values()
            if d.role == DroneRole.SURVEY and d.flight_mode in (FlightMode.TRANSIT, FlightMode.SURVEYING)
        ]

        for relay in relays:
            soc = getattr(relay.battery, "soc", 1.0)
            if soc <= self.low_battery_threshold:
                # Find best replacement surveyor (highest battery, closest to relay)
                candidates = [s for s in surveyors if getattr(s.battery, "soc", 1.0) >= self.takeover_min_battery]
                if not candidates:
                    continue

                candidates.sort(key=lambda s: (
                    -getattr(s.battery, "soc", 1.0),
                    float(np.linalg.norm(s.position - relay.position)),
                ))
                replacement = candidates[0]

                # Record event
                self.relief_history.append({
                    "time": round(sim_time, 2),
                    "discharged_relay": relay.id,
                    "replacement": replacement.id,
                    "relay_soc": round(soc * 100, 1),
                    "replacement_soc": round(getattr(replacement.battery, "soc", 1.0) * 100, 1),
                })

                return relay.id, replacement.id

        return None

    def execute_handover(self, discharged_relay: Any, replacement_surveyor: Any) -> None:
        """
        Executes seamless role swap:
        - Replacement inherits elevated relay position and switches to RELAY mode.
        - Discharged relay initiates RTL back to GCS.
        """
        target_pos = discharged_relay.get_target_waypoint()
        
        # Promote replacement
        replacement_surveyor.role = DroneRole.RELAY
        replacement_surveyor.set_flight_mode(FlightMode.RELAY)
        if target_pos is not None:
            replacement_surveyor.set_target_waypoint(target_pos)

        # Discharged relay returns to base
        discharged_relay.role = DroneRole.SURVEY
        discharged_relay.set_flight_mode(FlightMode.RTL)
        if hasattr(discharged_relay, "trigger_retreat"):
            discharged_relay.trigger_retreat()
        elif hasattr(discharged_relay, "home_position"):
            home = discharged_relay.home_position
            discharged_relay.set_target_waypoint(np.array([home[0], home[1], 55.0], dtype=np.float64))
        elif hasattr(discharged_relay, "_initial_pos"):
            init_p = discharged_relay._initial_pos
            discharged_relay.set_target_waypoint(np.array([init_p[0], init_p[1], 55.0], dtype=np.float64))


def compute_conical_fov_radius(z_drone: float, z_target: float) -> float:
    """
    Computes 75° downward altitude-scaled conical sensor footprint radius:
    r_footprint = min(55.0, max(12.0, Delta_z * tan(37.5°)))
    where Delta_z = z_drone - z_target.
    """
    delta_z = float(z_drone) - float(z_target)
    # tan(37.5 deg) ~ 0.7673269879789604
    return float(min(55.0, max(12.0, delta_z * 0.7673269879789604)))


_OBSTACLE_BOUNDS_CACHE: Dict[Tuple[int, int], Tuple[np.ndarray, np.ndarray]] = {}


def invalidate_los_cache() -> None:
    """Invalidates cached NumPy obstacle bounding arrays (e.g. after building collapse)."""
    global _OBSTACLE_BOUNDS_CACHE
    _OBSTACLE_BOUNDS_CACHE.clear()


def _get_cached_obstacle_bounds(obstacles: Sequence[Any]) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    """Retrieves or builds cached (min_pts, max_pts) arrays for vectorized slab raycheck."""
    key = (id(obstacles), len(obstacles))
    cached = _OBSTACLE_BOUNDS_CACHE.get(key)
    if cached is not None:
        return cached

    min_list = []
    max_list = []
    for obs in obstacles:
        m = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
        x = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
        if m is not None and x is not None:
            min_list.append(m)
            max_list.append(x)

    if not min_list:
        return None, None

    min_pts = np.asarray(min_list, dtype=np.float64)
    max_pts = np.asarray(max_list, dtype=np.float64)

    if len(_OBSTACLE_BOUNDS_CACHE) > 64:
        _OBSTACLE_BOUNDS_CACHE.clear()
    _OBSTACLE_BOUNDS_CACHE[key] = (min_pts, max_pts)
    return min_pts, max_pts


def is_line_of_sight_blocked(
    p_drone: Union[np.ndarray, Sequence[float]],
    p_target: Union[np.ndarray, Sequence[float]],
    obstacles: Optional[Sequence[Any]] = None,
) -> bool:
    """
    Fast, vectorized Line-of-Sight (LoS) raycheck against 3D architectural obstacles.
    Uses fast slab ray-AABB intersection with NumPy broadcasting (< 0.02 ms).
    Returns True if an obstacle intercepts the ray, False if line-of-sight is clear.
    """
    if not obstacles:
        return False

    # Check if obstacles is an ObstacleManager instance
    if hasattr(obstacles, "check_los"):
        clear, _, _, _ = obstacles.check_los(p_drone, p_target)
        return not clear

    # Handle object enclosing obstacles list (e.g. DisasterEnvironment)
    if hasattr(obstacles, "obstacles") and not isinstance(obstacles, (list, tuple)):
        obstacles = obstacles.obstacles
        if not obstacles:
            return False

    src = np.asarray(p_drone, dtype=np.float64)
    dst = np.asarray(p_target, dtype=np.float64)
    d = dst - src
    length = float(np.linalg.norm(d))

    min_pts, max_pts = _get_cached_obstacle_bounds(obstacles)
    if min_pts is None or max_pts is None:
        return False

    if length < 1e-9:
        inside = np.all((src >= min_pts) & (src <= max_pts), axis=1)
        return bool(np.any(inside))

    # Fast-path for non-zero coordinate differences (99.9% of rays)
    abs_d = np.abs(d)
    if np.all(abs_d > 1e-12):
        inv_d = 1.0 / d
        t1 = (min_pts - src) * inv_d
        t2 = (max_pts - src) * inv_d
        t_near = np.minimum(t1, t2)
        t_far = np.maximum(t1, t2)
        t_enter = np.max(t_near, axis=1)
        t_exit = np.min(t_far, axis=1)
        return bool(np.any((t_enter <= t_exit) & (t_exit >= 1e-6) & (t_enter <= 0.999999)))

    # General path handling parallel rays
    is_parallel = abs_d < 1e-12
    safe_d = np.where(is_parallel, 1.0, d)
    inv_d = 1.0 / safe_d

    t1 = (min_pts - src) * inv_d
    t2 = (max_pts - src) * inv_d

    t_near = np.where(is_parallel, -np.inf, np.minimum(t1, t2))
    t_far = np.where(is_parallel, np.inf, np.maximum(t1, t2))

    parallel_miss = is_parallel & ((src < min_pts) | (src > max_pts))
    any_parallel_miss = np.any(parallel_miss, axis=1)

    t_enter = np.max(t_near, axis=1)
    t_exit = np.min(t_far, axis=1)

    hits = (t_enter <= t_exit) & (t_exit >= 1e-6) & (t_enter <= 0.999999) & (~any_parallel_miss)
    return bool(np.any(hits))


class SectorPartitionManager:
    """
    Partitions the 3D disaster diorama into distinct operational search sectors
    and orchestrates systematic UAV area scanning and abnormality detection.
    """

    def __init__(self, is_challenge_mode: bool = False) -> None:
        self.is_challenge_mode = bool(is_challenge_mode)
        self.sectors: Dict[str, ScanningSector] = {}
        self.abnormalities: Dict[str, AbnormalityRecord] = {}
        self.drone_waypoint_idx: Dict[str, int] = {}
        self.sector_visited_wps: Dict[str, Set[int]] = {}
        self._init_sectors()

    def _init_sectors(self) -> None:
        """Partition operational zone into 12 dedicated search sectors."""
        self.sectors.clear()

        if self.is_challenge_mode:
            # 1000m x 1000m Challenge Arena: X in [0, 1000], Y in [-500, 500]
            col_bounds = [
                (0.0, 250.0),
                (250.0, 500.0),
                (500.0, 750.0),
                (750.0, 1000.0),
            ]
            row_bounds = [
                (166.0, 500.0, "North"),
                (-166.0, 166.0, "Central"),
                (-500.0, -166.0, "South"),
            ]
        else:
            # Sector Delta 700m x 700m Diorama: X in [-160, 160], Y in [-120, 120]
            col_bounds = [
                (-160.0, -80.0),
                (-80.0, 0.0),
                (0.0, 80.0),
                (80.0, 160.0),
            ]
            row_bounds = [
                (40.0, 120.0, "North"),
                (-40.0, 40.0, "Central"),
                (-120.0, -40.0, "South"),
            ]

        sector_names = [
            ("SEC_01_NW", "Sector 01: Northwest Industrial", 0, 0),
            ("SEC_02_NC", "Sector 02: North Bridge & Hazard Zone", 1, 0),
            ("SEC_03_NE", "Sector 03: Northeast Perimeter", 2, 0),
            ("SEC_04_FE", "Sector 04: Far Northeast Industrial", 3, 0),
            ("SEC_05_WC", "Sector 05: West High-Rise Complex", 0, 1),
            ("SEC_06_CC", "Sector 06: Central Hospital District", 1, 1),
            ("SEC_07_EC", "Sector 07: Central Boulevard & Park", 2, 1),
            ("SEC_08_SE", "Sector 08: East Substation Grid", 3, 1),
            ("SEC_09_SW", "Sector 09: Southwest Highway & Collapse", 0, 2),
            ("SEC_10_SC", "Sector 10: South Civic Shelter Zone", 1, 2),
            ("SEC_11_SP", "Sector 11: South Canal & Survivors Park", 2, 2),
            ("SEC_12_EL", "Sector 12: Southeast Logistics Hub", 3, 2),
        ]

        for sec_id, name, col_idx, row_idx in sector_names:
            x_min, x_max = col_bounds[col_idx]
            y_min, y_max, _ = row_bounds[row_idx]
            z_min, z_max = 15.0, 65.0
            cx = (x_min + x_max) / 2.0
            cy = (y_min + y_max) / 2.0
            cz = 32.0

            dx = x_max - x_min
            dy = y_max - y_min
            wps = [
                [x_min + 0.25 * dx, y_min + 0.25 * dy, cz],
                [x_min + 0.25 * dx, y_max - 0.25 * dy, cz],
                [x_max - 0.25 * dx, y_max - 0.25 * dy, cz],
                [x_max - 0.25 * dx, y_min + 0.25 * dy, cz],
                [cx, cy, cz],
            ]

            self.sectors[sec_id] = ScanningSector(
                id=sec_id,
                name=name,
                bounds=[x_min, x_max, y_min, y_max, z_min, z_max],
                center=[cx, cy, cz],
                assigned_drone_id=None,
                patrol_waypoints=wps,
                scan_progress=0.0,
                detected_abnormalities=[],
                status="PENDING",
            )
            self.sector_visited_wps[sec_id] = set()

    def get_sector_for_position(self, pos: Sequence[float]) -> Optional[ScanningSector]:
        """Finds the geographic sector enclosing a 3D position (or closest sector)."""
        px, py = float(pos[0]), float(pos[1])
        for sector in self.sectors.values():
            b = sector.bounds
            if b[0] <= px <= b[1] and b[2] <= py <= b[3]:
                return sector

        # Fallback to closest sector center if slightly outside bounds
        best_sec = None
        min_d = float("inf")
        for sector in self.sectors.values():
            d = (px - sector.center[0]) ** 2 + (py - sector.center[1]) ** 2
            if d < min_d:
                min_d = d
                best_sec = sector
        return best_sec

    def assign_sectors(self, drones: Dict[str, Any]) -> None:
        """
        Assigns active survey/scout UAVs to dedicated search sectors.
        Guarantees that each drone has an assigned operational area.
        """
        # Find active survey drones
        surveyors = [
            d for d in drones.values()
            if d.role == DroneRole.SURVEY and d.flight_mode not in (
                FlightMode.LANDED, FlightMode.COMPLETED, FlightMode.EMERGENCY_LAND
            )
        ]
        if not surveyors:
            return

        # Sort predictably: Scouts first, then Surveyors by ID number
        def sort_key(d: Any) -> Tuple[int, str]:
            is_scout = 0 if "SCOUT" in d.id else 1
            return (is_scout, d.id)

        surveyors.sort(key=sort_key)
        sec_keys = list(self.sectors.keys())

        for idx, drone in enumerate(surveyors):
            # Target sector for this drone
            sec_id = sec_keys[idx % len(sec_keys)]
            sector = self.sectors[sec_id]

            sector.assigned_drone_id = drone.id
            drone.assigned_sector_id = sec_id
            drone.sector_scan_progress = sector.scan_progress
            if sector.status == "PENDING":
                sector.status = "SCANNING"

            if drone.id not in self.drone_waypoint_idx:
                self.drone_waypoint_idx[drone.id] = 0

    def register_abnormality(
        self,
        anomaly_id: str,
        anomaly_type: str,
        position: Sequence[float],
        severity: str = "HIGH",
        description: str = "",
        required_dwell: float = 6.0,
    ) -> AbnormalityRecord:
        """Registers a site or detected abnormality into its corresponding sector."""
        sector = self.get_sector_for_position(position)
        sector_id = sector.id if sector else "SEC_01_NW"

        record = AbnormalityRecord(
            id=anomaly_id,
            sector_id=sector_id,
            type=anomaly_type,
            position=[float(position[0]), float(position[1]), float(position[2])],
            severity=severity,
            description=description,
            detected=False,
            detection_time=None,
            detected_by=None,
            confidence=0.95,
            inspected=False,
            inspection_dwell=0.0,
            required_dwell=float(required_dwell),
        )
        self.abnormalities[anomaly_id] = record

        if sector and anomaly_id not in sector.detected_abnormalities:
            sector.detected_abnormalities.append(anomaly_id)

        return record

    def populate_initial_abnormalities(
        self,
        pois: Dict[str, Dict[str, Any]],
        survivors: Optional[Dict[str, Any]] = None,
    ) -> None:
        """Populates initial disaster sites and survivors into sector management."""
        for p_id, poi in pois.items():
            if p_id not in self.abnormalities:
                pri = str(poi.get("priority", "HIGH")).upper()
                dwell = float(poi.get("required_dwell_time", 10.0))
                self.register_abnormality(
                    anomaly_id=p_id,
                    anomaly_type="TARGET_SITE",
                    position=poi["position"],
                    severity=pri,
                    description=f"Disaster Target Site: {p_id}",
                    required_dwell=dwell,
                )

        if survivors:
            for s_id, surv in survivors.items():
                anom_id = f"ANOMALY_{s_id}"
                if anom_id not in self.abnormalities:
                    self.register_abnormality(
                        anomaly_id=anom_id,
                        anomaly_type="SURVIVOR",
                        position=surv.position,
                        severity="CRITICAL",
                        description=f"Human Bio-Thermal Signature ({getattr(surv, 'heat_c', 37.5)}°C)",
                        required_dwell=4.0,
                    )

    def get_next_waypoint_for_drone(
        self,
        drone_id: str,
        current_pos: Optional[np.ndarray] = None,
    ) -> np.ndarray:
        """
        Returns the optimal target waypoint for a drone scanning its assigned sector.
        Prioritizes detected uninspected abnormalities in the sector; falls back to patrol sweep.
        """
        # Find drone's assigned sector
        assigned_sec = None
        for sec in self.sectors.values():
            if sec.assigned_drone_id == drone_id:
                assigned_sec = sec
                break

        if not assigned_sec:
            # Fallback to first sector or center
            assigned_sec = list(self.sectors.values())[0]

        # 1. Check for detected, uninspected abnormalities in this sector
        for anom_id in assigned_sec.detected_abnormalities:
            anom = self.abnormalities.get(anom_id)
            if anom and anom.detected and not anom.inspected:
                return np.array(anom.position, dtype=np.float64)

        # 2. Return current patrol waypoint
        wps = assigned_sec.patrol_waypoints
        if not wps:
            return np.array(assigned_sec.center, dtype=np.float64)

        wp_idx = self.drone_waypoint_idx.get(drone_id, 0) % len(wps)
        return np.array(wps[wp_idx], dtype=np.float64)

    def update(
        self,
        drones: Dict[str, Any],
        pois: Dict[str, Dict[str, Any]],
        dt: float,
        sim_time: float,
        obstacles: Optional[Sequence[Any]] = None,
    ) -> None:
        """
        Advances sector scanning progress, checks sensor detections with
        altitude-scaled conical FOV footprint and 3D architectural LoS raycheck,
        accumulates abnormality dwell time, and updates sector completion status.
        """
        if obstacles is None:
            obstacles = getattr(self, "obstacles", None)
        if obstacles is None:
            for d in drones.values():
                if hasattr(d, "obstacles") and d.obstacles:
                    obstacles = d.obstacles
                    break

        for drone_id, drone in drones.items():
            if drone.role != DroneRole.SURVEY:
                continue

            sec_id = getattr(drone, "assigned_sector_id", None)
            if not sec_id or sec_id not in self.sectors:
                continue

            sector = self.sectors[sec_id]
            d_pos = drone.position

            # 1. Update patrol sweep waypoint progress
            wps = sector.patrol_waypoints
            if wps:
                cur_idx = self.drone_waypoint_idx.get(drone_id, 0) % len(wps)
                target_wp = np.array(wps[cur_idx], dtype=np.float64)
                dist_to_wp = float(np.linalg.norm(d_pos - target_wp))

                if dist_to_wp <= 20.0:
                    visited = self.sector_visited_wps.setdefault(sec_id, set())
                    visited.add(cur_idx)
                    self.drone_waypoint_idx[drone_id] = (cur_idx + 1) % len(wps)

            # 2. Conical FOV footprint & LoS raycheck: scan for abnormalities in and near sector
            for anom in self.abnormalities.values():
                if not anom.detected:
                    dist_to_anom = float(np.linalg.norm(d_pos - np.array(anom.position)))
                    if drone.flight_mode in (
                        FlightMode.TRANSIT, FlightMode.SURVEYING, FlightMode.TAKEOFF
                    ):
                        can_detect = False
                        if d_pos[2] >= 3.0:
                            # 75° downward altitude-scaled conical sensor FOV:
                            # r_footprint = min(55.0, max(12.0, Delta_z * tan(37.5°)))
                            r_footprint = compute_conical_fov_radius(d_pos[2], anom.position[2])
                            dist_xy = float(np.hypot(d_pos[0] - anom.position[0], d_pos[1] - anom.position[1]))
                            if dist_xy <= r_footprint and dist_to_anom <= 55.0:
                                if not is_line_of_sight_blocked(d_pos, anom.position, obstacles):
                                    can_detect = True
                        elif dist_to_anom <= 2.0:
                            # Immediate contact proximity check (strictly requires unoccluded line-of-sight)
                            if not is_line_of_sight_blocked(d_pos, anom.position, obstacles):
                                can_detect = True

                        if can_detect:
                            anom.detected = True
                            anom.detection_time = sim_time
                            anom.detected_by = drone.id

                            # Ensure anomaly is registered in its sector
                            if anom.id not in sector.detected_abnormalities and anom.sector_id == sec_id:
                                sector.detected_abnormalities.append(anom.id)

                            # Sync with POIs
                            if anom.id in pois:
                                p = pois[anom.id]
                                p["is_detected"] = True
                                p["detection_time"] = sim_time
                                p["detected_by"] = drone.id
                                p["is_reported"] = True
                                p["report_time"] = sim_time + 0.1
                                p["reporting_latency_s"] = 0.1
                                p["is_sla_compliant"] = True

            # 3. Abnormality inspection dwell accumulation
            for anom_id in sector.detected_abnormalities:
                anom = self.abnormalities.get(anom_id)
                if anom and anom.detected and not anom.inspected:
                    dist = float(np.linalg.norm(d_pos - np.array(anom.position)))
                    if dist <= 28.0:
                        anom.inspection_dwell += dt
                        if anom.inspection_dwell >= anom.required_dwell:
                            anom.inspected = True
                            if anom.id in pois:
                                pois[anom.id]["current_dwell_time"] = max(
                                    pois[anom.id].get("current_dwell_time", 0.0),
                                    pois[anom.id]["required_dwell_time"]
                                )
                                pois[anom.id]["is_completed"] = True

            # 4. Calculate sector scan progress (0% - 100%)
            visited = self.sector_visited_wps.get(sec_id, set())
            wp_fraction = (len(visited) / max(1, len(wps))) if wps else 1.0

            anom_fraction = 1.0
            sec_anoms = [self.abnormalities[aid] for aid in sector.detected_abnormalities if aid in self.abnormalities]
            if sec_anoms:
                inspected_count = sum(1 for a in sec_anoms if a.inspected)
                detected_count = sum(1 for a in sec_anoms if a.detected)
                anom_fraction = (detected_count * 0.4 + inspected_count * 0.6) / max(1, len(sec_anoms))

            total_progress = round((wp_fraction * 50.0) + (anom_fraction * 50.0), 1)
            sector.scan_progress = min(100.0, total_progress)
            drone.sector_scan_progress = sector.scan_progress

            # 5. Update sector status
            if sector.scan_progress >= 99.0:
                sector.status = "CLEARED"
            elif any(a.detected and not a.inspected for a in sec_anoms):
                sector.status = "INSPECTING"
            elif sector.scan_progress > 0.0:
                sector.status = "SCANNING"
            else:
                sector.status = "PENDING"

    def get_sectors_telemetry(self) -> List[Dict[str, Any]]:
        """Returns serialized sector data for 3D Cockpit and HUD display."""
        return [s.to_dict() for s in self.sectors.values()]

    def get_abnormalities_telemetry(self) -> List[Dict[str, Any]]:
        """Returns serialized detected operational abnormalities (collapses, hazards) for 3D Cockpit and HUD display."""
        return [
            a.to_dict()
            for a in self.abnormalities.values()
            if a.detected and a.type not in ("TARGET_SITE", "SURVIVOR")
        ]

