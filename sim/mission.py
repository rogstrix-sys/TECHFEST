"""
sim/mission.py: Disaster PoI Survey Mission Control & Dynamic Role Allocation.

Implements:
1. Multi-priority Disaster Points of Interest (PoI) management (Survivor Search, Structural Collapse, Hazard Zone).
2. Autonomous MAVSDK-compliant Finite State Machine (FSM) transitions:
   TAKEOFF -> TRANSIT -> SURVEYING -> RELAY -> RTL -> LANDING -> LANDED -> COMPLETED.
3. Dynamic Fleet Role Allocation:
   - Survey UAVs: Dispatched to inspect high-priority PoIs and stream reconnaissance telemetry.
   - Relay UAVs: Deployed as elevated aerial communication bridges linking surveyors to GCS.
4. Comprehensive Autonomous Retreat (RTH/RTL) & Controlled Landing:
   - Triggered when a drone completes its assigned survey and no pending PoIs remain.
   - Triggered when all surveillance area / disaster PoIs have been covered.
   - Triggered when battery drops below RTB threshold or dynamic Return-to-Base energy reserve.
   - Failsafe immediate emergency landing when battery reaches critical threshold.
   - Safe cruise at Tier 3 obstacle clearance altitude (55m) followed by controlled descent and touchdown.
5. Multi-drone recovery pad deconfliction and fleet-wide retreat coordination.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union
import numpy as np

from sim.types import DroneRole, FlightMode, TacticalCommsEvent, SurvivorRecord


class DisasterSitePriorityQueue:
    """
    Priority Queue for Disaster Sites with deadline urgency scaling.
    Prioritizes:
    CRITICAL (Survivor Search, Hospital) > HIGH (Bridge Collapse, Substation) > MEDIUM > LOW
    and dynamically factors in mission deadline to prevent site starvation.
    """
    PRIORITY_WEIGHTS = {
        "CRITICAL": 1000.0,
        "HIGH": 500.0,
        "MEDIUM": 200.0,
        "LOW": 50.0,
    }

    def __init__(self) -> None:
        self.queue: List[Dict[str, Any]] = []

    def update_queue(
        self,
        pois: Dict[str, Dict[str, Any]],
        current_time: float,
        time_budget: float,
        drones: Optional[Dict[str, Any]] = None,
    ) -> List[Dict[str, Any]]:
        """Sorts pending disaster sites by priority weight, deadline urgency, and completion status."""
        pending = [p for p in pois.values() if not p.get("is_completed", False)]
        time_remaining = max(0.1, time_budget - current_time)
        urgency_factor = max(1.0, 1.0 + (current_time / time_remaining))

        def scoring_fn(p: Dict[str, Any]) -> float:
            base_w = self.PRIORITY_WEIGHTS.get(str(p.get("priority", "MEDIUM")).upper(), 200.0)
            req_dwell = float(p.get("required_dwell_time", 10.0))
            score = (base_w * urgency_factor) - (req_dwell * 2.0)
            if p.get("assigned_drone_id") is not None:
                score += 50.0  # Continuity bias
            return -score  # Negative for descending priority

        pending.sort(key=scoring_fn)
        self.queue = pending
        return self.queue

    def peek(self) -> Optional[Dict[str, Any]]:
        return self.queue[0] if self.queue else None

    def get_ordered_queue_telemetry(self) -> List[Dict[str, Any]]:
        """Returns structured data for HUD priority queue display."""
        items = []
        for rank, p in enumerate(self.queue, start=1):
            items.append({
                "rank": rank,
                "id": p["id"],
                "priority": p.get("priority", "MEDIUM"),
                "dwell_time": p.get("required_dwell_time", 10.0),
                "progress": round(float(min(1.0, p.get("current_dwell_time", 0.0) / max(0.1, p.get("required_dwell_time", 10.0))) * 100.0), 1),
                "assigned_drone": p.get("assigned_drone_id"),
                "status": "SURVEYING" if p.get("assigned_drone_id") else "PENDING",
            })
        return items


class DisasterMissionManager:
    """
    High-Level Disaster Survey Mission Orchestrator.

    Coordinates the multi-UAV fleet:
    - Allocates roles between Surveyors and Relays.
    - Dispatches Survey UAVs to pending PoIs via Priority Queue and CBBA auction.
    - Enforces strict mission completion time budget with dynamic airspeed scaling.
    - Network disconnect failsafe: triggers RTB on comms loss, automatically resumes tasks upon reconnection.
    - Coordinates autonomous retreat (RTL) upon survey completion, total area coverage, or low battery.
    - Executes controlled vertical descent, flare, and safe landing at recovery pads.
    """

    def __init__(
        self,
        gcs_position: Sequence[float] = (0.0, 0.0, 0.0),
        survey_dwell_radius: float = 35.0,
        low_battery_rtl_threshold: float = 0.25,
        emergency_battery_threshold: float = 0.10,
        rtl_altitude: float = 55.0,
        approach_arrival_radius: float = 6.0,
        landing_altitude_threshold: float = 0.25,
        mission_time_budget: float = 600.0,
        comms_loss_timeout: float = 12.0,
    ) -> None:
        self.gcs_position = np.array(gcs_position, dtype=np.float64)
        self.survey_dwell_radius = float(survey_dwell_radius)
        self.low_battery_rtl_threshold = float(low_battery_rtl_threshold)
        self.emergency_battery_threshold = float(emergency_battery_threshold)
        self.rtl_altitude = float(rtl_altitude)
        self.approach_arrival_radius = float(approach_arrival_radius)
        self.landing_altitude_threshold = float(landing_altitude_threshold)
        self.mission_time_budget = float(mission_time_budget)
        self.comms_loss_timeout = float(comms_loss_timeout)
        self.total_mission_time: float = 0.0
        self.completed_pois_count: int = 0
        self.retreat_all_requested: bool = False
        self._last_pois: Dict[str, Dict[str, Any]] = {}

        # Priority Queue for disaster survey sites
        self.priority_queue = DisasterSitePriorityQueue()

        # Swarm intelligence modules
        from sim.planning import CBBASolver, RelayReliefManager
        self.cbba_solver = CBBASolver(max_bundle_size=2)
        self.relief_manager = RelayReliefManager(low_battery_threshold=0.35, takeover_min_battery=0.65)

        # GCS Automated Battery Charging / Hot-Swap Pads
        gx, gy, gz = float(self.gcs_position[0]), float(self.gcs_position[1]), float(self.gcs_position[2])
        self.charging_pads: List[Dict[str, Any]] = [
            {"id": "PAD_1", "position": [gx - 15.0, gy, gz], "status": "AVAILABLE", "assigned_drone_id": None, "swap_timer": 0.0},
            {"id": "PAD_2", "position": [gx - 5.0, gy, gz], "status": "AVAILABLE", "assigned_drone_id": None, "swap_timer": 0.0},
            {"id": "PAD_3", "position": [gx + 5.0, gy, gz], "status": "AVAILABLE", "assigned_drone_id": None, "swap_timer": 0.0},
            {"id": "PAD_4", "position": [gx + 15.0, gy, gz], "status": "AVAILABLE", "assigned_drone_id": None, "swap_timer": 0.0},
        ]

        # Tactical Visual Comms Chatter Feed (No Audio)
        self.tactical_comms: List[TacticalCommsEvent] = []
        self.max_comms_history: int = 50
        self.emit_tactical_comms("INFO", "GCS", "Tactical Swarm Mission Operations Online. Network FANET initialized.", "SYSTEM")

        # Synthetic Thermal AI / Survivor SAR
        self.survivors: Dict[str, SurvivorRecord] = {}
        self._survivors_initialized: bool = False

    def emit_tactical_comms(self, level: str, callsign: str, message: str, category: str = "MISSION") -> None:
        """Log a real-time tactical comms chatter event for visual HUD display."""
        evt = TacticalCommsEvent(
            timestamp=self.total_mission_time,
            level=level,
            callsign=callsign,
            message=message,
            category=category,
        )
        self.tactical_comms.append(evt)
        if len(self.tactical_comms) > self.max_comms_history:
            self.tactical_comms.pop(0)

    def _init_survivors(self, pois: Dict[str, Dict[str, Any]]) -> None:
        """Seed simulated trapped human thermal body heat signatures around disaster sites."""
        self._survivors_initialized = True
        s_count = 1
        for p_id, poi in pois.items():
            pos = poi["position"]
            priority = str(poi.get("priority", "MEDIUM")).upper()
            num_survivors = 2 if priority in ("CRITICAL", "HIGH") else 1
            for k in range(num_survivors):
                offset_x = 4.0 * (k + 1) * (-1 if k % 2 == 0 else 1)
                offset_y = 3.0 * (k + 1) * (1 if k % 2 == 0 else -1)
                s_id = f"SURVIVOR_{s_count:02d}"
                heat = round(37.0 + (s_count % 5) * 0.3, 1)
                conf = round(0.91 + (s_count % 8) * 0.01, 2)
                self.survivors[s_id] = SurvivorRecord(
                    id=s_id,
                    poi_id=p_id,
                    position=[float(pos[0] + offset_x), float(pos[1] + offset_y), 0.0],
                    heat_c=heat,
                    confidence=conf,
                    discovered=False,
                )
                s_count += 1

    def get_survivors_telemetry(self) -> Dict[str, Any]:
        """Returns structured survivor counts and discovered locations for HUD and UI."""
        total = len(self.survivors)
        discovered = [s.to_dict() for s in self.survivors.values() if s.discovered]
        return {
            "total_count": total,
            "located_count": len(discovered),
            "pending_count": total - len(discovered),
            "discovered_survivors": discovered,
        }

    def get_tactical_comms_telemetry(self) -> List[Dict[str, Any]]:
        """Returns chronological tactical comms events for visual ticker and log."""
        return [evt.to_dict() for evt in self.tactical_comms[-20:]]

    def get_charging_pads_telemetry(self) -> List[Dict[str, Any]]:
        """Returns GCS automated charging pad statuses."""
        return self.charging_pads

    def get_time_remaining(self) -> float:
        """Returns remaining mission time in seconds before budget expires."""
        return max(0.0, self.mission_time_budget - self.total_mission_time)

    def get_budget_status(self) -> str:
        """Returns mission pace status based on remaining time and pending sites."""
        rem = self.get_time_remaining()
        has_pending = any(not p.get("is_completed", False) for p in getattr(self, "_last_pois", {}).values())
        if not has_pending and self.completed_pois_count > 0:
            return "COMPLETED"
        if rem <= 0.0:
            return "TIME_EXCEEDED"
        if rem <= 60.0:
            return "CRITICAL_DEADLINE"
        if rem <= 150.0:
            return "EXPEDITED"
        return "ON_SCHEDULE"

    def get_priority_queue_telemetry(self) -> List[Dict[str, Any]]:
        """Returns serialized priority queue entries for UI display."""
        return self.priority_queue.get_ordered_queue_telemetry()

    def trigger_fleet_retreat(self) -> None:
        """Command all drones in the swarm to immediately initiate autonomous retreat."""
        self.retreat_all_requested = True

    def trigger_drone_retreat(self, drone_id: str, drones: Dict[str, Any]) -> None:
        """Command an individual drone to immediately initiate autonomous retreat."""
        if drone_id in drones:
            self._initiate_drone_rtl(drones[drone_id])

    def trigger_chaos_fault(self, drones: Dict[str, Any], target_drone_id: Optional[str] = None) -> Optional[str]:
        """
        Swarm Self-Healing Chaos Fault Injection:
        Simulates a sudden catastrophic hardware failure / flameout in an airborne drone.
        Swarm autonomously detects failure, redistributes relays, and CBBA re-assigns orphaned PoIs.
        """
        airborne = [
            d for d in drones.values()
            if d.flight_mode in (FlightMode.TAKEOFF, FlightMode.TRANSIT, FlightMode.SURVEYING, FlightMode.RELAY)
            and not getattr(d, "is_fault_injected", False)
            and float(d.position[2]) > 5.0
        ]
        if not airborne and not target_drone_id:
            return None

        victim = None
        if target_drone_id and target_drone_id in drones:
            victim = drones[target_drone_id]
        elif airborne:
            import random
            victim = random.choice(airborne)

        if not victim:
            return None

        victim.is_fault_injected = True
        victim.set_flight_mode(FlightMode.EMERGENCY_LAND)
        self.emit_tactical_comms(
            "CRITICAL",
            victim.id,
            f"⚡ CRITICAL FAULT: {victim.id} MOTOR FLAMEOUT — Swarm Self-Healing Activated",
            "MISSION"
        )

        # Release assigned PoI if any so healthy fleet members immediately re-survey
        if victim.assigned_poi_id and hasattr(self, "_last_pois") and self._last_pois:
            p_id = victim.assigned_poi_id
            if p_id in self._last_pois:
                self._last_pois[p_id]["assigned_drone_id"] = None
            victim.assigned_poi_id = None
            self._assign_pending_pois(drones, self._last_pois)

        if victim.role == DroneRole.RELAY:
            self._apply_relay_spring_tethering(drones)

        return victim.id

    def get_recovery_pad(self, drone: Any, drone_index: int = 0) -> np.ndarray:
        """
        Determines the designated 3D recovery pad / landing location for a drone.
        Priority:
        1. Explicitly configured drone.recovery_pad
        2. Configured drone.home_position
        3. Drone's initial launch position (drone._initial_pos)
        4. Deconflicted multi-drone landing slot around GCS base station.
        """
        if hasattr(drone, "recovery_pad") and drone.recovery_pad is not None:
            return np.asarray(drone.recovery_pad, dtype=np.float64)

        if hasattr(drone, "home_position") and drone.home_position is not None:
            home = np.asarray(drone.home_position, dtype=np.float64)
            if not np.allclose(home, 0.0) or np.allclose(self.gcs_position, 0.0):
                return home

        if hasattr(drone, "_initial_pos") and drone._initial_pos is not None:
            init_p = np.asarray(drone._initial_pos, dtype=np.float64)
            if not np.allclose(init_p, 0.0) or np.allclose(self.gcs_position, 0.0):
                return init_p

        # Multi-drone deconflicted slot grid around GCS base
        idx = getattr(drone, "_fleet_index", drone_index)
        col = (idx % 4) - 1.5
        row = (idx // 4) - 1.5
        slot_offset = np.array([col * 6.0, row * 6.0, 0.0], dtype=np.float64)
        return self.gcs_position + slot_offset

    def calculate_required_soc_for_rtb(self, drone: Any, target_pad: np.ndarray) -> float:
        """
        Calculates dynamic State of Charge (SoC) fraction required for drone to return to base.
        Accounts for horizontal distance, climb to Tier 3 corridor, descent, flare, and safety margin.
        """
        if not hasattr(drone, "battery") or drone.battery is None:
            return self.low_battery_rtl_threshold

        diff_xy = drone.position[:2] - target_pad[:2]
        dist_xy = float(np.linalg.norm(diff_xy))

        # Vertical climb to Tier 3 corridor and descent to pad
        current_z = float(drone.position[2])
        dz_up = max(0.0, self.rtl_altitude - current_z)
        dz_down = max(0.0, self.rtl_altitude - float(target_pad[2]))

        v_cruise = min(8.0, getattr(drone.limits, "max_speed_xy", 10.0))
        v_climb = min(2.5, getattr(drone.limits, "max_speed_z_up", 3.5))
        v_descend = 1.2

        t_flight = (dist_xy / max(v_cruise, 1.0)) + (dz_up / max(v_climb, 1.0)) + (dz_down / max(v_descend, 0.5)) + 10.0
        avg_power = 195.0  # Watts
        energy_required = t_flight * avg_power * 1.25  # 25% safety reserve margin

        total_joules = getattr(drone.battery, "total_energy_joules", 266400.0)
        req_soc = energy_required / max(total_joules, 1.0) + self.emergency_battery_threshold
        return min(0.60, max(self.low_battery_rtl_threshold, req_soc))

    def _initiate_drone_rtl(self, drone: Any) -> None:
        """Initiates autonomous return to launch / recovery pad."""
        pad = self.get_recovery_pad(drone)
        drone.set_flight_mode(FlightMode.RTL)
        drone.set_target_waypoint(np.array([pad[0], pad[1], float(self.rtl_altitude)], dtype=np.float64))
        drone.assigned_poi_id = None
        drone.is_transmitting = False
        drone.dwell_time = 0.0

    def update(
        self,
        drones: Dict[str, Any],
        pois: Dict[str, Dict[str, Any]],
        dt: float,
        network_engine: Optional[Any] = None,
    ) -> None:
        """
        Advance mission state by dt seconds.
        Assigns pending PoIs via priority queue, updates dwell times, and transitions drone FSM states.
        Enforces mission time budget and network disconnect failsafe.
        """
        self.total_mission_time += dt
        self._last_pois = pois

        # 0. Initialize trapped survivor heat signatures at disaster sites if not already seeded
        if not self._survivors_initialized and pois:
            self._init_survivors(pois)

        # 1. Update disaster site priority queue and deadline urgency
        self.priority_queue.update_queue(pois, self.total_mission_time, self.mission_time_budget, drones)

        # 2. Dynamic airspeed pace adjustment to guarantee completion within budget
        time_rem = self.get_time_remaining()
        pending_dwell = sum(max(0.0, p.get("required_dwell_time", 10.0) - p.get("current_dwell_time", 0.0)) for p in pois.values() if not p.get("is_completed", False))

        pace_multiplier = 1.0
        if time_rem < 200.0 and pending_dwell > 0:
            pace_multiplier = min(1.45, max(1.0, (pending_dwell + 60.0) / max(10.0, time_rem)))

        for d in drones.values():
            if hasattr(d, "limits") and d.limits is not None:
                d.limits.max_speed_xy = min(14.5, 10.0 * pace_multiplier)

        # 3. Ensure role allocation for fleet
        self._ensure_role_allocation(drones)

        # 4. Check dynamic relay battery relief rotation
        handover = self.relief_manager.evaluate_relief_rotation(drones, self.total_mission_time)
        if handover:
            r_id, s_id = handover
            self.relief_manager.execute_handover(drones[r_id], drones[s_id])

        # 5. Apply dynamic relay spring-tethering to track surveyor fleet centroid
        self._apply_relay_spring_tethering(drones)

        # 5b. Evaluate aerodynamic wake drafting in V-formation corridors
        self._apply_formation_drafting(drones)

        # 6. Assign available survey drones to pending PoIs in strict priority order
        self._assign_pending_pois(drones, pois)

        # 7. Update drones and their active mission targets
        for idx, (drone_id, drone) in enumerate(drones.items()):
            setattr(drone, "_fleet_index", idx)
            self._update_drone_fsm(drone, pois, dt, network_engine=network_engine)

        # Count completed PoIs
        self.completed_pois_count = sum(1 for p in pois.values() if p.get("is_completed", False))

    def _apply_relay_spring_tethering(self, drones: Dict[str, Any]) -> None:
        """
        Multi-Sector Elevated Relay Spring-Tethering:
        Distributes relay drones across West and East operational sectors at elevated
        altitudes (75m-85m) to clear obstacles (Damaged Tower Beta, 65m) and ensure
        uninterrupted multi-hop FANET connectivity to both left and right disaster sites.
        """
        surveyors = [
            d for d in drones.values()
            if d.role == DroneRole.SURVEY and d.flight_mode in (FlightMode.TRANSIT, FlightMode.SURVEYING)
        ]
        relays = sorted(
            [d for d in drones.values() if d.role == DroneRole.RELAY and d.flight_mode in (FlightMode.RELAY, FlightMode.TRANSIT)],
            key=lambda d: d.id
        )
        if not relays:
            return

        gcs_xy = self.gcs_position[:2]
        num_relays = len(relays)

        # Split surveyors into West (x < 0) and East (x >= 0) sectors
        west_surveyors = [d for d in surveyors if d.position[0] < 0]
        east_surveyors = [d for d in surveyors if d.position[0] >= 0]

        west_target_xy = np.mean([d.position[:2] for d in west_surveyors], axis=0) if west_surveyors else np.array([-160.0, 100.0])
        east_target_xy = np.mean([d.position[:2] for d in east_surveyors], axis=0) if east_surveyors else np.array([160.0, 100.0])

        if num_relays >= 2:
            num_west = num_relays // 2
            west_relays = relays[:num_west]
            east_relays = relays[num_west:]
        else:
            west_relays = relays
            east_relays = []

        # Position West sector relays (elevated at 75-85m to clear 65m Damaged Tower Beta)
        west_vec = west_target_xy - gcs_xy
        west_dist = max(15.0, float(np.linalg.norm(west_vec)))
        west_u = west_vec / west_dist
        for k, relay in enumerate(west_relays):
            frac = (k + 1) / (len(west_relays) + 1)
            pos_xy = gcs_xy + west_u * (west_dist * frac)
            pos_z = 75.0 + (k % 3) * 8.0
            relay.set_target_waypoint(np.array([pos_xy[0], pos_xy[1], pos_z], dtype=np.float64))

        # Position East sector relays
        if east_relays:
            east_vec = east_target_xy - gcs_xy
            east_dist = max(15.0, float(np.linalg.norm(east_vec)))
            east_u = east_vec / east_dist
            for k, relay in enumerate(east_relays):
                frac = (k + 1) / (len(east_relays) + 1)
                pos_xy = gcs_xy + east_u * (east_dist * frac)
                pos_z = 75.0 + (k % 3) * 8.0
                relay.set_target_waypoint(np.array([pos_xy[0], pos_xy[1], pos_z], dtype=np.float64))

    def _apply_formation_drafting(self, drones: Dict[str, Any]) -> None:
        """
        V-Formation Aerodynamic Wake Drafting:
        Calculates tip-vortex upwash energy savings when drones fly in formation.
        Drones trailing 2.5m-22m behind a lead drone with 1.0m-6.0m lateral offset
        experience induced upwash reducing hover rotor power by 2% to 15%.
        """
        for d in drones.values():
            d.is_drafting = False
            d.drafting_leader_id = None
            d.drafting_saving_pct = 0.0

        active_drones = [
            d for d in drones.values()
            if d.flight_mode in (FlightMode.TRANSIT, FlightMode.SURVEYING, FlightMode.RELAY, FlightMode.RTL)
            and not getattr(d, "is_fault_injected", False)
            and float(d.position[2]) > 5.0
        ]

        for follower in active_drones:
            f_pos = follower.position
            best_saving = 0.0
            best_leader = None

            for leader in active_drones:
                if leader.id == follower.id:
                    continue

                l_pos = leader.position
                l_vel = getattr(leader, "velocity", np.zeros(3))
                speed = float(np.linalg.norm(l_vel[:2]))
                if speed < 1.5:
                    continue

                # Heading unit vector of leader
                u_head = l_vel[:2] / speed
                delta_xy = f_pos[:2] - l_pos[:2]

                # Longitudinal distance: how far follower is behind leader
                d_long = -float(np.dot(delta_xy, u_head))
                if d_long < 2.5 or d_long > 22.0:
                    continue

                # Lateral offset perpendicular to leader track
                u_perp = np.array([-u_head[1], u_head[0]])
                d_lat = abs(float(np.dot(delta_xy, u_perp)))
                if d_lat < 1.0 or d_lat > 6.0:
                    continue

                # Vertical height difference
                d_vert = abs(float(f_pos[2] - l_pos[2]))
                if d_vert > 4.0:
                    continue

                # Peak upwash at ~2.5m lateral offset, tapering smoothly
                lat_efficiency = max(0.0, 1.0 - abs(d_lat - 2.5) / 3.0)
                long_efficiency = max(0.0, 1.0 - (d_long - 2.5) / 19.5)
                vert_efficiency = max(0.0, 1.0 - d_vert / 4.0)

                saving = 15.0 * lat_efficiency * long_efficiency * vert_efficiency
                if saving > best_saving and saving >= 2.0:
                    best_saving = saving
                    best_leader = leader.id

            if best_leader and best_saving >= 2.0:
                follower.is_drafting = True
                follower.drafting_leader_id = best_leader
                follower.drafting_saving_pct = round(best_saving, 1)

    def _ensure_role_allocation(self, drones: Dict[str, Any]) -> None:
        """Ensure an optimal split of Survey and Relay drones in the fleet."""
        sorted_ids = sorted(drones.keys())
        n = len(sorted_ids)
        if n == 0:
            return

        # If roles already assigned, preserve them
        has_survey = any(d.role == DroneRole.SURVEY for d in drones.values())
        has_relay = any(d.role == DroneRole.RELAY for d in drones.values())
        if has_survey and (has_relay or n <= 2):
            return

        # Allocate approx 2/3 as Surveyors, 1/3 as Relays (minimum 1 relay if n >= 3)
        num_relays = max(1, n // 3) if n >= 3 else 0
        num_surveyors = n - num_relays

        for i, d_id in enumerate(sorted_ids):
            drone = drones[d_id]
            if i < num_surveyors:
                drone.role = DroneRole.SURVEY
            else:
                drone.role = DroneRole.RELAY

    def _assign_pending_pois(self, drones: Dict[str, Any], pois: Dict[str, Dict[str, Any]]) -> None:
        """Assign unassigned Survey drones to high-priority pending PoIs."""
        # 0. Clean stale PoI assignments: if a PoI has assigned_drone_id, but that drone is NOT
        # currently targeting this PoI or is not in active survey/transit, release the PoI!
        for p in pois.values():
            if not p.get("is_completed", False):
                assigned_id = p.get("assigned_drone_id")
                if assigned_id:
                    d = drones.get(assigned_id)
                    if (
                        not d
                        or d.assigned_poi_id != p["id"]
                        or d.flight_mode in (
                            FlightMode.RTL,
                            FlightMode.LANDING,
                            FlightMode.LANDED,
                            FlightMode.EMERGENCY_LAND,
                            FlightMode.COMPLETED,
                        )
                        or getattr(d, "is_comms_loss_rtl", False)
                        or getattr(d, "is_fault_injected", False)
                    ):
                        p["assigned_drone_id"] = None
                        p["_assignment_timer"] = 0.0
                    elif d.flight_mode == FlightMode.TRANSIT and p.get("current_dwell_time", 0.0) <= 0.0:
                        p["_assignment_timer"] = p.get("_assignment_timer", 0.0) + 0.05
                        # If a drone is blocked or delayed in transit > 45s without reaching dwell, re-auction!
                        if p["_assignment_timer"] > 45.0:
                            p["assigned_drone_id"] = None
                            d.assigned_poi_id = None
                            p["_assignment_timer"] = 0.0
                    else:
                        p["_assignment_timer"] = 0.0

        # Find unassigned pending PoIs drawn from Priority Queue
        pending_pois = [
            p for p in self.priority_queue.queue
            if not p.get("is_completed", False) and p.get("assigned_drone_id") is None
        ]

        # Find idle or unassigned survey drones with adequate battery
        available_surveyors = [
            d for d in drones.values()
            if d.role == DroneRole.SURVEY and d.assigned_poi_id is None
            and d.flight_mode not in (
                FlightMode.RTL,
                FlightMode.LANDING,
                FlightMode.LANDED,
                FlightMode.EMERGENCY_LAND,
                FlightMode.COMPLETED,
            )
            and not (hasattr(d, "battery") and d.battery.is_low())
            and not self.retreat_all_requested
            and not getattr(d, "is_comms_loss_rtl", False)
        ]

        # 1. Consensus-Based Bundle Algorithm (CBBA) distributed auction
        if hasattr(self, "cbba_solver") and pending_pois and available_surveyors:
            cbba_map = self.cbba_solver.solve(drones, pois)
            for p in pending_pois:
                p_id = p["id"]
                d_id = cbba_map.get(p_id)
                if d_id and d_id in drones and drones[d_id] in available_surveyors:
                    selected_drone = drones[d_id]
                    available_surveyors.remove(selected_drone)
                    p["assigned_drone_id"] = selected_drone.id
                    selected_drone.assigned_poi_id = p_id
                    selected_drone.set_target_waypoint(p["position"])

        # 2. Greedy fallback for any remaining unassigned
        for poi in pending_pois:
            if poi.get("assigned_drone_id") is not None or not available_surveyors:
                continue
            poi_pos = poi["position"]
            available_surveyors.sort(key=lambda d: float(np.linalg.norm(d.position - poi_pos)))
            selected_drone = available_surveyors.pop(0)
            poi["assigned_drone_id"] = selected_drone.id
            selected_drone.assigned_poi_id = poi["id"]
            selected_drone.set_target_waypoint(poi_pos)

        # 3. Swarm Cooperative Wide-Area Exploration & Sector Coverage
        # If survey/scout drones are free and not assigned to a primary PoI, dispatch them
        # to systematically sweep disaster sectors so the swarm covers all areas without hovering.
        if available_surveyors:
            # Check for unlocated survivor coordinates first
            unlocated_survivor_positions = [
                s.position for s in self.survivors.values() if not s.discovered
            ] if hasattr(self, "survivors") else []

            coverage_sectors = [
                [-210.0, 90.0, 30.0],   # West Sector (Tower Collapse)
                [-170.0, 240.0, 25.0],  # Far North-West Sector (Bridge)
                [-240.0, -120.0, 25.0], # South-West Sector (Highway)
                [-120.0, 160.0, 30.0],  # West-Central Sector
                [-40.0, 140.0, 32.0],   # North-Central Sector (Hospital)
                [30.0, 260.0, 28.0],    # North Sector (Hazard)
                [0.0, 30.0, 28.0],      # Central Boulevard
                [160.0, 40.0, 26.0],    # East-Central Sector (Substation)
                [220.0, 160.0, 28.0],   # East Sector (Survivor Zone)
                [190.0, -80.0, 25.0],   # South-East Sector (Shelter)
                [130.0, 220.0, 28.0],   # North-East Industrial Sector
                [-100.0, -80.0, 25.0],  # South-West Sector
            ]

            target_points = unlocated_survivor_positions if unlocated_survivor_positions else coverage_sectors

            for idx, drone in enumerate(available_surveyors):
                sec_pt = target_points[(idx + int(self.total_mission_time // 12)) % len(target_points)]
                drone.set_target_waypoint(np.array(sec_pt, dtype=np.float64))
                drone.set_flight_mode(FlightMode.TRANSIT)

    def _update_drone_fsm(
        self,
        drone: Any,
        pois: Dict[str, Dict[str, Any]],
        dt: float,
        network_engine: Optional[Any] = None,
    ) -> None:
        """Handle individual UAV FSM updates based on mission progress, retreat, and network connectivity."""
        pad = self.get_recovery_pad(drone)

        # ---------------------------------------------------------------------
        # Network Connectivity Check & Autonomous Failsafe
        # ---------------------------------------------------------------------
        is_connected = True
        if network_engine is not None and hasattr(network_engine, "is_connected_to_gcs"):
            is_connected = bool(network_engine.is_connected_to_gcs(drone.id))

        if getattr(drone, "is_comms_loss_rtl", False):
            if is_connected:
                # Network link recovered! Seamlessly resume interrupted task
                drone.is_comms_loss_rtl = False
                drone.comms_loss_duration = 0.0
                saved = getattr(drone, "_saved_task", None)
                if saved and saved.get("poi_id") and saved["poi_id"] in pois:
                    saved_poi = pois[saved["poi_id"]]
                    if not saved_poi.get("is_completed", False):
                        drone.assigned_poi_id = saved_poi["id"]
                        saved_poi["assigned_drone_id"] = drone.id
                        drone.set_target_waypoint(saved_poi["position"])
                        drone.set_flight_mode(FlightMode.TRANSIT)
                        drone._saved_task = None
                        self.emit_tactical_comms("SUCCESS", drone.id, f"🟢 MESH LINK RESTORED — Resuming task at {saved_poi['id']}", "COMMS")
                    else:
                        drone._saved_task = None
                        drone.set_flight_mode(FlightMode.TRANSIT)
                else:
                    drone._saved_task = None
                    if drone.role == DroneRole.RELAY:
                        drone.set_flight_mode(FlightMode.RELAY)
                    else:
                        drone.set_flight_mode(FlightMode.TRANSIT)
                    self.emit_tactical_comms("SUCCESS", drone.id, "🟢 MESH LINK RESTORED — Resuming assigned flight corridor", "COMMS")
        elif drone.flight_mode in (FlightMode.TAKEOFF, FlightMode.TRANSIT, FlightMode.SURVEYING, FlightMode.RELAY):
            if not is_connected:
                drone.comms_loss_duration = getattr(drone, "comms_loss_duration", 0.0) + dt
                if drone.comms_loss_duration >= self.comms_loss_timeout and not getattr(drone, "is_comms_loss_rtl", False):
                    # Trigger Network Loss Failsafe: return to base
                    drone.is_comms_loss_rtl = True
                    drone._saved_task = {
                        "poi_id": drone.assigned_poi_id,
                        "flight_mode": drone.flight_mode,
                        "target_position": drone.target_position.copy() if drone.target_position is not None else None,
                    }
                    if drone.assigned_poi_id and drone.assigned_poi_id in pois:
                        pois[drone.assigned_poi_id]["assigned_drone_id"] = None
                    drone.assigned_poi_id = None
                    self.emit_tactical_comms("WARN", drone.id, "⚠️ COMMS LOST (>2.0s) — Failsafe RTL Initiated", "COMMS")
                    self._initiate_drone_rtl(drone)
            else:
                drone.comms_loss_duration = 0.0

        # ---------------------------------------------------------------------
        # 1. Terminal / Landing / Hot-Swap States Handling
        # ---------------------------------------------------------------------
        if drone.flight_mode == FlightMode.DOCKED_SWAPPING:
            drone._swap_timer = getattr(drone, "_swap_timer", 25.0) - dt
            pad_id = getattr(drone, "_assigned_pad_id", None)
            if pad_id:
                for p in self.charging_pads:
                    if p["id"] == pad_id:
                        p["swap_timer"] = max(0.0, drone._swap_timer)
            if drone._swap_timer <= 0.0:
                if hasattr(drone, "battery") and drone.battery is not None:
                    drone.battery.soc = 1.0
                drone.is_low_battery_rtb = False
                drone.is_comms_loss_rtl = False
                if pad_id:
                    for p in self.charging_pads:
                        if p["id"] == pad_id:
                            p["status"] = "AVAILABLE"
                            p["assigned_drone_id"] = None
                            p["swap_timer"] = 0.0
                drone.set_flight_mode(FlightMode.TAKEOFF)
                drone.set_target_waypoint(drone.position + np.array([0.0, 0.0, 30.0]))
                self.emit_tactical_comms(
                    "SUCCESS",
                    drone.id,
                    "🔋 LiPo Hot-Swap complete (100% SoC) — Redeploying to active disaster grid",
                    "SWAP"
                )
            return

        if drone.flight_mode == FlightMode.LANDED:
            drone.velocity[:] = 0.0
            drone.acceleration[:] = 0.0
            drone.rotor_speeds[:] = 0.0
            drone.is_transmitting = False
            drone.target_position = None
            drone.assigned_poi_id = None

            # Check if drone landed due to low battery and can undergo automated hot-swap
            if getattr(drone, "is_low_battery_rtb", False) and not self.retreat_all_requested:
                available_pad = None
                for p in self.charging_pads:
                    if p["status"] == "AVAILABLE" or p["assigned_drone_id"] == drone.id:
                        available_pad = p
                        break
                if available_pad:
                    available_pad["status"] = "OCCUPIED"
                    available_pad["assigned_drone_id"] = drone.id
                    available_pad["swap_timer"] = 25.0
                    drone.set_flight_mode(FlightMode.DOCKED_SWAPPING)
                    drone._swap_timer = 25.0
                    drone._assigned_pad_id = available_pad["id"]
                    self.emit_tactical_comms(
                        "INFO",
                        drone.id,
                        f"🪫 Docked at {available_pad['id']} — Automated LiPo Hot-Swap initiated (25s turnaround)",
                        "SWAP"
                    )
            return

        if drone.flight_mode in (FlightMode.LANDING, FlightMode.EMERGENCY_LAND):
            # Target pad on ground
            target_pt = pad if drone.flight_mode == FlightMode.LANDING else np.array([drone.position[0], drone.position[1], 0.0])
            drone.set_target_waypoint(np.array([target_pt[0], target_pt[1], 0.0], dtype=np.float64))
            drone.is_transmitting = False
            drone.assigned_poi_id = None

            # Touchdown detection
            pz = float(drone.position[2])
            vz = float(drone.velocity[2])
            if pz <= self.landing_altitude_threshold and abs(vz) <= 1.0:
                drone.position[2] = 0.0
                drone.velocity[:] = 0.0
                drone.acceleration[:] = 0.0
                drone.rotor_speeds[:] = 0.0
                drone.set_flight_mode(FlightMode.LANDED)
                drone.target_position = None
            return

        if drone.flight_mode == FlightMode.RTL:
            # Cruising towards recovery pad at Tier 3 obstacle clearance corridor
            drone.set_target_waypoint(np.array([pad[0], pad[1], float(self.rtl_altitude)], dtype=np.float64))
            drone.assigned_poi_id = None
            drone.is_transmitting = False

            # Check arrival above recovery pad
            dist_to_pad_xy = float(np.linalg.norm(drone.position[:2] - pad[:2]))
            if dist_to_pad_xy <= self.approach_arrival_radius:
                # Reached airspace directly over pad: initiate controlled landing descent
                drone.set_flight_mode(FlightMode.LANDING)
                drone.set_target_waypoint(np.array([pad[0], pad[1], 0.0], dtype=np.float64))
            return

        if drone.flight_mode == FlightMode.COMPLETED:
            if drone.position[2] > self.landing_altitude_threshold:
                drone.set_flight_mode(FlightMode.LANDING)
                drone.set_target_waypoint(np.array([pad[0], pad[1], 0.0], dtype=np.float64))
            else:
                drone.set_flight_mode(FlightMode.LANDED)
            return

        # ---------------------------------------------------------------------
        # 2. Autonomous Retreat Triggers (Active Flight)
        # ---------------------------------------------------------------------
        # Trigger A: Fleet-wide retreat requested
        if self.retreat_all_requested:
            self._initiate_drone_rtl(drone)
            return

        # Trigger B: Low / Critical Battery
        if hasattr(drone, "battery") and drone.battery is not None:
            # Critical battery: emergency failsafe descent
            if drone.battery.is_critical() or drone.battery.soc <= self.emergency_battery_threshold:
                dist_to_pad = float(np.linalg.norm(drone.position[:2] - pad[:2]))
                if dist_to_pad <= self.approach_arrival_radius:
                    drone.set_flight_mode(FlightMode.LANDING)
                    drone.set_target_waypoint(np.array([pad[0], pad[1], 0.0]))
                else:
                    drone.set_flight_mode(FlightMode.EMERGENCY_LAND)
                    drone.set_target_waypoint(np.array([drone.position[0], drone.position[1], 0.0]))
                drone.assigned_poi_id = None
                drone.is_transmitting = False
                self.emit_tactical_comms("CRITICAL", drone.id, f"🚨 Battery CRITICAL ({int(drone.battery.soc*100)}%) — Emergency Landing", "BATTERY")
                return

            # Low battery: static threshold or dynamic return-to-base energy check
            dynamic_req_soc = self.calculate_required_soc_for_rtb(drone, pad)
            if drone.battery.is_low() or drone.battery.soc <= self.low_battery_rtl_threshold or drone.battery.soc <= dynamic_req_soc:
                if not getattr(drone, "is_low_battery_rtb", False):
                    drone.is_low_battery_rtb = True
                    self.emit_tactical_comms(
                        "CRITICAL",
                        drone.id,
                        f"🪫 Low Battery ({int(drone.battery.soc*100)}%) — Retreating to base charging pad",
                        "BATTERY"
                    )
                self._initiate_drone_rtl(drone)
                return

        # Comprehensive Sensor Footprint Survivor Detection (FLIR & LiDAR across active modes)
        if hasattr(self, "survivors") and drone.flight_mode in (FlightMode.TRANSIT, FlightMode.SURVEYING, FlightMode.TAKEOFF):
            for s_id, surv in self.survivors.items():
                if not surv.discovered:
                    dist_to_surv = float(np.linalg.norm(drone.position - np.array(surv.position)))
                    if dist_to_surv <= 55.0:
                        surv.discovered = True
                        surv.discovery_time = self.total_mission_time
                        surv.discovered_by = drone.id
                        self.emit_tactical_comms(
                            "SUCCESS",
                            drone.id,
                            f"👤 SURVIVOR LOCATED at [{int(surv.position[0])}, {int(surv.position[1])}] (IR Heat {surv.heat_c}°C, Conf: {int(surv.confidence*100)}%)",
                            "SAR"
                        )

        # Trigger C: All surveillance area / disaster PoIs covered and all survivors found
        all_pois_completed = (len(pois) > 0 and all(p.get("is_completed", False) for p in pois.values()))
        all_survivors_found = (len(self.survivors) > 0 and all(s.discovered for s in self.survivors.values()))
        if all_pois_completed and all_survivors_found:
            self._initiate_drone_rtl(drone)
            return

        # ---------------------------------------------------------------------
        # 3. Role-Specific Mission Logic
        # ---------------------------------------------------------------------
        # Relay drone mission logic
        if drone.role == DroneRole.RELAY:
            if drone.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                drone.set_flight_mode(FlightMode.TAKEOFF)
                drone.set_target_waypoint(drone.position + np.array([0.0, 0.0, 30.0]))
            elif drone.flight_mode == FlightMode.TAKEOFF and drone.position[2] >= 30.0:
                drone.set_flight_mode(FlightMode.RELAY)
            return

        # Survey drone mission logic
        if drone.role == DroneRole.SURVEY:
            if drone.flight_mode in (FlightMode.IDLE, FlightMode.LANDED):
                drone.set_flight_mode(FlightMode.TAKEOFF)
                drone.set_target_waypoint(drone.position + np.array([0.0, 0.0, 30.0]))

            elif drone.flight_mode == FlightMode.TAKEOFF:
                if drone.position[2] >= 25.0:
                    drone.set_flight_mode(FlightMode.TRANSIT)
                    if drone.assigned_poi_id is not None and drone.assigned_poi_id in pois:
                        poi = pois[drone.assigned_poi_id]
                        drone.set_target_waypoint(poi["position"])
                    else:
                        has_incomplete = any(not p.get("is_completed", False) for p in pois.values())
                        if not has_incomplete and all_survivors_found:
                            self._initiate_drone_rtl(drone)

            elif drone.flight_mode == FlightMode.TRANSIT:
                if drone.assigned_poi_id is not None and drone.assigned_poi_id in pois:
                    poi = pois[drone.assigned_poi_id]
                    drone.set_target_waypoint(poi["position"])
                    dist_to_poi = float(np.linalg.norm(drone.position[:2] - poi["position"][:2]))
                    if dist_to_poi < self.survey_dwell_radius:
                        # Arrived at PoI: begin survey dwelling
                        drone.set_flight_mode(FlightMode.SURVEYING)
                        drone.is_transmitting = True
                else:
                    # In TRANSIT without active assignment: check if pending tasks exist
                    all_done = all(p.get("is_completed", False) for p in pois.values())
                    all_survs_found = (len(self.survivors) > 0 and all(s.discovered for s in self.survivors.values()))
                    if all_done and all_survs_found:
                        self._initiate_drone_rtl(drone)

            elif drone.flight_mode == FlightMode.SURVEYING:
                if drone.assigned_poi_id is not None and drone.assigned_poi_id in pois:
                    poi = pois[drone.assigned_poi_id]
                    drone.set_target_waypoint(poi["position"])
                    poi["current_dwell_time"] += dt
                    drone.dwell_time = poi["current_dwell_time"]
                    drone.is_transmitting = True

                    # Check if inspection complete
                    if poi["current_dwell_time"] >= poi["required_dwell_time"]:
                        poi["is_completed"] = True
                        drone.assigned_poi_id = None
                        drone.dwell_time = 0.0
                        drone.is_transmitting = False
                        self.emit_tactical_comms(
                            "SUCCESS",
                            "GCS",
                            f"🎯 {poi['id']} SURVEY COMPLETE (100% Coverage)",
                            "MISSION"
                        )

                        # Trigger D: Survey complete check
                        all_done = all(p.get("is_completed", False) for p in pois.values())
                        all_survs_found = (len(self.survivors) > 0 and all(s.discovered for s in self.survivors.values()))

                        if all_done and all_survs_found:
                            # Assigned survey is complete & all surveillance area covered & all survivors located!
                            self._initiate_drone_rtl(drone)
                        else:
                            drone.set_flight_mode(FlightMode.TRANSIT)
                else:
                    drone.is_transmitting = False
                    drone.set_flight_mode(FlightMode.TRANSIT)
