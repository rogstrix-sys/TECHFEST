"""
tests/unit/test_retreat.py: Comprehensive Unit Tests for Autonomous UAV Retreat, RTL, and Safe Recovery.

Covers:
1. Low-battery RTL trigger and dynamic RTB SoC threshold calculation.
2. Critical-battery failsafe emergency descent.
3. Survey-completed individual drone RTL trigger when no pending PoIs remain.
4. Fleet-wide retreat when all surveillance area / disaster PoIs are completed.
5. Manual fleet-wide and per-drone retreat commands via mission manager and core.
6. Controlled descent, precision approach setpoint, and touchdown detection into LANDED state.
"""

import numpy as np
import pytest

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.mission import DisasterMissionManager
from sim.types import DroneRole, FlightMode


def test_low_battery_triggers_autonomous_rtl():
    """Verify that a drone with low battery automatically initiates RTL towards recovery pad."""
    mgr = DisasterMissionManager(gcs_position=[0.0, -250.0, 0.0])
    drone = Drone("SURVEY_1", initial_pos=np.array([100.0, 100.0, 30.0], dtype=np.float64))
    drone.role = DroneRole.SURVEY
    drone.assigned_poi_id = "POI_1"
    drone.set_flight_mode(FlightMode.SURVEYING)

    # Battery at nominal level
    drone.battery.soc = 0.85
    pois = {
        "POI_1": {
            "id": "POI_1",
            "position": np.array([100.0, 100.0, 30.0]),
            "priority": "HIGH",
            "required_dwell_time": 20.0,
            "current_dwell_time": 5.0,
            "is_completed": False,
            "assigned_drone_id": "SURVEY_1",
        }
    }
    drones = {"SURVEY_1": drone}

    mgr.update(drones, pois, dt=0.05)
    # Still surveying at 85% SoC
    assert drone.flight_mode == FlightMode.SURVEYING

    # Deplete battery below RTL threshold
    drone.battery.soc = 0.18
    mgr.update(drones, pois, dt=0.05)

    # Must autonomously transition to RTL and target recovery pad at safe altitude
    assert drone.flight_mode == FlightMode.RTL
    pad = mgr.get_recovery_pad(drone)
    target = drone.get_target_waypoint()
    assert np.allclose(target[:2], pad[:2])
    assert target[2] == mgr.rtl_altitude


def test_critical_battery_emergency_descent():
    """Verify that critical battery triggers emergency descent failsafe."""
    mgr = DisasterMissionManager(gcs_position=[0.0, -250.0, 0.0])
    drone = Drone("SCOUT_1", initial_pos=np.array([150.0, -50.0, 35.0], dtype=np.float64))
    drone.role = DroneRole.SURVEY
    drone.set_flight_mode(FlightMode.TRANSIT)

    drone.battery.soc = 0.06  # Critical (< 0.08)
    pois = {}
    drones = {"SCOUT_1": drone}

    mgr.update(drones, pois, dt=0.05)
    assert drone.flight_mode in (FlightMode.EMERGENCY_LAND, FlightMode.LANDING)
    assert drone.is_transmitting is False


def test_survey_completion_triggers_rtl():
    """Verify that a drone whose assigned survey completes with no pending tasks retreats."""
    mgr = DisasterMissionManager(gcs_position=[0.0, -250.0, 0.0])
    drone = Drone("SURVEY_2", initial_pos=np.array([50.0, 50.0, 25.0], dtype=np.float64))
    drone.role = DroneRole.SURVEY
    drone.set_flight_mode(FlightMode.SURVEYING)
    drone.assigned_poi_id = "POI_SOLO"
    drone.dwell_time = 9.95

    pois = {
        "POI_SOLO": {
            "id": "POI_SOLO",
            "position": np.array([50.0, 50.0, 25.0]),
            "priority": "HIGH",
            "required_dwell_time": 10.0,
            "current_dwell_time": 9.95,
            "is_completed": False,
            "assigned_drone_id": "SURVEY_2",
        }
    }
    drones = {"SURVEY_2": drone}

    # Step mission by 0.1s to exceed required dwell time
    mgr.update(drones, pois, dt=0.1)

    assert pois["POI_SOLO"]["is_completed"] is True
    # Since no more PoIs remain, drone must initiate RTL
    assert drone.flight_mode == FlightMode.RTL


def test_all_pois_completed_triggers_fleet_rtl():
    """Verify that when all PoIs are completed, all active drones (survey and relay) retreat."""
    mgr = DisasterMissionManager(gcs_position=[0.0, -250.0, 0.0])
    d1 = Drone("SURVEY_1", initial_pos=np.array([20.0, 20.0, 30.0], dtype=np.float64))
    d1.role = DroneRole.SURVEY
    d1.set_flight_mode(FlightMode.TRANSIT)

    d2 = Drone("RELAY_1", initial_pos=np.array([0.0, -100.0, 45.0], dtype=np.float64))
    d2.role = DroneRole.RELAY
    d2.set_flight_mode(FlightMode.RELAY)

    drones = {"SURVEY_1": d1, "RELAY_1": d2}
    pois = {
        "POI_A": {"id": "POI_A", "position": [10.0, 10.0, 20.0], "is_completed": True, "assigned_drone_id": None},
        "POI_B": {"id": "POI_B", "position": [30.0, 30.0, 20.0], "is_completed": True, "assigned_drone_id": None},
    }

    mgr.update(drones, pois, dt=0.05)
    assert d1.flight_mode == FlightMode.RTL
    assert d2.flight_mode == FlightMode.RTL


def test_core_trigger_fleet_and_drone_retreat():
    """Verify programmatic fleet and individual retreat calls on SwarmSimulationCore."""
    config = SimulationConfig(gcs_position=[0.0, -250.0, 0.0])
    core = SwarmSimulationCore(config)
    core.set_mission_manager(DisasterMissionManager(gcs_position=config.gcs_position))

    d1 = Drone("UAV_1", initial_pos=np.array([10.0, 10.0, 20.0], dtype=np.float64))
    d1.role = DroneRole.SURVEY
    d1.set_flight_mode(FlightMode.TRANSIT)
    core.add_drone(d1)

    d2 = Drone("UAV_2", initial_pos=np.array([20.0, 20.0, 20.0], dtype=np.float64))
    d2.role = DroneRole.SURVEY
    d2.set_flight_mode(FlightMode.TRANSIT)
    core.add_drone(d2)

    # Test individual retreat
    core.trigger_drone_retreat("UAV_1")
    assert d1.flight_mode == FlightMode.RTL
    assert d2.flight_mode == FlightMode.TRANSIT

    # Test fleet retreat
    core.trigger_fleet_retreat()
    assert d1.flight_mode == FlightMode.RTL
    assert d2.flight_mode == FlightMode.RTL


def test_controlled_landing_and_touchdown():
    """Verify precision landing approach and touchdown into LANDED mode with zero rotor speeds."""
    mgr = DisasterMissionManager(gcs_position=[0.0, -250.0, 0.0])
    pad = np.array([0.0, -250.0, 0.0], dtype=np.float64)

    # Drone arrives above pad in RTL mode
    drone = Drone("SURVEY_3", initial_pos=np.array([2.0, -251.0, 55.0], dtype=np.float64))
    drone.role = DroneRole.SURVEY
    drone.set_recovery_pad(pad)
    drone.set_flight_mode(FlightMode.RTL)

    pois = {}
    drones = {"SURVEY_3": drone}

    # Within approach arrival radius (dist_xy < 10m): transitions from RTL to LANDING
    mgr.update(drones, pois, dt=0.05)
    assert drone.flight_mode == FlightMode.LANDING

    # Simulate vertical descent near ground
    drone.position = np.array([0.05, -250.02, 0.20], dtype=np.float64)
    drone.velocity = np.array([0.0, 0.0, -0.3], dtype=np.float64)

    mgr.update(drones, pois, dt=0.05)
    assert drone.flight_mode == FlightMode.LANDED
    assert drone.position[2] == 0.0
    assert np.allclose(drone.velocity, 0.0)
    assert np.allclose(drone.rotor_speeds, 0.0)
