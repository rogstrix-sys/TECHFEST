"""
tests/unit/test_next_gen_features.py: Comprehensive unit tests for:
- Tactical Visual Comms Chatter Feed
- Synthetic Thermal AI Survivor Detection (SAR)
- Dynamic Relay Spring-Tethering
- Automated GCS Battery Hot-Swap & Redeployment
- Dryden Atmospheric Wind & Turbulence Telemetry
"""

import numpy as np
import pytest
from sim.core import SwarmSimulationCore, SimulationConfig
from sim.drone import Drone, DroneLimits
from sim.mission import DisasterMissionManager
from sim.types import DroneRole, FlightMode, BatteryModel, TacticalCommsEvent, SurvivorRecord
from sim.weather import DrydenTurbulenceModel, WindConfig


def test_tactical_comms_logging():
    """Verify DisasterMissionManager logs and limits tactical comms events."""
    mgr = DisasterMissionManager(mission_time_budget=300.0)
    assert len(mgr.tactical_comms) >= 1  # System online message

    # Emit multiple tactical events
    mgr.emit_tactical_comms("WARN", "SCOUT_1", "⚠️ Comms link degraded", "COMMS")
    mgr.emit_tactical_comms("SUCCESS", "GCS", "🎯 P1 Disaster Site Survey Complete", "MISSION")
    mgr.emit_tactical_comms("SUCCESS", "GCS", "👤 SURVIVOR LOCATED at POI_1 (IR Heat 38.2°C)", "SAR")
    mgr.emit_tactical_comms("INFO", "RELAY_2", "🪫 Docked at PAD_1 for LiPo Hot-Swap", "SWAP")

    history = mgr.get_tactical_comms_telemetry()
    assert len(history) >= 4
    categories = [e["category"] for e in history]
    assert "COMMS" in categories
    assert "SAR" in categories
    assert "SWAP" in categories

    # Test history cap
    for i in range(60):
        mgr.emit_tactical_comms("INFO", "TEST", f"Message {i}")
    assert len(mgr.tactical_comms) <= mgr.max_comms_history


def test_survivor_seeding_and_discovery():
    """Verify simulated trapped human thermal heat signatures are seeded and discovered."""
    mgr = DisasterMissionManager()
    pois = {
        "POI_CRIT": {
            "id": "POI_CRIT",
            "position": np.array([50.0, 50.0, 0.0]),
            "priority": "CRITICAL",
            "required_dwell_time": 10.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
        }
    }

    # Initialize survivors
    mgr._init_survivors(pois)
    surv_telemetry = mgr.get_survivors_telemetry()
    assert surv_telemetry["total_count"] == 2  # 2 survivors for CRITICAL
    assert surv_telemetry["located_count"] == 0
    assert surv_telemetry["pending_count"] == 2

    # Create surveyor drone
    drone = Drone("SCOUT_1", initial_position=np.array([52.0, 51.0, 15.0]))
    drone.role = DroneRole.SURVEY
    drone.flight_mode = FlightMode.SURVEYING
    drone.assigned_poi_id = "POI_CRIT"

    # Step FSM to simulate dwell inspection
    pois["POI_CRIT"]["current_dwell_time"] = 3.0
    mgr._update_drone_fsm(drone, pois, dt=0.5)

    updated_telemetry = mgr.get_survivors_telemetry()
    assert updated_telemetry["located_count"] >= 1
    found_survivor = updated_telemetry["discovered_survivors"][0]
    assert found_survivor["poi_id"] == "POI_CRIT"
    assert found_survivor["heat_c"] >= 36.5
    assert found_survivor["confidence"] >= 0.85
    assert found_survivor["discovered_by"] == "SCOUT_1"


def test_dynamic_relay_spring_tethering():
    """Verify relay drones dynamically position themselves toward surveyor centroid."""
    mgr = DisasterMissionManager(gcs_position=(0.0, -250.0, 0.0))

    # Scouts operating deep in the north disaster sector (y = +150)
    scout1 = Drone("SCOUT_1", initial_position=np.array([20.0, 150.0, 25.0]))
    scout1.role = DroneRole.SURVEY
    scout1.flight_mode = FlightMode.SURVEYING

    scout2 = Drone("SCOUT_2", initial_position=np.array([-20.0, 170.0, 25.0]))
    scout2.role = DroneRole.SURVEY
    scout2.flight_mode = FlightMode.SURVEYING

    relay1 = Drone("RELAY_1", initial_position=np.array([0.0, 0.0, 60.0]))
    relay1.role = DroneRole.RELAY
    relay1.flight_mode = FlightMode.RELAY

    relay2 = Drone("RELAY_2", initial_position=np.array([0.0, 50.0, 70.0]))
    relay2.role = DroneRole.RELAY
    relay2.flight_mode = FlightMode.RELAY

    drones = {
        "SCOUT_1": scout1,
        "SCOUT_2": scout2,
        "RELAY_1": relay1,
        "RELAY_2": relay2,
    }

    mgr._apply_relay_spring_tethering(drones)

    # Relays should have target waypoints pulled along GCS (y=-250) to Scout Centroid (y=+160)
    assert relay1.target_position is not None, "relay1 must have a target position"
    assert relay2.target_position is not None, "relay2 must have a target position"
    # Both relays should be positioned between GCS and scout centroid
    # (multi-sector logic splits relays into West/East corridors at elevated altitude)
    assert -250.0 < relay1.target_position[1] < 200.0, f"relay1 Y out of range: {relay1.target_position[1]}"
    assert -250.0 < relay2.target_position[1] < 200.0, f"relay2 Y out of range: {relay2.target_position[1]}"
    # Both relays should be at elevated altitude (75m+) for obstacle clearance
    assert relay1.target_position[2] >= 70.0, f"relay1 not at obstacle-clearing altitude: {relay1.target_position[2]}"
    assert relay2.target_position[2] >= 70.0, f"relay2 not at obstacle-clearing altitude: {relay2.target_position[2]}"


def test_autonomous_battery_hotswap_and_redeployment():
    """Verify low battery drone lands at base, enters hot-swap, restores 100% SoC, and takes off."""
    mgr = DisasterMissionManager(gcs_position=(0.0, -250.0, 0.0))

    drone = Drone("SCOUT_1", initial_position=np.array([-15.0, -250.0, 0.0]))
    drone.role = DroneRole.SURVEY
    drone.battery = BatteryModel(capacity_mah=4500.0, soc=0.15)  # 15% SoC (low)
    drone.flight_mode = FlightMode.LANDED
    drone.is_low_battery_rtb = True

    pois = {}
    # Touchdown on ground -> should transition to DOCKED_SWAPPING
    mgr._update_drone_fsm(drone, pois, dt=0.05)

    assert drone.flight_mode == FlightMode.DOCKED_SWAPPING
    assert hasattr(drone, "_swap_timer")
    assert drone._swap_timer > 0.0

    # Advance swap timer by 26 seconds to complete hot-swap
    for _ in range(55):
        mgr._update_drone_fsm(drone, pois, dt=0.5)

    # After timer completes: battery must be 100% and drone should takeoff
    assert drone.flight_mode == FlightMode.TAKEOFF
    assert drone.battery.soc >= 0.99
    assert not drone.is_low_battery_rtb
    assert drone.target_position is not None
    assert drone.target_position[2] > 0.0  # Ascending


def test_weather_and_parasite_drag_coupling():
    """Verify Dryden wind alters relative airspeed and scales LiPo battery draw."""
    wind_cfg = WindConfig(mean_speed_mps=8.0, direction_deg=90.0, turbulence_intensity="MODERATE")
    weather = DrydenTurbulenceModel(config=wind_cfg)

    pos = np.array([0.0, 0.0, 20.0])
    vel = np.array([0.0, 0.0, 0.0])
    wind_sample = weather.sample(pos, vel, dt=0.05)
    assert len(wind_sample) == 3
    wind_speed = float(np.linalg.norm(wind_sample))
    assert wind_speed > 2.0  # Substantial wind present

    # Test battery model with zero wind vs headwind
    bat_calm = BatteryModel()
    bat_wind = BatteryModel()

    # Hovering stationary in calm air: relative airspeed = 0
    bat_calm.step(dt=1.0, speed=0.0, airspeed=0.0, altitude=20.0, climb_rate=0.0)

    # Flying / hovering in severe 14.5 m/s headwind: relative airspeed = 14.5 m/s
    bat_wind.step(dt=1.0, speed=0.0, airspeed=14.5, altitude=20.0, climb_rate=0.0)

    # Parasite drag in 14.5 m/s wind causes severe power draw (> 240 W vs hover ~195 W)
    assert bat_wind.current_power_w > bat_calm.current_power_w
    assert bat_wind.last_p_parasite > bat_calm.last_p_parasite
    assert bat_wind.soc < bat_calm.soc
