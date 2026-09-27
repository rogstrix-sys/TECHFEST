"""
tests/unit/test_challenge.py: Unit tests for MeitY / IIT Bombay / IISER Bhopal UAV Swarm Challenge Engine.
Verifies all 11 official competition constraints and compliance monitoring logic.
"""

import numpy as np
import pytest

from sim.challenge import (
    ChallengeComplianceMonitor,
    ChallengeMissionConstraints,
    ChallengePOI,
    DynamicPoiSpawner,
    create_challenge_simulation,
)
from sim.drone import Drone
from sim.types import DroneLimits, DroneRole, FlightMode


def test_challenge_constraints_defaults():
    """Verify all 11 official challenge constraints are encoded with exact parameters."""
    c = ChallengeMissionConstraints()
    assert c.mission_duration_s == 2700.0  # 45 mins
    assert c.uav_max_flight_time_s == 1200.0  # 20 mins
    assert c.max_comm_range_m == 100.0  # 100 m
    assert c.operational_area_w_m == 1000.0  # 1000 m
    assert c.operational_area_l_m == 1000.0  # 1000 m
    assert c.operational_center_offset_m == 75.0  # 75 m
    assert c.max_altitude_m == 100.0  # 100 m
    assert c.max_speed_m_s == 5.0  # 5 m/s
    assert c.min_separation_m == 20.0  # 20 m
    assert c.max_poi_reporting_latency_s == 10.0  # 10 s SLA
    assert c.num_pois == 10  # 10 POIs

    d = c.to_dict()
    assert d["mission_duration_s"] == 2700.0
    assert d["max_speed_m_s"] == 5.0
    assert d["min_separation_m"] == 20.0


def test_dynamic_poi_spawner():
    """Verify dynamic POI spawner generates 10 POIs inside 1000m arena with progressive activation."""
    spawner = DynamicPoiSpawner(num_pois=10, seed=123)
    assert len(spawner.pois) == 10

    # Verify all POIs are within the 1000m x 1000m arena
    for poi in spawner.pois.values():
        assert 0.0 <= poi.position[0] <= 1000.0
        assert -500.0 <= poi.position[1] <= 500.0
        assert 0.0 <= poi.position[2] <= 100.0

    core_pois = {}
    # At t=0, only initial batch should spawn
    spawned_t0 = spawner.update(0.0, core_pois)
    assert len(spawned_t0) == 3
    assert len(core_pois) == 3

    # At t=350, all 10 should be spawned
    spawned_t350 = spawner.update(350.0, core_pois)
    assert len(core_pois) == 10
    for p in spawner.pois.values():
        assert p.is_spawned is True


def test_compliance_monitor_speed_check():
    """Verify compliance monitor detects overspeed violation (>5.0 m/s)."""
    monitor = ChallengeComplianceMonitor()
    d1 = Drone("UAV_1", limits=DroneLimits(max_speed_xy=10.0))
    d1.set_flight_mode(FlightMode.TRANSIT)
    d1.velocity = np.array([5.5, 0.0, 0.0])  # > 5.0 m/s
    spawner = DynamicPoiSpawner(num_pois=2)

    res = monitor.evaluate_step(
        sim_time=10.0,
        drones={"UAV_1": d1},
        spawner=spawner,
        active_links=[],
        gcs_position=np.array([-75.0, 0.0, 0.0]),
    )
    assert res["is_fully_compliant"] is False
    assert res["violations"]["speed"] > 0
    assert monitor.max_observed_speed_m_s == pytest.approx(5.5, rel=1e-3)


def test_compliance_monitor_altitude_check():
    """Verify compliance monitor detects ceiling breach (>100.0 m)."""
    monitor = ChallengeComplianceMonitor()
    d1 = Drone("UAV_1")
    d1.set_flight_mode(FlightMode.SURVEYING)
    d1.position = np.array([100.0, 50.0, 105.0])  # > 100m
    spawner = DynamicPoiSpawner(num_pois=2)

    res = monitor.evaluate_step(
        sim_time=10.0,
        drones={"UAV_1": d1},
        spawner=spawner,
        active_links=[],
        gcs_position=np.array([-75.0, 0.0, 0.0]),
    )
    assert res["is_fully_compliant"] is False
    assert res["violations"]["altitude"] > 0
    assert monitor.max_observed_altitude_m == pytest.approx(105.0, rel=1e-3)


def test_compliance_monitor_separation_check():
    """Verify compliance monitor detects drone inter-vehicle proximity violation (<20.0 m)."""
    monitor = ChallengeComplianceMonitor()
    d1 = Drone("UAV_1", initial_pos=np.array([100.0, 0.0, 30.0]))
    d2 = Drone("UAV_2", initial_pos=np.array([110.0, 0.0, 30.0]))  # 10m apart < 20m
    d1.set_flight_mode(FlightMode.TRANSIT)
    d2.set_flight_mode(FlightMode.TRANSIT)
    spawner = DynamicPoiSpawner(num_pois=2)

    res = monitor.evaluate_step(
        sim_time=10.0,
        drones={"UAV_1": d1, "UAV_2": d2},
        spawner=spawner,
        active_links=[],
        gcs_position=np.array([-75.0, 0.0, 0.0]),
    )
    assert res["is_fully_compliant"] is False
    assert res["violations"]["separation"] > 0
    assert monitor.min_observed_separation_m == pytest.approx(10.0, rel=1e-3)


def test_compliance_monitor_poi_sla_check():
    """Verify compliance monitor detects POI reporting SLA violation (>10s)."""
    monitor = ChallengeComplianceMonitor()
    spawner = DynamicPoiSpawner(num_pois=2)
    poi = spawner.pois["POI_1"]
    poi.is_detected = True
    poi.detection_time = 10.0
    poi.is_reported = True
    poi.report_time = 25.0
    poi.reporting_latency_s = 15.0  # > 10s SLA
    poi.is_sla_compliant = False

    res = monitor.evaluate_step(
        sim_time=30.0,
        drones={},
        spawner=spawner,
        active_links=[],
        gcs_position=np.array([-75.0, 0.0, 0.0]),
    )
    assert res["is_fully_compliant"] is False
    assert res["violations"]["poi_latency"] > 0


def test_challenge_simulation_factory_and_compliance():
    """Verify full factory simulation builds compliant swarm that steps cleanly without violations."""
    sim, spawner, monitor = create_challenge_simulation(seed=42)

    # 16 drones total
    assert len(sim.drones) == 16
    # GCS positioned 75m outside arena
    assert sim.config.gcs_position[0] == -75.0
    assert sim.environment.config.gcs_position[0] == -75.0

    # Verify all initial pads are spaced >= 20m apart
    positions = [d.position for d in sim.drones.values()]
    for i in range(len(positions)):
        for j in range(i + 1, len(positions)):
            dist = float(np.linalg.norm(positions[i] - positions[j]))
            assert dist >= 20.0, f"Pads too close: {dist}m"

    # Step simulation 100 times (5.0s sim time)
    for _ in range(100):
        snapshot = sim.step(0.05)

    comp = snapshot.challenge_constraints
    assert comp is not None
    assert comp["is_fully_compliant"] is True
    assert comp["violations"]["speed"] == 0
    assert comp["violations"]["altitude"] == 0
    assert comp["violations"]["separation"] == 0
    assert comp["violations"]["comm_range"] == 0
    assert comp["violations"]["poi_latency"] == 0
    assert comp["metrics"]["max_speed_m_s"] <= 5.0
    assert comp["metrics"]["max_altitude_m"] <= 100.0
    assert comp["metrics"]["min_separation_m"] >= 20.0
