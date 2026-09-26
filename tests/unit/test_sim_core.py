"""
tests/unit/test_sim_core.py: Unit tests for SwarmSimulationCore master engine and telemetry serializer.
"""

from __future__ import annotations

import json
import time
import pytest
import numpy as np

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.obstacles import ObstacleAABB
from sim.types import (
    DroneLimits,
    DroneRole,
    FlightMode,
    TelemetrySnapshot,
)


@pytest.fixture
def default_config() -> SimulationConfig:
    return SimulationConfig(
        dt=0.05,
        max_duration=60.0,
        gcs_position=(0.0, -200.0, 0.0),
    )


@pytest.fixture
def swarm_core(default_config: SimulationConfig) -> SwarmSimulationCore:
    core = SwarmSimulationCore(config=default_config)
    # 2 Survey drones
    d1 = Drone(drone_id="UAV_1", role=DroneRole.SURVEY, initial_position=np.array([10.0, 10.0, 30.0]))
    d2 = Drone(drone_id="UAV_2", role=DroneRole.SURVEY, initial_position=np.array([20.0, 20.0, 30.0]))
    # 2 Relay drones
    r1 = Drone(drone_id="RELAY_1", role=DroneRole.RELAY, initial_position=np.array([0.0, -100.0, 70.0]))
    r2 = Drone(drone_id="RELAY_2", role=DroneRole.RELAY, initial_position=np.array([0.0, -50.0, 80.0]))

    for d in [d1, d2, r1, r2]:
        d.set_flight_mode(FlightMode.TRANSIT)
        core.add_drone(d)
    return core


def test_core_initialization(default_config: SimulationConfig):
    """Verify clean instantiation of default environment, clocks, and empty fleet."""
    core = SwarmSimulationCore(config=default_config)
    assert core.sim_time == 0.0
    assert core.step_count == 0
    assert len(core.drones) == 0
    assert np.allclose(core.config.gcs_position, [0.0, -200.0, 0.0])
    assert core.environment.bounds_x == (-250.0, 250.0)
    assert core.environment.bounds_z == (0.0, 120.0)


def test_drone_registration_duplicate():
    """Verify fleet management and duplicate ID prevention."""
    core = SwarmSimulationCore()
    d1 = Drone("UAV_1")
    core.add_drone(d1)
    assert "UAV_1" in core.drones
    assert core.drones["UAV_1"] is d1

    # Attempt duplicate registration
    with pytest.raises(ValueError, match="already registered"):
        core.add_drone(Drone("UAV_1"))

    # Removal
    removed = core.remove_drone("UAV_1")
    assert removed is d1
    assert "UAV_1" not in core.drones


def test_step_deterministic_clock():
    """Verify simulation time advances strictly deterministically by dt each tick."""
    core = SwarmSimulationCore(config=SimulationConfig(dt=0.05))
    snap1 = core.step()
    assert core.sim_time == pytest.approx(0.05)
    assert core.step_count == 1
    assert snap1.sim_time == pytest.approx(0.05)

    # Invalid dt
    with pytest.raises(ValueError, match="strictly positive"):
        core.step(dt=0.0)
    with pytest.raises(ValueError, match="strictly positive"):
        core.step(dt=-0.05)

    # 9 more steps
    for _ in range(9):
        core.step(0.05)
    assert core.sim_time == pytest.approx(0.50)
    assert core.step_count == 10


def test_bit_for_bit_reproducibility(default_config: SimulationConfig):
    """Verify two identical simulations produce identical states over 200 ticks."""
    core_a = SwarmSimulationCore(config=default_config)
    core_b = SwarmSimulationCore(config=default_config)

    for core in [core_a, core_b]:
        d = Drone("UAV_1", initial_position=np.array([10.0, 20.0, 30.0]))
        d.set_flight_mode(FlightMode.TRANSIT)
        d.set_target_waypoint(np.array([100.0, 100.0, 50.0]))
        core.add_drone(d)

    for _ in range(200):
        core_a.step(0.05)
        core_b.step(0.05)

    pos_a = core_a.drones["UAV_1"].position
    pos_b = core_b.drones["UAV_1"].position
    vel_a = core_a.drones["UAV_1"].velocity
    vel_b = core_b.drones["UAV_1"].velocity

    assert np.allclose(pos_a, pos_b, atol=1e-12)
    assert np.allclose(vel_a, vel_b, atol=1e-12)


def test_waypoint_seeking():
    """Verify drone moves monotonically toward target waypoint under APF guidance."""
    core = SwarmSimulationCore(config=SimulationConfig(dt=0.05))
    drone = Drone("UAV_SEEK", initial_position=np.array([0.0, 0.0, 30.0]))
    drone.set_flight_mode(FlightMode.TRANSIT)
    target = np.array([50.0, 0.0, 30.0])
    drone.set_target_waypoint(target)
    core.add_drone(drone)

    d_initial = float(np.linalg.norm(drone.position - target))
    for _ in range(20):
        core.step(0.05)

    d_final = float(np.linalg.norm(drone.position - target))
    # Distance must decrease
    assert d_final < d_initial
    assert drone.position[0] > 0.0
    assert drone.velocity[0] > 0.0


def test_ground_clamping_and_bounds():
    """Verify drone does not penetrate ground surface (Z >= 0) and stays within world bounds."""
    core = SwarmSimulationCore()
    drone = Drone("UAV_BOUND", initial_position=np.array([0.0, 0.0, 0.5]))
    drone.set_flight_mode(FlightMode.TRANSIT)
    drone.velocity = np.array([0.0, 0.0, -10.0])
    core.add_drone(drone)

    for _ in range(10):
        core.step(0.05)

    assert drone.position[2] >= 0.0
    assert drone.position[2] == pytest.approx(0.0, abs=1e-4)
    assert drone.velocity[2] >= 0.0


def test_separation_and_downwash():
    """Verify inter-drone separation force and downwash avoidance."""
    core = SwarmSimulationCore()
    # Two drones close horizontally (d = 1.5m)
    d1 = Drone("UAV_1", initial_position=np.array([0.0, 0.0, 30.0]))
    d2 = Drone("UAV_2", initial_position=np.array([1.5, 0.0, 30.0]))
    d1.set_flight_mode(FlightMode.TRANSIT)
    d2.set_flight_mode(FlightMode.TRANSIT)
    core.add_drone(d1)
    core.add_drone(d2)

    f1 = core.compute_steering_forces(d1)
    f2 = core.compute_steering_forces(d2)
    # d1 pushed along -X, d2 pushed along +X
    assert f1[0] < 0.0
    assert f2[0] > 0.0


def test_obstacle_repulsion_in_core():
    """Verify static obstacle repulsion pushes drone away in core loop."""
    core = SwarmSimulationCore()
    obs = ObstacleAABB("OBS_TEST", "Test Obs", np.array([-10.0, -10.0, 0.0]), np.array([10.0, 10.0, 40.0]))
    core.add_obstacle(obs)

    # Drone placed 2m outside the +X face (x=12)
    drone = Drone("UAV_OBS", initial_position=np.array([12.0, 0.0, 20.0]))
    drone.set_flight_mode(FlightMode.TRANSIT)
    core.add_drone(drone)

    force = core.compute_steering_forces(drone)
    # Repulsion pushes along +X (away from obstacle)
    assert force[0] > 0.0


def test_vsm_relay_positioning(default_config: SimulationConfig):
    """Verify Virtual Spring Mesh positions Relay UAVs in Layer 4 along line of sight."""
    core = SwarmSimulationCore(config=default_config)
    # Survey cluster centered around [100, 100, 30]
    s1 = Drone("S1", role=DroneRole.SURVEY, initial_position=np.array([90.0, 90.0, 30.0]))
    s2 = Drone("S2", role=DroneRole.SURVEY, initial_position=np.array([110.0, 110.0, 30.0]))
    # 2 Relays
    r1 = Drone("R1", role=DroneRole.RELAY, initial_position=np.array([0.0, -150.0, 75.0]))
    r2 = Drone("R2", role=DroneRole.RELAY, initial_position=np.array([0.0, -100.0, 85.0]))

    for d in [s1, s2, r1, r2]:
        d.set_flight_mode(FlightMode.TRANSIT)
        core.add_drone(d)

    core.update_vsm_relay_setpoints()

    t1 = r1.get_target_waypoint()
    t2 = r2.get_target_waypoint()

    assert t1 is not None and t2 is not None
    # GCS is at [0, -200, 0]. Centroid is at [100, 100].
    # R1 is at 1/3 (target Y between -200 and 100, X between 0 and 100)
    assert 0.0 < t1[0] < t2[0] < 100.0
    assert -200.0 < t1[1] < t2[1] < 100.0
    # Both targets strictly in Layer 4 ([70, 90]m)
    assert 70.0 <= t1[2] <= 90.0
    assert 70.0 <= t2[2] <= 90.0


def test_telemetry_snapshot_schema(swarm_core: SwarmSimulationCore):
    """Verify TelemetrySnapshot serialization produces compact JSON conforming to schema."""
    snap = swarm_core.step()
    assert isinstance(snap, TelemetrySnapshot)
    d = swarm_core.to_dict()

    required_keys = ["sim_time", "drones", "swarm", "gcs", "pois", "active_routes", "links", "packets", "metrics"]
    for k in required_keys:
        assert k in d

    json_str = swarm_core.to_json()
    assert len(json_str.encode("utf-8")) < 2500  # Compact frame size < 2.5 KB


def test_poi_registration_and_progress():
    """Verify registration and telemetry formatting of Points of Interest."""
    core = SwarmSimulationCore()
    core.add_poi("POI_HOSPITAL", position=[100.0, 120.0, 25.0], priority="HIGH", required_dwell_time=10.0)
    assert "POI_HOSPITAL" in core.pois
    snap = core.get_telemetry_snapshot()
    poi_entry = snap.pois[0]
    assert poi_entry["id"] == "POI_HOSPITAL"
    assert poi_entry["priority"] == "HIGH"
    assert poi_entry["progress"] == 0.0
    assert not poi_entry["is_completed"]


def test_headless_throughput(swarm_core: SwarmSimulationCore):
    """Benchmark: 200 simulation steps execute with high throughput (< 0.25 s)."""
    t0 = time.perf_counter()
    for _ in range(200):
        swarm_core.step(0.05)
    elapsed = time.perf_counter() - t0
    assert elapsed < 0.25  # Well within headless real-time performance budget


def test_downwash_vertical_separation_concentric():
    """
    Verify downwash cone behavior in SwarmSimulationCore when lower drone is
    directly beneath upper drone (dz = -4.0m, d_xy = 0.0m).
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    # Compute steering forces
    f_bot = core.compute_steering_forces(d_bot)
    f_top = core.compute_steering_forces(d_top)

    # Lower drone: lateral escape along +X must be non-zero and substantial
    assert f_bot[0] > 10.0, f"Lower drone did not receive sufficient lateral escape force: {f_bot}"
    # Lower drone: downward sink force must be negative
    assert f_bot[2] < 0.0, f"Lower drone did not receive downward downwash sink: {f_bot}"

    # Upper drone: downwash component on upper drone must be 0
    assert f_top[0] == pytest.approx(0.0, abs=1e-5)
    assert f_top[1] == pytest.approx(0.0, abs=1e-5)


def test_downwash_with_lateral_offset_in_core():
    """
    Verify downwash radial push on lower drone when laterally offset inside cone
    (dz = -4.0m, dx = 1.0m, cone radius = 4 * 0.4663 + 1.0 = 2.865m).
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([1.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    # Lower drone must be pushed outward along +X away from (0, 0)
    assert f_bot[0] > 0.0
    assert f_bot[2] < 0.0


def test_downwash_outside_cone_in_core():
    """
    Verify that when lower drone is outside the 25-degree downwash cone
    (dz = -4.0m, dx = 7.0m > r_cone = 2.865m), downwash force is 0.
    """
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([7.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    assert np.allclose(f_bot, 0.0)


def test_downwash_disabled_configuration():
    """Verify that when enable_downwash=False, no downwash force is computed."""
    core = SwarmSimulationCore(SimulationConfig(enable_downwash=False))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    f_bot = core.compute_steering_forces(d_bot)
    # Only vertical separation force exists (along -Z), zero lateral force
    assert f_bot[0] == pytest.approx(0.0, abs=1e-6)
    assert f_bot[1] == pytest.approx(0.0, abs=1e-6)


def test_downwash_dynamic_lateral_escape_step():
    """
    Verify full dynamic simulation step: lower drone escapes the concentric
    downwash column across successive simulation steps.
    """
    core = SwarmSimulationCore(SimulationConfig(dt=0.05, enable_downwash=True))
    d_top = Drone("UAV_TOP", initial_position=np.array([0.0, 0.0, 35.0]))
    d_bot = Drone("UAV_BOT", initial_position=np.array([0.0, 0.0, 31.0]))
    d_top.set_flight_mode(FlightMode.SURVEYING)
    d_bot.set_flight_mode(FlightMode.SURVEYING)
    core.add_drone(d_top)
    core.add_drone(d_bot)

    initial_x = d_bot.position[0]
    for _ in range(20):  # 1.0 second simulation
        core.step(0.05)

    # Lower drone must have moved significantly along +X (> 0.5 meter)
    assert d_bot.position[0] - initial_x > 0.5, f"Lower drone failed to dynamically escape downwash cone: {d_bot.position}"
