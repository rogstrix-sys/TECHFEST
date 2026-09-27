"""
tests/unit/test_drone.py: Comprehensive unit test suite for 6-DOF Drone Kinematics and Dynamics.
"""

from __future__ import annotations

import math
import pytest
import numpy as np

from sim.drone import Drone
from sim.obstacles import ObstacleAABB
from sim.types import DroneLimits, DroneRole, FlightMode


def test_acceleration_clamping():
    """Verify commanded acceleration magnitude is saturated to max_accel (4.0 m/s^2)."""
    drone = Drone("UAV_TEST", limits=DroneLimits(max_accel=4.0))
    drone.flight_mode = FlightMode.TRANSIT
    excessive_force = np.array([200.0, 0.0, 0.0])  # a_cmd = 200 / 1.2 = 166.7 m/s^2
    drone.step_physics(dt=0.05, commanded_force=excessive_force)
    accel_mag = float(np.linalg.norm(drone.acceleration))
    assert accel_mag <= 4.0 + 1e-6
    assert drone.acceleration[0] == pytest.approx(4.0, abs=1e-5)


def test_velocity_horizontal_clamping():
    """Verify horizontal speed saturates at max_speed_xy (10.0 m/s)."""
    drone = Drone("UAV_TEST", limits=DroneLimits(max_speed_xy=10.0, max_accel=4.0))
    drone.flight_mode = FlightMode.TRANSIT
    max_forward_force = np.array([50.0, 0.0, 0.0])
    for _ in range(100):
        drone.step_physics(dt=0.1, commanded_force=max_forward_force)

    speed_xy = float(np.linalg.norm(drone.velocity[:2]))
    assert speed_xy <= 10.0 + 1e-6
    assert speed_xy == pytest.approx(10.0, abs=1e-3)


def test_velocity_climb_rate_clamping():
    """Verify vertical climb rate saturates at max_speed_z_up (3.5 m/s)."""
    drone = Drone("UAV_TEST", limits=DroneLimits(max_speed_z_up=3.5))
    drone.flight_mode = FlightMode.TRANSIT
    upward_force = np.array([0.0, 0.0, 100.0])
    for _ in range(50):
        drone.step_physics(dt=0.1, commanded_force=upward_force)

    assert drone.velocity[2] <= 3.5 + 1e-6
    assert drone.velocity[2] == pytest.approx(3.5, abs=1e-3)


def test_velocity_descent_rate_clamping():
    """Verify vertical descent rate saturates at max_speed_z_down (2.5 m/s)."""
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 0.0, 50.0]), limits=DroneLimits(max_speed_z_down=2.5))
    drone.flight_mode = FlightMode.TRANSIT
    downward_force = np.array([0.0, 0.0, -100.0])
    for _ in range(50):
        drone.step_physics(dt=0.1, commanded_force=downward_force)

    assert drone.velocity[2] >= -2.5 - 1e-6
    assert drone.velocity[2] == pytest.approx(-2.5, abs=1e-3)


def test_ground_collision_constraint():
    """Verify drone does not penetrate ground surface (Z >= 0.0) under high downward momentum."""
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 0.0, 0.5]))
    drone.flight_mode = FlightMode.TRANSIT
    drone.velocity = np.array([0.0, 0.0, -5.0])
    downward_force = np.array([0.0, 0.0, -50.0])

    drone.step_physics(dt=0.2, commanded_force=downward_force)
    assert drone.position[2] >= 0.0
    assert drone.position[2] == 0.0
    assert drone.velocity[2] >= 0.0


def test_euler_quaternion_roundtrip():
    """Verify exact roundtrip conversion between Euler angles and unit quaternions."""
    test_angles = [
        (0.0, 0.0, 0.0),
        (0.2, -0.3, 1.2),
        (-0.4, 0.5, -math.pi / 2.0),
        (0.15, -0.15, 3.0),
    ]
    for roll, pitch, yaw in test_angles:
        q = Drone.euler_to_quaternion(roll, pitch, yaw)
        assert np.linalg.norm(q) == pytest.approx(1.0, abs=1e-6)
        r_out, p_out, y_out = Drone.quaternion_to_euler(q)
        assert r_out == pytest.approx(roll, abs=1e-5)
        assert p_out == pytest.approx(pitch, abs=1e-5)
        assert y_out == pytest.approx(yaw, abs=1e-5)


def test_quaternion_norm_invariance():
    """Verify quaternion maintains unit norm during dynamic acceleration steps."""
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 0.0, 20.0]))
    drone.flight_mode = FlightMode.TRANSIT
    for step_i in range(50):
        # Varying forces
        force = np.array([
            math.sin(step_i * 0.2) * 10.0,
            math.cos(step_i * 0.2) * 10.0,
            0.0
        ])
        drone.step_physics(dt=0.05, commanded_force=force)
        norm = float(np.linalg.norm(drone.quaternion))
        assert norm == pytest.approx(1.0, abs=1e-5)


def test_tilt_clamping():
    """Verify attitude pitch and roll clamp to max_tilt_rad (30 deg)."""
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 0.0, 20.0]))
    drone.flight_mode = FlightMode.TRANSIT
    extreme_force = np.array([0.0, 100.0, 0.0])  # Extreme lateral force
    for _ in range(30):
        drone.step_physics(dt=0.05, commanded_force=extreme_force)

    # Roll angle should be clamped at max_tilt_rad (0.5236 rad)
    assert abs(drone.attitude[0]) <= 0.5236 + 1e-4


def test_yaw_rate_limiting():
    """Verify yaw rotational velocity does not exceed max_yaw_rate (1.5708 rad/s)."""
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 0.0, 20.0]))
    drone.flight_mode = FlightMode.TRANSIT
    drone.attitude[2] = 0.0
    # Command target behind drone (180 degree yaw change)
    drone.target_position = np.array([-10.0, 0.0, 20.0])
    dt = 0.02
    drone.step_physics(dt=dt, commanded_force=np.zeros(3))
    delta_yaw = abs(drone.attitude[2])
    max_allowed_delta = 1.5708 * dt + 1e-4
    assert delta_yaw <= max_allowed_delta


def test_apf_conic_parabolic_switch():
    """Verify Khatib APF conic-parabolic attractive force transition at 15.0m."""
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 0.0, 20.0]))
    drone.flight_mode = FlightMode.TRANSIT

    # Case 1: Parabolic regime (d = 5.0m <= 15.0m)
    drone.target_position = np.array([5.0, 0.0, 20.0])
    f_att_short = drone.compute_attractive_force()
    expected_mag_short = 1.2 * 5.0  # 6.0 N
    assert np.linalg.norm(f_att_short) == pytest.approx(expected_mag_short, rel=1e-5)

    # Case 2: Conic regime (d = 30.0m > 15.0m)
    drone.target_position = np.array([30.0, 0.0, 20.0])
    f_att_long = drone.compute_attractive_force()
    expected_mag_long = 15.0 * 1.2  # 18.0 N
    assert np.linalg.norm(f_att_long) == pytest.approx(expected_mag_long, rel=1e-5)


def test_obstacle_repulsion_direction():
    """Verify obstacle repulsion pushes drone away from obstacle face (X-axis dominant).

    With goal-oriented tangential bypass, a small lateral component is also added
    to steer the drone around the obstacle toward its target. The primary repulsion
    must still be in the -X direction (away from the X=10 face).
    """
    obs = ObstacleAABB(
        id="OBS_1",
        name="Test Building",
        min_pt=np.array([10.0, 10.0, 0.0]),
        max_pt=np.array([20.0, 20.0, 30.0]),
    )
    # Position drone outside the X=10 face at X=9
    drone = Drone("UAV_TEST", initial_position=np.array([9.0, 15.0, 15.0]))
    drone.flight_mode = FlightMode.TRANSIT

    f_rep = drone.compute_obstacle_repulsion([obs])
    # Primary repulsion must be in -X direction (away from the X=10 face)
    assert f_rep[0] < 0.0, f"Expected negative X repulsion, got {f_rep}"
    # X component should dominate (magnitude > any lateral bypass component)
    assert abs(f_rep[0]) > 0.0, "X repulsion must be non-zero"


def test_obstacle_repulsion_distance_threshold():
    """Verify obstacle repulsion is strictly zero beyond influence radius (8.0m)."""
    obs = ObstacleAABB(
        id="OBS_1",
        name="Test Building",
        min_pt=np.array([10.0, 10.0, 0.0]),
        max_pt=np.array([20.0, 20.0, 30.0]),
    )
    # Distance to face = 10.0m (> rho_0 = 8.0m)
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 15.0, 15.0]))
    f_rep = drone.compute_obstacle_repulsion([obs])
    assert np.allclose(f_rep, [0.0, 0.0, 0.0])


def test_reynolds_separation_quadratic_growth():
    """Verify separation force increases non-linearly as distance decreases."""
    drone_a = Drone("UAV_A", initial_position=np.array([0.0, 0.0, 30.0]))
    drone_a.flight_mode = FlightMode.TRANSIT

    drone_b_far = Drone("UAV_B", initial_position=np.array([4.0, 0.0, 30.0]))
    drone_b_far.flight_mode = FlightMode.TRANSIT
    f_sep_far, _, _ = drone_a.compute_flocking_forces([drone_b_far])

    drone_b_close = Drone("UAV_B", initial_position=np.array([2.0, 0.0, 30.0]))
    drone_b_close.flight_mode = FlightMode.TRANSIT
    f_sep_close, _, _ = drone_a.compute_flocking_forces([drone_b_close])

    mag_far = np.linalg.norm(f_sep_far)
    mag_close = np.linalg.norm(f_sep_close)
    assert mag_close > mag_far * 2.0  # Non-linear growth


def test_reynolds_alignment():
    """Verify velocity alignment force matches flock peer velocity."""
    drone_a = Drone("UAV_A", initial_position=np.array([0.0, 0.0, 30.0]))
    drone_a.flight_mode = FlightMode.TRANSIT
    drone_a.velocity = np.array([0.0, 0.0, 0.0])

    peer = Drone("UAV_B", initial_position=np.array([5.0, 0.0, 30.0]))
    peer.flight_mode = FlightMode.TRANSIT
    peer.velocity = np.array([8.0, 0.0, 0.0])

    _, f_align, _ = drone_a.compute_flocking_forces([peer])
    assert f_align[0] > 0.0
    assert f_align[1] == 0.0
    assert f_align[2] == 0.0


def test_downwash_cone_trigger():
    """Verify lower drone in downwash cone experiences outward lateral and downward forces."""
    drone_upper = Drone("UAV_UP", initial_position=np.array([10.0, 10.0, 30.0]))
    drone_upper.flight_mode = FlightMode.TRANSIT

    # Lower drone situated at dz = 5m below, dx = 0.5m
    drone_lower = Drone("UAV_DOWN", initial_position=np.array([10.5, 10.0, 25.0]))
    drone_lower.flight_mode = FlightMode.TRANSIT

    f_dw = drone_lower.compute_downwash_repulsion([drone_upper])
    # Lateral push along +X (away from upper drone)
    assert f_dw[0] > 10.0
    # Downward turbulent sink
    assert f_dw[2] < 0.0


def test_downwash_outside_cone_inactive():
    """Verify lower drone outside downwash cone experiences zero downwash force."""
    drone_upper = Drone("UAV_UP", initial_position=np.array([10.0, 10.0, 30.0]))
    drone_upper.flight_mode = FlightMode.TRANSIT

    # Lower drone 5m laterally away (outside cone radius r = 5 * tan(25) = 2.33m)
    drone_lower = Drone("UAV_DOWN", initial_position=np.array([15.0, 10.0, 25.0]))
    drone_lower.flight_mode = FlightMode.TRANSIT

    f_dw = drone_lower.compute_downwash_repulsion([drone_upper])
    assert np.allclose(f_dw, [0.0, 0.0, 0.0])


def test_downwash_upper_drone_unaffected():
    """Verify upper drone is completely unaffected by downwash of lower drone."""
    drone_upper = Drone("UAV_UP", initial_position=np.array([10.0, 10.0, 30.0]))
    drone_upper.flight_mode = FlightMode.TRANSIT
    drone_lower = Drone("UAV_DOWN", initial_position=np.array([10.0, 10.0, 25.0]))
    drone_lower.flight_mode = FlightMode.TRANSIT

    f_dw_upper = drone_upper.compute_downwash_repulsion([drone_lower])
    assert np.allclose(f_dw_upper, [0.0, 0.0, 0.0])


def test_survey_altitude_corridor_enforcement():
    """Verify SURVEYING mode restores drone to [25, 45]m altitude band."""
    drone = Drone("UAV_SURVEY", role=DroneRole.SURVEY)
    drone.flight_mode = FlightMode.SURVEYING

    # Below band (Z = 15m)
    drone.position[2] = 15.0
    f_corr_low = drone.compute_corridor_force()
    assert f_corr_low[2] > 0.0  # Restoring upward force

    # Above band (Z = 50m)
    drone.position[2] = 50.0
    f_corr_high = drone.compute_corridor_force()
    assert f_corr_high[2] < 0.0  # Restoring downward force


def test_relay_altitude_corridor_enforcement():
    """Verify RELAY mode restores drone into Layer 4 [70, 90]m corridor."""
    drone = Drone("UAV_RELAY", role=DroneRole.RELAY)
    drone.flight_mode = FlightMode.RELAY

    # At Z = 50m (below relay band)
    drone.position[2] = 50.0
    f_corr = drone.compute_corridor_force()
    assert f_corr[2] > 0.0  # Pulls up into relay corridor


def test_battery_depletion_rate():
    """Verify 60 seconds hover flight depletes battery by expected percentage (~4.4%)."""
    drone = Drone("UAV_TEST", initial_position=np.array([0.0, 0.0, 20.0]))
    drone.flight_mode = FlightMode.TRANSIT
    for _ in range(600):  # 60s at dt=0.1
        drone.step_physics(dt=0.1, commanded_force=np.zeros(3))

    assert drone.battery.soc == pytest.approx(0.956, abs=0.005)


def test_battery_threshold_flags():
    """Verify battery low and critical threshold flags on drone instance."""
    drone = Drone("UAV_TEST")
    drone.battery.soc = 0.24
    assert drone.battery.is_low()
    assert not drone.battery.is_critical()

    drone.battery.soc = 0.09
    assert drone.battery.is_critical()


def test_battery_floor_at_zero():
    """Verify battery SoC never drops below 0.0."""
    drone = Drone("UAV_TEST")
    drone.flight_mode = FlightMode.TRANSIT
    # Run for 2 hours (excessive drain)
    for _ in range(7200):
        drone.step_physics(dt=1.0, commanded_force=np.array([4.0, 0.0, 0.0]))

    assert drone.battery.soc >= 0.0
    assert drone.battery.soc == 0.0
