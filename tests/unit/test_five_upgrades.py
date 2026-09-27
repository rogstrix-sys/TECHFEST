"""
tests/unit/test_five_upgrades.py: Test suite for 5 Key Upgrades:
1. Swarm Self-Healing Chaos Fault Injection
2. 1-Click 3D Point Cloud Export (.PLY format)
3. Manual FPV Controller Mode (Velocity Tracking)
4. V-Formation Aerodynamic Wake Drafting & Power Reduction
5. REST API Endpoints for Chaos, Point Cloud, and Manual Control
"""

import numpy as np
import pytest

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.mapping import LiDARScan, LiDARPoint, OccupancyGridMap3D
from sim.mission import DisasterMissionManager
from sim.types import DroneRole, FlightMode


def test_swarm_chaos_fault_injection():
    """Verify dynamic hardware fault injection, flameout physics, and task release."""
    gcs = (0.0, -250.0, 0.0)
    mission = DisasterMissionManager(gcs_position=gcs)

    # Create 3 drones
    d1 = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, 50.0, 30.0]))
    d2 = Drone("UAV_2", role=DroneRole.SURVEY, initial_pos=np.array([20.0, 60.0, 30.0]))
    r1 = Drone("RELAY_1", role=DroneRole.RELAY, initial_pos=np.array([0.0, -100.0, 70.0]))
    drones = {d1.id: d1, d2.id: d2, r1.id: r1}

    pois = {
        "POI_TEST": {
            "id": "POI_TEST",
            "position": np.array([0.0, 55.0, 20.0]),
            "priority": "HIGH",
            "required_dwell_time": 10.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
            "assigned_drone_id": "UAV_1",
        }
    }
    d1.assigned_poi_id = "POI_TEST"
    d1.set_flight_mode(FlightMode.SURVEYING)

    mission._last_pois = pois

    # Inject fault on UAV_1
    victim_id = mission.trigger_chaos_fault(drones, target_drone_id="UAV_1")
    assert victim_id == "UAV_1"
    assert d1.is_fault_injected is True
    assert d1.flight_mode == FlightMode.EMERGENCY_LAND
    assert d1.assigned_poi_id is None

    # Verify PoI was released for self-healing reassignment
    assert pois["POI_TEST"]["assigned_drone_id"] != "UAV_1"

    # Verify physical free-fall when fault is injected
    z_init = float(d1.position[2])
    d1.step(dt=0.1)
    # With motor flameout, drone accelerates downward under gravity
    assert float(d1.position[2]) < z_init


def test_point_cloud_ply_export():
    """Verify Stanford ASCII PLY export format, vertex count, and RGB altitude colors."""
    grid = OccupancyGridMap3D(voxel_size_m=4.0)

    # Insert a synthetic scan with points
    scan = LiDARScan(
        timestamp=1.0,
        drone_id="UAV_1",
        sensor_origin=np.array([0.0, 0.0, 25.0]),
        points=[
            LiDARPoint(x=10.0, y=15.0, z=5.0, range_m=18.0, intensity=0.8),
            LiDARPoint(x=20.0, y=25.0, z=15.0, range_m=28.0, intensity=0.9),
            LiDARPoint(x=30.0, y=35.0, z=30.0, range_m=45.0, intensity=0.6),
        ]
    )
    grid.insert_scan(scan)

    ply_str = grid.export_point_cloud_ply()
    assert isinstance(ply_str, str)
    assert ply_str.startswith("ply\n")
    assert "format ascii 1.0" in ply_str
    assert "element vertex" in ply_str
    assert "property float x" in ply_str
    assert "property float y" in ply_str
    assert "property float z" in ply_str
    assert "property uchar red" in ply_str
    assert "property uchar green" in ply_str
    assert "property uchar blue" in ply_str
    assert "property float intensity" in ply_str
    assert "end_header" in ply_str

    lines = ply_str.strip().split("\n")
    header_end = lines.index("end_header")
    data_lines = lines[header_end + 1:]
    assert len(data_lines) >= 3

    # Check first data line format: x y z r g b intensity
    first_pt = data_lines[0].split()
    assert len(first_pt) == 7
    float(first_pt[0])  # x
    float(first_pt[1])  # y
    float(first_pt[2])  # z
    r, g, b = int(first_pt[3]), int(first_pt[4]), int(first_pt[5])
    assert 0 <= r <= 255
    assert 0 <= g <= 255
    assert 0 <= b <= 255


def test_point_cloud_las_export():
    """Verify ASPRS LAS 1.2 binary export format, header signature, and record packing."""
    grid = OccupancyGridMap3D(voxel_size_m=4.0)

    scan = LiDARScan(
        timestamp=1.0,
        drone_id="UAV_1",
        sensor_origin=np.array([0.0, 0.0, 25.0]),
        points=[
            LiDARPoint(x=10.0, y=15.0, z=5.0, range_m=18.0, intensity=0.8),
            LiDARPoint(x=20.0, y=25.0, z=15.0, range_m=28.0, intensity=0.9),
            LiDARPoint(x=30.0, y=35.0, z=30.0, range_m=45.0, intensity=0.6),
        ]
    )
    grid.insert_scan(scan)

    las_bytes = grid.export_point_cloud_las()
    assert isinstance(las_bytes, bytes)
    assert len(las_bytes) >= 227 + 3 * 26

    # Verify LAS 1.2 Magic Header Signature
    assert las_bytes[:4] == b"LASF"
    # Version 1.2
    assert las_bytes[24] == 1
    assert las_bytes[25] == 2
    # System Identifier
    assert b"UAV-X AUTONOMOUS SLAM" in las_bytes[:60]
    # Header size 227
    header_size = int.from_bytes(las_bytes[94:96], "little")
    assert header_size == 227



def test_manual_fpv_velocity_control():
    """Verify manual override control, velocity tracking, and mode toggling."""
    drone = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, 0.0, 20.0]))
    assert drone.is_manual_override is False

    # Engage manual mode with target velocity [5.0, 0.0, 1.0]
    drone.set_manual_control(vx=5.0, vy=0.0, vz=1.0, enabled=True)
    assert drone.is_manual_override is True
    assert drone.flight_mode == FlightMode.MANUAL
    assert np.allclose(drone.manual_vel_cmd, [5.0, 0.0, 1.0])

    # Step physics for several steps; velocity should accelerate towards commanded velocity
    for _ in range(10):
        drone.step(dt=0.05)

    assert drone.velocity[0] > 1.0
    assert drone.velocity[2] > 0.1

    # Disengage manual mode
    drone.set_manual_control(enabled=False)
    assert drone.is_manual_override is False
    assert drone.flight_mode == FlightMode.TRANSIT


def test_v_formation_aerodynamic_drafting():
    """Verify tip-vortex upwash energy savings calculation in V-formation corridors."""
    mission = DisasterMissionManager(gcs_position=(0.0, -250.0, 0.0))

    # Leader drone flying north at 8 m/s
    leader = Drone("LEADER", role=DroneRole.SURVEY, initial_pos=np.array([0.0, 50.0, 25.0]))
    leader.velocity = np.array([0.0, 8.0, 0.0], dtype=np.float64)
    leader.flight_mode = FlightMode.TRANSIT

    # Follower drone positioned in V-formation sweet spot:
    # 8 meters behind (y = 42.0), 2.5 meters to the right (x = 2.5), same altitude
    follower = Drone("FOLLOWER", role=DroneRole.SURVEY, initial_pos=np.array([2.5, 42.0, 25.0]))
    follower.velocity = np.array([0.0, 8.0, 0.0], dtype=np.float64)
    follower.flight_mode = FlightMode.TRANSIT

    # Outsider drone far away
    outsider = Drone("OUTSIDER", role=DroneRole.SURVEY, initial_pos=np.array([100.0, -100.0, 25.0]))
    outsider.velocity = np.array([0.0, 8.0, 0.0], dtype=np.float64)
    outsider.flight_mode = FlightMode.TRANSIT

    drones = {leader.id: leader, follower.id: follower, outsider.id: outsider}

    # Evaluate drafting
    mission._apply_formation_drafting(drones)

    assert follower.is_drafting is True
    assert follower.drafting_leader_id == "LEADER"
    assert follower.drafting_saving_pct >= 5.0  # Significant power savings
    assert follower.drafting_saving_pct <= 15.0

    assert outsider.is_drafting is False
    assert outsider.drafting_saving_pct == 0.0
    assert leader.is_drafting is False  # Leader does not draft anyone ahead

    # Verify battery model applies drafting factor to reduce power
    soc_before = follower.battery.soc
    follower.step(dt=1.0)
    energy_consumed_drafting = (soc_before - follower.battery.soc)

    # Compare with non-drafting drone
    outsider_soc_before = outsider.battery.soc
    outsider.step(dt=1.0)
    energy_consumed_solo = (outsider_soc_before - outsider.battery.soc)

    # Follower should consume less energy than solo flyer
    assert energy_consumed_drafting < energy_consumed_solo


def test_core_simulation_wrappers():
    """Verify SwarmSimulationCore wrapper methods for chaos fault and manual control."""
    core = SwarmSimulationCore()
    d1 = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, 0.0, 20.0]))
    core.add_drone(d1)
    d1.flight_mode = FlightMode.TRANSIT

    # Manual control wrapper
    success = core.set_drone_manual_control("UAV_1", vx=3.0, vy=2.0, vz=1.0, enabled=True)
    assert success is True
    assert d1.is_manual_override is True

    # Fault injection wrapper
    victim = core.trigger_chaos_fault("UAV_1")
    assert victim == "UAV_1"
    assert d1.is_fault_injected is True
