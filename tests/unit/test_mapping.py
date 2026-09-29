"""
tests/unit/test_mapping.py: Unit tests for 3D LiDAR perception and Occupancy Voxel Grid mapping.
"""

from __future__ import annotations

import numpy as np
import pytest

from sim.drone import Drone
from sim.mapping import LiDARScanner, OccupancyGridMap3D, LiDARPoint, LiDARScan
from sim.obstacles import ObstacleAABB
from sim.types import DroneRole


@pytest.fixture
def sample_obstacles():
    """Create a sample building obstacle for LiDAR testing."""
    return [
        ObstacleAABB(
            id="BLD_TEST",
            name="Test Tower",
            min_pt=np.array([20.0, 20.0, 0.0]),
            max_pt=np.array([40.0, 40.0, 30.0]),
            material="reinforced_concrete"
        )
    ]


def test_lidar_scanner_initialization():
    """Verify LiDAR scanner initializes with valid ray precomputations."""
    scanner = LiDARScanner(
        max_range_m=50.0,
        horizontal_fov_deg=360.0,
        horizontal_resolution_deg=30.0,
        vertical_channels=4
    )
    assert scanner.max_range == 50.0
    assert len(scanner._body_ray_dirs) == 12 * 4  # 12 azimuth * 4 vertical
    # Check that ray direction vectors are unit length
    norms = np.linalg.norm(scanner._body_ray_dirs, axis=1)
    np.testing.assert_allclose(norms, 1.0, atol=1e-5)


def test_lidar_scanner_raycast_hits(sample_obstacles):
    """Verify LiDAR scanner raycasts hit obstacles and ground correctly."""
    scanner = LiDARScanner(
        max_range_m=60.0,
        horizontal_fov_deg=360.0,
        horizontal_resolution_deg=15.0,
        vertical_channels=8
    )

    # Place drone at [10, 30, 15] looking directly at the building at [20, 20, 0]..[40, 40, 30]
    pos = np.array([10.0, 30.0, 15.0])
    attitude = np.array([0.0, 0.0, 0.0])

    scan = scanner.scan("UAV_1", pos, attitude, sample_obstacles, sim_time=1.0)

    assert isinstance(scan, LiDARScan)
    assert scan.drone_id == "UAV_1"
    assert len(scan.points) > 0

    # Ensure some points hit the obstacle
    obs_hits = [p for p in scan.points if p.obstacle_id == "BLD_TEST"]
    assert len(obs_hits) > 0

    # Ensure range is within bounds
    for p in scan.points:
        assert scanner.min_range <= p.range_m <= scanner.max_range
        assert 0.0 <= p.intensity <= 1.0

    # Check to_dict() serialization
    data = scan.to_dict()
    assert data["drone_id"] == "UAV_1"
    assert "points" in data
    assert len(data["points"]) == len(scan.points)


def test_occupancy_grid_map_log_odds_updates(sample_obstacles):
    """Verify 3D occupancy voxel grid updates log-odds and computes probabilities."""
    grid = OccupancyGridMap3D(voxel_size_m=4.0)

    # Create synthetic scan hitting obstacle at [25, 25, 10]
    scan = LiDARScan(
        timestamp=0.5,
        drone_id="UAV_1",
        sensor_origin=np.array([0.0, 0.0, 10.0]),
        points=[
            LiDARPoint(x=25.0, y=25.0, z=10.0, range_m=35.35, intensity=0.9, obstacle_id="BLD_TEST")
        ]
    )

    grid.insert_scan(scan)

    # Check that hit voxel is recorded and probability increased
    hit_key = grid.world_to_grid(np.array([25.0, 25.0, 10.0]))
    assert hit_key in grid.voxels
    assert grid.voxels[hit_key].log_odds > 0.0
    assert grid.voxels[hit_key].occupancy_prob > 0.5
    assert grid.voxels[hit_key].is_occupied

    # Check metrics computation
    metrics = grid.compute_metrics()
    assert metrics["occupied_voxels"] >= 1.0
    assert metrics["mapped_volume_m3"] > 0.0

    # Check get_occupied_voxels serialization
    occ_list = grid.get_occupied_voxels()
    assert len(occ_list) >= 1
    assert "pos" in occ_list[0]
    assert "prob" in occ_list[0]


def test_individual_drone_lidar_sensor(sample_obstacles):
    """Verify every individual drone possesses its own onboard LiDAR scanner and point cloud buffer."""
    d1 = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([10.0, 30.0, 15.0]))
    d2 = Drone("UAV_2", role=DroneRole.SURVEY, initial_pos=np.array([10.0, 10.0, 15.0]))

    # Verify each drone has its own individual LiDARScanner instance
    assert hasattr(d1, "lidar") and isinstance(d1.lidar, LiDARScanner)
    assert hasattr(d2, "lidar") and isinstance(d2.lidar, LiDARScanner)
    assert d1.lidar is not d2.lidar  # Separate physical sensor instances

    # Execute scan on d1
    scan1 = d1.perform_lidar_scan(sample_obstacles, sim_time=1.0)
    assert isinstance(scan1, LiDARScan)
    assert scan1.drone_id == "UAV_1"
    assert len(scan1.points) > 0
    assert len(d1.scanned_points) == len(scan1.points)
    assert len(d2.scanned_points) == 0  # d2 hasn't scanned yet

    # Verify individual drone PLY export
    d1_ply = d1.export_point_cloud_ply()
    assert d1_ply.startswith("ply\n")
    assert "comment UAV UAV_1 Onboard LiDAR Point Cloud" in d1_ply
    assert f"element vertex {len(d1.scanned_points)}" in d1_ply


def test_net_combined_ply_export_fused_from_all_drones(sample_obstacles):
    """Verify net PLY combines point cloud data from each individual drone into unified 3D map."""
    d1 = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([10.0, 30.0, 15.0]))
    d2 = Drone("UAV_2", role=DroneRole.SURVEY, initial_pos=np.array([30.0, 10.0, 15.0]))
    r1 = Drone("RELAY_1", role=DroneRole.RELAY, initial_pos=np.array([30.0, 50.0, 25.0]))
    drones = {d1.id: d1, d2.id: d2, r1.id: r1}

    # Each individual drone scans the environment from its unique perspective
    s1 = d1.perform_lidar_scan(sample_obstacles, sim_time=1.0)
    s2 = d2.perform_lidar_scan(sample_obstacles, sim_time=1.0)
    s3 = r1.perform_lidar_scan(sample_obstacles, sim_time=1.0)

    assert len(d1.scanned_points) > 0
    assert len(d2.scanned_points) > 0
    assert len(r1.scanned_points) > 0

    grid = OccupancyGridMap3D(voxel_size_m=4.0)
    # Insert scans from all drones
    grid.insert_scan(s1)
    grid.insert_scan(s2)
    grid.insert_scan(s3)

    # Export Net Combined PLY
    net_ply = grid.export_point_cloud_ply(drones=drones)
    assert net_ply.startswith("ply\n")
    assert "comment Net 3D City & Structural Map Combined from All Individual Drone Sensors" in net_ply
    assert "UAV_1" in net_ply
    assert "UAV_2" in net_ply
    assert "RELAY_1" in net_ply

    lines = net_ply.strip().split("\n")
    header_end = lines.index("end_header")
    data_lines = lines[header_end + 1:]

    # Combined total points must equal sum of all individual drones' points
    total_expected = len(d1.scanned_points) + len(d2.scanned_points) + len(r1.scanned_points)
    assert len(data_lines) == total_expected

