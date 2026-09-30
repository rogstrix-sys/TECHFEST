"""
tests/unit/test_sector_scanning.py: Unit tests for Spatial Sector Partitioning & Abnormality Detection.
"""

import numpy as np
import pytest

from sim.core import SwarmSimulationCore, SimulationConfig
from sim.drone import Drone
from sim.mission import DisasterMissionManager
from sim.network import FANETNetworkEngine
from sim.obstacles import ObstacleAABB, create_default_disaster_obstacles
from sim.types import DroneRole, FlightMode
from sim.planning import (
    SectorPartitionManager,
    compute_conical_fov_radius,
    is_line_of_sight_blocked,
    invalidate_los_cache,
)


def test_sector_partition_initialization():
    """Verify that SectorPartitionManager initializes 12 geographic search sectors."""
    mgr = SectorPartitionManager(is_challenge_mode=False)
    assert len(mgr.sectors) == 12

    # Check bounds and patrol waypoints for each sector
    for sec_id, sector in mgr.sectors.items():
        assert sector.id == sec_id
        assert len(sector.bounds) == 6
        x_min, x_max, y_min, y_max, z_min, z_max = sector.bounds
        assert x_min < x_max
        assert y_min < y_max
        assert len(sector.patrol_waypoints) == 5
        assert sector.status == "PENDING"
        assert sector.scan_progress == 0.0

    # Challenge mode check
    mgr_ch = SectorPartitionManager(is_challenge_mode=True)
    assert len(mgr_ch.sectors) == 12
    # Verify bounds cover challenge arena [0, 1000]
    all_x = [sec.bounds[1] for sec in mgr_ch.sectors.values()]
    assert max(all_x) == 1000.0


def test_sector_assignment_to_fleet():
    """Verify that survey/scout UAVs are assigned dedicated scanning sectors."""
    mgr = SectorPartitionManager(is_challenge_mode=False)

    drones = {}
    # 4 Scouts
    for i in range(1, 5):
        d_id = f"SCOUT_{i}"
        drones[d_id] = Drone(d_id, role=DroneRole.SURVEY, initial_pos=np.array([0.0, 0.0, 0.0]))
    # 8 Surveyors
    for i in range(1, 9):
        d_id = f"UAV_{i}"
        drones[d_id] = Drone(d_id, role=DroneRole.SURVEY, initial_pos=np.array([0.0, 0.0, 0.0]))
    # 4 Relays (should NOT be assigned survey sectors)
    for i in range(1, 5):
        d_id = f"RELAY_{i}"
        drones[d_id] = Drone(d_id, role=DroneRole.RELAY, initial_pos=np.array([0.0, 0.0, 50.0]))

    mgr.assign_sectors(drones)

    # All 12 survey drones should have unique sectors
    assigned_sectors = set()
    for d_id in [f"SCOUT_{i}" for i in range(1, 5)] + [f"UAV_{i}" for i in range(1, 9)]:
        sec_id = drones[d_id].assigned_sector_id
        assert sec_id is not None
        assigned_sectors.add(sec_id)
        assert mgr.sectors[sec_id].assigned_drone_id == d_id
        assert mgr.sectors[sec_id].status == "SCANNING"

    assert len(assigned_sectors) == 12

    # Relays must not have assigned survey sectors
    for i in range(1, 5):
        assert drones[f"RELAY_{i}"].assigned_sector_id is None


def test_abnormality_registration_and_detection():
    """Verify abnormality detection within 55m sensor radius and sync with POIs."""
    mgr = SectorPartitionManager(is_challenge_mode=False)

    pois = {
        "POI_HOSPITAL": {
            "id": "POI_HOSPITAL",
            "position": np.array([-60.0, 25.0, 30.0]),
            "priority": "CRITICAL",
            "required_dwell_time": 10.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
            "is_detected": False,
            "is_reported": False,
            "is_spawned": True,
        }
    }
    mgr.populate_initial_abnormalities(pois)

    assert "POI_HOSPITAL" in mgr.abnormalities
    anom = mgr.abnormalities["POI_HOSPITAL"]
    assert not anom.detected
    assert anom.sector_id == "SEC_06_CC"

    # Drone placed far away (>100m)
    drone = Drone("UAV_2", role=DroneRole.SURVEY, initial_pos=np.array([100.0, 100.0, 30.0]))
    drone.assigned_sector_id = "SEC_06_CC"
    drone.set_flight_mode(FlightMode.TRANSIT)
    drones = {"UAV_2": drone}

    mgr.update(drones, pois, dt=0.1, sim_time=1.0)
    assert not anom.detected
    assert not pois["POI_HOSPITAL"]["is_detected"]

    # Move drone inside 55m sensor radius
    drone.position = np.array([-50.0, 20.0, 30.0])  # ~11m distance
    mgr.update(drones, pois, dt=0.1, sim_time=2.0)

    assert anom.detected
    assert anom.detection_time == 2.0
    assert anom.detected_by == "UAV_2"
    assert pois["POI_HOSPITAL"]["is_detected"]
    assert pois["POI_HOSPITAL"]["detected_by"] == "UAV_2"
    assert pois["POI_HOSPITAL"]["is_reported"]


def test_sector_scanning_progress_and_dwell():
    """Verify patrol waypoints advancing, dwell accumulation, and scan progress."""
    mgr = SectorPartitionManager(is_challenge_mode=False)

    pois = {
        "POI_TEST": {
            "id": "POI_TEST",
            "position": np.array([-120.0, 80.0, 30.0]),
            "priority": "HIGH",
            "required_dwell_time": 2.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
            "is_detected": False,
            "is_reported": False,
            "is_spawned": True,
        }
    }
    mgr.populate_initial_abnormalities(pois)

    drone = Drone("SCOUT_1", role=DroneRole.SURVEY, initial_pos=np.array([-120.0, 80.0, 30.0]))
    drone.assigned_sector_id = "SEC_01_NW"
    drone.set_flight_mode(FlightMode.SURVEYING)
    drones = {"SCOUT_1": drone}
    mgr.sectors["SEC_01_NW"].assigned_drone_id = "SCOUT_1"

    # Step simulation for 2.5 seconds near POI
    for t in range(25):
        mgr.update(drones, pois, dt=0.1, sim_time=t * 0.1)

    anom = mgr.abnormalities["POI_TEST"]
    assert anom.detected
    assert anom.inspected
    assert pois["POI_TEST"]["is_completed"]
    assert mgr.sectors["SEC_01_NW"].scan_progress > 0.0


def test_telemetry_snapshot_integration():
    """Verify that SwarmSimulationCore packs sectors, abnormalities, and drone sector fields into telemetry."""
    config = SimulationConfig(gcs_position=(0.0, -145.0, 0.0))
    sim = SwarmSimulationCore(config)

    sim.add_poi("POI_HOSPITAL", position=[-60.0, 25.0, 30.0], priority="CRITICAL", required_dwell_time=10.0)

    for i in range(1, 5):
        role = DroneRole.RELAY if i == 4 else DroneRole.SURVEY
        d = Drone(f"UAV_{i}", role=role, initial_pos=np.array([i * 10.0, -140.0, 0.5]))
        sim.add_drone(d)

    sim.set_network_engine(FANETNetworkEngine())
    sim.set_mission_manager(DisasterMissionManager(gcs_position=config.gcs_position))

    snap = sim.step(dt=0.1)
    telemetry = snap.to_dict()

    assert "sectors" in telemetry
    assert len(telemetry["sectors"]) == 12
    
    # Check POI detection fields
    poi_entry = next(p for p in telemetry["pois"] if p["id"] == "POI_HOSPITAL")
    assert "is_detected" in poi_entry
    assert "is_reported" in poi_entry

    # Register and detect a collapse abnormality
    sim.mission_manager.sector_manager.register_abnormality(
        anomaly_id="ANOMALY_COLLAPSE_01",
        anomaly_type="STRUCTURAL_COLLAPSE",
        position=[10.0, -140.0, 0.5],
        severity="CRITICAL",
    )
    # Step again so the drone at [10, -140, 0.5] detects it
    snap2 = sim.step(dt=0.1)
    tel2 = snap2.to_dict()
    assert "abnormalities" in tel2
    assert any(a["id"] == "ANOMALY_COLLAPSE_01" for a in tel2["abnormalities"])

    # Check drones
    drones_data = telemetry["drones"]
    survey_drones = [d for d in drones_data if d["role"] == "SURVEY"]
    for d in survey_drones:
        assert "sector" in d


def test_conical_fov_radius_calculation():
    """Verify 75° downward altitude-scaled conical sensor footprint: min 12m, max 55m."""
    # Floor check: Delta_z <= 0 or small Delta_z should clamp to 12.0m
    assert compute_conical_fov_radius(z_drone=10.0, z_target=10.0) == 12.0
    assert compute_conical_fov_radius(z_drone=5.0, z_target=10.0) == 12.0
    assert compute_conical_fov_radius(z_drone=15.0, z_target=0.0) == 12.0  # 15 * tan(37.5) ~ 11.51 <= 12

    # Linear scaling zone: Delta_z * tan(37.5°)
    r30 = compute_conical_fov_radius(z_drone=30.0, z_target=0.0)
    assert 22.5 <= r30 <= 23.5  # 30 * 0.7673 ~ 23.02m

    r50 = compute_conical_fov_radius(z_drone=50.0, z_target=0.0)
    assert 38.0 <= r50 <= 39.0  # 50 * 0.7673 ~ 38.37m

    # Ceiling check: clamped to max 55.0m
    assert compute_conical_fov_radius(z_drone=80.0, z_target=0.0) == 55.0  # 80 * 0.7673 ~ 61.39 > 55
    assert compute_conical_fov_radius(z_drone=120.0, z_target=0.0) == 55.0


def test_los_raycheck_vectorized_slab():
    """Verify fast vectorized slab ray-AABB occlusion check against 3D buildings."""
    import time
    building = ObstacleAABB("BLD_1", "Test Building", [40.0, -10.0, 0.0], [60.0, 10.0, 30.0])
    obstacles = [building]

    # Direct ray straight through building center
    blocked = is_line_of_sight_blocked(
        p_drone=np.array([0.0, 0.0, 15.0]),
        p_target=np.array([100.0, 0.0, 15.0]),
        obstacles=obstacles,
    )
    assert blocked is True

    # Drone climbs above building (z=45m > 30m) -> ray clears obstacle
    clear_above = is_line_of_sight_blocked(
        p_drone=np.array([0.0, 0.0, 45.0]),
        p_target=np.array([100.0, 0.0, 45.0]),
        obstacles=obstacles,
    )
    assert clear_above is False

    # Drone navigates around building laterally (y=30m outside [-10, 10])
    clear_lateral = is_line_of_sight_blocked(
        p_drone=np.array([0.0, 30.0, 15.0]),
        p_target=np.array([100.0, 30.0, 15.0]),
        obstacles=obstacles,
    )
    assert clear_lateral is False

    # Empty obstacles list -> never blocked
    assert is_line_of_sight_blocked([0, 0, 15], [100, 0, 15], []) is False

    # Micro-benchmark: Vectorized raycheck takes < 0.05 ms across full city diorama
    disaster_obs = create_default_disaster_obstacles()
    drone_p = np.array([0.0, 0.0, 35.0])
    target_p = np.array([80.0, 80.0, 0.0])

    t0 = time.perf_counter()
    N = 500
    for _ in range(N):
        is_line_of_sight_blocked(drone_p, target_p, disaster_obs)
    dt_ms = (time.perf_counter() - t0) * 1000.0 / N
    assert dt_ms < 0.05, f"Vectorized LoS check too slow: {dt_ms:.4f} ms"


def test_sector_scanning_los_occlusion_and_altitude_gate():
    """Verify SectorPartitionManager blocks detection if occluded by building or altitude < 3m."""
    mgr = SectorPartitionManager(is_challenge_mode=False)
    # 20m building placed between x=20 and x=30
    building = ObstacleAABB("OBS_BLD", "Concrete Structure", [20.0, -10.0, 0.0], [30.0, 10.0, 20.0])
    obstacles = [building]

    pois = {
        "POI_OCCLUDED": {
            "id": "POI_OCCLUDED",
            "position": np.array([35.0, 0.0, 10.0]),
            "priority": "HIGH",
            "required_dwell_time": 5.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
            "is_detected": False,
            "is_reported": False,
            "is_spawned": True,
        }
    }
    mgr.populate_initial_abnormalities(pois)
    anom = mgr.abnormalities["POI_OCCLUDED"]

    # 1. Drone placed at [15, 0, 10] on West side: ray to [35, 0, 10] hits building [20, 30] -> blocked!
    drone = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([15.0, 0.0, 10.0]))
    drone.assigned_sector_id = anom.sector_id
    drone.set_flight_mode(FlightMode.SURVEYING)
    drones = {"UAV_1": drone}

    # Step update with building obstacle: ray hits building -> blocked!
    mgr.update(drones, pois, dt=0.1, sim_time=1.0, obstacles=obstacles)
    assert not anom.detected
    assert not pois["POI_OCCLUDED"]["is_detected"]

    # 2. Drone moves to East side [33.0, 0.0, 30.0]: inside conical FOV footprint and clear LoS!
    drone.position = np.array([33.0, 0.0, 30.0])
    mgr.update(drones, pois, dt=0.1, sim_time=2.0, obstacles=obstacles)
    assert anom.detected
    assert anom.detected_by == "UAV_1"
    assert pois["POI_OCCLUDED"]["is_detected"]

    # 3. Test minimum airborne altitude gate (z >= 3.0m)
    anom2_id = "POI_LOW_ALT"
    pois[anom2_id] = {
        "id": anom2_id,
        "position": np.array([20.0, 20.0, 0.0]),
        "priority": "MEDIUM",
        "required_dwell_time": 5.0,
        "current_dwell_time": 0.0,
        "is_completed": False,
        "is_detected": False,
        "is_reported": False,
        "is_spawned": True,
    }
    mgr.register_abnormality(anom2_id, "TARGET_SITE", [20.0, 20.0, 0.0])
    anom2 = mgr.abnormalities[anom2_id]

    # Drone airborne at z=2.0m (< 3.0m requirement) at 6m horizontal distance
    drone_low = Drone("UAV_2", role=DroneRole.SURVEY, initial_pos=np.array([20.0, 26.0, 2.0]))
    drone_low.assigned_sector_id = anom2.sector_id
    drone_low.set_flight_mode(FlightMode.TRANSIT)
    drones["UAV_2"] = drone_low

    mgr.update(drones, pois, dt=0.1, sim_time=3.0, obstacles=[])
    assert not anom2.detected  # blocked by altitude < 3.0m gate!

    # Drone climbs to z=8.0m (>= 3.0m) -> detected!
    drone_low.position = np.array([20.0, 26.0, 8.0])
    mgr.update(drones, pois, dt=0.1, sim_time=4.0, obstacles=[])
    assert anom2.detected


def test_survivor_detection_los_occlusion_in_mission():
    """Verify survivor discovery in DisasterMissionManager respects LoS obstruction."""
    mgr = DisasterMissionManager()
    building = ObstacleAABB("BLD_SURV", "Hospital Wing", [10.0, 10.0, 0.0], [30.0, 30.0, 25.0])
    mgr.obstacles = [building]

    pois = {
        "POI_SITE": {
            "id": "POI_SITE",
            "position": np.array([35.0, 20.0, 0.0]),
            "priority": "CRITICAL",
            "required_dwell_time": 10.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
        }
    }
    mgr._init_survivors(pois)
    # Relocate one survivor right behind building at [32, 20, 0]
    first_surv = list(mgr.survivors.values())[0]
    first_surv.position = [32.0, 20.0, 0.0]

    # Surveyor drone placed on opposite side at [5.0, 20.0, 15.0]
    drone = Drone("SCOUT_1", initial_position=np.array([5.0, 20.0, 15.0]))
    drone.role = DroneRole.SURVEY
    drone.flight_mode = FlightMode.SURVEYING
    drones = {"SCOUT_1": drone}

    # Step update: building intercepts ray between [5, 20, 15] and [32, 20, 0] -> blocked!
    mgr.update(drones, pois, dt=0.1, obstacles=[building])
    assert not first_surv.discovered

    # Drone circles around building to East side at [35.0, 20.0, 15.0] (unobstructed line-of-sight & inside footprint)
    drone.position = np.array([35.0, 20.0, 15.0])
    mgr.update(drones, pois, dt=0.1, obstacles=[building])
    assert first_surv.discovered
    assert first_surv.discovered_by == "SCOUT_1"


def test_los_prevents_wall_clipping_in_close_proximity():
    """Verify that thin walls strictly block detection even when distance <= 2.0m."""
    wall = ObstacleAABB("THIN_WALL", "Concrete Wall", [0.0, -10.0, 0.0], [0.2, 10.0, 10.0])
    obstacles = [wall]

    # 1. SectorPartitionManager: Drone at [-0.5, 0, 5], target at [0.5, 0, 5] (1m distance <= 2m)
    mgr = SectorPartitionManager(is_challenge_mode=False)
    pois = {
        "POI_WALL": {
            "id": "POI_WALL",
            "position": np.array([0.5, 0.0, 5.0]),
            "priority": "HIGH",
            "required_dwell_time": 5.0,
            "current_dwell_time": 0.0,
            "is_completed": False,
            "is_detected": False,
            "is_reported": False,
            "is_spawned": True,
        }
    }
    mgr.populate_initial_abnormalities(pois)
    anom = mgr.abnormalities["POI_WALL"]

    drone = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([-0.5, 0.0, 5.0]))
    drone.assigned_sector_id = anom.sector_id
    drone.set_flight_mode(FlightMode.SURVEYING)
    drones = {"UAV_1": drone}

    # Step: distance is 1.0m, but wall is in between -> must NOT detect!
    mgr.update(drones, pois, dt=0.1, sim_time=1.0, obstacles=obstacles)
    assert not anom.detected, "Detection through wall must be occluded!"
    assert not pois["POI_WALL"]["is_detected"]

    # Drone climbs above wall to z=25m so LoS ray clears the 10m wall -> clear LoS -> detected!
    drone.position = np.array([-0.5, 0.0, 25.0])
    mgr.update(drones, pois, dt=0.1, sim_time=2.0, obstacles=obstacles)
    assert anom.detected
    assert pois["POI_WALL"]["is_detected"]


def test_los_dynamic_collapse_cache_invalidation():
    """Verify that dynamic building collapse invalidates LoS cache and clears raycheck."""
    config = SimulationConfig(gcs_position=(0.0, -145.0, 0.0))
    sim = SwarmSimulationCore(config)

    # Add 45m tall building between x=40 and x=60
    tower = ObstacleAABB("TOWER_COLLAPSE", "Concrete Tower", [40.0, -10.0, 0.0], [60.0, 10.0, 45.0])
    sim.add_obstacle(tower)

    p_drone = np.array([0.0, 0.0, 35.0])
    p_target = np.array([100.0, 0.0, 35.0])

    # Initial raycheck at z=35m: occluded by 45m tower
    assert is_line_of_sight_blocked(p_drone, p_target, sim.obstacles) is True

    # Trigger structural collapse: tower collapses to ~25m
    res = sim.trigger_obstacle_collapse(obstacle_id="TOWER_COLLAPSE", collapse_ratio=0.45)
    assert res is not None
    assert tower.max_pt[2] < 30.0

    # Raycheck at z=35m: must be clear immediately with zero cache staleness!
    assert is_line_of_sight_blocked(p_drone, p_target, sim.obstacles) is False


def test_los_raycheck_extreme_and_grazing_angles():
    """Verify slab ray-AABB edge cases: axis-aligned rays, grazing boundaries, zero distance."""
    box = ObstacleAABB("BOX", "Box", [10.0, 10.0, 10.0], [20.0, 20.0, 20.0])
    obstacles = [box]

    # 1. Zero distance check (point inside vs outside)
    assert is_line_of_sight_blocked([15.0, 15.0, 15.0], [15.0, 15.0, 15.0], obstacles) is True
    assert is_line_of_sight_blocked([5.0, 5.0, 5.0], [5.0, 5.0, 5.0], obstacles) is False

    # 2. Parallel along 2 axes (pure X travel through box)
    assert is_line_of_sight_blocked([0.0, 15.0, 15.0], [30.0, 15.0, 15.0], obstacles) is True
    # Pure X travel outside box
    assert is_line_of_sight_blocked([0.0, 5.0, 15.0], [30.0, 5.0, 15.0], obstacles) is False

    # 3. Ray grazing box boundary face exactly (y=10.0, z=15.0)
    assert is_line_of_sight_blocked([0.0, 10.0, 15.0], [30.0, 10.0, 15.0], obstacles) is True

    # 4. Ray in reverse direction through box
    assert is_line_of_sight_blocked([30.0, 15.0, 15.0], [0.0, 15.0, 15.0], obstacles) is True

    # 5. Box entirely behind drone
    assert is_line_of_sight_blocked([25.0, 15.0, 15.0], [35.0, 15.0, 15.0], obstacles) is False

    # 6. Box entirely behind target
    assert is_line_of_sight_blocked([-10.0, 15.0, 15.0], [0.0, 15.0, 15.0], obstacles) is False
