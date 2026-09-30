"""
tests/unit/test_tier1_features.py: Comprehensive Verification for Tier 1 Features.
1. Predictive Survivor Thermal Probability Heatmap (Bayesian Belief Grid)
2. Dynamic Obstacle Collapse Simulation (AABB structural failure & emergency reroute)
3. Byzantine-Tolerant Consensus-Based Mission Voting (BFT-CBBA)
"""

import numpy as np
import pytest

from sim.core import SwarmSimulationCore
from sim.drone import Drone, DroneLimits
from sim.mapping import BayesianThermalGrid
from sim.mission import DisasterMissionManager
from sim.obstacles import ObstacleAABB
from sim.planning import CBBASolver
from sim.types import DroneRole, FlightMode


def test_bayesian_thermal_grid_recursive_estimation():
    """Verify recursive Bayesian prior/posterior probability density updates."""
    grid = BayesianThermalGrid(bounds_x=(-100.0, 100.0), bounds_y=(-100.0, 100.0), cell_size=20.0, prior_p=0.04)
    assert grid.grid.shape == (10, 10)
    assert np.allclose(grid.grid, 0.04)

    # 1. Seed priors near a disaster site
    poi_pos = [[20.0, 20.0, 0.0]]
    grid.seed_priors(poi_pos, elevated_p=0.25)
    # Check that cells near (20, 20) have elevated prior
    assert np.max(grid.grid) >= 0.25

    # 2. Simulate FLIR sweep over a trapped survivor
    survivor_pos = [[20.0, 20.0, 0.0]]
    drone_pos = [20.0, 20.0, 30.0]
    p_before = float(np.max(grid.grid))
    grid.update_flir_scan(drone_pos, survivor_pos, fov_radius=40.0)
    p_after = float(np.max(grid.grid))
    assert p_after > p_before, f"Posterior probability should increase on detection: {p_before} -> {p_after}"
    assert p_after > 0.50

    # 3. Simulate FLIR sweep over an empty sector (no survivor)
    empty_drone_pos = [-60.0, -60.0, 30.0]
    grid.update_flir_scan(empty_drone_pos, survivor_pos, fov_radius=40.0)
    # The cells around (-60, -60) should decrease in probability
    cell_idx_x = int(math_floor := ((-60.0 - (-100.0)) // 20.0))
    cell_idx_y = int(((-60.0 - (-100.0)) // 20.0))
    assert grid.grid[cell_idx_x, cell_idx_y] < 0.04

    # 4. Target retrieval
    target = grid.get_highest_entropy_target([0.0, 0.0, 0.0])
    assert target is not None
    assert target.shape == (3,)
    assert target[2] == 28.0

    # 5. Serialization
    payload = grid.to_dict()
    assert "hotspots" in payload
    assert len(payload["hotspots"]) >= 1
    assert payload["max_prob"] == pytest.approx(p_after, abs=0.01)


def test_dynamic_obstacle_collapse_geometry_and_sar():
    """Verify mid-mission obstacle collapse, AABB geometry alteration, and survivor spawn."""
    sim = SwarmSimulationCore()
    obs = ObstacleAABB(
        id="OBS_TOWER_BETA",
        name="Damaged Tower Beta",
        min_pt=np.array([-50.0, -50.0, 0.0]),
        max_pt=np.array([0.0, 0.0, 60.0]),
    )
    sim.add_obstacle(obs)
    mm = DisasterMissionManager(gcs_position=np.array([0.0, -100.0, 0.0]))
    sim.set_mission_manager(mm)

    old_height = float(obs.max_pt[2] - obs.min_pt[2])
    assert old_height == 60.0

    # Trigger collapse with 45% reduction
    res = sim.trigger_obstacle_collapse(obstacle_id="OBS_TOWER_BETA", collapse_ratio=0.45)
    assert res is not None
    assert res["obstacle_id"] == "OBS_TOWER_BETA"
    assert res["new_height_m"] == 33.0
    assert obs.max_pt[2] == 33.0
    assert getattr(obs, "is_collapsed", False) is True

    # Check rubble expansion
    assert obs.min_pt[0] == -54.0
    assert obs.max_pt[0] == 4.0

    # Check that secondary survivor was spawned under rubble
    assert len(mm.survivors) >= 1
    collapse_survs = [s for s in mm.survivors.values() if "COLLAPSE" in s.id]
    assert len(collapse_survs) == 1
    assert collapse_survs[0].discovered is False
    assert collapse_survs[0].heat_c == 38.4


def test_byzantine_fault_tolerant_cbba_auction():
    """Verify BFT-CBBA detects spoofed rogue bids and awards task to honest winner."""
    solver = CBBASolver(max_bundle_size=1, enable_bft=True)

    drones = {
        "UAV_1": Drone(drone_id="UAV_1", role=DroneRole.SURVEY, initial_position=np.array([10.0, 0.0, 25.0])),
        "UAV_2": Drone(drone_id="UAV_2", role=DroneRole.SURVEY, initial_position=np.array([50.0, 0.0, 25.0])),
        "UAV_ROGUE": Drone(drone_id="UAV_ROGUE", role=DroneRole.SURVEY, initial_position=np.array([200.0, 0.0, 25.0])),
    }
    pois = {
        "POI_A": {
            "id": "POI_A",
            "position": np.array([0.0, 0.0, 0.0]),
            "priority": "HIGH",
            "is_completed": False,
            "assigned_drone_id": None,
        }
    }

    # 1. Normal auction: UAV_1 is closest (dist=10m) -> highest score, should win
    res = solver.solve(drones, pois)
    assert res["POI_A"] == "UAV_1"

    # 2. Rogue drone injects an artificially inflated spoofed bid of 9999.0
    solver.inject_byzantine_bid(drone_id="UAV_ROGUE", poi_id="POI_A", spoofed_score=9999.0)

    # 3. Solve with BFT enabled: Rogue bid should be caught and rejected by BFT consensus filter!
    res_bft = solver.solve(drones, pois)
    assert res_bft["POI_A"] == "UAV_1", "Honest UAV_1 should win despite rogue drone submitting 9999.0"

    # Check BFT audit log
    assert len(solver.bft_audit_log) >= 1
    entry = solver.bft_audit_log[-1]
    assert entry["rogue_drone_id"] == "UAV_ROGUE"
    assert entry["action"] == "REJECTED_BFT_OUTLIER"
    assert entry["spoofed_score"] == 9999.0
