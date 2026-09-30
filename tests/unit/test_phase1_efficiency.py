"""
tests/unit/test_phase1_efficiency.py: Validation suite for Phase 1 performance optimizations:
- BayesianThermalGrid dirty-flag caching
- Unified handle_command command routing
- Obstacle list dirty-flag caching
"""

import numpy as np
from sim.mapping import BayesianThermalGrid
from vis.server import server_manager, handle_command


def test_bayesian_thermal_grid_dirty_flag_caching():
    """Verify that BayesianThermalGrid caches serialization until modified."""
    grid = BayesianThermalGrid(bounds_x=(-100.0, 100.0), bounds_y=(-100.0, 100.0), cell_size=20.0)
    assert grid._dirty is True

    # First serialization computes and caches
    d1 = grid.to_dict()
    assert grid._dirty is False
    assert grid._cached_dict is not None

    # Second serialization returns cached object reference immediately
    d2 = grid.to_dict()
    assert d1 is d2

    # Seeding priors invalidates cache
    grid.seed_priors([[10.0, 10.0]])
    assert grid._dirty is True

    # Recomputes and updates cache
    d3 = grid.to_dict()
    assert grid._dirty is False
    assert d3 is not d1
    assert d3["max_prob"] >= 0.25

    # Update FLIR scan invalidates cache
    grid.update_flir_scan(drone_pos=[10.0, 10.0, 20.0], survivor_positions=[[10.0, 10.0, 0.0]])
    assert grid._dirty is True
    d4 = grid.to_dict()
    assert grid._dirty is False
    assert d4 is not d3


def test_server_unified_handle_command():
    """Verify handle_command processes various commands cleanly."""
    # Test pause & resume
    r_pause = handle_command(server_manager, {"command": "pause"})
    assert r_pause["status"] == "ok"
    assert server_manager.is_running is False

    r_resume = handle_command(server_manager, {"cmd": "resume"})
    assert r_resume["status"] == "ok"
    assert server_manager.is_running is True

    # Test speed
    r_speed = handle_command(server_manager, {"command": "speed", "value": 3.0})
    assert r_speed["status"] == "ok"
    assert server_manager.sim_speed == 3.0
    handle_command(server_manager, {"command": "speed", "value": 1.0})

    # Test formation
    r_form = handle_command(server_manager, {"command": "formation", "value": "V_FORMATION"})
    assert r_form["status"] == "ok"
    assert r_form["formation"] == "V_FORMATION"

    # Test dispatch
    r_disp = handle_command(server_manager, {"command": "dispatch", "drone_id": "UAV_1", "target": [10.0, 20.0, 30.0]})
    assert r_disp["status"] == "ok"
    assert r_disp["target"] == [10.0, 20.0, 30.0]

    # Test obstacle collapse sets dirty flag
    server_manager._obs_dirty = False
    r_col = handle_command(server_manager, {"command": "collapse"})
    assert r_col["status"] == "ok"
    assert server_manager._obs_dirty is True
    assert server_manager._cached_obs_list is None
