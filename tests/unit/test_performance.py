"""
tests/unit/test_performance.py: Automated performance and latency benchmark suite.
Guarantees physics loop runtime and WebSocket bandwidth budgets.
"""

import time
import json
import pytest
from vis.server import server_manager, prune_mesh_links


def test_simulation_physics_step_latency():
    """Verify 16-drone simulation step completes in under 12ms per step on CPU."""
    sim = server_manager.sim
    # Warmup
    for _ in range(5):
        sim.step(dt=0.033)

    t0 = time.perf_counter()
    steps = 30
    for _ in range(steps):
        sim.step(dt=0.033)
    t1 = time.perf_counter()

    avg_ms = (t1 - t0) * 1000.0 / steps
    assert avg_ms < 12.0, f"Expected physics step < 12ms, got {avg_ms:.2f}ms"


def test_websocket_payload_compact_size():
    """Verify WebSocket broadcast payload respects strict bandwidth budgets."""
    snap = server_manager.sim.step()
    data = snap.to_dict()

    # Full frame with pruned links
    data["links"] = prune_mesh_links(data.get("links", []), data.get("active_routes", []))
    full_payload = json.dumps(data)
    assert len(full_payload) < 22000, f"Full payload exceeded 22 KB budget: {len(full_payload)} bytes"

    # Delta frame (slow channels omitted)
    data.pop("links", None)
    data.pop("pois", None)
    data.pop("priority_queue", None)
    data.pop("charging_pads", None)
    delta_payload = json.dumps(data)
    assert len(delta_payload) < 12000, f"Delta payload exceeded 12 KB budget: {len(delta_payload)} bytes"


def test_steering_force_broadphase_efficiency():
    """Verify broadphase AABB culling completes steering force calculations in < 1ms."""
    sim = server_manager.sim
    drone = list(sim.drones.values())[0]

    t0 = time.perf_counter()
    iterations = 100
    for _ in range(iterations):
        sim.compute_steering_forces(drone)
    t1 = time.perf_counter()

    per_call_ms = (t1 - t0) * 1000.0 / iterations
    assert per_call_ms < 1.0, f"Steering force took too long: {per_call_ms:.3f}ms per drone"


def test_network_los_cache_consistency():
    """Verify that network engine _los_cache returns valid links and maintains routing."""
    net = server_manager.sim.network_engine
    drones = server_manager.sim.drones
    obstacles = server_manager.sim.obstacles
    gcs_pos = server_manager.sim.config.gcs_position

    # First update seeds cache
    net.update(drones, gcs_pos, obstacles, dt=0.033)
    routes1 = list(net.active_routes)
    links1 = len(net.link_cache)

    # Second update uses cache
    net.update(drones, gcs_pos, obstacles, dt=0.033)
    routes2 = list(net.active_routes)
    links2 = len(net.link_cache)

    assert links1 == links2
    assert routes1 == routes2
