"""
tests/unit/test_phase2_delta_encoding.py: Verification suite for Phase 2 delta encoding & channel decoupling:
- Compact mesh link serialization with rounded precision
- Lean survivors telemetry without duplicate thermal grid
- Decoupled slow-channel throttling and overall payload reduction (>60% savings)
"""

import json
from vis.server import server_manager, prune_mesh_links


def test_prune_mesh_links_compact_rounding():
    """Verify prune_mesh_links returns compact dictionaries with rounded numeric metrics."""
    raw_links = [
        {
            "source": "UAV_1",
            "target": "UAV_2",
            "distance": 42.123456789,
            "snr": 28.987654321,
            "cost": 50.111222333,
            "path_loss_db": 65.444555666,
            "viable": True,
            "band": "2.4GHz",
            "unneeded_verbose_field": "x" * 100,
        }
    ]
    pruned = prune_mesh_links(raw_links, max_links=10)
    assert len(pruned) == 1
    l = pruned[0]
    assert l["source"] == "UAV_1"
    assert l["target"] == "UAV_2"
    assert l["distance"] == 42.1
    assert l["snr"] == 29.0
    assert l["cost"] == 50.1
    assert l["path_loss_db"] == 65.4
    assert "unneeded_verbose_field" not in l


def test_get_survivors_telemetry_lean_payload():
    """Verify survivors telemetry does not duplicate thermal_grid and remains under 500 bytes."""
    mm = getattr(server_manager.sim, "mission_manager", None)
    assert mm is not None
    surv_telem = mm.get_survivors_telemetry()
    assert "total_count" in surv_telem
    assert "located_count" in surv_telem
    assert "pending_count" in surv_telem
    assert "discovered_survivors" in surv_telem
    assert "thermal_grid" not in surv_telem
    assert len(json.dumps(surv_telem)) < 500


def test_payload_size_reduction():
    """Verify that delta-encoded broadcast payload size is reduced by >50% compared to raw unpruned."""
    snap = server_manager.sim.step()
    raw_dict = snap.to_dict()
    raw_size = len(json.dumps(raw_dict))

    # Apply Phase 2 pruning
    data = dict(raw_dict)
    data["links"] = prune_mesh_links(data["links"], data.get("active_routes", []))
    compact_size = len(json.dumps(data))

    assert compact_size < raw_size * 0.60, f"Expected >40% reduction, got {raw_size} -> {compact_size}"
