"""
tests/unit/test_vis_server.py: Tests for FastAPI visualizer server and endpoints.
"""

from __future__ import annotations

import csv
import io
import pytest
from fastapi.testclient import TestClient

from vis.server import app, server_manager


@pytest.fixture
def client():
    return TestClient(app)


def test_server_telemetry_endpoint(client: TestClient):
    """Verify /api/telemetry returns full schema snapshot."""
    response = client.get("/api/telemetry")
    assert response.status_code == 200
    data = response.json()
    assert "drones" in data
    assert "pois" in data
    assert "active_routes" in data
    assert "links" in data
    assert "gcs" in data
    assert len(data["drones"]) >= 5


def test_server_control_endpoint(client: TestClient):
    """Verify /api/control processes pause, resume, speed, and reset."""
    resp_pause = client.post("/api/control", json={"command": "pause"})
    assert resp_pause.status_code == 200
    assert resp_pause.json()["running"] is False

    resp_resume = client.post("/api/control", json={"command": "resume"})
    assert resp_resume.status_code == 200
    assert resp_resume.json()["running"] is True

    resp_speed = client.post("/api/control", json={"command": "speed", "value": 2.5})
    assert resp_speed.status_code == 200
    assert resp_speed.json()["speed"] == 2.5

    resp_reset = client.post("/api/control", json={"command": "reset"})
    assert resp_reset.status_code == 200


def test_server_export_csv_endpoint(client: TestClient):
    """Verify /api/export_telemetry returns downloadable CSV flight log."""
    # Step the simulation a few ticks to populate history
    for _ in range(5):
        server_manager.sim.step()

    response = client.get("/api/export_telemetry")
    assert response.status_code == 200
    assert "text/csv" in response.headers.get("content-type", "")
    assert "uav_swarm_flight_recorder.csv" in response.headers.get("content-disposition", "")

    reader = csv.reader(io.StringIO(response.text))
    rows = list(reader)
    assert len(rows) > 5  # Header + multiple drone state entries
    header = rows[0]
    assert "sim_time" in header
    assert "drone_id" in header
    assert "pos_x" in header
    assert "est_x" in header
    assert "battery_pct" in header


def test_server_websocket(client: TestClient):
    """Verify /ws accepts WebSocket connection and receives real-time JSON."""
    with client.websocket_connect("/ws") as websocket:
        # Send a control command over websocket
        websocket.send_json({"command": "speed", "value": 1.5})
        import time
        time.sleep(0.05)
        # Verify server handled command
        assert server_manager.sim_speed == 1.5


def test_server_scenario_endpoints(client: TestClient):
    """Verify GET and POST /api/scenario for MeitY / IIT Bombay Challenge and Sector Delta."""
    # 1. Check default scenario is sector_delta
    resp = client.get("/api/scenario")
    assert resp.status_code == 200
    assert resp.json()["scenario"] in ("sector_delta", "challenge")

    # 2. Switch to challenge scenario
    resp_ch = client.post("/api/scenario", json={"scenario": "challenge"})
    assert resp_ch.status_code == 200
    assert resp_ch.json()["status"] == "ok"
    assert resp_ch.json()["scenario"] == "challenge"
    assert server_manager.scenario == "challenge"
    assert server_manager.sim.config.gcs_position[0] == -75.0

    # Verify telemetry snapshot includes challenge constraints
    telem = client.get("/api/telemetry").json()
    assert "challenge_constraints" in telem
    assert telem["challenge_constraints"] is not None
    assert telem["challenge_constraints"]["is_fully_compliant"] is True

    # 3. Switch back to sector_delta
    resp_sd = client.post("/api/scenario", json={"scenario": "sector_delta"})
    assert resp_sd.status_code == 200
    assert resp_sd.json()["scenario"] == "sector_delta"
    assert server_manager.scenario == "sector_delta"


def test_server_export_point_cloud_endpoints(client: TestClient):
    """Verify /api/export_point_cloud produces Net Combined PLY/LAS and individual drone PLY."""
    # 1. Net Combined PLY Export
    resp_ply = client.get("/api/export_point_cloud?format=ply")
    assert resp_ply.status_code == 200
    assert "UAVX_Net_Combined_City_PointCloud.ply" in resp_ply.headers.get("content-disposition", "")
    assert resp_ply.text.startswith("ply\n")
    assert "element vertex" in resp_ply.text

    # 2. Net Combined LAS Export
    resp_las = client.get("/api/export_point_cloud?format=las")
    assert resp_las.status_code == 200
    assert "UAVX_Net_Combined_City_PointCloud.las" in resp_las.headers.get("content-disposition", "")
    assert resp_las.content[:4] == b"LASF"

    # 3. Individual Drone PLY Export
    resp_drone = client.get("/api/export_point_cloud?format=ply&drone_id=UAV_1")
    assert resp_drone.status_code == 200
    assert "UAV_1_PointCloud.ply" in resp_drone.headers.get("content-disposition", "")
    assert resp_drone.text.startswith("ply\n")

    # 4. Unknown drone ID 404 check
    resp_404 = client.get("/api/export_point_cloud?format=ply&drone_id=INVALID_UAV_999")
    assert resp_404.status_code == 404


def test_fast_json_dumps_and_link_pruning():
    """Verify fast_json_dumps serialization and mesh link pruning."""
    from vis.server import fast_json_dumps, prune_mesh_links
    import json

    # 1. Test fast_json_dumps
    sample_data = {"drone": "UAV_1", "battery": 98.5, "tags": [1, 2, 3]}
    serialized = fast_json_dumps(sample_data)
    assert isinstance(serialized, str)
    deserialized = json.loads(serialized)
    assert deserialized["drone"] == "UAV_1"
    assert deserialized["battery"] == 98.5

    # 2. Test prune_mesh_links
    dense_links = []
    for i in range(10):
        for j in range(i + 1, 10):
            dense_links.append({
                "source": f"UAV_{i}",
                "target": f"UAV_{j}",
                "viable": True,
                "cost": float(abs(i - j)),
                "snr": 25.0 - float(abs(i - j)),
            })
    # 45 total links
    assert len(dense_links) == 45
    active_routes = [["UAV_0", "UAV_1", "UAV_2", "GCS"]]
    pruned = prune_mesh_links(dense_links, active_routes=active_routes, max_links=20)
    assert len(pruned) <= 20
    has_route_edge = any(
        (l["source"] == "UAV_0" and l["target"] == "UAV_1") or (l["source"] == "UAV_1" and l["target"] == "UAV_0")
        for l in pruned
    )
    assert has_route_edge

    # 3. Test _json_default fallback with NumPy types
    import numpy as np
    from vis.server import _json_default
    np_payload = {
        "pos": np.array([10.5, 20.25, 30.125], dtype=np.float64),
        "speed": np.float64(12.5),
        "count": np.int32(42),
        "active": np.bool_(True),
    }
    dumped_fallback = json.dumps(np_payload, default=_json_default)
    loaded_fallback = json.loads(dumped_fallback)
    assert loaded_fallback["pos"] == [10.5, 20.25, 30.125]
    assert loaded_fallback["speed"] == 12.5
    assert loaded_fallback["count"] == 42
    assert loaded_fallback["active"] is True



