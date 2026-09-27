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

