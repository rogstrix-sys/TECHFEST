"""
tests/unit/test_formations_and_recharge.py: Comprehensive tests for Tactical Swarm Formations
and Automated GCS Battery Fast Recharging with Continuous Mission Persistence.
"""

import numpy as np
import pytest
from starlette.testclient import TestClient

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.types import DroneRole, FlightMode, TelemetrySnapshot
from vis.server import app, create_default_simulation, server_manager


class TestTacticalSwarmFormations:
    """Tests for multi-UAV tactical formation geometries and kinematic setpoint calculation."""

    def test_formation_mode_switching(self):
        """Test setting valid formation modes and rejecting invalid formation strings."""
        sim = SwarmSimulationCore()
        assert sim.active_formation == "AUTONOMOUS"

        assert sim.set_swarm_formation("V_FORMATION") is True
        assert sim.active_formation == "V_FORMATION"

        assert sim.set_swarm_formation("LINE_SWEEP") is True
        assert sim.active_formation == "LINE_SWEEP"

        assert sim.set_swarm_formation("PERIMETER_ORBIT") is True
        assert sim.active_formation == "PERIMETER_ORBIT"

        assert sim.set_swarm_formation("v-shape") is True
        # Normalized or rejected if unrecognized:
        assert sim.set_swarm_formation("AUTONOMOUS") is True
        assert sim.active_formation == "AUTONOMOUS"

        # Invalid formation should return False and keep existing state
        assert sim.set_swarm_formation("INVALID_FORMATION_XYZ") is False
        assert sim.active_formation == "AUTONOMOUS"

    def test_v_formation_setpoints_and_drafting(self):
        """Verify V-formation (Chevron/Wedge) assigns staggered wingman positions and drafting bonus."""
        sim = SwarmSimulationCore()
        d1 = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, 0.0, 30.0]))
        d2 = Drone("UAV_2", role=DroneRole.SURVEY, initial_pos=np.array([-10.0, -10.0, 30.0]))
        d3 = Drone("UAV_3", role=DroneRole.SURVEY, initial_pos=np.array([10.0, -10.0, 30.0]))
        sim.add_drone(d1)
        sim.add_drone(d2)
        sim.add_drone(d3)

        for d in [d1, d2, d3]:
            d.flight_mode = FlightMode.TRANSIT
            d.position[2] = 30.0

        d1.velocity = np.array([0.0, 10.0, 0.0])  # Flying North
        sim.set_swarm_formation("V_FORMATION")
        sim.update_formation_setpoints()

        # Wingmen should have target waypoints behind and to the sides of UAV_1
        tgt2 = d2.get_target_waypoint()
        tgt3 = d3.get_target_waypoint()

        assert tgt2 is not None
        assert tgt3 is not None
        # Both wingmen should be trailing in Y
        assert tgt2[1] < d1.position[1]
        assert tgt3[1] < d1.position[1]
        # Wingman 2 is left (-X) and Wingman 3 is right (+X)
        assert tgt2[0] < tgt3[0]
        # Wingmen in V-formation should be marked drafting
        assert d2.is_drafting is True
        assert d2.drafting_leader_id == "UAV_1"
        assert d2.drafting_saving_pct > 0.0

    def test_line_sweep_setpoints(self):
        """Verify LINE_SWEEP assigns lateral line-abreast waypoints across the fleet."""
        sim = SwarmSimulationCore()
        drones = [
            Drone(f"UAV_{i+1}", role=DroneRole.SURVEY, initial_pos=np.array([i * 10.0, 0.0, 30.0]))
            for i in range(4)
        ]
        for d in drones:
            sim.add_drone(d)
            d.flight_mode = FlightMode.TRANSIT
            d.position[2] = 30.0

        sim.set_swarm_formation("LINE_SWEEP")
        sim.update_formation_setpoints()

        targets = [d.get_target_waypoint() for d in drones]
        for tgt in targets:
            assert tgt is not None
        # Waypoints should have monotonically increasing or ordered lateral offsets
        xs = [tgt[0] for tgt in targets]
        assert xs[0] < xs[1] < xs[2] < xs[3]

    def test_perimeter_orbit_setpoints(self):
        """Verify PERIMETER_ORBIT places drones along a circular perimeter around origin."""
        sim = SwarmSimulationCore()
        drones = [
            Drone(f"UAV_{i+1}", role=DroneRole.SURVEY, initial_pos=np.array([20.0 * i, 0.0, 35.0]))
            for i in range(4)
        ]
        for d in drones:
            sim.add_drone(d)
            d.flight_mode = FlightMode.TRANSIT
            d.position[2] = 35.0

        sim.set_swarm_formation("PERIMETER_ORBIT")
        sim.update_formation_setpoints()

        for d in drones:
            tgt = d.get_target_waypoint()
            assert tgt is not None
            r = np.linalg.norm(tgt[:2])
            # Radius should be close to 80m nominal radius
            assert pytest.approx(r, abs=1.0) == 80.0
            assert tgt[2] >= 30.0


class TestAutomatedGCSRechargeAndPersistence:
    """Tests for automated GCS pad fast charging and continuous mission persistence."""

    def test_landed_drone_recharges_battery(self):
        """Verify that a landed drone with depleted battery automatically recharges at the GCS pad."""
        sim = SwarmSimulationCore()
        drone = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, -145.0, 0.0]))
        sim.add_drone(drone)
        drone.flight_mode = FlightMode.LANDED
        drone.position[2] = 0.0
        drone.battery.soc = 0.20  # 20% remaining
        assert drone.battery.soc == 0.20

        # Step simulation by 1.0 second
        sim.step(dt=1.0)

        # Battery SoC should have increased due to fast charging (+0.05/s)
        assert drone.battery.soc > 0.20
        assert getattr(drone, "_is_charging", False) is True

    def test_full_recharge_triggers_auto_relaunch(self):
        """Verify that once fully charged (100% SoC), the drone automatically relaunches into active patrol."""
        sim = SwarmSimulationCore()
        drone = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, -145.0, 0.0]))
        sim.add_drone(drone)
        drone.flight_mode = FlightMode.LANDED
        drone.position[2] = 0.0
        drone.battery.soc = 0.98
        drone._landed_for_recharge = True
        drone.is_low_battery_rtb = True

        # Advance step to reach 100%
        sim.step(dt=1.0)

        assert drone.battery.soc >= 0.999
        # Auto-relaunch should trigger
        assert drone.flight_mode == FlightMode.TAKEOFF
        assert drone.is_low_battery_rtb is False
        assert drone.get_target_waypoint() is not None
        assert drone.get_target_waypoint()[2] >= 30.0

    def test_telemetry_includes_charging_and_formation_fields(self):
        """Verify TelemetrySnapshot accurately serializes active_formation and is_charging status."""
        sim = SwarmSimulationCore()
        drone = Drone("UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, -145.0, 0.0]))
        sim.add_drone(drone)
        drone.flight_mode = FlightMode.LANDED
        drone.position[2] = 0.0
        drone.battery.soc = 0.50

        sim.set_swarm_formation("V_FORMATION")
        sim.step(dt=0.05)

        snap = sim.get_telemetry_snapshot()
        assert snap.active_formation == "V_FORMATION"

        d_dict = snap.to_dict()
        assert d_dict["active_formation"] == "V_FORMATION"
        assert d_dict["formation"] == "V_FORMATION"

        uav_data = d_dict["drones"][0]
        assert uav_data["is_charging"] is True


class TestFormationServerAPI:
    """Verify FastAPI server endpoint /api/formation and /api/control integration."""

    @pytest.fixture
    def client(self):
        server_manager.sim = create_default_simulation()
        return TestClient(app)

    def test_post_formation_endpoint(self, client):
        """Test POST /api/formation changes formation mode."""
        resp = client.post("/api/formation", json={"formation": "LINE_SWEEP"})
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "ok"
        assert data["formation"] == "LINE_SWEEP"

        resp2 = client.post("/api/formation", json={"formation": "INVALID"})
        assert resp2.status_code == 200
        assert resp2.json()["status"] == "error"

    def test_post_control_formation_command(self, client):
        """Test /api/control accepts formation commands."""
        resp = client.post("/api/control", json={"command": "formation", "value": "PERIMETER_ORBIT"})
        assert resp.status_code == 200
        data = resp.json()
        assert data["status"] == "ok"
        assert data["formation"] == "PERIMETER_ORBIT"
