"""
Unit tests for Advanced LiPo Battery Physics (Airspeed & Altitude Scaling),
Network Disconnect RTB & Reconnection Resumption, and
Disaster Site Priority Queue with Mission Time Budget.
"""

import pytest
import numpy as np
from sim.types import BatteryModel, DroneRole, FlightMode
from sim.drone import Drone
from sim.network import FANETNetworkEngine
from sim.mission import DisasterMissionManager, DisasterSitePriorityQueue


class TestBatteryPhysicsScaling:
    """Test electro-mechanical battery depletion against airspeed, altitude, and climb work."""

    def test_airspeed_parasite_drag_depletion(self):
        """Verify that high airspeed consumes more power than hover due to cubic parasite drag."""
        bat_hover = BatteryModel(capacity_mah=5000.0, nominal_voltage=22.2)
        bat_high_speed = BatteryModel(capacity_mah=5000.0, nominal_voltage=22.2)

        # 100 steps of 0.1s at hover (speed=0)
        for _ in range(100):
            bat_hover.step(dt=0.1, airspeed=0.0, altitude=10.0, climb_rate=0.0)

        # 100 steps of 0.1s at high airspeed (14.0 m/s)
        for _ in range(100):
            bat_high_speed.step(dt=0.1, airspeed=14.0, altitude=10.0, climb_rate=0.0)

        assert bat_high_speed.current_power_w > bat_hover.current_power_w
        assert bat_high_speed.soc < bat_hover.soc
        assert bat_high_speed.last_p_parasite > 0.0

    def test_altitude_air_density_scaling(self):
        """Verify that higher altitude (thinner air) increases induced hover power."""
        bat_sea_level = BatteryModel(capacity_mah=5000.0, nominal_voltage=22.2)
        bat_high_alt = BatteryModel(capacity_mah=5000.0, nominal_voltage=22.2)

        # Low altitude (5m)
        bat_sea_level.step(dt=0.1, airspeed=0.0, altitude=5.0, climb_rate=0.0)
        # High altitude (100m)
        bat_high_alt.step(dt=0.1, airspeed=0.0, altitude=100.0, climb_rate=0.0)

        assert bat_high_alt.last_altitude_factor > bat_sea_level.last_altitude_factor
        assert bat_high_alt.current_power_w > bat_sea_level.current_power_w

    def test_climb_work_consumption(self):
        """Verify that climbing consumes potential energy power."""
        bat_level = BatteryModel(capacity_mah=5000.0, nominal_voltage=22.2)
        bat_climbing = BatteryModel(capacity_mah=5000.0, nominal_voltage=22.2)

        bat_level.step(dt=0.1, airspeed=5.0, altitude=20.0, climb_rate=0.0)
        bat_climbing.step(dt=0.1, airspeed=5.0, altitude=20.0, climb_rate=3.0)

        assert bat_climbing.last_p_climb > 0.0
        assert bat_climbing.current_power_w > bat_level.current_power_w


class TestNetworkDisconnectRTBAndResume:
    """Test autonomous return-to-base on comms loss and task resumption upon reconnection."""

    def test_comms_loss_triggers_rtb_and_saves_task(self):
        """When a drone loses network connection to GCS, it must save active task and trigger RTL."""
        drone = Drone(drone_id="UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, -250.0, 0.0]))
        drone.position = np.array([120.0, 150.0, 30.0])  # Airborne far from base
        drone.flight_mode = FlightMode.SURVEYING
        drone.assigned_poi_id = "POI_SITE_A"

        mission = DisasterMissionManager(comms_loss_timeout=1.0)
        pois = {
            "POI_SITE_A": {
                "id": "POI_SITE_A",
                "position": [120.0, 150.0, 30.0],
                "priority": "HIGH",
                "required_dwell_time": 10.0,
                "current_dwell_time": 2.0,
                "is_completed": False,
                "assigned_drone_id": "UAV_1",
            }
        }
        drones = {"UAV_1": drone}

        # Mock network engine with no path to GCS for UAV_1 (far distance: 1000m exceeds max RF range)
        network = FANETNetworkEngine()
        network.update_topology({"GCS": np.array([0.0, -250.0, 15.0]), "UAV_1": np.array([1000.0, 1000.0, 20.0])})

        # Step until timeout exceeds 1.0s (15 * 0.1s = 1.5s)
        for _ in range(15):
            mission.update(drones=drones, pois=pois, dt=0.1, network_engine=network)

        assert drone.is_comms_loss_rtl is True
        assert drone.flight_mode in (FlightMode.RTL, FlightMode.LANDING)
        assert drone._saved_task is not None
        assert drone._saved_task["poi_id"] == "POI_SITE_A"

    def test_reconnection_resumes_saved_task(self):
        """When a disconnected drone regains network connection, it resumes its saved task."""
        drone = Drone(drone_id="UAV_1", role=DroneRole.SURVEY, initial_pos=np.array([0.0, -250.0, 0.0]))
        drone.position = np.array([120.0, 150.0, 30.0])
        drone.is_comms_loss_rtl = True
        drone.flight_mode = FlightMode.RTL
        drone._saved_task = {
            "poi_id": "POI_SITE_A",
            "target_pos": np.array([120.0, 150.0, 30.0]),
            "flight_mode": FlightMode.SURVEYING,
        }

        mission = DisasterMissionManager(comms_loss_timeout=1.0)
        pois = {
            "POI_SITE_A": {
                "id": "POI_SITE_A",
                "position": [120.0, 150.0, 30.0],
                "priority": "HIGH",
                "required_dwell_time": 10.0,
                "current_dwell_time": 2.0,
                "is_completed": False,
                "assigned_drone_id": None,
            }
        }
        drones = {"UAV_1": drone}

        # Mock network engine WITH viable direct link to GCS (distance 50m < 80m max 2.4GHz)
        network = FANETNetworkEngine()
        network.update_topology({"GCS": np.array([0.0, -250.0, 15.0]), "UAV_1": np.array([0.0, -200.0, 20.0])})

        # Run mission update with active network connection
        mission.update(drones=drones, pois=pois, dt=0.1, network_engine=network)

        # UAV_1 should have cleared comms loss and resumed task
        assert drone.is_comms_loss_rtl is False
        assert drone.assigned_poi_id == "POI_SITE_A"
        assert drone.flight_mode in (FlightMode.TRANSIT, FlightMode.SURVEYING)


class TestPriorityQueueAndBudget:
    """Test disaster site priority queue and mission time budget pacing."""

    def test_priority_queue_ordering(self):
        """Verify sites are sorted by priority: CRITICAL > HIGH > MEDIUM > LOW."""
        pq = DisasterSitePriorityQueue()
        pois = {
            "SITE_LOW": {"id": "SITE_LOW", "priority": "LOW", "required_dwell_time": 10.0, "is_completed": False},
            "SITE_CRIT": {"id": "SITE_CRIT", "priority": "CRITICAL", "required_dwell_time": 10.0, "is_completed": False},
            "SITE_MED": {"id": "SITE_MED", "priority": "MEDIUM", "required_dwell_time": 10.0, "is_completed": False},
            "SITE_HIGH": {"id": "SITE_HIGH", "priority": "HIGH", "required_dwell_time": 10.0, "is_completed": False},
        }

        queue = pq.update_queue(pois=pois, current_time=10.0, time_budget=300.0)
        ordered_ids = [p["id"] for p in queue]
        assert ordered_ids == ["SITE_CRIT", "SITE_HIGH", "SITE_MED", "SITE_LOW"]

    def test_mission_budget_status_and_pacing(self):
        """Verify budget status progression: ON_SCHEDULE -> EXPEDITED / CRITICAL_DEADLINE."""
        mission = DisasterMissionManager(mission_time_budget=300.0)
        assert mission.get_time_remaining() == 300.0
        assert mission.get_budget_status() == "ON_SCHEDULE"

        # Advance mission time
        mission.total_mission_time = 200.0  # 100s remaining
        assert mission.get_budget_status() == "EXPEDITED"

        mission.total_mission_time = 260.0  # 40s remaining
        assert mission.get_budget_status() == "CRITICAL_DEADLINE"

        mission.total_mission_time = 301.0  # Expired
        assert mission.get_budget_status() == "TIME_EXCEEDED"
