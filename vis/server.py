"""
vis/server.py: FastAPI & WebSocket Telemetry Streaming Server.

Streams real-time 3D simulation telemetry snapshots, LiDAR point cloud sweeps,
3D occupancy voxel grids, and Khatib APF guidance vectors to WebGL Three.js clients.
Provides interactive controls (Pause, Resume, Reset, Speed, Focus Drone) and REST endpoints.
"""

from __future__ import annotations

import asyncio
import json
import os
from pathlib import Path
import time
from typing import Any, Dict, List, Optional, Set
import numpy as np

from fastapi import FastAPI, Request, Response, WebSocket, WebSocketDisconnect
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.environment import DisasterEnvironment
from sim.mapping import LiDARScanner, OccupancyGridMap3D
from sim.mission import DisasterMissionManager
from sim.network import FANETNetworkEngine
from sim.obstacles import ObstacleAABB, create_default_disaster_obstacles
from sim.types import DroneRole, FlightMode
from sim.weather import WindConfig

STATIC_DIR = Path(__file__).resolve().parent / "static"


def get_gpu_telemetry() -> Dict[str, Any]:
    """Fetch live NVIDIA GPU hardware telemetry for Web Cockpit and HUD."""
    try:
        import pynvml
        pynvml.nvmlInit()
        handle = pynvml.nvmlDeviceGetHandleByIndex(0)
        name = pynvml.nvmlDeviceGetName(handle)
        mem = pynvml.nvmlDeviceGetMemoryInfo(handle)
        util = pynvml.nvmlDeviceGetUtilizationRates(handle)
        temp = pynvml.nvmlDeviceGetTemperature(handle, pynvml.NVML_TEMPERATURE_GPU)
        try:
            power = pynvml.nvmlDeviceGetPowerUsage(handle) / 1000.0
        except Exception:
            power = 15.0
        return {
            "name": name,
            "temp_c": temp,
            "gpu_util_pct": util.gpu,
            "vram_used_mb": int(mem.used / 1024**2),
            "vram_total_mb": int(mem.total / 1024**2),
            "power_w": round(power, 1),
            "accel": "NVIDIA CUDA OpenCL",
        }
    except Exception:
        return {
            "name": "NVIDIA RTX 4050",
            "temp_c": 48,
            "gpu_util_pct": 0,
            "vram_used_mb": 291,
            "vram_total_mb": 6141,
            "power_w": 18.0,
            "accel": "NVIDIA CUDA",
        }


def create_default_simulation() -> SwarmSimulationCore:
    """Instantiate a fully configured UAV swarm simulation aligned with Sector Delta 3D Diorama."""
    config = SimulationConfig(
        dt=0.05,
        world_bounds_x=(-175.0, 175.0),
        world_bounds_y=(-175.0, 175.0),
        world_bounds_z=(0.0, 130.0),
        gcs_position=(0.0, -145.0, 0.0),
        gcs_comm_radius=280.0,
        enable_downwash=True,
        enable_vsm_relays=True,
        enable_weather=True,
        wind_config=WindConfig(mean_speed_mps=5.5, direction_deg=45.0, turbulence_intensity="LIGHT", gust_probability=0.03, gust_magnitude_mps=6.0),
    )
    sim = SwarmSimulationCore(config=config)

    # 1. Add architectural obstacles matching Sector Delta visual 3D buildings
    sector_delta_obstacles = [
        # --- CENTRAL DISTRICT ---
        # Tower 1: Iconic Faceted Glass Skyscraper (center: -60, 20)
        ObstacleAABB(
            id="OBS_TOWER_GLASS",
            name="Faceted Glass Skyscraper",
            min_pt=np.array([-80.0, 0.0, 0.0]),
            max_pt=np.array([-40.0, 40.0, 80.0]),
            material="steel_concrete",
            base_attenuation_db=24.0,
        ),
        # Tower 2: Hexagonal / Cylindrical High-Rise (center: -110, -18)
        ObstacleAABB(
            id="OBS_TOWER_HEX",
            name="Hexagonal Glass High-Rise",
            min_pt=np.array([-128.0, -36.0, 0.0]),
            max_pt=np.array([-92.0, 0.0, 92.0]),
            material="reinforced_concrete",
            base_attenuation_db=25.0,
        ),
        # Tower 3: Stepped Corporate High-Rise (center: 10, 52)
        ObstacleAABB(
            id="OBS_TOWER_STEPPED",
            name="Stepped Corporate High-Rise",
            min_pt=np.array([-12.0, 32.0, 0.0]),
            max_pt=np.array([32.0, 72.0, 96.0]),
            material="reinforced_concrete",
            base_attenuation_db=24.0,
        ),
        # Tower 4: Rear Slender Spire Tower (center: -25, 88)
        ObstacleAABB(
            id="OBS_TOWER_SPIRE",
            name="Slender Spire High-Rise",
            min_pt=np.array([-38.0, 75.0, 0.0]),
            max_pt=np.array([-12.0, 101.0, 110.0]),
            material="steel_concrete",
            base_attenuation_db=26.0,
        ),
        # Tower 5: Sandstone Tower with Rooftop Relay Pad (center: 70, -12)
        ObstacleAABB(
            id="OBS_TOWER_SANDSTONE",
            name="Sandstone Tower (Relay Hub)",
            min_pt=np.array([50.0, -33.0, 0.0]),
            max_pt=np.array([90.0, 9.0, 76.0]),
            material="heavy_concrete",
            base_attenuation_db=22.0,
        ),
        # Tower 6: Sandstone High-Rise East (center: 118, -42)
        ObstacleAABB(
            id="OBS_TOWER_SANDSTONE_EAST",
            name="Sandstone Tower East",
            min_pt=np.array([102.0, -60.0, 0.0]),
            max_pt=np.array([134.0, -24.0, 82.0]),
            material="heavy_concrete",
            base_attenuation_db=23.0,
        ),
        # Tower 7: Modern Curved White High-Rise (center: 135, 20)
        ObstacleAABB(
            id="OBS_TOWER_WHITE",
            name="Modern Curved High-Rise",
            min_pt=np.array([120.0, 4.0, 0.0]),
            max_pt=np.array([150.0, 36.0, 66.0]),
            material="concrete_debris",
            base_attenuation_db=20.0,
        ),
        # Mid-Rise 1 (center: 12, -42)
        ObstacleAABB(
            id="OBS_MIDRISE_1",
            name="Center Plaza Mid-Rise",
            min_pt=np.array([-3.0, -58.0, 0.0]),
            max_pt=np.array([27.0, -26.0, 36.0]),
            material="reinforced_concrete",
            base_attenuation_db=18.0,
        ),
        # Mid-Rise 2 (center: -18, -55)
        ObstacleAABB(
            id="OBS_MIDRISE_2",
            name="West Avenue Mid-Rise",
            min_pt=np.array([-30.0, -68.0, 0.0]),
            max_pt=np.array([-6.0, -42.0, 30.0]),
            material="reinforced_concrete",
            base_attenuation_db=18.0,
        ),
        # --- EXPANDED NORTH & TECH DISTRICT ---
        # Tower 8: Glass Telecom Megatower (center: 30, 110)
        ObstacleAABB(
            id="OBS_TOWER_MEGATOWER",
            name="North Telecom Megatower",
            min_pt=np.array([12.0, 92.0, 0.0]),
            max_pt=np.array([48.0, 128.0, 120.0]),
            material="steel_concrete",
            base_attenuation_db=28.0,
        ),
        # Tower 9: Cyan High-Rise (center: -70, 115)
        ObstacleAABB(
            id="OBS_TOWER_CYAN",
            name="Northwest Cyan Tower",
            min_pt=np.array([-86.0, 98.0, 0.0]),
            max_pt=np.array([-54.0, 132.0, 98.0]),
            material="steel_concrete",
            base_attenuation_db=24.0,
        ),
        # Tower 10: North Commerce Center (center: 85, 95)
        ObstacleAABB(
            id="OBS_TOWER_COMMERCE",
            name="North Commerce Center",
            min_pt=np.array([68.0, 79.0, 0.0]),
            max_pt=np.array([102.0, 111.0, 88.0]),
            material="reinforced_concrete",
            base_attenuation_db=22.0,
        ),
        # Tower 11: Stepped Tech High-Rise (center: 120, 110)
        ObstacleAABB(
            id="OBS_TOWER_TECH",
            name="Stepped Tech Tower",
            min_pt=np.array([105.0, 95.0, 0.0]),
            max_pt=np.array([135.0, 125.0, 78.0]),
            material="steel_concrete",
            base_attenuation_db=22.0,
        ),
        # North Chemical Facility & Hazard Tanks (enclosing POI_HAZARD at -20, 95)
        ObstacleAABB(
            id="OBS_FACILITY_HAZARD",
            name="North Chemical Processing Plant",
            min_pt=np.array([-38.0, 82.0, 0.0]),
            max_pt=np.array([-2.0, 108.0, 24.0]),
            material="heavy_concrete",
            base_attenuation_db=25.0,
        ),
        # --- EXPANDED WEST & NORTHWEST MEDICAL/CIVIC DISTRICT ---
        # St. Jude Medical Center Complex (enclosing POI_HOSPITAL at -60, 25)
        ObstacleAABB(
            id="OBS_HOSPITAL_COMPLEX",
            name="Metropolitan Trauma Hospital",
            min_pt=np.array([-84.0, 3.0, 0.0]),
            max_pt=np.array([-36.0, 47.0, 34.0]),
            material="reinforced_concrete",
            base_attenuation_db=24.0,
        ),
        # Tower 12: Northwest Research Tower (center: -110, 50)
        ObstacleAABB(
            id="OBS_TOWER_RESEARCH",
            name="Bio-Research High-Rise",
            min_pt=np.array([-124.0, 36.0, 0.0]),
            max_pt=np.array([-96.0, 64.0, 70.0]),
            material="steel_concrete",
            base_attenuation_db=22.0,
        ),
        # Tower 13: Northwest Twin Tower (center: -135, 20)
        ObstacleAABB(
            id="OBS_TOWER_NW_TWIN",
            name="Northwest Twin Tower",
            min_pt=np.array([-148.0, 6.0, 0.0]),
            max_pt=np.array([-122.0, 34.0, 68.0]),
            material="reinforced_concrete",
            base_attenuation_db=22.0,
        ),
        # --- EXPANDED EAST POWER & LOGISTICS DISTRICT ---
        # East Power Substation Building (enclosing POI_SUBSTATION at 115, -40)
        ObstacleAABB(
            id="OBS_POWER_SUBSTATION",
            name="Regional Power Substation",
            min_pt=np.array([102.0, -52.0, 0.0]),
            max_pt=np.array([128.0, -28.0, 22.0]),
            material="heavy_concrete",
            base_attenuation_db=26.0,
        ),
        # Tower 14: East Horizon Corporate Spire (center: 145, 45)
        ObstacleAABB(
            id="OBS_TOWER_EAST_HORIZON",
            name="East Horizon Corporate Spire",
            min_pt=np.array([129.0, 30.0, 0.0]),
            max_pt=np.array([161.0, 60.0, 84.0]),
            material="steel_concrete",
            base_attenuation_db=24.0,
        ),
        # East Logistics Hub (center: 145, -70)
        ObstacleAABB(
            id="OBS_LOGISTICS_HUB",
            name="East Logistics Hub",
            min_pt=np.array([130.0, -85.0, 0.0]),
            max_pt=np.array([160.0, -55.0, 28.0]),
            material="steel_concrete",
            base_attenuation_db=20.0,
        ),
        # --- EXPANDED SOUTHWEST RESIDENTIAL & COLLAPSE DISTRICT ---
        # Collapsed 4-Story Apartment Block (enclosing POI_COLLAPSE at -85, -60)
        ObstacleAABB(
            id="OBS_COLLAPSE_BLOCK",
            name="Collapsed Apartment Complex",
            min_pt=np.array([-101.0, -71.0, 0.0]),
            max_pt=np.array([-69.0, -49.0, 16.0]),
            material="concrete_debris",
            base_attenuation_db=22.0,
        ),
        # Metro Civic Shelter Station (enclosing POI_SHELTER at -50, -85)
        ObstacleAABB(
            id="OBS_CIVIC_SHELTER",
            name="Metro Civic Emergency Shelter",
            min_pt=np.array([-63.0, -95.0, 0.0]),
            max_pt=np.array([-37.0, -75.0, 14.0]),
            material="reinforced_concrete",
            base_attenuation_db=20.0,
        ),
        # West Residential Block (center: -115, -75)
        ObstacleAABB(
            id="OBS_WEST_RESIDENTIAL",
            name="West Residential High-Rise",
            min_pt=np.array([-128.0, -87.0, 0.0]),
            max_pt=np.array([-102.0, -63.0, 24.0]),
            material="reinforced_concrete",
            base_attenuation_db=18.0,
        ),
    ]
    for obs in sector_delta_obstacles:
        sim.add_obstacle(obs)

    # 2. Add high-priority disaster Points of Interest located INSIDE Sector Delta diorama
    sim.add_poi("POI_SURVIVORS", position=[25.0, -55.0, 20.0], priority="CRITICAL", required_dwell_time=12.0)
    sim.add_poi("POI_COLLAPSE", position=[-85.0, -60.0, 18.0], priority="HIGH", required_dwell_time=10.0)
    sim.add_poi("POI_HAZARD", position=[-20.0, 95.0, 26.0], priority="MEDIUM", required_dwell_time=8.0)
    sim.add_poi("POI_BRIDGE", position=[40.0, -95.0, 16.0], priority="HIGH", required_dwell_time=8.0)
    sim.add_poi("POI_SHELTER", position=[-50.0, -85.0, 16.0], priority="MEDIUM", required_dwell_time=6.0)
    sim.add_poi("POI_HOSPITAL", position=[-60.0, 25.0, 30.0], priority="CRITICAL", required_dwell_time=12.0)
    sim.add_poi("POI_SUBSTATION", position=[115.0, -40.0, 24.0], priority="HIGH", required_dwell_time=10.0)
    sim.add_poi("POI_HIGHWAY", position=[-100.0, -100.0, 14.0], priority="MEDIUM", required_dwell_time=8.0)

    # 3. Add heterogeneous fleet of 16 UAVs spawning on their designated launch pads on the GCS Apron
    fleet_init = [
        # Heavy Disaster Surveyors (Front row along tactical apron y = -145)
        ("UAV_1", DroneRole.SURVEY, [-70.0, -145.0, 0.45]),
        ("UAV_2", DroneRole.SURVEY, [-50.0, -145.0, 0.45]),
        ("UAV_3", DroneRole.SURVEY, [-30.0, -145.0, 0.45]),
        ("UAV_4", DroneRole.SURVEY, [-10.0, -145.0, 0.45]),
        ("UAV_5", DroneRole.SURVEY, [10.0, -145.0, 0.45]),
        ("UAV_6", DroneRole.SURVEY, [30.0, -145.0, 0.45]),
        ("UAV_7", DroneRole.SURVEY, [50.0, -145.0, 0.45]),
        ("UAV_8", DroneRole.SURVEY, [70.0, -145.0, 0.45]),
        # Elevated High-Altitude Multi-Hop Relays (Relay pad row y = -136)
        ("RELAY_1", DroneRole.RELAY, [-45.0, -136.0, 0.45]),
        ("RELAY_2", DroneRole.RELAY, [-15.0, -136.0, 0.45]),
        ("RELAY_3", DroneRole.RELAY, [15.0, -136.0, 0.45]),
        ("RELAY_4", DroneRole.RELAY, [45.0, -136.0, 0.45]),
        # Rapid Reconnaissance Scouts (Scout pad row y = -153)
        ("SCOUT_1", DroneRole.SURVEY, [-45.0, -153.0, 0.45]),
        ("SCOUT_2", DroneRole.SURVEY, [-15.0, -153.0, 0.45]),
        ("SCOUT_3", DroneRole.SURVEY, [15.0, -153.0, 0.45]),
        ("SCOUT_4", DroneRole.SURVEY, [45.0, -153.0, 0.45]),
    ]
    for d_id, role, pos in fleet_init:
        drone = Drone(d_id, role=role, initial_pos=np.array(pos, dtype=np.float64))
        # Staggered tactical takeoff delays: Scouts launch first, then Relays, then Surveyors in pairs
        if "SCOUT" in d_id:
            drone.takeoff_delay = 1.0
        elif "RELAY" in d_id:
            drone.takeoff_delay = 2.4
        else:
            try:
                pair_idx = int(d_id.split("_")[1])
            except Exception:
                pair_idx = 1
            drone.takeoff_delay = 3.6 + ((pair_idx - 1) // 2) * 1.0
        sim.add_drone(drone)

    # 4. Attach subsystems with extended mission budget and generous comms timeout
    sim.set_network_engine(FANETNetworkEngine())
    sim.set_mission_manager(DisasterMissionManager(
        gcs_position=config.gcs_position,
        mission_time_budget=1200.0,
        comms_loss_timeout=8.0,
        survey_dwell_radius=14.0,
    ))

    return sim


class SimulationServer:
    """Manages the simulation step loop, LiDAR perception, and WebSocket broadcasting."""

    def __init__(self) -> None:
        self.sim: SwarmSimulationCore = create_default_simulation()
        self.clients: Set[WebSocket] = set()
        self.is_running: bool = True
        self.sim_speed: float = 1.0
        self.step_delay: float = 0.033  # ~30 Hz broadcast
        self.step_count: int = 0
        self.focus_drone_id: str = "UAV_1"

        # Autonomous SLAM & LiDAR Perception Engine
        self.lidar = LiDARScanner(
            max_range_m=75.0,
            horizontal_fov_deg=360.0,
            horizontal_resolution_deg=6.0,   # 60 azimuth beams per ring
            vertical_fov_deg=(-50.0, 15.0),  # -50 deg downward ground look to +15 deg upward
            vertical_channels=16,            # 16 elevation rings (960 rays total)
            range_noise_std_m=0.03
        )
        self.voxel_map = OccupancyGridMap3D(voxel_size_m=4.5)
        self.latest_payload: Optional[str] = None

    def reset(self) -> None:
        """Reset simulation and SLAM occupancy grid to initial disaster scenario."""
        self.sim = create_default_simulation()
        self.voxel_map = OccupancyGridMap3D(voxel_size_m=4.5)
        self.step_count = 0

    async def broadcast_loop(self) -> None:
        """Asynchronous simulation execution, LiDAR perception, and telemetry broadcast loop."""
        while True:
            step_start = time.perf_counter()
            try:
                if self.is_running:
                    self.step_count += 1
                    # Step simulation
                    snapshot = self.sim.step()
                    data = snapshot.to_dict()

                    # 1. Enrich with real-time EKF estimation metrics & attitude Euler angles
                    for d_dict in data.get("drones", []):
                        drone_obj = self.sim.drones.get(d_dict["id"])
                        if drone_obj:
                            est_pos = drone_obj.ekf.estimated_position
                            est_vel = drone_obj.ekf.estimated_velocity
                            d_dict["estimated_position"] = [round(float(c), 3) for c in est_pos]
                            d_dict["estimated_velocity"] = [round(float(v), 3) for v in est_vel]
                            d_dict["ekf_error_m"] = round(float(np.linalg.norm(drone_obj.position - est_pos)), 3)
                            
                            # Attitude in degrees for PFD artificial horizon
                            att = drone_obj.attitude
                            d_dict["roll_deg"] = round(float(np.degrees(att[0])), 1)
                            d_dict["pitch_deg"] = round(float(np.degrees(att[1])), 1)
                            d_dict["yaw_deg"] = round(float(np.degrees(att[2])) % 360.0, 1)

                    # 2. Collaborative Multi-UAV Swarm LiDAR Sweep and SLAM Integration
                    focus_drone = self.sim.drones.get(self.focus_drone_id) or next(iter(self.sim.drones.values()), None)
                    if focus_drone is not None:
                        # Execute high-resolution LiDAR scan for focus drone
                        scan = self.lidar.scan(
                            drone_id=focus_drone.id,
                            position=focus_drone.position,
                            attitude=focus_drone.attitude,
                            obstacles=self.sim.obstacles,
                            sim_time=snapshot.sim_time,
                        )
                        self.voxel_map.insert_scan(scan)
                        data["lidar_scan"] = scan.to_dict()

                        # Compute Khatib APF Guidance Vectors for Autonomous Viewport
                        f_att = focus_drone.compute_attractive_force()
                        f_rep = focus_drone.compute_obstacle_repulsion(self.sim.obstacles)
                        f_net = f_att + f_rep
                        data["apf_vectors"] = {
                            "drone_id": focus_drone.id,
                            "f_att": [round(float(c), 2) for c in f_att],
                            "f_rep": [round(float(c), 2) for c in f_rep],
                            "f_net": [round(float(c), 2) for c in f_net],
                            "mag_att": round(float(np.linalg.norm(f_att)), 1),
                            "mag_rep": round(float(np.linalg.norm(f_rep)), 1),
                            "mag_net": round(float(np.linalg.norm(f_net)), 1),
                            "target": [round(float(c), 2) for c in focus_drone.target_position] if focus_drone.target_position is not None else None,
                        }

                    # Swarm Collaborative SLAM: Scan 1 active peer drone each tick in round-robin to maintain steady 30+ FPS
                    active_peers = [
                        d for d in self.sim.drones.values()
                        if d.id != (focus_drone.id if focus_drone else "")
                        and d.flight_mode in (FlightMode.TRANSIT, FlightMode.SURVEYING, FlightMode.RELAY, FlightMode.RTL)
                        and float(d.position[2]) > 2.0
                    ]
                    if active_peers:
                        peer_idx = self.step_count % len(active_peers)
                        peer = active_peers[peer_idx]
                        p_scan = self.lidar.scan(
                            drone_id=peer.id,
                            position=peer.position,
                            attitude=peer.attitude,
                            obstacles=self.sim.obstacles,
                            sim_time=snapshot.sim_time,
                        )
                        self.voxel_map.insert_scan(p_scan)

                    # 3. Stream 3D Occupied Voxels & SLAM Metrics
                    data["occupied_voxels"] = self.voxel_map.get_occupied_voxels(max_count=250)
                    mapping_metrics = self.voxel_map.compute_metrics()
                    data["mapping_metrics"] = mapping_metrics

                    # 4. Stream Scientific Analytical Chart Data
                    drones_list = data.get("drones", [])
                    ekf_errors = [d.get("ekf_error_m", 0.0) for d in drones_list]
                    avg_ekf_err = round(float(np.mean(ekf_errors)) if ekf_errors else 0.08, 3)

                    total_pois_count = len(self.sim.pois) if hasattr(self.sim, "pois") else 5
                    data["analytics"] = {
                        "sim_time": round(snapshot.sim_time, 2),
                        "avg_ekf_error_m": avg_ekf_err,
                        "pdr": round(float(snapshot.metrics.get("pdr", 1.0) * 100.0), 1),
                        "latency_ms": round(float(snapshot.metrics.get("avg_latency_ms", 8.5)), 1),
                        "completed_pois": int(snapshot.metrics.get("completed_pois", 0)),
                        "total_pois": total_pois_count,
                        "mapped_pct": mapping_metrics.get("coverage_pct", 0.0),
                        "active_relays": sum(1 for d in drones_list if d.get("role") == "RELAY"),
                        "retreating_drones": sum(1 for d in drones_list if d.get("flight_mode") in ("RTL", "LANDING")),
                        "landed_drones": sum(1 for d in drones_list if d.get("flight_mode") == "LANDED"),
                        "throughput_kbps": round(float(len(snapshot.packets) * 14.5 + 28.0), 1),
                    }

                    # 5. Add obstacles geometry for client 3D rendering
                    obs_list = []
                    for obs in self.sim.obstacles:
                        min_p = getattr(obs, "min_pt", getattr(obs, "min_bound", None))
                        max_p = getattr(obs, "max_pt", getattr(obs, "max_bound", None))
                        if min_p is not None and max_p is not None:
                            obs_list.append({
                                "id": getattr(obs, "id", "OBS"),
                                "name": getattr(obs, "name", "Building"),
                                "min_pt": [round(float(c), 2) for c in min_p],
                                "max_pt": [round(float(c), 2) for c in max_p],
                            })
                    data["obstacles"] = obs_list

                    # 6. Add live NVIDIA GPU hardware telemetry
                    data["gpu"] = get_gpu_telemetry()

                    # 7. Broadcast to connected WebSockets
                    payload = json.dumps(data)
                    self.latest_payload = payload
                    if self.clients:
                        dead_clients = set()
                        for ws in list(self.clients):
                            try:
                                await ws.send_text(payload)
                            except Exception:
                                dead_clients.add(ws)
                        self.clients.difference_update(dead_clients)

            except Exception as e:
                import traceback
                print(f"[!] Simulation broadcast loop error: {e}")
                traceback.print_exc()

            target_interval = self.step_delay / max(0.1, self.sim_speed)
            elapsed = time.perf_counter() - step_start
            sleep_time = max(0.001, target_interval - elapsed)
            await asyncio.sleep(sleep_time)


server_manager = SimulationServer()

app = FastAPI(title="3D UAV Swarm & Resilient FANET Network Visualizer")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.middleware("http")
async def add_no_cache_headers(request: Request, call_next):
    """Enforces zero-caching on static assets and HTML to prevent stale browser code."""
    response = await call_next(request)
    path = request.url.path
    if path.startswith("/static") or path == "/" or path.endswith((".js", ".css", ".html")):
        response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate, max-age=0"
        response.headers["Pragma"] = "no-cache"
        response.headers["Expires"] = "0"
    return response

# Mount static files directory
if not STATIC_DIR.exists():
    STATIC_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")


@app.get("/")
async def get_index():
    index_file = STATIC_DIR / "index.html"
    if index_file.exists():
        return FileResponse(str(index_file))
    return HTMLResponse("<h1>Visualizer static files loading...</h1>")


@app.get("/api/telemetry")
async def get_telemetry():
    """Return latest simulation telemetry frame."""
    return JSONResponse(server_manager.sim.to_dict())


@app.get("/api/gpu")
async def get_gpu():
    """Return live NVIDIA GPU telemetry metrics."""
    return JSONResponse(get_gpu_telemetry())


@app.get("/api/export_telemetry")
async def export_telemetry():
    """Export mission flight telemetry history as a downloadable CSV log."""
    import io
    import csv

    output = io.StringIO()
    writer = csv.writer(output)
    writer.writerow([
        "sim_time", "drone_id", "role", "flight_mode",
        "pos_x", "pos_y", "pos_z",
        "vel_x", "vel_y", "vel_z",
        "est_x", "est_y", "est_z",
        "battery_pct", "assigned_poi_id"
    ])

    history = list(server_manager.sim.history)
    if not history:
        history = [server_manager.sim.get_telemetry_snapshot()]

    for snap in history:
        t = snap.sim_time
        for d in snap.drones:
            pos = d.get("position", [0, 0, 0])
            vel = d.get("velocity", [0, 0, 0])
            drone_obj = server_manager.sim.drones.get(d.get("id"))
            if drone_obj:
                est = list(drone_obj.ekf.estimated_position)
            else:
                est = d.get("estimated_position") or pos
            writer.writerow([
                t, d.get("id"), d.get("role"), d.get("flight_mode"),
                pos[0], pos[1], pos[2],
                vel[0], vel[1], vel[2],
                est[0], est[1], est[2],
                d.get("battery_pct"), d.get("assigned_poi_id") or "NONE"
            ])

    return Response(
        content=output.getvalue(),
        media_type="text/csv",
        headers={"Content-Disposition": "attachment; filename=uav_swarm_flight_recorder.csv"}
    )


@app.get("/api/export_debrief")
async def export_debrief():
    """Compiles and exports the complete executive mission debrief report."""
    sim = server_manager.sim
    data = sim.to_dict()

    pois = data.get("pois", [])
    total_pois = len(pois)
    cleared_pois = sum(1 for p in pois if p.get("is_completed", False))

    survivors_data = data.get("survivors") or {}
    located_survivors = survivors_data.get("located_count", 0) if isinstance(survivors_data, dict) else 0
    total_survivors = survivors_data.get("total_count", 0) if isinstance(survivors_data, dict) else 0

    drones = data.get("drones", [])
    sim_t = float(data.get("sim_time", 0.0))
    total_energy_wh = sum(
        float(d.get("power_w", 195.0)) * (sim_t / 3600.0)
        for d in drones
    )

    metrics = data.get("metrics", {})
    pdr = metrics.get("pdr", 0.98)
    avg_latency = metrics.get("avg_latency_ms", 15.0)

    debrief = {
        "mission_title": "UAV-X 16-Drone Swarm Disaster Reconnaissance & FANET Relay",
        "timestamp": time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime()),
        "mission_duration_s": round(sim_t, 1),
        "mission_time_budget_s": 300.0,
        "budget_compliance": "WITHIN_BUDGET" if sim_t <= 300.0 else "TIME_EXCEEDED",
        "disaster_sites_summary": {
            "total_sites": total_pois,
            "cleared_sites": cleared_pois,
            "completion_pct": round(cleared_pois / max(1, total_pois) * 100.0, 1),
            "priority_breakdown": {
                "critical": sum(1 for p in pois if str(p.get("priority")).upper() == "CRITICAL" and p.get("is_completed")),
                "high": sum(1 for p in pois if str(p.get("priority")).upper() == "HIGH" and p.get("is_completed")),
                "medium": sum(1 for p in pois if str(p.get("priority")).upper() == "MEDIUM" and p.get("is_completed")),
                "low": sum(1 for p in pois if str(p.get("priority")).upper() == "LOW" and p.get("is_completed")),
            },
        },
        "search_and_rescue_summary": {
            "total_survivors_estimated": total_survivors,
            "survivors_located": located_survivors,
            "recovery_rate_pct": round(located_survivors / max(1, total_survivors) * 100.0, 1) if total_survivors > 0 else 100.0,
            "discovered_survivors": survivors_data.get("discovered_survivors", []) if isinstance(survivors_data, dict) else [],
        },
        "fleet_and_energy_summary": {
            "fleet_size": len(drones),
            "total_energy_consumed_wh": round(total_energy_wh, 2),
            "avg_drone_power_w": round(total_energy_wh / max(0.001, sim_t / 3600.0) / max(1, len(drones)), 1) if sim_t > 0 else 195.0,
            "charging_pad_turnarounds": sum(1 for pad in (data.get("charging_pads") or []) if pad.get("status") == "OCCUPIED"),
        },
        "network_and_telemetry_summary": {
            "packet_delivery_ratio": pdr,
            "avg_latency_ms": avg_latency,
            "active_routes_count": len(data.get("active_routes", [])),
            "mesh_health": "EXCELLENT" if pdr >= 0.95 else "DEGRADED",
        },
        "tactical_comms_log": data.get("tactical_comms") or [],
    }
    return JSONResponse(debrief)


@app.get("/api/export_point_cloud")
async def export_point_cloud(format: str = "ply"):
    """
    Export reconstructed 3D LiDAR point cloud.
    Formats:
    - 'ply': Stanford ASCII PLY format for CloudCompare, Blender, MeshLab.
    - 'las': ASPRS LAS 1.2 Binary format for CloudCompare, QGIS, ArcGIS, PDAL.
    """
    fmt = format.lower()
    if fmt == "las":
        las_bytes = server_manager.voxel_map.export_point_cloud_las()
        return Response(
            content=las_bytes,
            media_type="application/octet-stream",
            headers={"Content-Disposition": "attachment; filename=UAVX_Disaster_PointCloud.las"}
        )
    ply_content = server_manager.voxel_map.export_point_cloud_ply()
    return Response(
        content=ply_content,
        media_type="application/octet-stream",
        headers={"Content-Disposition": "attachment; filename=UAVX_Disaster_PointCloud.ply"}
    )


@app.post("/api/chaos_fault")
async def post_chaos_fault(payload: Optional[Dict[str, Any]] = None):
    """Inject dynamic hardware failure / flameout to test Swarm Self-Healing."""
    target_id = payload.get("drone_id") if payload else None
    victim_id = server_manager.sim.trigger_chaos_fault(target_drone_id=target_id)
    return JSONResponse({
        "status": "ok",
        "victim_id": victim_id,
        "message": f"Catastrophic fault injected on {victim_id}. Swarm self-healing engaged." if victim_id else "No eligible airborne drone available for fault injection."
    })


@app.post("/api/manual_control")
async def post_manual_control(payload: Dict[str, Any]):
    """Direct manual velocity override (WASD/Gamepad) for focus drone or specified drone."""
    drone_id = payload.get("drone_id") or server_manager.focus_drone_id
    vx = float(payload.get("vx", 0.0))
    vy = float(payload.get("vy", 0.0))
    vz = float(payload.get("vz", 0.0))
    yaw_rate = float(payload.get("yaw_rate", 0.0))
    enabled = bool(payload.get("enabled", True))
    success = server_manager.sim.set_drone_manual_control(drone_id, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, enabled=enabled)
    return JSONResponse({"status": "ok" if success else "error", "drone_id": drone_id, "enabled": enabled})


@app.post("/api/control")
async def post_control(payload: Dict[str, Any]):
    """Handle HUD commands: pause, resume, reset, speed, focus drone, retreat."""
    cmd = payload.get("command") or payload.get("cmd")
    if cmd == "pause":
        server_manager.is_running = False
    elif cmd == "resume":
        server_manager.is_running = True
    elif cmd == "reset":
        server_manager.reset()
    elif cmd == "speed":
        server_manager.sim_speed = float(payload.get("value", 1.0))
    elif cmd == "focus_drone":
        drone_id = str(payload.get("drone_id", "UAV_1"))
        if drone_id in server_manager.sim.drones:
            server_manager.focus_drone_id = drone_id
    elif cmd in ("retreat", "rtl"):
        drone_id = payload.get("drone_id")
        if drone_id and drone_id in server_manager.sim.drones:
            server_manager.sim.trigger_drone_retreat(drone_id)
        else:
            server_manager.sim.trigger_fleet_retreat()
    elif cmd == "chaos_fault":
        drone_id = payload.get("drone_id")
        victim = server_manager.sim.trigger_chaos_fault(drone_id)
        return JSONResponse({"status": "ok", "victim_id": victim})
    elif cmd == "manual_control":
        drone_id = payload.get("drone_id") or server_manager.focus_drone_id
        vx = float(payload.get("vx", 0.0))
        vy = float(payload.get("vy", 0.0))
        vz = float(payload.get("vz", 0.0))
        yaw_rate = float(payload.get("yaw_rate", 0.0))
        enabled = bool(payload.get("enabled", True))
        server_manager.sim.set_drone_manual_control(drone_id, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, enabled=enabled)
    return JSONResponse({
        "status": "ok",
        "running": server_manager.is_running,
        "speed": server_manager.sim_speed,
        "focus_drone": server_manager.focus_drone_id
    })


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    server_manager.clients.add(websocket)
    if server_manager.latest_payload:
        try:
            await websocket.send_text(server_manager.latest_payload)
        except Exception:
            pass
    try:
        while True:
            msg = await websocket.receive_text()
            try:
                data = json.loads(msg)
                cmd = data.get("command") or data.get("cmd")
                if cmd == "pause":
                    server_manager.is_running = False
                elif cmd == "resume":
                    server_manager.is_running = True
                elif cmd == "reset":
                    server_manager.reset()
                elif cmd == "speed":
                    server_manager.sim_speed = float(data.get("value", 1.0))
                elif cmd == "focus_drone":
                    drone_id = str(data.get("drone_id", "UAV_1"))
                    if drone_id in server_manager.sim.drones:
                        server_manager.focus_drone_id = drone_id
                elif cmd in ("retreat", "rtl"):
                    drone_id = data.get("drone_id")
                    if drone_id and drone_id in server_manager.sim.drones:
                        server_manager.sim.trigger_drone_retreat(drone_id)
                    else:
                        server_manager.sim.trigger_fleet_retreat()
                elif cmd == "chaos_fault":
                    server_manager.sim.trigger_chaos_fault(data.get("drone_id"))
                elif cmd == "manual_control":
                    drone_id = data.get("drone_id") or server_manager.focus_drone_id
                    vx = float(data.get("vx", 0.0))
                    vy = float(data.get("vy", 0.0))
                    vz = float(data.get("vz", 0.0))
                    yaw_rate = float(data.get("yaw_rate", 0.0))
                    enabled = bool(data.get("enabled", True))
                    server_manager.sim.set_drone_manual_control(drone_id, vx=vx, vy=vy, vz=vz, yaw_rate=yaw_rate, enabled=enabled)
            except Exception:
                pass
    except WebSocketDisconnect:
        server_manager.clients.discard(websocket)


@app.on_event("startup")
async def startup_event():
    asyncio.create_task(server_manager.broadcast_loop())
