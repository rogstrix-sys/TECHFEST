#!/usr/bin/env python3
"""
run_simulation.py: Unified Launcher for 3D Resilient Multi-Hop UAV Swarm Simulation.

Features:
- Automated dependency verification & pip install.
- Launches 3D WebGL Three.js Cockpit over FastAPI/WebSocket on http://localhost:8000.
- Automatically opens the default browser to the interactive 3D cockpit.
- Supports headless mode (--headless) for automated benchmarking and testing.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
import webbrowser

REQUIRED_PACKAGES = [
    ("numpy", "numpy>=1.24.0"),
    ("fastapi", "fastapi>=0.100.0"),
    ("uvicorn", "uvicorn>=0.22.0"),
    ("websockets", "websockets>=11.0"),
]


def check_and_install_dependencies() -> None:
    """Ensure all required Python packages are installed without friction."""
    missing = []
    for mod_name, pkg_spec in REQUIRED_PACKAGES:
        try:
            __import__(mod_name)
        except ImportError:
            missing.append(pkg_spec)

    if missing:
        print(f"[*] Installing missing dependencies: {', '.join(missing)}...")
        try:
            subprocess.check_call(
                [sys.executable, "-m", "pip", "install", *missing],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print("[+] Dependencies installed successfully.")
        except Exception as e:
            print(f"[!] Warning: Auto-install failed ({e}). Proceeding anyway...")


def run_headless_simulation(
    duration: float = 30.0,
    dt: float = 0.05,
    num_drones: int = 8,
    num_pois: int = 5,
    enable_weather: bool = False,
) -> None:
    """Run pure-Python simulation in headless mode with real-time ASCII telemetry."""
    from sim.core import SimulationConfig, SwarmSimulationCore
    from sim.drone import Drone
    from sim.mission import DisasterMissionManager
    from sim.network import FANETNetworkEngine
    from sim.obstacles import create_default_disaster_obstacles
    from sim.types import DroneRole

    print("=" * 72)
    print("  UAV-X: RESILIENT MULTI-HOP AERIAL SWARM SIMULATION (HEADLESS MODE)")
    print("=" * 72)
    weather_desc = "ACTIVE (Dryden MIL-F-8785C + Shear)" if enable_weather else "OFF (Calm Air)"
    print(f"[*] Configuration: Fleet={num_drones} UAVs, PoIs={num_pois}, Duration={duration}s, dt={dt}s, Weather={weather_desc}")

    config = SimulationConfig(
        dt=dt,
        max_duration=duration,
        gcs_position=(0.0, -150.0, 0.0),
        enable_weather=enable_weather,
    )
    sim = SwarmSimulationCore(config)

    for obs in create_default_disaster_obstacles():
        sim.add_obstacle(obs)

    default_pois = [
        ("POI_SURVIVORS", [115.0, 75.0, 25.0], "CRITICAL", 10.0),
        ("POI_COLLAPSE", [-100.0, 35.0, 28.0], "HIGH", 8.0),
        ("POI_HAZARD", [10.0, 110.0, 30.0], "MEDIUM", 6.0),
        ("POI_BRIDGE", [-40.0, 120.0, 22.0], "HIGH", 8.0),
        ("POI_SHELTER", [60.0, -20.0, 20.0], "MEDIUM", 6.0),
    ]
    for i in range(min(num_pois, len(default_pois))):
        p_id, pos, pri, dwell = default_pois[i]
        sim.add_poi(p_id, position=pos, priority=pri, required_dwell_time=dwell)

    for i in range(num_drones):
        role = DroneRole.RELAY if i >= max(2, num_drones - 2) else DroneRole.SURVEY
        d_id = f"UAV_{i+1}"
        drone = Drone(d_id, role=role, initial_pos=[-30.0 + i * 15.0, -130.0, 0.0])
        sim.add_drone(drone)

    sim.set_network_engine(FANETNetworkEngine())
    sim.set_mission_manager(DisasterMissionManager(gcs_position=config.gcs_position))

    print("[+] Swarm initialized. Running simulation...")
    t0 = time.perf_counter()
    steps = int(duration / dt)

    for step in range(steps):
        snap = sim.step(dt)

        if step % 40 == 0 or step == steps - 1:
            routes_str = " | ".join([" -> ".join(r) for r in snap.active_routes[:2]]) or "DIRECT"
            pdr = snap.metrics.get("pdr", 1.0) * 100.0
            lat = snap.metrics.get("avg_latency_ms", 0.0)
            cmpl = snap.metrics.get("completed_pois", 0)
            print(
                f"[T={snap.sim_time:5.1f}s] PDR={pdr:5.1f}% | Latency={lat:4.1f}ms | "
                f"PoIs Done={cmpl}/{num_pois} | Routes: {routes_str}"
            )

    elapsed = time.perf_counter() - t0
    rate = steps / max(elapsed, 1e-4)
    print("=" * 72)
    print(f"[+] Simulation Completed! Executed {steps} steps in {elapsed:.2f}s ({rate:.1f} Hz)")
    print(f"[+] Packets Delivered to GCS: {sim.network_engine.gcs_packet_count}")
    print("=" * 72)


def find_free_port(preferred_port: int = 8000) -> int:
    """Check if preferred port is free; if not, find the next available port."""
    import socket
    for port in range(preferred_port, preferred_port + 50):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(("127.0.0.1", port))
                return port
            except OSError:
                continue
    return preferred_port


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV-X 3D Resilient Swarm Simulator")
    parser.add_argument("--headless", action="store_true", help="Run simulation headless without GUI")
    parser.add_argument("--duration", type=float, default=30.0, help="Simulation duration (seconds)")
    parser.add_argument("--drones", type=int, default=8, help="Number of UAVs in fleet")
    parser.add_argument("--pois", type=int, default=5, help="Number of Points of Interest")
    parser.add_argument("--port", type=int, default=8000, help="Web server port (default: 8000)")
    parser.add_argument("--hud", action="store_true", help="Launch native desktop military HUD (no web browser)")
    parser.add_argument("--no-browser", action="store_true", help="Do not auto-open web browser")
    parser.add_argument("--weather", action="store_true", help="Enable Dryden atmospheric wind & turbulence")
    parser.add_argument("--challenge", action="store_true", help="Launch MeitY / IIT Bombay / IISER Bhopal 1000m Challenge scenario")
    args = parser.parse_args()

    # Enforce NVIDIA GeForce RTX 4050 high-performance GPU binding
    try:
        from run_hud import configure_windows_gpu_preference
        configure_windows_gpu_preference()
    except Exception:
        pass

    os.environ["SHIM_MCCOMPAT"] = "0x800000001"
    os.environ["CUDA_VISIBLE_DEVICES"] = "0"
    os.environ["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    os.environ["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"

    check_and_install_dependencies()

    if args.hud:
        from run_hud import ensure_gui_dependencies
        ensure_gui_dependencies()
        from vis.desktop_hud import TacticalDesktopHUD
        hud = TacticalDesktopHUD(width=1280, height=720)
        if args.weather and hasattr(hud.sim, "config"):
            hud.sim.config.enable_weather = True
        hud.run()
        return

    if args.headless:
        run_headless_simulation(
            duration=args.duration,
            num_drones=args.drones,
            num_pois=args.pois,
            enable_weather=args.weather,
        )
    else:
        import uvicorn
        from vis.server import app, server_manager

        if args.challenge:
            server_manager.reset(scenario="challenge")
            print("[+] Initialized MeitY / IIT Bombay / IISER Bhopal 1000m Challenge Scenario.")

        active_port = find_free_port(args.port)
        if active_port != args.port:
            print(f"[!] Port {args.port} is in use by another process.")
            print(f"[+] Automatically selecting available port: {active_port}")

        url = f"http://127.0.0.1:{active_port}"
        print("=" * 72)
        print("  UAV-X: 3D RESILIENT MULTI-HOP AERIAL SWARM COCKPIT")
        print("=" * 72)
        print(f"[*] Starting local WebGL telemetry server on: {url}")
        print("[*] Press Ctrl+C in terminal to stop server.")
        print("=" * 72)

        if not args.no_browser:
            def open_browser():
                time.sleep(1.2)
                try:
                    from run_hud import find_browser_app_executable
                    browser_exe = find_browser_app_executable()
                    if browser_exe:
                        app_flags = [
                            browser_exe,
                            "--force-high-performance-gpu",
                            "--gpu-preference=2",
                            url,
                        ]
                        hud_env = os.environ.copy()
                        hud_env["SHIM_MCCOMPAT"] = "0x800000001"
                        hud_env["CUDA_VISIBLE_DEVICES"] = "0"
                        hud_env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
                        hud_env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"
                        subprocess.Popen(app_flags, env=hud_env)
                        return
                except Exception:
                    pass
                try:
                    webbrowser.open(url)
                except Exception:
                    pass

            import threading
            threading.Thread(target=open_browser, daemon=True).start()

        uvicorn.run(app, host="127.0.0.1", port=active_port, log_level="warning", ws_per_message_deflate=True)


if __name__ == "__main__":
    main()
