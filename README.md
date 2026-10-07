<div align="center">

# 🚁 UAV-X Swarm Simulation
### 3D Resilient Multi-Hop Aerial Communication Network & SLAM Autonomy Cockpit

[![Branch](https://img.shields.io/badge/branch-main%20%28stable%29-brightgreen.svg)](https://github.com/rogstrix-sys/TECHFEST/tree/main)
[![Python](https://img.shields.io/badge/Python-3.9%2B-green.svg)](https://python.org)
[![Three.js](https://img.shields.io/badge/Three.js-r128-blue.svg)](#)
[![IIT Bombay](https://img.shields.io/badge/IIT%20Bombay-TechFest-red.svg)](#)
[![GPU](https://img.shields.io/badge/GPU-NVIDIA%20RTX%204050-76b900.svg)](#)

> A production-grade, physics-based simulation of an autonomous UAV swarm conducting post-disaster aerial survey, multi-hop FANET communication relay, and real-time 3D SLAM mapping — rendered in a military-grade WebGL cockpit with native desktop HUD.

</div>

---

## 📋 Table of Contents
- [Overview](#-overview)
- [Key Features](#-key-features)
- [Architecture](#-architecture)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Desktop HUD](#-desktop-hud)
- [Project Structure](#-project-structure)
- [Technology Stack](#-technology-stack)
- [Simulation Parameters](#-simulation-parameters)
- [Test Suite](#-test-suite)
- [Releases & Changelog](#-releases--changelog)

---

## 🌐 Overview

**UAV-X Swarm Simulation** is a decoupled, high-performance simulation framework for a **Flying Ad-Hoc Network (FANET)** formed by a fleet of up to **16 autonomous UAVs** operating in a post-disaster urban zone. The swarm autonomously:

- 🗺️ **Surveys & Scans Sectors** — Autonomous spatial sector partitioning, coverage path planning, and multi-priority Points of Interest (Survivors, Structural Collapse, Hazard Zones)
- 📡 **Maintains Resilient Multi-Hop Communication** — Ground Control Station (GCS) connectivity via Dynamic Link-State routing with Ray-AABB LoS occlusion and 2D broadphase caching
- 🔄 **Self-Heals with DTN** — Store-and-Forward FIFO buffering during link dropouts and topological partitions
- 🧠 **Builds Collaborative 3D Maps** — Multi-drone LiDAR fusion, instanced OctoMap voxel SLAM, Bayesian survivor thermal heatmap, and PLY/LAS point cloud export
- 📐 **Executes Tactical Swarm Formations** — V-Formation, Diamond, Echelon, Line, and Circle with automated continuous GCS battery fast recharging
- 🛡️ **Fault-Tolerant Decision Making** — Byzantine-Fault-Tolerant (BFT) CBBA mission consensus with MAD outlier rejection and dynamic obstacle collapse avoidance
- 🛬 **Autonomously Retreats** — Coordinated full-fleet Return-to-Launch (RTL) on low battery or operator command
- 🏙️ **Renders in a Unified 3D City Diorama** — Sector Delta urban theater with LERP/SLERP motion smoothing, smoke/fire particle physics, trajectory ribbons, and interactive click-to-dispatch waypoints
- 📺 **Provides Real-Time Telemetry** — Dual-viewport WebGL cockpit, native military HUD (MIL-STD-1787D), delta telemetry encoding (-81% bandwidth), and analytical Chart.js graphs

Developed for **IIT Bombay TechFest** and national aerospace UAV challenges (MeitY / IISER Bhopal).

---

## ✨ Key Features

### Simulation Engine & Dynamics
| # | Feature | Details |
|---|---------|---------|
| 1 | **6-DOF Quadcopter Dynamics** | Newton-Euler rigid body, position/velocity integration, quaternion attitude |
| 2 | **LERP/SLERP Motion Smoothing** | Zero-stutter drone motion via linear position lerp + spherical quaternion interpolation |
| 3 | **Flocking & Collision Avoidance** | Khatib Artificial Potential Fields + Reynolds Boids + downwash repulsion |
| 4 | **4-Tier Altitude Corridors** | Launch [0–20m] → Survey [25–45m] → Transit [50–65m] → Relay [70–90m] |
| 5 | **Battery & Energy Model** | Electro-mechanical power draw; Return-to-Launch (RTL) on low battery threshold |
| 6 | **Automated GCS Fast Recharging** | Continuous docking pad rotation, rapid battery replenishment, and persistent fleet cycling |
| 7 | **Dynamic Obstacle Collapse** | Real-time building collapse simulation with dynamic collision envelopes & tangential bypass |
| 8 | **Sector Delta 3D Diorama** | Large-scale city environment; disaster sites embedded in urban fabric; 3D launch pads |
| 9 | **1000m Challenge Mode** | Support for MeitY / IIT Bombay / IISER Bhopal 1000m large-scale arena benchmarks |
| 10 | **Weather & Atmospheric Disturbances** | Wind gusts and turbulence affecting drone dynamics |

### FANET Communication & Routing
| # | Feature | Details |
|---|---------|---------|
| 11 | **RF Propagation (2.4 GHz + 915 MHz)** | Friis FSPL (PL₀ = 40.05 dB) + Log-distance (η_LoS=2.05, η_NLoS=3.60); dual-band |
| 12 | **Realistic LoS Ray-AABB Occlusion** | 3D ray-slab intersection test; +22 dB building penetration forces multi-hop routing |
| 13 | **Fast Broadphase Culling & Caching** | 2D AABB broadphase steering culling & LoS raycast caching for 2.1x network update speedup |
| 14 | **Dynamic Link-State Routing (DLS)** | Sub-ms Dijkstra; composite cost (SNR, distance, obstacle penalty, battery weight) |
| 15 | **DTN Store-and-Forward** | 250-packet FIFO ring buffer for transient link interruptions |
| 16 | **Delta Telemetry Encoding** | Multi-rate telemetry decoupling and link quantization reducing bandwidth by -81% |
| 17 | **Network Telemetry** | Real-time PDR, end-to-end latency, hop-count distribution |

### Mission Control & Swarm Autonomy
| # | Feature | Details |
|---|---------|---------|
| 18 | **Spatial Sector Scanning** | Autonomous sector partitioning, sweeping lawnmower scan patterns, and coverage path planning |
| 19 | **Tactical Swarm Formations** | Dynamic geometry switching: V-Formation, Diamond, Echelon, Line, and Circle |
| 20 | **Byzantine-Fault-Tolerant CBBA** | Consensus-Based Bundle Algorithm with Median Absolute Deviation (MAD) outlier rejection |
| 21 | **10-State MAVSDK-Compliant FSM** | IDLE → TAKEOFF → TRANSIT → SURVEYING → RELAY → DATA_TX → RTL → LANDING |
| 22 | **MAVLink Bridge** | `sim/mavlink_bridge.py` — real-drone MAVLink protocol relay (SITL / hardware) |
| 23 | **Virtual Spring Mesh (VSM)** | Relay UAVs self-position via spring-damper forces between GCS and Surveyors |
| 24 | **Battery-Comms Priority System** | Battery-aware dynamic role reassignment; comms degradation priority fallback |
| 25 | **Interactive Click-to-Dispatch** | Click-to-target waypoint dispatching directly in the 3D diorama |
| 26 | **Autonomous Retreat (RTL)** | Full-fleet coordinated return-to-launch on command or low-battery threshold |
| 27 | **Survey Progress Tracking** | Per-PoI survey completion percentage, dwell-time tracking, data-volume counters |

### SLAM, Perception & Mapping
| # | Feature | Details |
|---|---------|---------|
| 28 | **3D LiDAR Simulation** | Multi-beam rotating LiDAR with configurable FOV and range noise |
| 29 | **Multi-Drone Collaborative SLAM** | Global point cloud fusion from decentralized drone LiDAR sweeps |
| 30 | **Instanced OctoMap Voxel SLAM** | Log-odds 3D occupancy grid with GPU-instanced voxel rendering |
| 31 | **Bayesian Thermal Heatmap** | Recursive Bayes survivor probability grid updated from FLIR thermal sensor detections |
| 32 | **Multi-Colormap Rendering** | Real-time point cloud visualization in Elevation, Intensity, and Thermal colormaps |
| 33 | **PLY & LAS 3D Export** | Direct export of unified 3D point cloud maps to industry-standard PLY and LAS formats |
| 34 | **9-State EKF Localization** | Extended Kalman Filter fusing IMU + GPS + barometer for state estimation |
| 35 | **Volumetric Coverage Metrics** | Occupied voxels, surveyed volume (m³), mapping density |

### Visualization, HUD & Performance
| # | Feature | Details |
|---|---------|---------|
| 36 | **Unified 3D City Diorama** | High-fidelity urban diorama with animated rotors, runway, and embedded PoIs |
| 37 | **Three.js 3D WebGL Cockpit** | Dual-viewport split screen: External theater + Autonomous SLAM perception |
| 38 | **Zero-GC WebGL Optimization** | TypedArray object pooling and reusable geometry buffers eliminating GC stutter |
| 39 | **Smoke & Fire Particle Physics** | Dynamic GPU particle emitters for disaster zones and building collapses |
| 40 | **3D Flight Path Ribbons** | Trajectory history ribbons color-coded by UAV role and velocity |
| 41 | **Native Desktop HUD (MIL-STD-1787D)** | Military aerospace HUD @ 60 FPS — pitch ladder, FPM, compass, CAS/ALT tapes |
| 42 | **PPI Radar** | 360° sweep with active RF mesh link visualization |
| 43 | **FLIR Thermal & NVG Modes** | Simulated thermal infrared and night-vision sensor overlays |
| 44 | **Multi-Hop Link Tubes** | Glowing 3D links color-coded by SNR/hop count at 30 Hz |
| 45 | **Catmull-Rom Packet Pulses** | Animated photon packets traveling relay splines to GCS |
| 46 | **Chart.js Analytics Drawer** | EKF convergence, PDR, SNR vs distance, battery depletion curves |
| 47 | **2-Row Anti-Overflow Toolbar** | All HUD buttons always fully visible at any screen resolution |
| 48 | **Investor Presentation Mode** | 60s choreographed cinematic tour with KPI callout banners |
| 49 | **Demonstration Video Generator** | Headless automated mission video recording (`scripts/generate_demonstration_video.py`) |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SIMULATION CORE (sim/)                        │
│                                                                       │
│  ┌──────────────┐  ┌───────────────────┐  ┌──────────────────────┐  │
│  │ 6-DOF Drone  │  │ Disaster Environment│  │  Mission FSM & CBBA  │  │
│  │ LERP/SLERP   │  │ Expanded City Arena │  │  VSM + Retreat (RTL) │  │
│  │ APF Flocking │  │ 3D Launch Pads      │  │  Battery Priority    │  │
│  └──────────────┘  └───────────────────┘  └──────────────────────┘  │
│          │                  │                          │              │
│          └──────────────────┼──────────────────────────┘              │
│                             ▼                                         │
│              ┌─────────────────────────────┐                         │
│              │    FANET Comm & Routing      │                         │
│              │  Dual-band 2.4GHz + 915MHz   │                         │
│              │  3D Ray-AABB Occlusion       │                         │
│              │  Dynamic Link-State (DLS)    │                         │
│              │  DTN Store-and-Forward       │                         │
│              └─────────────────────────────┘                         │
│                             │                                         │
│              ┌──────────────┼──────────────┐                         │
│              ▼              ▼              ▼                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  LiDAR SLAM  │  │  9-State EKF │  │ MAVLink Bridge│              │
│  │  OctoMap 3D  │  │ IMU+GPS+Baro │  │  Real-Drone   │              │
│  │  Voxel Recon │  │  Fusion      │  │  SITL/HW Link │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ Serialized State Frames (30 Hz JSON)
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    VISUALIZATION ENGINE (vis/)                        │
│                                                                       │
│  ┌──────────────────────┐     ┌───────────────────────────────────┐  │
│  │ FastAPI / WebSocket  │────▶│  Three.js WebGL 3D Cockpit        │  │
│  │ vis/server.py        │     │  - Sector Delta 3D Diorama        │  │
│  │ 30 Hz broadcast      │     │  - Dual Viewport: Theater + SLAM  │  │
│  │ MAVLink relay        │     │  - Multi-hop RF link tubes        │  │
│  │ Survey progress API  │     │  - Chart.js analytics drawer      │  │
│  └──────────────────────┘     │  - 2-row anti-overflow toolbar    │  │
│                               └───────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                               ▲
┌─────────────────────────────────────────────────────────────────────┐
│              DESKTOP HUD LAYER (vis/desktop_hud.py)                   │
│  MIL-STD-1787D Military Aerospace HUD @ 60 FPS                       │
│  Pitch Ladder · FPM · Compass · CAS/ALT Tapes · PPI Radar · FLIR    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## ⚙️ Installation

### Prerequisites
- Python **3.9+**
- Git
- *(Recommended)* NVIDIA GPU for hardware-accelerated desktop HUD

### Steps

```bash
# 1. Clone the repository
git clone https://github.com/rogstrix-sys/TECHFEST.git
cd TECHFEST

# 2. (Optional) Create a virtual environment
python -m venv .venv
.venv\Scripts\activate         # Windows
# source .venv/bin/activate    # Linux/macOS

# 3. Install all dependencies
pip install -r requirements.txt
```

### Dependencies (`requirements.txt`)
```text
numpy>=1.24.0
fastapi>=0.100.0
uvicorn>=0.22.0
websockets>=11.0
pytest>=7.0.0
pandas>=2.0.0
```

---

## 🚀 Quick Start

### 1. Web Simulation (3D Cockpit in Browser)
```bash
python run_simulation.py
```
Opens automatically at **http://localhost:8000**

### 2. CLI Options
| Flag | Default | Description |
|------|---------|-------------|
| `--drones N` | 8 | Number of UAVs in the swarm |
| `--duration S` | 300 | Simulation duration in seconds |
| `--pois N` | 5 | Number of Points of Interest |
| `--port N` | 8000 | WebSocket server port |
| `--speed X` | 1.0 | Simulation speed multiplier |
| `--headless` | off | Run without browser (pure physics & networking) |

```bash
# Example: 16-drone, 8-PoI, 5-minute simulation
python run_simulation.py --drones 16 --pois 8 --duration 300

# Headless mode for automated testing
python run_simulation.py --headless --duration 60
```

---

## 🪟 Desktop HUD

> Hardware-accelerated native desktop application — no browser required.

```bash
# Default 1920×1080 desktop window (NVIDIA RTX 4050 GPU auto-enforced)
python run_hud.py

# Fullscreen Tactical Kiosk Mode
python run_hud.py --fullscreen

# OpenCV SVS fallback (no GPU needed)
python run_hud.py --opencv
```
Or double-click **`run_hud.bat`** on Windows.

### HUD Symbology (MIL-STD-1787D)
- Pitch ladder with positive/negative rungs & dynamic bank/roll rotation
- Flight Path Marker (FPM) velocity vector
- Top magnetic heading compass tape with target steering bug
- Left calibrated airspeed (CAS) tape + acceleration trend vector
- Right barometric altitude tape + Vertical Speed Indicator (VSI)
- 3D Target Acquisition Lock Boxes on PoIs and peer UAVs
- Corner PPI Radar — 360° sweep with active RF mesh links
- FLIR Thermal IR and NVG Night Vision sensor simulation modes
- **Hotkeys**: UAV selection, camera switching, sensor mode, pause/reset

---

## 📁 Project Structure

```
TECHFEST/                              (branch: main)
├── run_simulation.py                  Main entrypoint (web cockpit)
├── run_hud.py                         Native desktop HUD launcher
├── run_hud.bat                        Windows one-click launcher (NVIDIA GPU forced)
├── requirements.txt                   Python dependencies (NumPy, Pandas, FastAPI, etc.)
├── GEMINI.md                          Project development guidelines
│
├── sim/                               Simulation core (15 modules)
│   ├── core.py                        Main simulation loop & orchestration
│   ├── drone.py                       6-DOF UAV kinematics + LERP/SLERP smoothing
│   ├── environment.py                 Disaster zone & urban environment
│   ├── obstacles.py                   3D AABB building obstacles & launch pads
│   ├── network.py                     FANET topology, link management & LoS caching
│   ├── mission.py                     Next-gen mission FSM, BFT CBBA, VSM, retreat logic
│   ├── mapping.py                     Collaborative 3D LiDAR, OctoMap voxel SLAM, PLY/LAS export
│   ├── physics.py                     Aerodynamic forces, wind turbulence & collision dynamics
│   ├── planning.py                    Spatial sector scanning & coverage path planning
│   ├── sensors.py                     EKF sensor fusion (IMU + GPS + Baro)
│   ├── swarm_sim.py                   High-level swarm simulation helpers
│   ├── mavlink_bridge.py              MAVLink protocol bridge (SITL / hardware)
│   ├── challenge.py                   1000m national challenge scenario runner
│   ├── weather.py                     Wind & atmospheric disturbances
│   └── types.py                       Shared data types, formations & priority states
│
├── vis/                               Visualization engine
│   ├── server.py                      FastAPI / WebSocket telemetry server & PLY/LAS export
│   ├── desktop_hud.py                 MIL-STD-1787D military HUD (1,155 lines)
│   └── static/
│       ├── index.html                 Cockpit shell (survey progress + MAVLink UI)
│       ├── js/
│       │   ├── cockpit.js             Full 3D WebGL cockpit engine (Zero-GC, 4,000+ lines)
│       │   └── sector_delta_diorama.js  Sector Delta 3D city diorama engine
│       ├── css/style.css              Glassmorphic HUD styles (2-row anti-overflow)
│       └── screenshots/               Diorama orbit, top, night, and challenge captures
│
├── tests/
│   ├── unit/                          30 unit test modules
│   │   ├── test_sector_scanning.py    Spatial sector scanning & coverage paths
│   │   ├── test_formations_and_recharge.py Tactical formations & GCS fast recharge
│   │   ├── test_tier1_features.py     Bayesian thermal heatmap & obstacle collapse
│   │   ├── test_performance.py        Delta telemetry & broadphase benchmark
│   │   ├── test_drone.py              6-DOF dynamics & LERP/SLERP motion
│   │   ├── test_network.py            FANET routing & LoS occlusion
│   │   ├── test_mission.py            FSM & CBBA consensus logic
│   │   ├── test_mapping.py            LiDAR SLAM & OctoMap voxels
│   │   ├── test_sensors_ekf.py        9-state EKF sensor fusion
│   │   ├── test_mavlink_bridge.py     MAVLink hardware & SITL bridge
│   │   ├── test_battery_comms_priority.py Energy & comms priority fallback
│   │   ├── test_retreat.py            Autonomous RTL retreat
│   │   ├── test_challenge.py          1000m national arena benchmark
│   │   ├── test_desktop_hud.py        MIL-STD-1787D desktop HUD
│   │   └── ...
│   └── e2e/                           End-to-End test suite (Tiers 1–4)
│       ├── test_tier1_features.py
│       ├── test_tier2_boundaries.py
│       ├── test_tier3_combinations.py
│       └── test_tier4_scenarios.py
│
└── scripts/                           Verification & utility scripts
    ├── generate_demonstration_video.py Headless automated demo video generator
    ├── verify_autonomous_retreat.py
    ├── verify_battery_comms_priority_ui.py
    ├── verify_header_anti_clipping.py
    ├── verify_theater_mode_and_hud_off.py
    ├── verify_next_gen_features_ui.py
    ├── verify_nvidia_gpu_cockpit.py
    ├── check_chrome_gpu.py
    └── ...
```

---

## 🛠️ Technology Stack

| Layer | Technology |
|-------|-----------|
| Simulation Engine | Python 3.9+, NumPy, SciPy, Pandas |
| Protocols | MAVLink (SITL + hardware), WebSocket JSON @ 30 Hz |
| Web Server | FastAPI, Uvicorn |
| 3D Visualization | Three.js r128 (WebGL), Catmull-Rom splines, Zero-GC TypedArray pooling |
| 3D Point Cloud Export | PLY, LAS industry-standard point cloud formats |
| Analytics | Chart.js 4.4 |
| Desktop HUD | Python, OpenCV / CEF, Direct3D11 / ANGLE |
| GPU Acceleration | NVIDIA CUDA, DirectX UserGpuPreferences enforcement |
| Testing | pytest, custom E2E Tier framework (345/345 passing tests) |

---

## 🔧 Simulation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Arena | Expanded urban zone | City-scale disaster environment (up to 1000m) |
| Fleet Size | Up to 16 UAVs | 4 Relays + 8 Surveyors + 4 Pathfinders |
| RF Bands | 2.4 GHz + 915 MHz LoRa | Dual-band FANET mesh |
| Update Rate | 30 Hz | Simulation & WebSocket tick |
| HUD Frame Rate | 60 FPS | Desktop HUD (GPU accelerated) |
| Motion Smoothing | LERP + SLERP | Position lerp + quaternion slerp |
| Altitude Layers | 4 tiers | Launch / Survey / Transit / Relay |
| DTN Buffer | 250 packets | FIFO store-and-forward ring |
| FSPL Reference | 40.05 dB | Friis free-space path loss @ 1m |
| LoS Path Loss exp. | 2.05 | Log-distance (line of sight) |
| NLoS Path Loss exp. | 3.60 | Log-distance (obstructed) |
| Building Penetration | +22 dB | Forces multi-hop routing |

---

## 🧪 Test Suite

The simulation is validated by an extensive test suite comprising **345 automated tests** (unit and opaque-box E2E) with a **100% pass rate**.

```bash
# Run all unit tests and E2E suites
pytest -v

# Run spatial sector scanning tests
pytest tests/unit/test_sector_scanning.py -v

# Run tactical formations and battery recharge tests
pytest tests/unit/test_formations_and_recharge.py -v

# Run performance and bandwidth delta benchmark suite
pytest tests/unit/test_performance.py -v

# Run full E2E suite (Tiers 1–4)
pytest tests/e2e/ -v

# Run specific feature tests
pytest tests/unit/test_mavlink_bridge.py -v
pytest tests/unit/test_battery_comms_priority.py -v
pytest tests/unit/test_retreat.py -v
pytest tests/unit/test_challenge.py -v
```

---

## 📦 Releases & Changelog

| Release / Merge | Highlights |
|-----------------|------------|
| **v2.5.0 (Latest)** | **Spatial Sector Scanning & Realistic LoS**: Autonomous sector allocation, multi-drone sweeping coverage path planning, 2D AABB broadphase steering culling & LoS raycast caching (2.1x speedup), Zero-GC WebGL optimizations. |
| **v2.4.0** | **Tier-1 Features & Bandwidth Optimization**: Bayesian survivor thermal probability heatmap, dynamic obstacle collapse with tangential bypass, Byzantine-Fault-Tolerant CBBA with MAD outlier rejection, delta telemetry encoding (-81% bandwidth). |
| **v2.3.0** | **Collaborative LiDAR SLAM & Point Cloud Export**: Multi-drone point cloud fusion, instanced OctoMap voxels, elevation/intensity/thermal colormaps, direct PLY/LAS point cloud download, and automated demo video generator. |
| **v2.2.0** | **Tactical Formations & Continuous Recharging**: Dynamic swarm formations (V-Formation, Diamond, Echelon, Line, Circle), automated continuous GCS battery fast recharging dock, smoke/fire particle physics, and 3D flight trajectory ribbons. |
| **v2.1.0** | **Sector Delta 3D Diorama & MAVLink**: Sector Delta 3D City Diorama, LERP/SLERP motion smoothing, MAVLink hardware/SITL bridge, 1000m national challenge mode, NVIDIA GPU DirectX enforcement. |
| **v2.0.0** | **Desktop HUD (MIL-STD-1787D)**: Hardware-accelerated native desktop HUD, `run_hud.py` launcher, Chart.js analytics, dual-viewport split screen. |
| **v1.0.0** | Initial release — core 6-DOF dynamics, FANET routing, Three.js cockpit, and E2E test framework. |

---

<div align="center">

**Built for IIT Bombay TechFest** | Aashutosh Kedia

</div>

