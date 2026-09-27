<div align="center">

# 🚁 UAV-X Swarm Simulation
### 3D Resilient Multi-Hop Aerial Communication Network & SLAM Autonomy Cockpit

[![Version](https://img.shields.io/badge/version-2.1.0-blue.svg)](https://github.com/rogstrix-sys/TECHFEST/releases)
[![Branch](https://img.shields.io/badge/branch-main%20%28stable%29-brightgreen.svg)](https://github.com/rogstrix-sys/TECHFEST/tree/main)
[![Python](https://img.shields.io/badge/Python-3.9%2B-green.svg)](https://python.org)
[![Three.js](https://img.shields.io/badge/Three.js-r128-blue.svg)](#)
[![IIT Bombay](https://img.shields.io/badge/IIT%20Bombay-TechFest-red.svg)](#)
[![GPU](https://img.shields.io/badge/GPU-NVIDIA%20RTX%204050-76b900.svg)](#)

> A production-grade, physics-based simulation of an autonomous UAV swarm conducting post-disaster aerial survey, multi-hop FANET communication relay, and real-time 3D SLAM mapping — rendered in a military-grade WebGL cockpit with native desktop HUD.

**Branch `main`** — Stable release branch. Latest merged version includes autonomous retreat, battery-comms priority, UI overflow fix, and all features up to the merged `aashutosh` PR.

</div>

---

## 📋 Table of Contents
- [Overview](#-overview)
- [Features](#-features)
- [Architecture](#-architecture)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Desktop HUD](#-desktop-hud)
- [Project Structure](#-project-structure)
- [Technology Stack](#-technology-stack)
- [Simulation Parameters](#-simulation-parameters)
- [Test Suite](#-test-suite)
- [Releases](#-releases)

---

## 🌐 Overview

**UAV-X Swarm Simulation** is a decoupled, high-performance simulation framework for a **Flying Ad-Hoc Network (FANET)** formed by a fleet of up to **16 autonomous UAVs** operating in a post-disaster zone. The swarm autonomously:

- 🗺️ **Surveys** multi-priority Points of Interest — Survivor Search, Structural Collapse, Hazard Zones
- 📡 **Maintains resilient multi-hop communication** back to a Ground Control Station (GCS)
- 🔄 **Self-heals** routing paths with DTN Store-and-Forward buffering during link interruptions
- 🧠 **Builds a 3D map** of the disaster zone using simulated LiDAR + OctoMap voxel SLAM
- 🛬 **Autonomously retreats** the full fleet on low-battery or command (Return-to-Launch)
- 📺 **Renders everything** in a real-time 3D WebGL cockpit with military HUD and live analytics

The project targets **IIT Bombay TechFest** and demonstrates production-grade autonomous swarm robotics.

---

## ✨ Features

### Core Simulation Engine
| # | Feature | Details |
|---|---------|---------|
| 1 | **6-DOF Quadcopter Dynamics** | Newton-Euler rigid body, position/velocity integration, quaternion attitude |
| 2 | **Flocking & Collision Avoidance** | Khatib Artificial Potential Fields + Reynolds Boids + downwash repulsion |
| 3 | **4-Tier Altitude Corridors** | Launch [0–20m] → Survey [25–45m] → Transit [50–65m] → Relay [70–90m] |
| 4 | **Battery & Energy Model** | Electro-mechanical power draw; Return-to-Launch trigger on low battery |
| 5 | **Battery-Comms Priority System** | Battery-aware dynamic role reassignment; comms degradation priority fallback |
| 6 | **500m × 500m Disaster Environment** | 3D AABB building obstacles, GCS base, multi-priority PoI beacons |
| 7 | **Weather & Atmospheric Disturbances** | Wind gusts and turbulence affecting drone dynamics |

### FANET Communication & Routing
| # | Feature | Details |
|---|---------|---------|
| 8 | **RF Propagation (2.4 GHz + 915 MHz)** | Friis FSPL (PL₀ = 40.05 dB) + Log-distance (η_LoS=2.05, η_NLoS=3.60) |
| 9 | **3D Ray-AABB Occlusion Engine** | Ray-slab intersection test; +22 dB building penetration forces multi-hop |
| 10 | **Dynamic Link-State Routing** | Sub-ms Dijkstra shortest-path; composite cost (SNR, distance, obstacle, battery) |
| 11 | **DTN Store-and-Forward** | 250-packet FIFO ring buffer for transient link interruptions |
| 12 | **Network Telemetry** | Real-time PDR, end-to-end latency, hop-count distribution |

### Mission Control & Autonomy
| # | Feature | Details |
|---|---------|---------|
| 13 | **10-State MAVSDK-Compliant FSM** | IDLE → TAKEOFF → TRANSIT → SURVEYING → RELAY → DATA_TX → RTL → LANDING |
| 14 | **CBBA Consensus Role Allocation** | Dynamic assignment of Surveyor / Relay / Pathfinder UAV roles |
| 15 | **Virtual Spring Mesh (VSM)** | Relay UAVs self-position via spring-damper forces between GCS and Surveyors |
| 16 | **Autonomous Retreat (RTL)** | Full-fleet return-to-launch on command or battery threshold |
| 17 | **Adaptive Task Scheduling** | Multi-priority mission queues; battery-aware reassignment |

### SLAM & Perception
| # | Feature | Details |
|---|---------|---------|
| 18 | **3D LiDAR Simulation** | Multi-beam rotating LiDAR with configurable FOV and range noise |
| 19 | **OctoMap Voxel SLAM** | Log-odds 3D occupancy grid; real-time voxel reconstruction |
| 20 | **9-State EKF Localization** | Extended Kalman Filter fusing IMU + GPS + barometer |

### Visualization & HUD
| # | Feature | Details |
|---|---------|---------|
| 21 | **Three.js 3D WebGL Cockpit** | Dual-viewport split screen: External theater + SLAM perception |
| 22 | **Native Desktop HUD (MIL-STD-1787D)** | Military aerospace HUD @ 60 FPS — pitch ladder, FPM, compass, CAS/ALT tapes |
| 23 | **PPI Radar** | 360° sweep with active RF mesh link visualization |
| 24 | **FLIR Thermal & NVG Modes** | Simulated thermal infrared and night-vision sensor overlays |
| 25 | **Multi-Hop Link Tubes** | Glowing 3D links color-coded by SNR/hop count at 30 Hz |
| 26 | **Catmull-Rom Packet Pulses** | Animated photon packets traveling relay splines to GCS |
| 27 | **Chart.js Analytics Drawer** | EKF convergence, PDR, SNR vs distance, battery depletion curves |
| 28 | **2-Row Anti-Overflow Toolbar** | All HUD buttons always fully visible at any screen resolution |
| 29 | **Battery/Comms Priority Indicators** | Live priority status overlays on drone cards |
| 30 | **Investor Presentation Mode** | 60s choreographed cinematic tour with KPI callout banners |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SIMULATION CORE (sim/)                        │
│                                                                       │
│  ┌──────────────┐  ┌───────────────────┐  ┌──────────────────────┐  │
│  │ 6-DOF Drone  │  │Disaster Environment│  │  Mission FSM & CBBA  │  │
│  │ APF Flocking │  │500×500m AABB obs.  │  │  VSM + Retreat (RTL) │  │
│  │ Battery Model│  │Multi-priority PoIs │  │  Battery Priority    │  │
│  └──────────────┘  └───────────────────┘  └──────────────────────┘  │
│                             │                                         │
│                             ▼                                         │
│              ┌─────────────────────────────┐                         │
│              │    FANET Comm & Routing      │                         │
│              │  2.4 GHz + 915 MHz LoRa      │                         │
│              │  3D Ray-AABB Occlusion       │                         │
│              │  Dynamic Link-State (DLS)    │                         │
│              │  DTN Store-and-Forward       │                         │
│              └─────────────────────────────┘                         │
│                             │                                         │
│              ┌──────────────┴──────────────┐                         │
│              ▼                             ▼                         │
│  ┌─────────────────────┐      ┌──────────────────────┐              │
│  │  LiDAR + OctoMap    │      │   9-State EKF Fusion  │              │
│  │  3D Voxel SLAM      │      │   IMU + GPS + Baro    │              │
│  └─────────────────────┘      └──────────────────────┘              │
└──────────────────────────────┬──────────────────────────────────────┘
                               │ Serialized State Frames @ 30 Hz
                               ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    VISUALIZATION ENGINE (vis/)                        │
│                                                                       │
│  ┌──────────────────────┐     ┌───────────────────────────────────┐  │
│  │ FastAPI / WebSocket  │────▶│  Three.js WebGL 3D Cockpit        │  │
│  │ vis/server.py        │     │  Dual Viewport: Theater + SLAM    │  │
│  │ 30 Hz broadcast      │     │  Multi-hop RF link tubes          │  │
│  └──────────────────────┘     │  Chart.js analytics drawer        │  │
│                               │  2-row anti-overflow HUD toolbar  │  │
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
```
numpy
scipy
fastapi
uvicorn[standard]
websockets
pytest
```

---

## 🚀 Quick Start

### 1. Launch the Web Simulation (3D Cockpit in Browser)
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
| `--headless` | off | Run without browser (pure simulation) |

```bash
# Example: 16-drone, 8-PoI, 5-minute simulation
python run_simulation.py --drones 16 --pois 8 --duration 300

# Headless (CI/testing)
python run_simulation.py --headless --duration 60
```

---

## 🪟 Desktop HUD

> Hardware-accelerated native desktop application — no browser required.

```bash
# Default 1920×1080 desktop window (NVIDIA RTX 4050 GPU auto-forced)
python run_hud.py

# Fullscreen Tactical Kiosk Mode
python run_hud.py --fullscreen

# OpenCV SVS fallback (no GPU needed)
python run_hud.py --opencv
```
Or double-click **`run_hud.bat`** on Windows.

### HUD Features (MIL-STD-1787D)
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
TECHFEST/                              (branch: main — stable)
├── run_simulation.py                  Main entrypoint (web cockpit)
├── run_hud.py                         Native desktop HUD launcher
├── run_hud.bat                        Windows one-click launcher
├── requirements.txt
│
├── sim/                               Simulation core (13 modules)
│   ├── core.py                        Main simulation loop & orchestration
│   ├── drone.py                       6-DOF UAV kinematics & dynamics
│   ├── environment.py                 Disaster zone environment
│   ├── obstacles.py                   3D AABB building obstacles
│   ├── network.py                     FANET topology & link management
│   ├── mission.py                     Mission FSM, CBBA, VSM, retreat logic
│   ├── mapping.py                     3D LiDAR / OctoMap voxel SLAM
│   ├── sensors.py                     EKF sensor fusion (IMU + GPS + Baro)
│   ├── planning.py                    Path planning & task scheduling
│   ├── challenge.py                   Adversarial scenario challenges
│   ├── weather.py                     Wind & atmospheric disturbances
│   └── types.py                       Shared data types & battery/comms states
│
├── vis/                               Visualization engine
│   ├── server.py                      FastAPI / WebSocket telemetry server
│   ├── desktop_hud.py                 MIL-STD-1787D military HUD (1,155 lines)
│   └── static/
│       ├── index.html                 Three.js cockpit shell
│       ├── js/cockpit.js              Full 3D WebGL cockpit engine
│       └── css/style.css              Glassmorphic HUD styles
│
├── tests/
│   ├── unit/                          Unit tests (22 modules)
│   │   ├── test_drone.py
│   │   ├── test_network.py
│   │   ├── test_mission.py
│   │   ├── test_mapping.py
│   │   ├── test_sensors_ekf.py
│   │   ├── test_battery_comms_priority.py
│   │   ├── test_retreat.py
│   │   ├── test_desktop_hud.py
│   │   └── ...
│   └── e2e/                           End-to-End test suite (Tiers 1–4)
│       ├── test_tier1_features.py
│       ├── test_tier2_boundaries.py
│       ├── test_tier3_combinations.py
│       └── test_tier4_scenarios.py
│
└── scripts/                           Verification & utility scripts
    ├── verify_autonomous_retreat.py
    ├── verify_battery_comms_priority_ui.py
    ├── verify_header_anti_clipping.py
    ├── verify_theater_mode_and_hud_off.py
    ├── verify_nvidia_gpu_cockpit.py
    ├── check_chrome_gpu.py
    └── verify_browser.py
```

---

## 🛠️ Technology Stack

| Layer | Technology |
|-------|-----------|
| Simulation Engine | Python 3.9+, NumPy, SciPy |
| Web Server | FastAPI, Uvicorn, WebSockets |
| 3D Visualization | Three.js r128 (WebGL), Catmull-Rom splines |
| Analytics | Chart.js 4.4 |
| Desktop HUD | Python, OpenCV / CEF, Direct3D11 / ANGLE |
| GPU Acceleration | NVIDIA CUDA, DirectX UserGpuPreferences enforcement |
| Testing | pytest, custom E2E Tier framework |
| Telemetry Protocol | WebSocket JSON @ 30 Hz |

---

## 🔧 Simulation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Arena | 500 × 500 m | Disaster zone dimensions |
| Fleet Size | 4–16 UAVs | Configurable via `--drones` |
| RF Bands | 2.4 GHz + 915 MHz LoRa | Dual-band FANET mesh |
| Update Rate | 30 Hz | Simulation & WebSocket tick |
| HUD Frame Rate | 60 FPS | Desktop HUD (GPU accelerated) |
| Altitude Layers | 4 tiers | Launch / Survey / Transit / Relay |
| DTN Buffer | 250 packets | FIFO store-and-forward ring |
| FSPL Reference | 40.05 dB | Friis free-space path loss @ 1m |
| LoS Path Loss exp. | 2.05 | Log-distance (line of sight) |
| NLoS Path Loss exp. | 3.60 | Log-distance (obstructed) |
| Building Penetration | +22 dB | Forces multi-hop routing |

---

## 🧪 Test Suite

```bash
# Run all unit tests
pytest tests/unit/ -v

# Run full E2E suite (Tiers 1–4)
pytest tests/e2e/ -v

# Run specific modules
pytest tests/unit/test_battery_comms_priority.py -v
pytest tests/unit/test_retreat.py -v
```

**Coverage:**
- ✅ **Tier 1** — Feature verification (all 30 core features)
- ✅ **Tier 2** — Boundary conditions & edge cases
- ✅ **Tier 3** — Feature interaction & combinations
- ✅ **Tier 4** — Full mission scenarios (multi-UAV, obstacle-dense, battery-critical)
- ✅ **Tier 5** — Adversarial stress & chaos testing
- ✅ **Battery/Comms Priority** — Priority system unit tests
- ✅ **Retreat** — Autonomous RTL logic tests

---

## 📦 Releases

| Version | Date | Highlights |
|---------|------|-----------|
| **v2.1.0** | 2026-09-26 | NVIDIA GPU enforcement via Windows registry; GPU verification scripts |
| **v2.0.0** | 2026-09-26 | Desktop HUD (MIL-STD-1787D), `run_hud.py` launcher, Chart.js analytics, dual-viewport |
| **v1.0.0** | 2026-09-26 | Initial release — core 6-DOF dynamics, FANET, Three.js cockpit, E2E test suite |
| **main (HEAD)** | 2026-09-27 | Merged: autonomous retreat, battery-comms priority, 2-row toolbar fix |

> **Active development** happens on [`aashutosh`](https://github.com/rogstrix-sys/TECHFEST/tree/aashutosh) and is periodically merged here.

---

<div align="center">

**Built for IIT Bombay TechFest** | Aashutosh Kedia

[📦 Releases](https://github.com/rogstrix-sys/TECHFEST/releases) · [🌿 Dev Branch](https://github.com/rogstrix-sys/TECHFEST/tree/aashutosh) · [📋 Compare](https://github.com/rogstrix-sys/TECHFEST/compare/main...aashutosh)

</div>
