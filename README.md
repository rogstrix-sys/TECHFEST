<div align="center">

# 🚁 UAV-X Swarm Simulation
### 3D Resilient Multi-Hop Aerial Communication Network & SLAM Autonomy Cockpit

[![Branch](https://img.shields.io/badge/branch-aashutosh-blueviolet.svg)](https://github.com/rogstrix-sys/TECHFEST/tree/aashutosh)
[![Python](https://img.shields.io/badge/Python-3.9%2B-green.svg)](https://python.org)
[![Three.js](https://img.shields.io/badge/Three.js-r128-blue.svg)](#)
[![IIT Bombay](https://img.shields.io/badge/IIT%20Bombay-TechFest-red.svg)](#)
[![GPU](https://img.shields.io/badge/GPU-NVIDIA%20RTX%204050-76b900.svg)](#)

> A production-grade, physics-based simulation of an autonomous UAV swarm conducting post-disaster aerial survey, multi-hop FANET communication relay, and real-time 3D SLAM mapping — rendered in a military-grade WebGL cockpit with native desktop HUD.

**Branch `aashutosh`** — Active development branch. Includes all features from `main` plus autonomous retreat, MAVLink bridge, battery-comms priority, next-gen mission system, Sector Delta 3D diorama, and LERP/SLERP motion smoothing.

</div>

---

## 📋 Table of Contents
- [Overview](#-overview)
- [What's New on This Branch](#-whats-new-on-this-branch)
- [Full Feature Set](#-full-feature-set)
- [Architecture](#-architecture)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Desktop HUD](#-desktop-hud)
- [Project Structure](#-project-structure)
- [Technology Stack](#-technology-stack)
- [Simulation Parameters](#-simulation-parameters)
- [Test Suite](#-test-suite)
- [Changelog](#-changelog)

---

## 🌐 Overview

**UAV-X Swarm Simulation** is a decoupled, high-performance simulation framework for a **Flying Ad-Hoc Network (FANET)** formed by a fleet of up to **16 autonomous UAVs** operating in a post-disaster urban zone. The swarm autonomously:

- 🗺️ **Surveys** multi-priority Points of Interest — Survivor Search, Structural Collapse, Hazard Zones
- 📡 **Maintains resilient multi-hop communication** back to a Ground Control Station (GCS) via Dynamic Link-State routing
- 🔄 **Self-heals** routing paths with DTN Store-and-Forward buffering during link interruptions
- 🧠 **Builds a 3D map** of the disaster zone using simulated LiDAR + OctoMap voxel SLAM
- 🛬 **Autonomously retreats** the full fleet on low-battery or command with Return-to-Launch logic
- 📺 **Renders everything** in a real-time WebGL cockpit with military HUD and live analytics

---

## 🆕 What's New on This Branch

> All changes on `aashutosh` beyond the `main` baseline (v2.1.0).

### Latest Commits
| # | Commit | Key Changes |
|---|--------|-------------|
| 1 | `feat: expand city + 3D launch pads` | Expanded urban area, embedded disaster sites in city fabric, 3D launch pad geometry |
| 2 | `perf: LERP/SLERP + render pipeline` | Eliminated drone motion stutter; full LERP position + SLERP quaternion interpolation; render loop optimized |
| 3 | `feat: Sector Delta 3D diorama` | Merged swarm autonomy with Sector Delta city diorama; `sector_delta_diorama.js`; screenshot assets |
| 4 | `feat: MAVLink bridge + next-gen mission` | `sim/mavlink_bridge.py`; SLAM upgrade; full UI overhaul (+2,991 lines) |
| 5 | `feat: battery-comms priority` | Battery-aware role reassignment; comms degradation handling |
| 6 | `fix: header 2-row layout` | All toolbar buttons always fully visible — no overflow at any resolution |
| 7 | `branch/aashutosh: autonomous retreat` | `sim/mission.py` major overhaul; autonomous RTL logic |

---

## ✨ Full Feature Set

### Simulation Engine
| # | Feature | Details |
|---|---------|---------|
| 1 | **6-DOF Quadcopter Dynamics** | Newton-Euler rigid body, position/velocity integration, quaternion attitude |
| 2 | **LERP/SLERP Motion Smoothing** | Zero-stutter drone motion via linear position lerp + spherical quaternion interpolation |
| 3 | **Flocking & Collision Avoidance** | Khatib Artificial Potential Fields + Reynolds Boids + downwash repulsion |
| 4 | **4-Tier Altitude Corridors** | Launch [0–20m] → Survey [25–45m] → Transit [50–65m] → Relay [70–90m] |
| 5 | **Battery & Energy Model** | Electro-mechanical power draw; Return-to-Launch (RTL) on low-battery |
| 6 | **Battery-Comms Priority System** | Battery-aware dynamic role reassignment; comms degradation priority fallback |
| 7 | **Expanded Urban Disaster Zone** | Large-scale city environment; disaster sites embedded in urban fabric; 3D launch pads |
| 8 | **Weather & Atmospheric Disturbances** | Wind gusts, turbulence affecting drone dynamics |

### FANET Communication & Routing
| # | Feature | Details |
|---|---------|---------|
| 9 | **RF Propagation (2.4 GHz + 915 MHz)** | Friis FSPL (PL₀ = 40.05 dB) + Log-distance (η_LoS=2.05, η_NLoS=3.60); dual-band |
| 10 | **3D Ray-AABB Occlusion Engine** | Ray-slab intersection; +22 dB building penetration forces multi-hop |
| 11 | **Dynamic Link-State Routing (FANET-DLS)** | Sub-ms Dijkstra; composite cost (SNR, distance, obstacle penalty, battery weight) |
| 12 | **DTN Store-and-Forward** | 250-packet FIFO ring buffer for transient link interruptions |
| 13 | **Network Telemetry** | Real-time PDR, end-to-end latency, hop-count distribution |

### Mission Control & Autonomy
| # | Feature | Details |
|---|---------|---------|
| 14 | **10-State MAVSDK-Compliant FSM** | IDLE → TAKEOFF → TRANSIT → SURVEYING → RELAY → DATA_TX → RTL → LANDING |
| 15 | **MAVLink Bridge** | `sim/mavlink_bridge.py` — real-drone MAVLink protocol relay (SITL/hardware) |
| 16 | **CBBA Consensus Role Allocation** | Dynamic assignment of Surveyor / High-Relay / Pathfinder roles |
| 17 | **Virtual Spring Mesh (VSM)** | Relay UAVs self-position via spring-damper forces between GCS and Surveyors |
| 18 | **Autonomous Retreat (RTL)** | Full-fleet autonomous return on command or low-battery threshold |
| 19 | **Survey Progress Tracking** | Per-PoI survey completion percentage, dwell-time tracking, data-volume counters |
| 20 | **Next-Gen Task Scheduling** | Multi-priority adaptive task queue; battery-aware role reassignment |

### SLAM & Perception
| # | Feature | Details |
|---|---------|---------|
| 21 | **3D LiDAR Simulation** | Multi-beam rotating LiDAR scanner with configurable FOV and range noise |
| 22 | **OctoMap Voxel SLAM** | Log-odds 3D occupancy grid; real-time voxel reconstruction from LiDAR sweeps |
| 23 | **9-State EKF Localization** | Extended Kalman Filter fusing IMU + GPS + barometer for state estimation |
| 24 | **Volumetric Coverage Metrics** | Occupied voxels, surveyed volume (m³), mapping density |

### Visualization & HUD
| # | Feature | Details |
|---|---------|---------|
| 25 | **Sector Delta 3D Diorama** | High-fidelity city environment merged with swarm autonomy (`sector_delta_diorama.js`) |
| 26 | **Three.js 3D WebGL Cockpit** | Dual-viewport split screen: External theater + Autonomous SLAM perception (3,878-line JS) |
| 27 | **Native Desktop HUD (MIL-STD-1787D)** | Military aerospace HUD @ 60 FPS — pitch ladder, FPM, compass, airspeed/alt tapes |
| 28 | **PPI Radar** | 360° sweep with active RF mesh link visualization |
| 29 | **FLIR Thermal & NVG Modes** | Simulated thermal infrared and night-vision sensor overlay modes |
| 30 | **Multi-Hop Link Tubes** | Glowing 3D links color-coded by SNR/hop count at 30 Hz |
| 31 | **Catmull-Rom Packet Pulses** | Animated photon packets traveling relay splines to GCS |
| 32 | **Chart.js Analytics Drawer** | EKF convergence, PDR & throughput, SNR vs distance, battery depletion curves |
| 33 | **Glassmorphic HUD (2-row, anti-overflow)** | All toolbar buttons always fully visible; brand + metrics row + controls row |
| 34 | **Survey Progress Panel** | Live per-PoI survey completion bars and data-volume indicators |
| 35 | **MAVLink Status Overlay** | Live MAVLink bridge connection status in cockpit |
| 36 | **Investor Presentation Mode** | 60s choreographed cinematic tour with KPI callout banners |
| 37 | **Screenshot Capture Assets** | `vis/static/` orbit/top/night views: `sector_delta_merged_*.png` |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        SIMULATION CORE (sim/)                        │
│                                                                       │
│  ┌──────────────┐  ┌───────────────────┐  ┌──────────────────────┐  │
│  │ 6-DOF Drone  │  │ Disaster Environment│  │  Next-Gen Mission    │  │
│  │ LERP/SLERP   │  │ Expanded city zone  │  │  FSM + CBBA + VSM   │  │
│  │ APF Flocking │  │ 3D launch pads      │  │  Autonomous Retreat  │  │
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
│              │  Battery-Comms Priority      │                         │
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
│  │ FastAPI/WebSocket     │────▶│  Three.js WebGL Cockpit           │  │
│  │ vis/server.py         │     │  - Sector Delta 3D Diorama        │  │
│  │ 30 Hz telemetry      │     │  - Dual Viewport Split Screen     │  │
│  │ MAVLink relay        │     │  - Multi-hop RF link tubes        │  │
│  │ Survey progress API  │     │  - Chart.js analytics drawer      │  │
│  └──────────────────────┘     │  - Survey progress panels         │  │
│                               │  - 2-row anti-overflow toolbar    │  │
│                               └───────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
                               ▲
┌─────────────────────────────────────────────────────────────────────┐
│              DESKTOP HUD LAYER (vis/desktop_hud.py)                   │
│  MIL-STD-1787D Aerospace HUD @ 60 FPS                                │
│  Pitch Ladder · FPM · Compass · CAS/ALT Tapes · PPI Radar · FLIR    │
└─────────────────────────────────────────────────────────────────────┘
```

---

## ⚙️ Installation

### Prerequisites
- Python **3.9+**
- Git
- *(Recommended)* NVIDIA GPU (RTX series) for hardware-accelerated desktop HUD

### Steps

```bash
# 1. Clone the repository (this branch)
git clone -b aashutosh https://github.com/rogstrix-sys/TECHFEST.git
cd TECHFEST

# 2. (Optional) Create a virtual environment
python -m venv .venv
.venv\Scripts\activate    # Windows
# source .venv/bin/activate  # Linux/macOS

# 3. Install dependencies
pip install -r requirements.txt
```

### `requirements.txt`
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

### Web Simulation (3D Cockpit in Browser)
```bash
python run_simulation.py
```
Opens automatically at **http://localhost:8000**

### CLI Options
```bash
# Custom fleet and duration
python run_simulation.py --drones 16 --duration 300 --pois 8

# Headless mode (no browser, pure physics + networking)
python run_simulation.py --headless

# Speed multiplier
python run_simulation.py --speed 2.0

# Custom port
python run_simulation.py --port 9000
```

---

## 🪟 Desktop HUD

```bash
# Default 1920×1080 desktop application (no browser needed)
python run_hud.py

# Fullscreen Tactical Kiosk Mode
python run_hud.py --fullscreen

# OpenCV Synthetic Vision System fallback
python run_hud.py --opencv
```
Or double-click **`run_hud.bat`** on Windows (auto-enforces NVIDIA RTX 4050 GPU).

### HUD Capabilities
- **MIL-STD-1787D** collimated aerospace symbology
- Pitch ladder, Flight Path Marker (FPM), boresight crosshairs, roll scale
- Top magnetic heading compass tape with target steering bug
- Left calibrated airspeed (CAS) tape + acceleration trend vector
- Right barometric altitude tape + Vertical Speed Indicator (VSI)
- 3D Target Acquisition Lock Boxes on PoIs and peer UAVs
- Corner PPI Radar — 360° sweep with active RF mesh links
- FLIR Thermal IR and NVG Night Vision sensor modes
- **Hotkeys**: UAV select, camera switch, sensor mode toggle, pause/reset

---

## 📁 Project Structure

```
TECHFEST/                              (branch: aashutosh)
├── run_simulation.py                  Main entrypoint (web cockpit)
├── run_hud.py                         Native desktop HUD launcher
├── run_hud.bat                        Windows one-click launcher (NVIDIA GPU forced)
├── requirements.txt
├── GEMINI.md                          Project modification guidelines
│
├── sim/                               Simulation core (13 modules)
│   ├── core.py                        Main simulation loop & orchestration
│   ├── drone.py                       6-DOF UAV kinematics + LERP/SLERP smoothing
│   ├── environment.py                 Disaster zone + expanded urban environment
│   ├── obstacles.py                   3D AABB building obstacles + launch pads
│   ├── network.py                     FANET topology, link management, priority weights
│   ├── mission.py                     Next-gen mission FSM, CBBA, VSM, retreat logic (922 lines)
│   ├── mapping.py                     3D LiDAR / OctoMap voxel SLAM (502 lines)
│   ├── sensors.py                     EKF sensor fusion (IMU + GPS + Baro)
│   ├── planning.py                    Path planning & task scheduling
│   ├── mavlink_bridge.py              MAVLink protocol bridge (251 lines) ← NEW
│   ├── challenge.py                   Adversarial scenario challenges
│   ├── weather.py                     Wind & atmospheric disturbances
│   └── types.py                       Shared data types, battery/comms priority states
│
├── vis/                               Visualization engine
│   ├── server.py                      FastAPI / WebSocket server (survey progress, MAVLink relay)
│   ├── desktop_hud.py                 MIL-STD-1787D military HUD (1,155 lines)
│   └── static/
│       ├── index.html                 Cockpit shell (survey progress + MAVLink UI)
│       ├── js/
│       │   ├── cockpit.js             Full WebGL cockpit engine (3,878 lines)
│       │   └── sector_delta_diorama.js  Sector Delta 3D city diorama ← NEW
│       ├── css/style.css              Glassmorphic HUD styles + priority styling
│       └── (screenshot assets)        sector_delta_merged_*.png, green_land_*.png
│
├── tests/
│   ├── conftest.py
│   ├── unit/                          24 unit test modules
│   │   ├── test_drone.py
│   │   ├── test_network.py
│   │   ├── test_mission.py
│   │   ├── test_mapping.py
│   │   ├── test_sensors_ekf.py
│   │   ├── test_mavlink_bridge.py     ← NEW
│   │   ├── test_battery_comms_priority.py ← NEW
│   │   ├── test_retreat.py            ← NEW
│   │   ├── test_five_upgrades.py      ← NEW
│   │   ├── test_next_gen_features.py  ← NEW
│   │   ├── test_desktop_hud.py
│   │   └── ...
│   └── e2e/                           E2E test suite (Tiers 1–4)
│       ├── test_tier1_features.py
│       ├── test_tier2_boundaries.py
│       ├── test_tier3_combinations.py
│       └── test_tier4_scenarios.py
│
└── scripts/                           15 utility & verification scripts
    ├── verify_autonomous_retreat.py
    ├── verify_battery_comms_priority_ui.py
    ├── verify_header_anti_clipping.py
    ├── verify_next_gen_features_ui.py
    ├── verify_nvidia_gpu_cockpit.py
    ├── check_chrome_gpu.py
    ├── test_survey_progress.py
    ├── capture_night.py               Screenshot capture (night mode)
    ├── capture_ss.py                  Screenshot capture (standard)
    └── ...
```

---

## 🛠️ Technology Stack

| Layer | Technology |
|-------|-----------|
| Simulation Engine | Python 3.9+, NumPy, SciPy |
| Communication Protocol | MAVLink (SITL + hardware via `mavlink_bridge.py`) |
| Web Server | FastAPI, Uvicorn, WebSockets |
| 3D Visualization | Three.js r128 (WebGL), Catmull-Rom splines |
| Analytics | Chart.js 4.4 |
| Desktop HUD | Python, OpenCV / CEF, Direct3D11 / ANGLE |
| GPU Acceleration | NVIDIA CUDA, DirectX UserGpuPreferences registry enforcement |
| Testing | pytest, custom E2E Tier framework |
| Telemetry Protocol | WebSocket JSON @ 30 Hz |

---

## 🔧 Simulation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Arena | Expanded urban zone | City-scale disaster environment |
| Fleet Size | Up to 16 UAVs | 4 Relays + 8 Surveyors + 4 Pathfinders |
| RF Bands | 2.4 GHz + 915 MHz LoRa | Dual-band FANET mesh |
| Update Rate | 30 Hz | Simulation & WebSocket telemetry tick |
| HUD Frame Rate | 60 FPS | Desktop HUD rendering (GPU accelerated) |
| Motion Smoothing | LERP + SLERP | Position lerp + quaternion slerp |
| Altitude Layers | 4 tiers | Launch / Survey / Transit / Relay |
| DTN Buffer | 250 packets | FIFO store-and-forward ring |
| FSPL Reference | 40.05 dB | Friis free-space path loss @ 1m |
| LoS Path Loss exp. | 2.05 | Log-distance (line of sight) |
| NLoS Path Loss exp. | 3.60 | Log-distance (obstructed) |
| Building Penetration | +22 dB | Forces multi-hop routing |
| LiDAR Beams | Multi-beam | Rotating LiDAR with range noise |

---

## 🧪 Test Suite

```bash
# Run all unit tests (24 modules)
pytest tests/unit/ -v

# Run full E2E suite (Tiers 1–4)
pytest tests/e2e/ -v

# Run specific new tests
pytest tests/unit/test_mavlink_bridge.py -v
pytest tests/unit/test_battery_comms_priority.py -v
pytest tests/unit/test_retreat.py -v
pytest tests/unit/test_next_gen_features.py -v
```

**Coverage:**
- ✅ **Tier 1** — Feature verification (all 37 features)
- ✅ **Tier 2** — Boundary conditions & edge cases
- ✅ **Tier 3** — Feature interaction & combinations
- ✅ **Tier 4** — Full mission scenarios (multi-UAV, obstacle-dense, battery-critical)
- ✅ **Tier 5** — Adversarial stress & chaos testing
- ✅ **MAVLink** — Protocol bridge unit tests
- ✅ **Battery/Comms Priority** — Priority system unit tests
- ✅ **Retreat** — Autonomous RTL logic tests

---

## 📈 Changelog

### `aashutosh` branch (beyond v2.1.0)
- ✨ **feat**: Expanded urban city zone with disaster sites embedded; 3D launch pads
- ✨ **feat**: `sector_delta_diorama.js` — Sector Delta 3D city diorama merged with swarm autonomy
- ✨ **feat**: `sim/mavlink_bridge.py` — MAVLink protocol bridge for real-drone SITL/hardware
- ✨ **feat**: Next-gen mission system — adaptive task scheduling, survey progress tracking, multi-priority queues
- ✨ **feat**: SLAM upgrade — expanded OctoMap voxel reconstruction, volumetric coverage metrics
- ✨ **feat**: Battery-comms priority system — battery-aware role reassignment, comms degradation handling
- ✨ **feat**: Autonomous fleet retreat — full-fleet RTL logic on command or low-battery
- 🔧 **perf**: LERP/SLERP motion smoothing — zero drone motion stutter
- 🔧 **perf**: Render pipeline optimization — improved Three.js frame efficiency
- 🔧 **fix**: 2-row header layout — all toolbar buttons always fully on-screen
- ✅ **tests**: 5 new unit test modules, 3 new verification scripts

### v2.1.0 — NVIDIA GPU Enforcement (main)
- NVIDIA RTX 4050 GPU forced via Windows DirectX registry + env vars
- `check_chrome_gpu.py`, `verify_nvidia_gpu_cockpit.py` scripts

### v2.0.0 — Desktop HUD & Expanded Cockpit (main)
- `vis/desktop_hud.py` — MIL-STD-1787D military HUD
- `run_hud.py` / `run_hud.bat` launcher
- Cockpit massively expanded: dual-viewport, Chart.js analytics, investor demo

### v1.0.0 — Initial Release (main)
- Core 6-DOF UAV dynamics, FANET networking, mission FSM
- Three.js 3D WebGL cockpit with telemetry HUD
- Full E2E test suite (Tiers 1–4)

---

<div align="center">

**Built for IIT Bombay TechFest** | Aashutosh Kedia | Branch: `aashutosh`

[🔀 Compare with main](https://github.com/rogstrix-sys/TECHFEST/compare/main...aashutosh) · [📋 Open PR](https://github.com/rogstrix-sys/TECHFEST/pull/new/aashutosh)

</div>
