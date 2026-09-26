<div align="center">

# 🚁 UAV-X Swarm Simulation
### 3D Resilient Multi-Hop Aerial Communication Network

[![Version](https://img.shields.io/badge/version-2.0.0-blue.svg)](https://github.com/rogstrix-sys/TECHFEST/releases)
[![Python](https://img.shields.io/badge/Python-3.9%2B-green.svg)](https://python.org)
[![License](https://img.shields.io/badge/license-MIT-orange.svg)](#)
[![IIT Bombay](https://img.shields.io/badge/IIT%20Bombay-TechFest-red.svg)](#)

> A high-fidelity, physics-based simulation of an autonomous UAV swarm conducting post-disaster aerial survey and multi-hop communication relay — with a military-grade 3D cockpit and native desktop HUD.

</div>

---

## 📋 Table of Contents
- [Overview](#-overview)
- [Features](#-features)
- [Architecture](#-architecture)
- [Installation](#-installation)
- [Quick Start](#-quick-start)
- [Desktop HUD](#-desktop-hud-v2)
- [Project Structure](#-project-structure)
- [Technology Stack](#-technology-stack)
- [Simulation Parameters](#-simulation-parameters)
- [Test Suite](#-test-suite)

---

## 🌐 Overview

**UAV-X Swarm Simulation** is a decoupled, high-performance simulation framework for a **Flying Ad-Hoc Network (FANET)** formed by a fleet of 8 autonomous UAVs operating in a post-disaster zone. The swarm autonomously:

- 🗺️ **Surveys** multi-priority Points of Interest (PoIs): Survivor Search, Structural Collapse, Hazard Zones
- 📡 **Maintains resilient communication** via dynamic multi-hop mesh routing back to a Ground Control Station (GCS)
- 🔄 **Self-heals** routing paths using DTN Store-and-Forward buffering during link interruptions
- 📊 **Renders everything** in a real-time 3D WebGL cockpit with telemetry analytics

The project targets **IIT Bombay TechFest** and demonstrates production-grade autonomous swarm robotics.

---

## ✨ Features

### Core Simulation Engine
| # | Feature | Details |
|---|---------|---------|
| 1 | **6-DOF Quadcopter Dynamics** | Newton-Euler rigid body, position/velocity integration, quaternion attitude |
| 2 | **Flocking & Collision Avoidance** | Khatib Artificial Potential Fields + Reynolds Boids with downwash repulsion |
| 3 | **4-Tier Altitude Corridors** | Launch [0–20m] → Survey [25–45m] → Transit [50–65m] → Relay [70–90m] |
| 4 | **Battery & Energy Model** | Electro-mechanical power draw with Return-to-Launch low-battery trigger |
| 5 | **500m × 500m Disaster Environment** | 3D AABB building obstacles, GCS base, multi-priority PoI beacons |

### FANET Communication & Routing
| # | Feature | Details |
|---|---------|---------|
| 6 | **RF Propagation (2.4 GHz)** | Friis FSPL (PL₀ = 40.05 dB) + Log-distance (η_LoS=2.05, η_NLoS=3.60) |
| 7 | **3D Ray-AABB Occlusion Engine** | Ray-slab intersection test; +22 dB building penetration loss forces multi-hop |
| 8 | **Dynamic Link-State Routing** | Sub-ms Dijkstra shortest-path with composite cost (SNR, distance, obstacle penalty) |
| 9 | **DTN Store-and-Forward** | 250-packet FIFO ring buffer for transient link interruptions |
| 10 | **Network Telemetry** | Real-time PDR, end-to-end latency, and hop-count distribution |

### Mission Control & Autonomy
| # | Feature | Details |
|---|---------|---------|
| 11 | **10-State MAVSDK-Compliant FSM** | IDLE → TAKEOFF → TRANSIT → SURVEYING → RELAY → DATA_TX → RTL → LANDING |
| 12 | **Dynamic Role Allocation** | CBBA consensus-based assignment into Surveyor / Relay UAV roles |
| 13 | **Virtual Spring Mesh (VSM)** | Relay UAVs self-position via spring-damper forces between GCS and Surveyors |
| 14 | **3D LiDAR + Voxel SLAM** | Multi-beam rotating LiDAR, range noise, OctoMap log-odds 3D reconstruction |
| 15 | **9-State EKF Localization** | Extended Kalman Filter fusing IMU + GPS + barometer for state estimation |

### Visualization & HUD
| # | Feature | Details |
|---|---------|---------|
| 16 | **Three.js 3D WebGL Cockpit** | Dual-viewport split screen: External theater + Autonomous SLAM perception |
| 17 | **Multi-Hop Link Tubes** | Glowing 3D links color-coded by SNR/hop count, updating at 30 Hz |
| 18 | **Catmull-Rom Packet Pulses** | Animated photon packets traveling multi-hop relay splines to GCS |
| 19 | **Glassmorphic Telemetry HUD** | Real-time swarm status, routing table, PDR, latency, PoI progress |
| 20 | **Chart.js Analytics Drawer** | EKF convergence, PDR & throughput, SNR vs distance, battery curves |
| 21 | **Investor Presentation Mode** | 60s choreographed cinematic tour with commercial KPI callout banners |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                     SIMULATION CORE (sim/)                       │
│                                                                   │
│  ┌──────────────┐  ┌──────────────────┐  ┌───────────────────┐  │
│  │ UAV Kinematics│  │Disaster Environment│  │  Mission FSM &   │  │
│  │ 6-DOF, APF   │  │500×500m, AABB obs │  │ PoI Survey Logic  │  │
│  │ Flocking     │  │Multi-priority PoIs│  │ VSM Relay Mesh   │  │
│  └──────────────┘  └──────────────────┘  └───────────────────┘  │
│                            │                                      │
│                            ▼                                      │
│               ┌────────────────────────┐                         │
│               │  FANET Comm & Routing  │                         │
│               │ Friis / Log-distance PL│                         │
│               │ 3D Ray-AABB Occlusion  │                         │
│               │ Dynamic Link-State DLS │                         │
│               │ DTN Store-and-Forward  │                         │
│               └────────────────────────┘                         │
└──────────────────────────┬──────────────────────────────────────┘
                           │ Serialized State Frames (30 Hz)
                           ▼
┌─────────────────────────────────────────────────────────────────┐
│                  VISUALIZATION ENGINE (vis/)                      │
│                                                                   │
│  ┌─────────────────────┐     ┌─────────────────────────────┐    │
│  │ FastAPI/WebSocket   │────▶│  Three.js WebGL 3D Cockpit  │    │
│  │ vis/server.py       │     │  - Dual Viewport Split Screen│    │
│  │ Broadcasts JSON     │     │  - Animated UAV rotors       │    │
│  │ Handles HUD commands│     │  - Glowing multi-hop links   │    │
│  └─────────────────────┘     │  - Chart.js Analytics Drawer │    │
│                               └─────────────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
                           ▲
┌─────────────────────────────────────────────────────────────────┐
│              DESKTOP HUD LAYER (vis/desktop_hud.py) — NEW v2     │
│                                                                   │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  MIL-STD-1787D Military Aerospace HUD @ 60 FPS            │  │
│  │  Pitch Ladder · Flight Path Marker · Compass Tape          │  │
│  │  Airspeed / Altitude Tapes · PPI Radar · FLIR / NVG Modes │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

---

## ⚙️ Installation

### Prerequisites
- Python **3.9+**
- Git
- *(Optional)* NVIDIA GPU for hardware-accelerated desktop HUD

### Setup

```bash
# Clone the repository
git clone https://github.com/rogstrix-sys/TECHFEST.git
cd TECHFEST

# Install dependencies
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

### 1. Full Web Simulation (3D Cockpit in Browser)
```bash
python run_simulation.py
```
Opens automatically at **http://localhost:8000** with the full 3D Three.js cockpit.

### 2. CLI Options
```bash
# Custom fleet size and duration
python run_simulation.py --drones 8 --duration 300 --pois 5

# Headless (no browser, pure simulation)
python run_simulation.py --headless

# Custom port
python run_simulation.py --port 9000

# Faster simulation
python run_simulation.py --speed 2.0
```

---

## 🪟 Desktop HUD (v2)

> **New in v2.0.0** — Native desktop application window with military-grade HUD.

### Launch Options
```bash
# Default 1920×1080 desktop application window (no browser needed)
python run_hud.py

# Fullscreen Tactical Kiosk Mode
python run_hud.py --fullscreen

# Classic OpenCV Synthetic Vision System fallback
python run_hud.py --opencv
```
Or double-click `run_hud.bat` on Windows.

### HUD Features
- **MIL-STD-1787D** collimated aerospace symbology
- **Pitch ladder** with positive/negative rungs & dynamic bank/roll rotation
- **Flight Path Marker (FPM)** velocity vector
- **Heading compass tape** with target steering bug
- **Airspeed (CAS) & Altitude tapes** with trend vectors
- **Vertical Speed Indicator (VSI)**
- **3D Target Acquisition Lock Boxes** on PoIs and peer UAVs
- **PPI Radar** — 360° sweep with active RF mesh links
- **FLIR Thermal** and **NVG Night Vision** sensor modes
- **Hotkeys**: UAV selection, camera switching, sensor mode toggle, pause/reset

---

## 📁 Project Structure

```
TECHFEST/
├── run_simulation.py          # Main entrypoint (web simulation)
├── run_hud.py                 # Desktop HUD launcher (NEW v2)
├── run_hud.bat                # Windows one-click HUD launcher (NEW v2)
├── requirements.txt           # Python dependencies
│
├── sim/                       # Simulation core modules
│   ├── core.py                # Main simulation loop & orchestration
│   ├── drone.py               # 6-DOF UAV kinematics & dynamics
│   ├── environment.py         # Disaster zone environment
│   ├── obstacles.py           # 3D AABB building obstacles
│   ├── network.py             # FANET topology & link management
│   ├── mission.py             # PoI mission control & FSM
│   ├── mapping.py             # 3D LiDAR / OctoMap voxel SLAM
│   ├── sensors.py             # EKF sensor fusion (IMU + GPS + Baro)
│   ├── planning.py            # Path planning & CBBA consensus
│   ├── weather.py             # Wind & atmospheric disturbances
│   └── types.py               # Shared data types & dataclasses
│
├── vis/                       # Visualization engine
│   ├── server.py              # FastAPI / WebSocket telemetry server
│   ├── desktop_hud.py         # Military desktop HUD (NEW v2)
│   └── static/
│       ├── index.html         # Three.js cockpit shell
│       ├── js/cockpit.js      # Full 3D WebGL cockpit (1200+ lines)
│       └── css/style.css      # Glassmorphic HUD styles
│
├── tests/                     # Test suite
│   ├── conftest.py
│   ├── unit/                  # Unit tests (16 modules)
│   │   ├── test_drone.py
│   │   ├── test_network.py
│   │   ├── test_mission.py
│   │   ├── test_mapping.py
│   │   ├── test_sensors_ekf.py
│   │   ├── test_desktop_hud.py    # NEW v2
│   │   ├── test_hud_launcher.py   # NEW v2
│   │   └── ...
│   └── e2e/                   # End-to-End test suite (Tiers 1–4)
│       ├── test_tier1_features.py
│       ├── test_tier2_boundaries.py
│       ├── test_tier3_combinations.py
│       └── test_tier4_scenarios.py
│
└── scripts/                   # Utility & verification scripts
    ├── verify_browser.py
    ├── verify_desktop_hud_app.py  # NEW v2
    ├── verify_expanded_cockpit.py # NEW v2
    └── verify_web_cockpit_hud.py  # NEW v2
```

---

## 🛠️ Technology Stack

| Layer | Technology |
|-------|-----------|
| Simulation Engine | Python 3.9+, NumPy, SciPy |
| Web Server | FastAPI, Uvicorn, WebSockets |
| 3D Visualization | Three.js (WebGL), Catmull-Rom splines |
| Analytics | Chart.js |
| Desktop HUD | Python (OpenCV / CEF), Direct3D11/ANGLE |
| Testing | pytest, custom E2E Tier framework |
| Communication | WebSocket JSON @ 30 Hz |

---

## 🔧 Simulation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| Arena | 500 × 500 m | Disaster zone dimensions |
| Fleet Size | 4–8 UAVs | Configurable via CLI |
| Frequency | 2.4 GHz | RF communication band |
| Update Rate | 30 Hz | Simulation & visualization tick |
| HUD Frame Rate | 60 FPS | Desktop HUD rendering |
| Altitude Layers | 4 tiers | Launch / Survey / Transit / Relay |
| DTN Buffer | 250 packets | FIFO store-and-forward ring |
| FSPL Reference | 40.05 dB | Friis free-space path loss @ 1m |
| LoS Path Loss exp. | 2.05 | Log-distance model (line of sight) |
| NLoS Path Loss exp. | 3.60 | Log-distance model (obstructed) |
| Building Penetration | +22 dB | Forced multi-hop occlusion loss |

---

## 🧪 Test Suite

```bash
# Run all unit tests
pytest tests/unit/ -v

# Run full E2E suite (Tiers 1–4)
pytest tests/e2e/ -v

# Run specific tier
pytest tests/e2e/test_tier4_scenarios.py -v
```

**Coverage:**
- ✅ **Tier 1** — Feature verification (all 29 core features)
- ✅ **Tier 2** — Boundary conditions & edge cases
- ✅ **Tier 3** — Feature interaction & combinations
- ✅ **Tier 4** — Full mission scenarios (multi-UAV, obstacle-dense, battery-critical)
- ✅ **Tier 5** — Adversarial stress & chaos testing

---

## 📈 Changelog

### v2.0.0 — Desktop HUD & Cockpit Expansion
- ✨ **New**: `vis/desktop_hud.py` — MIL-STD-1787D military aerospace HUD at 60 FPS
- ✨ **New**: `run_hud.py` & `run_hud.bat` — Native desktop application launcher
- ✨ **New**: FLIR Thermal & NVG Night Vision sensor simulation modes
- ✨ **New**: PPI Radar with 360° sweep and active RF mesh visualization
- 🔧 **Enhanced**: `cockpit.js` massively expanded (+1,200 lines) — dual-viewport, 16-drone support, Chart.js analytics drawer, investor presentation bar
- 🔧 **Enhanced**: WebSocket server with additional HUD command channels
- 🔧 **Enhanced**: CSS glassmorphic refinements for split-screen layout
- ✅ **New Tests**: `test_desktop_hud.py`, `test_hud_launcher.py`, 3 new verification scripts

### v1.0.0 — Initial Release
- Core 6-DOF UAV dynamics, FANET networking, mission FSM
- Three.js 3D WebGL cockpit with telemetry HUD
- Full E2E test suite (Tiers 1–4)

---

<div align="center">

**Built for IIT Bombay TechFest** | Aashutosh Kedia

</div>
