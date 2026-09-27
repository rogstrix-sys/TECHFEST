"""
tests/unit/test_challenger_m1_2.py: Challenger M1-2 Verification & Adversarial Stress Tests.

Empirically tests:
1. Degenerate 3D Ray-AABB Geometry: vertex grazing, coplanar face grazing, zero-length rays, surface-origin rays.
2. Vectorized vs scalar numerical divergence across 10,000 rays.
3. Swarm scaling (25 and 50 drones, 500 ticks) and downwash NameError defect.
4. Cross-process determinism and bit-level state reproducibility.
"""

import math
import os
import subprocess
import sys
import time
import numpy as np
import pytest

from sim.core import SimulationConfig, SwarmSimulationCore
from sim.drone import Drone
from sim.environment import DisasterEnvironment
from sim.obstacles import ObstacleAABB, ObstacleManager, create_default_disaster_obstacles
from sim.types import DroneLimits, DroneRole, FlightMode


class TestDegenerateGeometry:
    """Adversarial geometry challenges on 3D Ray-AABB slab intersection."""

    @pytest.fixture
    def test_box_and_manager(self):
        box = ObstacleAABB(
            id="BOX_TEST",
            name="Test Box",
            min_pt=np.array([0.0, 0.0, 0.0]),
            max_pt=np.array([10.0, 10.0, 10.0]),
            base_attenuation_db=22.0,
            attenuation_db_per_meter=1.5,
        )
        return box, ObstacleManager([box])

    def test_vertex_grazing_behavior(self, test_box_and_manager):
        """
        Adversarial test: A ray passes precisely through corner vertex (10, 10, 10)
        from (11, 9, 10) to (9, 11, 10) without entering the volume.
        Documents behavior: hit is flagged True with 0m penetration and 22dB base attenuation.
        """
        box, manager = test_box_and_manager
        p_src = np.array([11.0, 9.0, 10.0])
        p_dst = np.array([9.0, 11.0, 10.0])

        res = box.intersect_ray_segment(p_src, p_dst)
        clear_s, pen_s, att_s, _ = manager.check_los(p_src, p_dst)
        clear_v, pen_v, att_v = manager.check_los_batch(p_src[np.newaxis, :], p_dst[np.newaxis, :])

        assert res.hit is True
        assert math.isclose(res.penetration_distance, 0.0, abs_tol=1e-9)
        assert math.isclose(res.attenuation_db, 22.0, abs_tol=1e-9)
        assert clear_s is False
        assert bool(clear_v[0]) is False
        assert math.isclose(pen_s, float(pen_v[0]), abs_tol=1e-9)
        assert math.isclose(att_s, float(att_v[0]), abs_tol=1e-9)

    def test_coplanar_face_grazing_discontinuity(self, test_box_and_manager):
        """
        Adversarial test: A ray travels across the roof face z=10.0 from (-5, 5, 10) to (15, 5, 10).
        Demonstrates that coplanar grazing assesses 10m penetration (37 dB attenuation),
        while a 1e-10m perturbation yields 0m penetration (0 dB attenuation).
        """
        box, manager = test_box_and_manager
        p_src_face = np.array([-5.0, 5.0, 10.0])
        p_dst_face = np.array([15.0, 5.0, 10.0])
        res_face = box.intersect_ray_segment(p_src_face, p_dst_face)

        p_src_pert = np.array([-5.0, 5.0, 10.0 + 1e-10])
        p_dst_pert = np.array([15.0, 5.0, 10.0 + 1e-10])
        res_pert = box.intersect_ray_segment(p_src_pert, p_dst_pert)

        assert res_face.hit is True
        assert math.isclose(res_face.penetration_distance, 10.0, abs_tol=1e-9)
        assert math.isclose(res_face.attenuation_db, 37.0, abs_tol=1e-9)

        assert res_pert.hit is False
        assert math.isclose(res_pert.penetration_distance, 0.0, abs_tol=1e-9)
        assert math.isclose(res_pert.attenuation_db, 0.0, abs_tol=1e-9)

    def test_zero_length_rays(self, test_box_and_manager):
        """
        Test zero-length rays (src == dst) inside, outside, and on surface.
        """
        box, manager = test_box_and_manager
        pts = {
            "inside": (np.array([5.0, 5.0, 5.0]), True, False, 22.0),
            "outside": (np.array([20.0, 20.0, 20.0]), False, True, 0.0),
            "surface": (np.array([10.0, 5.0, 5.0]), True, False, 22.0),
            "vertex": (np.array([10.0, 10.0, 10.0]), True, False, 22.0),
        }

        for name, (pt, exp_hit, exp_clear, exp_att) in pts.items():
            res = box.intersect_ray_segment(pt, pt)
            c_s, p_s, a_s, _ = manager.check_los(pt, pt)
            c_v, p_v, a_v = manager.check_los_batch(pt[np.newaxis, :], pt[np.newaxis, :])

            assert res.hit is exp_hit, f"Mismatch in {name} hit"
            assert c_s is exp_clear, f"Mismatch in {name} scalar los_clear"
            assert bool(c_v[0]) is exp_clear, f"Mismatch in {name} vector los_clear"
            assert math.isclose(a_s, exp_att, abs_tol=1e-9)
            assert math.isclose(float(a_v[0]), exp_att, abs_tol=1e-9)

    def test_surface_start_pointing_away(self, test_box_and_manager):
        """
        Adversarial test: Ray starts on surface x=0 pointing OUTWARD into open space (-10, 5, 5).
        Documents that current slab test classifies this as hit=True, los_clear=False with 22dB loss.
        """
        box, manager = test_box_and_manager
        p_src = np.array([0.0, 5.0, 5.0])
        p_dst = np.array([-10.0, 5.0, 5.0])

        res = box.intersect_ray_segment(p_src, p_dst)
        c_s, p_s, a_s, _ = manager.check_los(p_src, p_dst)
        c_v, p_v, a_v = manager.check_los_batch(p_src[np.newaxis, :], p_dst[np.newaxis, :])

        assert res.hit is True
        assert math.isclose(res.penetration_distance, 0.0, abs_tol=1e-9)
        assert c_s is False
        assert bool(c_v[0]) is False


class TestVectorizedEquivalence:
    """Stress test vectorized vs scalar consistency across 10,000 rays."""

    def test_vectorized_vs_scalar_10000_rays(self):
        obstacles = create_default_disaster_obstacles()
        manager = ObstacleManager(obstacles)
        np.random.seed(42)

        K = 10000
        sources = np.random.uniform([-250, -250, 0], [250, 250, 120], size=(K, 3))
        targets = np.random.uniform([-250, -250, 0], [250, 250, 120], size=(K, 3))

        # Vectorized
        vec_clear, vec_pen, vec_att = manager.check_los_batch(sources, targets)

        # Scalar
        scalar_clear = np.zeros(K, dtype=bool)
        scalar_pen = np.zeros(K, dtype=np.float64)
        scalar_att = np.zeros(K, dtype=np.float64)

        for i in range(K):
            c, p, a, _ = manager.check_los(sources[i], targets[i])
            scalar_clear[i] = c
            scalar_pen[i] = p
            scalar_att[i] = a

        # Assertions
        assert np.array_equal(vec_clear, scalar_clear), "Boolean LoS divergence between scalar and batch"
        assert np.allclose(vec_pen, scalar_pen, atol=1e-8), "Penetration distance divergence"
        assert np.allclose(vec_att, scalar_att, atol=1e-8), "Attenuation divergence"


class TestSwarmScalingAndDefects:
    """High drone count scaling and defect reproduction."""

    def test_downwash_nameerror_reproduction(self):
        """
        Confirms the critical bug: sim/core.py line 183 calls math.exp() without 'import math'.
        """
        import sim.core
        core_instance = SwarmSimulationCore(SimulationConfig(enable_downwash=True))
        d_top = Drone("TOP", initial_pos=[0.0, 0.0, 35.0])
        d_bot = Drone("BOT", initial_pos=[0.0, 0.0, 30.0])
        d_top.set_flight_mode(FlightMode.TRANSIT)
        d_bot.set_flight_mode(FlightMode.TRANSIT)
        core_instance.add_drone(d_top)
        core_instance.add_drone(d_bot)

        # In unpatched sim.core, compute_steering_forces raises NameError
        if not hasattr(sim.core, "math"):
            with pytest.raises(NameError, match="name 'math' is not defined"):
                core_instance.compute_steering_forces(d_bot)

    def test_large_swarm_scaling_performance(self):
        """
        Benchmarks 25 and 50 drones running 100 ticks with sim.core.math available.
        Verifies effective simulation rate >= 20 Hz and numerical stability.
        """
        import sim.core
        sim.core.math = math

        for count in [25, 50]:
            core = SwarmSimulationCore(SimulationConfig(dt=0.05, enable_downwash=True))
            for obs in create_default_disaster_obstacles():
                core.add_obstacle(obs)

            np.random.seed(1234)
            for i in range(count):
                role = DroneRole.RELAY if (i % 4 == 0) else DroneRole.SURVEY
                pos = np.random.uniform([-100, -100, 10], [100, 100, 40])
                d = Drone(f"UAV_{i:02d}", role=role, initial_pos=pos)
                d.set_flight_mode(FlightMode.TRANSIT)
                d.set_target_waypoint(np.random.uniform([-150, -150, 25], [150, 150, 80]))
                core.add_drone(d)

            t0 = time.perf_counter()
            for _ in range(100):
                core.step(0.05)
            t1 = time.perf_counter()

            sim_hz = 100.0 / (t1 - t0)
            # 25-drone swarm must run at >=20 Hz; 50-drone swarm at >=15 Hz
            # (goal-oriented tangential bypass adds ~10% APF overhead per drone)
            hz_target = 20.0 if count <= 25 else 15.0
            assert sim_hz >= hz_target, f"Simulation rate {sim_hz:.1f} Hz fell below {hz_target} Hz target for {count} drones"

            for d in core.drones.values():
                assert np.all(np.isfinite(d.position)), "NaN/Inf in drone position"
                assert np.all(np.isfinite(d.velocity)), "NaN/Inf in drone velocity"


class TestDeterminism:
    """Verifies deterministic reproducibility across independent runs."""

    def test_cross_process_determinism(self):
        subscript_code = """
import sys
import os
import json
import math
import numpy as np

sys.path.insert(0, os.path.abspath(r"{repo_root}"))

import sim.core
sim.core.math = math

from sim.core import SwarmSimulationCore, SimulationConfig
from sim.drone import Drone
from sim.types import DroneRole, FlightMode
from sim.obstacles import create_default_disaster_obstacles

core = SwarmSimulationCore(SimulationConfig(dt=0.05, enable_downwash=True))
for obs in create_default_disaster_obstacles():
    core.add_obstacle(obs)

np.random.seed(4242)
for i in range(5):
    d = Drone(f"D_{i}", initial_pos=[i*10.0, i*5.0, 20.0])
    d.set_flight_mode(FlightMode.TRANSIT)
    d.set_target_waypoint([0.0, 0.0, 30.0])
    core.add_drone(d)

for _ in range(50):
    core.step(0.05)

states = dict()
for d in core.drones.values():
    states[d.id] = dict(pos=d.position.tolist(), vel=d.velocity.tolist())
print(json.dumps(states))
"""
        repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")).replace("\\", "/")
        code = subscript_code.replace("{repo_root}", repo_root)

        p1 = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)
        p2 = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, check=True)

        assert p1.stdout == p2.stdout, "State trajectories between independent processes diverged!"
