"""
tests/unit/test_desktop_hud.py: Unit and Integration Tests for Native Desktop HUD & SVS.
"""

import numpy as np
import pytest

from vis.desktop_hud import Camera3D, TacticalDesktopHUD
from vis.server import create_default_simulation


class TestCamera3D:
    def test_camera_init_and_basis(self):
        cam = Camera3D(width=1280, height=720, fov_deg=65.0)
        assert cam.width == 1280
        assert cam.height == 720
        assert cam.fwd.shape == (3,)
        assert cam.right.shape == (3,)
        assert cam.up.shape == (3,)
        # Check orthogonality
        assert abs(float(np.dot(cam.fwd, cam.right))) < 1e-4
        assert abs(float(np.dot(cam.fwd, cam.up))) < 1e-4

    def test_camera_projection(self):
        cam = Camera3D(width=1280, height=720, fov_deg=65.0)
        cam.set_pose(np.array([0.0, -100.0, 50.0]), np.array([0.0, 0.0, 10.0]))

        # Target should project close to center
        proj = cam.project_point(np.array([0.0, 0.0, 10.0]))
        assert proj is not None
        u, v, zc = proj
        assert abs(u - 640) < 5
        assert abs(v - 360) < 5
        assert zc > 0

    def test_camera_behind_clipping(self):
        cam = Camera3D(width=1280, height=720)
        cam.set_pose(np.array([0.0, 0.0, 10.0]), np.array([0.0, 100.0, 10.0]))

        # Point behind camera should return None
        behind_pt = np.array([0.0, -50.0, 10.0])
        assert cam.project_point(behind_pt) is None


class TestTacticalDesktopHUD:
    def test_hud_initialization(self):
        sim = create_default_simulation()
        hud = TacticalDesktopHUD(sim=sim, width=1280, height=720)
        assert hud.width == 1280
        assert hud.height == 720
        assert hud.focused_drone_id in sim.drones
        assert len(sim.drones) == 16
        assert len(sim.pois) == 8

    def test_hud_drone_and_mode_cycling(self):
        hud = TacticalDesktopHUD(width=1280, height=720)
        first_drone = hud.focused_drone_id

        hud.cycle_drone(forward=True)
        assert hud.focused_drone_id != first_drone

        hud.set_drone("RELAY_1")
        assert hud.focused_drone_id == "RELAY_1"

        cam_mode_1 = hud.camera_mode
        hud.cycle_camera()
        assert hud.camera_mode != cam_mode_1

        sensor_mode_1 = hud.sensor_mode
        hud.cycle_sensor()
        assert hud.sensor_mode != sensor_mode_1

    def test_hud_render_frame_modes(self):
        hud = TacticalDesktopHUD(width=1280, height=720)
        # Advance simulation to engage controllers
        for _ in range(10):
            hud.sim.step(0.05)
        hud.sim_time = hud.sim.sim_time

        # Test Tactical RGB
        hud.sensor_mode = "TACTICAL"
        frame_tactical = hud.render_frame()
        assert isinstance(frame_tactical, np.ndarray)
        assert frame_tactical.shape == (720, 1280, 3)
        assert frame_tactical.dtype == np.uint8

        # Test FLIR Thermal White-Hot
        hud.sensor_mode = "FLIR_THERMAL"
        frame_flir = hud.render_frame()
        assert frame_flir.shape == (720, 1280, 3)

        # Test NVG Night Vision
        hud.sensor_mode = "NVG_NIGHT"
        frame_nvg = hud.render_frame()
        assert frame_nvg.shape == (720, 1280, 3)

    def test_hud_camera_perspectives(self):
        hud = TacticalDesktopHUD(width=1280, height=720)
        for cam_mode in ["FPV_CHASE", "FPV_COCKPIT", "TACTICAL_TOP", "GCS_MAST", "ORBIT", "SPLIT_SLAM"]:
            hud.camera_mode = cam_mode
            frame = hud.render_frame()
            assert frame.shape == (720, 1280, 3)

    def test_hud_mouse_orbit_pan_zoom(self):
        hud = TacticalDesktopHUD(width=1280, height=720)
        initial_yaw = hud.orbit_yaw
        initial_pitch = hud.orbit_pitch
        initial_dist = hud.orbit_dist

        # Simulate left button down + drag (Orbit)
        hud.on_mouse(1, 100, 100, 0, None)  # cv2.EVENT_LBUTTONDOWN
        hud.on_mouse(0, 150, 80, 0, None)   # cv2.EVENT_MOUSEMOVE
        hud.on_mouse(4, 150, 80, 0, None)   # cv2.EVENT_LBUTTONUP

        assert hud.orbit_yaw != initial_yaw
        assert hud.orbit_pitch != initial_pitch
        assert hud.is_custom_orbit is True

        # Simulate mouse wheel zoom (cv2.EVENT_MOUSEWHEEL)
        hud.on_mouse(10, 150, 80, 120, None)
        assert hud.orbit_dist < initial_dist

        hud.on_mouse(10, 150, 80, -120, None)
        assert hud.orbit_dist > 4.0
