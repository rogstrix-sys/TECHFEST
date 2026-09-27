"""
tests/unit/test_hud_launcher.py: Unit tests for Standalone Desktop HUD Launcher (run_hud.py).
"""

import os
from run_hud import find_browser_app_executable, is_server_alive


def test_find_browser_app_executable():
    exe = find_browser_app_executable()
    assert exe is not None
    assert os.path.isfile(exe)
    lower = exe.lower()
    assert any(b in lower for b in ["chrome", "msedge", "edge", "brave", "chromium"])


def test_is_server_alive_closed_port():
    # An unused high port should cleanly return False without crashing
    assert is_server_alive(port=59123, timeout=0.2) is False
