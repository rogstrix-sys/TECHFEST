@echo off
REM ==============================================================================
REM UAV-X: Standalone Desktop Heads-Up Display (HUD) Launcher
REM MIL-STD-1787D Military Aerospace Cockpit & 3D SLAM Perception
REM NVIDIA GeForce RTX 4050 GPU Accelerated
REM ==============================================================================

echo [*] Initializing UAV-X Standalone Desktop HUD Cockpit...
cd /d "%~dp0"
python run_hud.py %*
if errorlevel 1 (
    echo [!] Launcher exited with code %errorlevel%.
    pause
)
