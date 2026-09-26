@echo off
REM ==============================================================================
REM UAV-X: Standalone Desktop Heads-Up Display (HUD) Launcher
REM MIL-STD-1787D Military Aerospace Cockpit & 3D SLAM Perception
REM NVIDIA GeForce RTX 4050 GPU Accelerated
REM ==============================================================================

echo [*] Initializing UAV-X Standalone Desktop HUD Cockpit...
cd /d "%~dp0"
set SHIM_MCCOMPAT=0x800000001
set CUDA_VISIBLE_DEVICES=0
set __NV_PRIME_RENDER_OFFLOAD=1
set __GLX_VENDOR_LIBRARY_NAME=nvidia
python run_hud.py %*
if errorlevel 1 (
    echo [!] Launcher exited with code %errorlevel%.
    pause
)
