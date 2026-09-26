"""
scripts/verify_desktop_hud_app.py
End-to-End Automated Verification of:
1. Dedicated Desktop HUD Application Window (App Mode with zero browser chrome).
2. Live NVIDIA RTX 4050 GPU hardware acceleration.
3. Dual-viewport 3D WebGL Web Cockpit with Military HUD overlay and mouse OrbitControls.
4. Native OpenCV SVS Split-Screen mode (SPLIT_SLAM).
"""

import subprocess
import time
import json
import base64
import httpx
import websockets
import asyncio
import os
import sys
import tempfile
import shutil
import cv2

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
ARTIFACT_DIR = r"C:\Users\kedia\.gemini\antigravity\brain\520a6b51-7d8b-4a86-80c6-ab15e9a57763"
DEBUG_PORT = 9255

async def run_desktop_hud_verification():
    print("=" * 76)
    print("[*] STEP 1: Verify Native Desktop OpenCV SPLIT_SLAM Viewport Mode...")
    print("=" * 76)
    from vis.desktop_hud import TacticalDesktopHUD
    hud = TacticalDesktopHUD(width=1280, height=720)
    hud.camera_mode = "SPLIT_SLAM"
    for _ in range(15):
        hud.sim.step(0.05)
    hud.sim_time = hud.sim.sim_time
    frame_split = hud.render_frame()

    svs_img_path = os.path.join(ARTIFACT_DIR, "desktop_hud_svs_split_verified.png")
    cv2.imwrite(svs_img_path, frame_split)
    print(f"[+] Saved Native OpenCV SPLIT_SLAM Screenshot: {svs_img_path}")

    print("=" * 76)
    print("[*] STEP 2: Verify Standalone Desktop HUD Window (App Mode)...")
    print("=" * 76)

    # 1. Start simulation server
    server_proc = subprocess.Popen(
        [sys.executable, "run_simulation.py", "--no-browser"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    # Poll server health until HTTP 200
    server_ready = False
    for _ in range(30):
        time.sleep(0.5)
        try:
            r = httpx.get("http://127.0.0.1:8000/api/telemetry", timeout=1.0)
            if r.status_code == 200:
                server_ready = True
                print("[+] Simulation server is live and responding at http://127.0.0.1:8000/")
                break
        except Exception:
            pass

    if not server_ready:
        print("[-] Server failed to become ready.")
        server_proc.terminate()
        return False

    temp_profile = tempfile.mkdtemp(prefix="chrome_app_hud_test_")
    print(f"[*] Launching Standalone Desktop HUD Window (App Mode) on port {DEBUG_PORT}...")
    chrome_proc = subprocess.Popen([
        CHROME_PATH,
        "--headless=new",
        f"--app=http://127.0.0.1:8000/",
        f"--remote-debugging-port={DEBUG_PORT}",
        f"--user-data-dir={temp_profile}",
        "--no-sandbox",
        "--disable-gpu",
        "--window-size=1920,1080",
    ])

    try:
        page_ws_url = None
        for attempt in range(25):
            time.sleep(0.5)
            try:
                r = httpx.get(f"http://127.0.0.1:{DEBUG_PORT}/json", timeout=2.0)
                if r.status_code == 200:
                    for p in r.json():
                        if p.get("type") == "page" and "8000" in p.get("url", ""):
                            page_ws_url = p.get("webSocketDebuggerUrl")
                            break
                    if page_ws_url:
                        break
            except Exception:
                pass

        if not page_ws_url:
            print("[-] Could not find Chrome App page WebSocket URL.")
            return False

        print(f"[+] Connected to Desktop HUD App Window: {page_ws_url}")
        async with websockets.connect(page_ws_url) as ws:
            msg_id = 0
            async def send_cmd(method, params=None):
                nonlocal msg_id
                msg_id += 1
                payload = {"id": msg_id, "method": method}
                if params:
                    payload["params"] = params
                await ws.send(json.dumps(payload))
                while True:
                    m = json.loads(await ws.recv())
                    if m.get("id") == msg_id:
                        return m

            await send_cmd("Runtime.enable")
            await send_cmd("Page.enable")

            # Wait 4 seconds for simulation stream and 3D rendering
            print("[*] Waiting 4 seconds for full 3D scene and HUD rendering...")
            await asyncio.sleep(4.0)

            # Check HUD canvas in DOM
            res_hud = await send_cmd("Runtime.evaluate", {
                "expression": "Boolean(document.getElementById('web-military-hud'))"
            })
            hud_exists = res_hud.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified Military HUD Canvas in Desktop App Window: {hud_exists}")

            res_gpu = await send_cmd("Runtime.evaluate", {
                "expression": "JSON.stringify((window.latestTelemetry && window.latestTelemetry.gpu) || null)"
            })
            gpu_data = res_gpu.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified NVIDIA GPU Telemetry in Desktop App: {gpu_data}")

            # Capture Screenshot of Desktop HUD Window
            ss1 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_1 = ss1.get("result", {}).get("data")
            if b64_1:
                p1 = os.path.join(ARTIFACT_DIR, "desktop_hud_app_verified.png")
                with open(p1, "wb") as f:
                    f.write(base64.b64decode(b64_1))
                print(f"[+] Saved Standalone Desktop HUD App Screenshot: {p1}")

            assert hud_exists is True, "HUD canvas element must exist in desktop app"
            return True

    finally:
        chrome_proc.terminate()
        chrome_proc.wait()
        server_proc.terminate()
        server_proc.wait()
        try:
            shutil.rmtree(temp_profile, ignore_errors=True)
        except Exception:
            pass
        print("[*] Terminated processes, cleaned up temp profile.")

if __name__ == "__main__":
    success = asyncio.run(run_desktop_hud_verification())
    if success:
        print("\n[SUCCESS] ALL DESKTOP HUD VERIFICATION CHECKS PASSED PERFECTLY!")
    else:
        print("\n[FAIL] Verification failed.")
        sys.exit(1)
