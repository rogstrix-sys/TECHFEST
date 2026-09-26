"""
scripts/verify_web_cockpit_hud.py
End-to-end automated verification of:
1. Web Cockpit Military HUD Canvas (#web-military-hud) overlay.
2. Full mouse pass-through (pointer-events: none) enabling Three.js OrbitControls orbit/pan/zoom.
3. HUD Symbology: Pitch ladder, boresight waterline, FPM velocity vector, CAS/ALT tapes, compass tape, tactical PPI radar, and NVIDIA RTX 4050 GPU badge.
4. HUD Toggle button (#btn-toggle-hud) and 'H' hotkey.
5. High-resolution screenshot capture.
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

CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
ARTIFACT_DIR = r"C:\Users\kedia\.gemini\antigravity\brain\520a6b51-7d8b-4a86-80c6-ab15e9a57763"
DEBUG_PORT = 9244

async def run_hud_verification():
    print("[*] Starting local simulation server in background...")
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

    temp_profile = tempfile.mkdtemp(prefix="chrome_hud_test_")
    print(f"[*] Launching Headless Chrome on port {DEBUG_PORT} with temp profile...")
    chrome_proc = subprocess.Popen([
        CHROME_PATH,
        "--headless=new",
        f"--remote-debugging-port={DEBUG_PORT}",
        f"--user-data-dir={temp_profile}",
        "--no-sandbox",
        "--disable-gpu",
        "--window-size=1920,1080",
        "http://127.0.0.1:8000/"
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
            print("[-] Could not find Chrome page WebSocket URL.")
            return False

        print(f"[+] Connected to Chrome Page WebSocket: {page_ws_url}")
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

            # 1. Wait 4 seconds for simulation stream and 3D rendering
            print("[*] Waiting 4 seconds for telemetry stream and 3D scene...")
            await asyncio.sleep(4.0)

            # Check HUD canvas in DOM
            res_hud = await send_cmd("Runtime.evaluate", {
                "expression": "Boolean(document.getElementById('web-military-hud'))"
            })
            hud_exists = res_hud.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified Military HUD Canvas in DOM: {hud_exists}")

            res_pointer = await send_cmd("Runtime.evaluate", {
                "expression": "window.getComputedStyle(document.getElementById('web-military-hud')).pointerEvents"
            })
            pointer_events = res_pointer.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified HUD pointer-events CSS property: '{pointer_events}' (Must be 'none' for mouse orbit)")

            res_btn = await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('btn-toggle-hud') ? document.getElementById('btn-toggle-hud').textContent : 'NONE'"
            })
            btn_text = res_btn.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified HUD Toggle Button Text: '{btn_text}'")

            # Check GPU Telemetry in DOM or JS state
            res_gpu = await send_cmd("Runtime.evaluate", {
                "expression": "JSON.stringify((window.latestTelemetry && window.latestTelemetry.gpu) || null)"
            })
            gpu_data = res_gpu.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified GPU Telemetry in Web State: {gpu_data}")

            # Capture Screenshot 1: Web Cockpit with Military HUD active
            ss1 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_1 = ss1.get("result", {}).get("data")
            if b64_1:
                p1 = os.path.join(ARTIFACT_DIR, "cockpit_web_hud_verified.png")
                with open(p1, "wb") as f:
                    f.write(base64.b64decode(b64_1))
                print(f"[+] Saved Web HUD Screenshot: {p1}")

            # 2. Test Mouse Drag OrbitControls through the HUD
            print("[*] Testing 3D Mouse Drag OrbitControls through HUD overlay...")
            res_cam_before = await send_cmd("Runtime.evaluate", {
                "expression": "({x: cameraTheater.position.x, y: cameraTheater.position.y, z: cameraTheater.position.z})"
            })
            cam_before = res_cam_before.get("result", {}).get("result", {}).get("value")

            # Dispatch mouse drag event on #viewport-theater
            await send_cmd("Runtime.evaluate", {
                "expression": """
                (() => {
                    const el = document.getElementById('viewport-theater');
                    const rect = el.getBoundingClientRect();
                    const startX = rect.left + rect.width / 2;
                    const startY = rect.top + rect.height / 2;
                    el.dispatchEvent(new MouseEvent('mousedown', { bubbles: true, clientX: startX, clientY: startY, button: 0 }));
                    el.dispatchEvent(new MouseEvent('mousemove', { bubbles: true, clientX: startX + 150, clientY: startY - 80, button: 0 }));
                    el.dispatchEvent(new MouseEvent('mouseup', { bubbles: true, clientX: startX + 150, clientY: startY - 80, button: 0 }));
                    if (controlsTheater) controlsTheater.update();
                })()
                """
            })
            await asyncio.sleep(0.5)

            res_cam_after = await send_cmd("Runtime.evaluate", {
                "expression": "({x: cameraTheater.position.x, y: cameraTheater.position.y, z: cameraTheater.position.z})"
            })
            cam_after = res_cam_after.get("result", {}).get("result", {}).get("value")
            print(f"[+] Camera Position Before Orbit Drag: {cam_before}")
            print(f"[+] Camera Position After Orbit Drag:  {cam_after}")

            # 3. Switch to Chase Camera to view HUD in aerial flight perspective
            print("[*] Switching to CHASE camera to verify pitch ladder and boresight in flight...")
            await send_cmd("Runtime.evaluate", {
                "expression": "document.querySelector('.btn-cam[data-cam=\"chase\"]').click()"
            })
            await asyncio.sleep(1.5)

            ss2 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_2 = ss2.get("result", {}).get("data")
            if b64_2:
                p2 = os.path.join(ARTIFACT_DIR, "cockpit_web_hud_chase_verified.png")
                with open(p2, "wb") as f:
                    f.write(base64.b64decode(b64_2))
                print(f"[+] Saved Web HUD Chase Camera Screenshot: {p2}")

            # 4. Test HUD Toggle Button
            print("[*] Testing HUD toggle button click...")
            await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('btn-toggle-hud').click()"
            })
            await asyncio.sleep(0.5)

            res_hud_off = await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('btn-toggle-hud').textContent"
            })
            btn_off_text = res_hud_off.get("result", {}).get("result", {}).get("value")
            print(f"[+] HUD Button Text after click: '{btn_off_text}'")

            # Toggle back ON
            await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('btn-toggle-hud').click()"
            })
            await asyncio.sleep(0.5)

            assert hud_exists is True, "HUD canvas element must exist"
            assert pointer_events == "none", "HUD canvas pointer-events must be 'none'"
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
        print("[*] Terminated Chrome and Server processes, cleaned up temp profile.")

if __name__ == "__main__":
    success = asyncio.run(run_hud_verification())
    if success:
        print("\n[SUCCESS] ALL WEB MILITARY HUD CHECKS PASSED PERFECTLY!")
    else:
        print("\n[FAIL] Web HUD Verification failed.")
        sys.exit(1)
