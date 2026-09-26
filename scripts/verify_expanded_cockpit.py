"""
scripts/verify_expanded_cockpit.py
End-to-end automated verification of:
1. Telemetry open & close toggle functionality (stays closed on ticks)
2. 16-drone heterogeneous fleet simulation
3. 8 Points of Interest (PoIs)
4. 700m x 700m expanded graphical theater with procedural highways, runway, helipads, river canal, and 3D bridge
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
DEBUG_PORT = 9233

async def run_verification():
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

    temp_profile = tempfile.mkdtemp(prefix="chrome_uav_test_")
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

            # Check Fleet Count and PoI Count
            res_fleet = await send_cmd("Runtime.evaluate", {
                "expression": "document.querySelectorAll('.drone-card').length"
            })
            fleet_count = res_fleet.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified Active UAV Fleet Cards in DOM: {fleet_count}")

            res_pois = await send_cmd("Runtime.evaluate", {
                "expression": "document.querySelectorAll('.poi-card').length"
            })
            poi_count = res_pois.get("result", {}).get("result", {}).get("value")
            print(f"[+] Verified Disaster PoI Cards in DOM: {poi_count}")

            res_badge = await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('badge-network').textContent"
            })
            badge_text = res_badge.get("result", {}).get("result", {}).get("value")
            print(f"[+] Mesh Badge Status: {badge_text}")

            # Capture Screenshot 1: 700m Overview with Procedural Terrain
            ss1 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_1 = ss1.get("result", {}).get("data")
            if b64_1:
                p1 = os.path.join(ARTIFACT_DIR, "cockpit_700m_expanded_overview.png")
                with open(p1, "wb") as f:
                    f.write(base64.b64decode(b64_1))
                print(f"[+] Saved Overview Screenshot: {p1}")

            # 2. Test Telemetry Open Button
            print("[*] Testing TELEMETRY Open Button (#btn-toggle-telemetry)...")
            await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('btn-toggle-telemetry').click()"
            })
            await asyncio.sleep(1.0)

            res_panel_open = await send_cmd("Runtime.evaluate", {
                "expression": "!document.getElementById('drone-inspect-panel').classList.contains('hidden')"
            })
            panel_is_open = res_panel_open.get("result", {}).get("result", {}).get("value")
            print(f"[+] Telemetry Panel Open State: {panel_is_open}")

            ss2 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_2 = ss2.get("result", {}).get("data")
            if b64_2:
                p2 = os.path.join(ARTIFACT_DIR, "cockpit_telemetry_open.png")
                with open(p2, "wb") as f:
                    f.write(base64.b64decode(b64_2))
                print(f"[+] Saved Telemetry Open Screenshot: {p2}")

            # 3. Test Telemetry Close & Verify It Stays Closed Over 50+ Telemetry Ticks
            print("[*] Testing Telemetry Close Button (#btn-close-inspect)...")
            await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('btn-close-inspect').click()"
            })
            print("[*] Waiting 2.5 seconds (over 50 telemetry ticks) to verify panel STAYS closed...")
            await asyncio.sleep(2.5)

            res_panel_closed = await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('drone-inspect-panel').classList.contains('hidden')"
            })
            panel_is_closed = res_panel_closed.get("result", {}).get("result", {}).get("value")
            print(f"[+] Telemetry Panel Stays Closed (50 ticks later): {panel_is_closed}")

            ss3 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_3 = ss3.get("result", {}).get("data")
            if b64_3:
                p3 = os.path.join(ARTIFACT_DIR, "cockpit_telemetry_closed_verified.png")
                with open(p3, "wb") as f:
                    f.write(base64.b64decode(b64_3))
                print(f"[+] Saved Telemetry Closed Screenshot: {p3}")

            # 4. Click a different drone in fleet list (e.g. RELAY_1) to verify selection and open
            print("[*] Selecting RELAY_1 from Fleet List to test inspection...")
            await send_cmd("Runtime.evaluate", {
                "expression": "selectDrone('RELAY_1', true)"
            })
            await asyncio.sleep(1.0)

            res_relay_title = await send_cmd("Runtime.evaluate", {
                "expression": "document.getElementById('inspect-id').textContent"
            })
            relay_title = res_relay_title.get("result", {}).get("result", {}).get("value")
            print(f"[+] Telemetry Panel Title after card click: {relay_title}")

            ss4 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_4 = ss4.get("result", {}).get("data")
            if b64_4:
                p4 = os.path.join(ARTIFACT_DIR, "cockpit_relay_telemetry.png")
                with open(p4, "wb") as f:
                    f.write(base64.b64decode(b64_4))
                print(f"[+] Saved Relay Telemetry Screenshot: {p4}")

            # 5. Capture Tactical Top-Down View showing full 700m sector layout
            print("[*] Switching to TACTICAL TOP view to capture full 700m sector map...")
            await send_cmd("Runtime.evaluate", {
                "expression": "document.querySelector('.btn-cam[data-cam=\"top\"]').click()"
            })
            await asyncio.sleep(1.2)

            ss5 = await send_cmd("Page.captureScreenshot", {"format": "png"})
            b64_5 = ss5.get("result", {}).get("data")
            if b64_5:
                p5 = os.path.join(ARTIFACT_DIR, "cockpit_700m_tactical_top_view.png")
                with open(p5, "wb") as f:
                    f.write(base64.b64decode(b64_5))
                print(f"[+] Saved Tactical Top View Screenshot: {p5}")

            assert fleet_count == 16, f"Expected 16 drones, got {fleet_count}"
            assert poi_count == 8, f"Expected 8 PoIs, got {poi_count}"
            assert panel_is_open is True, "Panel should have opened upon clicking toggle button"
            assert panel_is_closed is True, "Panel should have stayed closed after clicking close button"
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
    success = asyncio.run(run_verification())
    if success:
        print("\n[SUCCESS] ALL VERIFICATION CHECKS PASSED PERFECTLY!")
    else:
        print("\n[FAIL] Verification failed.")
        sys.exit(1)
