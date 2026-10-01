"""
scripts/capture_angles.py: Capture multiple close-up viewpoints of the destroyed diorama via CDP.
"""
import subprocess
import time
import os
import sys
import json
import base64
import tempfile
import asyncio
import httpx
import websockets

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from run_hud import find_browser_app_executable

CHROME_BIN = find_browser_app_executable() or r"C:\Program Files\Google\Chrome\Application\chrome.exe"
URL = "http://localhost:8000/"
DEBUG_PORT = 9246

VIEWS = [
    {
        "name": "destroyed_overview",
        "path": "vis/static/screenshots/destroyed_overview.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(220, -850, 580); controlsTheater.target.set(0, -30, 20); controlsTheater.update(); }"
    },
    {
        "name": "landslide_mountain_closeup",
        "path": "vis/static/screenshots/landslide_mountain_closeup.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(130, 45, 42); controlsTheater.target.set(145, 155, 18); controlsTheater.update(); }"
    },
    {
        "name": "severed_bridge_closeup",
        "path": "vis/static/screenshots/severed_bridge_closeup.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(30, -175, 55); controlsTheater.target.set(38, -95, 10); controlsTheater.update(); }"
    },
    {
        "name": "city_ruins_closeup",
        "path": "vis/static/screenshots/city_ruins_closeup.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(-45, -170, 95); controlsTheater.target.set(-55, -20, 25); controlsTheater.update(); }"
    },
    {
        "name": "gcs_hub_view",
        "path": "vis/static/screenshots/gcs_hub_view.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(0, -780, 160); controlsTheater.target.set(0, -575, 10); controlsTheater.update(); }"
    },
    {
        "name": "pancake_collapse_closeup",
        "path": "vis/static/screenshots/pancake_collapse_closeup.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(-85, -125, 45); controlsTheater.target.set(-85, -60, 10); controlsTheater.update(); }"
    }
]

async def capture_views():
    temp_profile = tempfile.mkdtemp(prefix="chrome_views_")
    cmd = [
        CHROME_BIN,
        "--headless=new",
        "--no-sandbox",
        "--disable-gpu-watchdog",
        "--use-gl=angle",
        f"--remote-debugging-port={DEBUG_PORT}",
        f"--user-data-dir={temp_profile}",
        "--window-size=1920,1080",
        URL
    ]
    print(f"[*] Launching Chrome headless at {URL}...")
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        page_ws_url = None
        for _ in range(30):
            await asyncio.sleep(0.3)
            try:
                r = httpx.get(f"http://127.0.0.1:{DEBUG_PORT}/json", timeout=1.0)
                if r.status_code == 200:
                    for p in r.json():
                        if p.get("type") == "page":
                            page_ws_url = p.get("webSocketDebuggerUrl")
                            break
                    if page_ws_url:
                        break
            except Exception:
                pass

        if not page_ws_url:
            print("[-] Could not connect to Chrome CDP.")
            return False

        print(f"[+] Connected to Chrome CDP at: {page_ws_url}")
        async with websockets.connect(page_ws_url, max_size=50 * 1024 * 1024) as ws:
            msg_id = 1
            async def cdp_call(method, params=None):
                nonlocal msg_id
                c_id = msg_id
                msg_id += 1
                await ws.send(json.dumps({"id": c_id, "method": method, "params": params or {}}))
                while True:
                    resp = json.loads(await ws.recv())
                    if resp.get("id") == c_id:
                        return resp.get("result", {})

            await cdp_call("Page.enable")
            await asyncio.sleep(2.5)

            for v in VIEWS:
                print(f"[*] Positioning camera for: {v['name']}...")
                await cdp_call("Runtime.evaluate", {"expression": v["eval"]})
                await asyncio.sleep(1.0)
                snap_res = await cdp_call("Page.captureScreenshot", {"format": "png"})
                data_b64 = snap_res.get("data", "")
                if data_b64:
                    os.makedirs(os.path.dirname(os.path.abspath(v["path"])), exist_ok=True)
                    with open(v["path"], "wb") as f:
                        f.write(base64.b64decode(data_b64))
                    print(f"[+] Captured {v['name']} -> {v['path']}")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()

if __name__ == "__main__":
    asyncio.run(capture_views())
