import asyncio
import subprocess
import time
import os
import sys
import json
import base64
import tempfile
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
        "name": "wide_forest_view",
        "path": "vis/static/screenshots/wide_forest_view.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(0, -1650, 1250); controlsTheater.target.set(0, -50, 0); controlsTheater.update(); }"
    },
    {
        "name": "user_angle_view",
        "path": "vis/static/screenshots/user_angle_view.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(0, -980, 780); controlsTheater.target.set(0, 0, 0); controlsTheater.update(); }"
    },
    {
        "name": "overhead_view",
        "path": "vis/static/screenshots/overhead_view.png",
        "eval": "if (cameraTheater && controlsTheater) { cameraTheater.position.set(0, 0, 1200); controlsTheater.target.set(0, 0, 0); controlsTheater.update(); }"
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
            return False

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
                await cdp_call("Runtime.evaluate", {"expression": v["eval"]})
                await asyncio.sleep(1.0)
                snap_res = await cdp_call("Page.captureScreenshot", {"format": "png"})
                data_b64 = snap_res.get("data", "")
                if data_b64:
                    os.makedirs(os.path.dirname(os.path.abspath(v["path"])), exist_ok=True)
                    with open(v["path"], "wb") as f:
                        f.write(base64.b64decode(data_b64))
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()

if __name__ == "__main__":
    asyncio.run(capture_views())
