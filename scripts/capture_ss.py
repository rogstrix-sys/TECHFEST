"""
scripts/capture_ss.py: Capture 1920x1080 screenshot of the WebGL 3D world via CDP.
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

CHROME_BIN = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
OUT_PATH = sys.argv[1] if len(sys.argv) > 1 else "/tmp/sector_delta_view.png"
URL = sys.argv[2] if len(sys.argv) > 2 else "http://localhost:8000/"
DEBUG_PORT = 9245

async def capture_cdp():
    temp_profile = tempfile.mkdtemp(prefix="chrome_snap_")
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
            # Wait for 3D textures, WebGL frame loop, and telemetry WebSocket
            print("[*] Waiting 2.5s for WebGL scene and telemetry stabilization...")
            await asyncio.sleep(2.5)

            # Capture screenshot
            snap_res = await cdp_call("Page.captureScreenshot", {"format": "png"})
            data_b64 = snap_res.get("data", "")
            if data_b64:
                os.makedirs(os.path.dirname(os.path.abspath(OUT_PATH)), exist_ok=True)
                with open(OUT_PATH, "wb") as f:
                    f.write(base64.b64decode(data_b64))
                print(f"[+] Screenshot captured successfully: {OUT_PATH} ({os.path.getsize(OUT_PATH)} bytes)")
                return True
            else:
                print("[-] Failed to capture screenshot data.")
                return False
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()

if __name__ == "__main__":
    asyncio.run(capture_cdp())
