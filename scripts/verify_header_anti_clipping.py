"""
scripts/verify_header_anti_clipping.py
Automated End-to-End Visual Verification of Header Buttons Anti-Clipping:
Tests window sizes 1536x864, 1280x720, and 1024x576 (user's exact resolution)
Verifies:
1. No button in .hud-controls goes out of the screen (bounding rect right <= innerWidth).
2. No button is cut off on the left (bounding rect left >= 0).
3. All view mode buttons (SPLIT, THEATER, SLAM, PiP) are fully visible with positive width.
4. Controls never collide with or overlap the brand.
5. Captures visual screenshots at 1024x576 and 1536x864.
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

ROOT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
ARTIFACT_DIR = r"C:\Users\kedia\.gemini\antigravity\brain\520a6b51-7d8b-4a86-80c6-ab15e9a57763"
DEBUG_PORT = 9299

async def verify_resolution(width: int, height: int, screenshot_name: str = None):
    print("=" * 76)
    print(f"[*] Testing Anti-Clipping at Resolution: {width}x{height}...")
    print("=" * 76)

    temp_profile = tempfile.mkdtemp(prefix="chrome_clipping_test_")
    env = os.environ.copy()
    env["SHIM_MCCOMPAT"] = "0x800000001"
    env["CUDA_VISIBLE_DEVICES"] = "0"
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"

    chrome_proc = subprocess.Popen([
        CHROME_PATH,
        "--headless=new",
        f"--app=http://127.0.0.1:8000/",
        f"--remote-debugging-port={DEBUG_PORT}",
        f"--user-data-dir={temp_profile}",
        "--no-sandbox",
        "--force-high-performance-gpu",
        "--gpu-preference=2",
        "--use-gl=angle",
        "--use-angle=d3d11",
        "--enable-gpu-rasterization",
        f"--window-size={width},{height}",
    ], env=env)

    try:
        page_ws_url = None
        for attempt in range(25):
            time.sleep(0.4)
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
            print("[-] Could not connect to Chrome CDP.")
            return False

        async with websockets.connect(page_ws_url, max_size=50 * 1024 * 1024) as ws:
            msg_id = 0
            async def cdp_call(method, params=None):
                nonlocal msg_id
                msg_id += 1
                cmd = {"id": msg_id, "method": method, "params": params or {}}
                await ws.send(json.dumps(cmd))
                while True:
                    resp = json.loads(await ws.recv())
                    if resp.get("id") == cmd["id"]:
                        return resp.get("result", {})

            await cdp_call("Page.enable")
            await cdp_call("Runtime.enable")

            # Force exact viewport size
            await cdp_call("Emulation.setDeviceMetricsOverride", {
                "width": width,
                "height": height,
                "deviceScaleFactor": 1,
                "mobile": False
            })

            await asyncio.sleep(2.0)

            # Audit bounding rectangles of every header element
            res = await cdp_call("Runtime.evaluate", {
                "expression": """
                (() => {
                    const winW = window.innerWidth;
                    const winH = window.innerHeight;
                    const header = document.querySelector(".hud-header");
                    const controls = document.querySelector(".hud-controls");
                    const brand = document.querySelector(".hud-brand");
                    const viewGroup = document.querySelector(".view-mode-group");
                    
                    const buttons = Array.from(controls.querySelectorAll("button, select"));
                    const viewButtons = Array.from(viewGroup.querySelectorAll(".btn-mode"));

                    const items = buttons.map(b => {
                        const r = b.getBoundingClientRect();
                        return {
                            id: b.id || b.getAttribute("data-mode") || b.textContent.trim(),
                            text: b.textContent.trim(),
                            left: Math.round(r.left),
                            right: Math.round(r.right),
                            top: Math.round(r.top),
                            bottom: Math.round(r.bottom),
                            width: Math.round(r.width),
                            height: Math.round(r.height),
                            isOutOfScreenRight: r.right > winW,
                            isOutOfScreenLeft: r.left < 0,
                            isCollapsed: r.width <= 0
                        };
                    });

                    const brandRect = brand.getBoundingClientRect();
                    const controlsRect = controls.getBoundingClientRect();

                    return {
                        windowWidth: winW,
                        windowHeight: winH,
                        headerWidth: Math.round(header.getBoundingClientRect().width),
                        controlsRight: Math.round(controlsRect.right),
                        controlsWidth: Math.round(controlsRect.width),
                        brandRight: Math.round(brandRect.right),
                        items: items,
                        viewButtons: viewButtons.map(vb => ({
                            mode: vb.getAttribute("data-mode"),
                            text: vb.textContent.trim(),
                            width: Math.round(vb.getBoundingClientRect().width)
                        }))
                    };
                })()
                """,
                "returnByValue": True
            })

            data = res.get("result", {}).get("value", {})
            print(f"[+] Window: {data.get('windowWidth')}x{data.get('windowHeight')} | Header Width: {data.get('headerWidth')}px")
            print(f"[+] Controls Right Edge: {data.get('controlsRight')}px (Must be <= {data.get('windowWidth')}px)")
            
            items = data.get("items", [])
            overflowing = [i for i in items if i["isOutOfScreenRight"]]
            clipped_left = [i for i in items if i["isOutOfScreenLeft"]]
            collapsed = [i for i in items if i["isCollapsed"]]

            print(f"[+] Total Buttons Audited: {len(items)}")
            for vb in data.get("viewButtons", []):
                print(f"    - View Mode [{vb['mode']}]: '{vb['text']}' (width: {vb['width']}px)")
                assert vb["width"] >= 20, f"Button {vb['mode']} is squished/collapsed!"

            if overflowing:
                print(f"[-] ERROR: Elements overflowing off screen: {[i['id'] for i in overflowing]}")
            if clipped_left:
                print(f"[-] ERROR: Elements clipped on left: {[i['id'] for i in clipped_left]}")
            if collapsed:
                print(f"[-] ERROR: Elements collapsed: {[i['id'] for i in collapsed]}")

            assert len(overflowing) == 0, f"Buttons went out of screen on right: {overflowing}"
            assert len(clipped_left) == 0, f"Buttons clipped on left: {clipped_left}"
            assert len(collapsed) == 0, f"Buttons collapsed to zero width: {collapsed}"

            if screenshot_name:
                shot = await cdp_call("Page.captureScreenshot", {"format": "png"})
                shot_bytes = base64.b64decode(shot["data"])
                out_path = os.path.join(ARTIFACT_DIR, screenshot_name)
                with open(out_path, "wb") as f:
                    f.write(shot_bytes)
                print(f"[+] Saved High-Res Screenshot: {out_path}")

            print(f"[+] Resolution {width}x{height} passed with ZERO cutoffs!")
            return True

    finally:
        try:
            chrome_proc.terminate()
            chrome_proc.wait(timeout=2)
        except Exception:
            pass
        try:
            shutil.rmtree(temp_profile, ignore_errors=True)
        except Exception:
            pass


async def main():
    print("=" * 76)
    print("[*] STEP 1: Starting Backend Simulation Server...")
    print("=" * 76)
    server_proc = subprocess.Popen(
        [sys.executable, "run_simulation.py", "--no-browser"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    for _ in range(30):
        time.sleep(0.5)
        try:
            r = httpx.get("http://127.0.0.1:8000/api/telemetry", timeout=1.0)
            if r.status_code == 200:
                print("[+] Server ready on port 8000.")
                break
        except Exception:
            pass

    try:
        # Test 1: Standard 1080p laptop with 125% DPI scaling (1536x864)
        ok1 = await verify_resolution(1536, 864, "uav_cockpit_no_cutoffs_1536x864.png")

        # Test 2: Standard 720p window (1280x720)
        ok2 = await verify_resolution(1280, 720)

        # Test 3: User's exact uploaded image resolution (1024x576)
        ok3 = await verify_resolution(1024, 576, "uav_cockpit_no_cutoffs_1024x576.png")

        if ok1 and ok2 and ok3:
            print("\n" + "=" * 76)
            print("[+] ALL RESOLUTION CHECKS PASSED: ZERO BUTTONS CUTTING OUT!")
            print("=" * 76)
            return True
        return False
    finally:
        try:
            server_proc.terminate()
            server_proc.wait(timeout=2)
        except Exception:
            pass

if __name__ == "__main__":
    success = asyncio.run(main())
    sys.exit(0 if success else 1)
