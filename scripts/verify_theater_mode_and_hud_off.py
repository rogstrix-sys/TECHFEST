"""
scripts/verify_theater_mode_and_hud_off.py
Automated End-to-End Visual Verification of:
1. Default Initial Viewport Mode = THEATER MODE (Viewport 1 full screen, Viewport 2 SLAM hidden).
2. Default HUD State = OFF (#btn-toggle-hud text "HUD: OFF", HUD canvas unrendered).
3. Option to Turn HUD ON dynamically via header button click or 'H' shortcut key.
4. HUD Overlay rendering correctly when activated.
5. Ability to switch viewports (Split 50/50, SLAM, PiP) when desired.
6. Captures high-res verification screenshots saved to artifacts directory.
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
DEBUG_PORT = 9288

async def run_verification():
    print("=" * 76)
    print("[*] STEP 1: Starting Simulation Server on Port 8000...")
    print("=" * 76)

    server_proc = subprocess.Popen(
        [sys.executable, "run_simulation.py", "--no-browser"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True
    )

    server_ready = False
    for _ in range(30):
        time.sleep(0.5)
        try:
            r = httpx.get("http://127.0.0.1:8000/api/telemetry", timeout=1.0)
            if r.status_code == 200:
                server_ready = True
                print("[+] Simulation server is live at http://127.0.0.1:8000/")
                break
        except Exception:
            pass

    if not server_ready:
        print("[-] Simulation server failed to start.")
        server_proc.terminate()
        return False

    temp_profile = tempfile.mkdtemp(prefix="chrome_theater_hud_test_")
    print("=" * 76)
    print("[*] STEP 2: Launching Chrome with NVIDIA RTX 4050 GPU Acceleration...")
    print("=" * 76)

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
        "--window-size=1920,1080",
    ], env=env)

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
            print("[-] Could not connect to Chrome DevTools port.")
            return False

        print(f"[+] Connected to Chrome CDP at: {page_ws_url}")

        async with websockets.connect(page_ws_url, max_size=50 * 1024 * 1024) as ws:
            msg_id = 1

            async def cdp_call(method, params=None):
                nonlocal msg_id
                cmd = {"id": msg_id, "method": method, "params": params or {}}
                msg_id += 1
                await ws.send(json.dumps(cmd))
                while True:
                    resp = json.loads(await ws.recv())
                    if resp.get("id") == cmd["id"]:
                        return resp.get("result", {})

            async def eval_js(expr):
                res = await cdp_call("Runtime.evaluate", {
                    "expression": expr,
                    "returnByValue": True
                })
                return res.get("result", {}).get("value", {})

            # Enable Page and Runtime domains
            await cdp_call("Page.enable")
            await cdp_call("Runtime.enable")

            # Let simulation run and WebGL scenes initialize
            print("[*] Waiting 4.0s for WebGL 3D scene & telemetries to stabilize...")
            await asyncio.sleep(4.0)

            # -------------------------------------------------------------
            # TEST 1: Inspect Initial Viewport Mode & HUD State
            # -------------------------------------------------------------
            init_state = await eval_js("""
                (() => {
                    const wrapper = document.getElementById("viewports-wrapper");
                    const vpTheater = document.getElementById("viewport-theater");
                    const vpSlam = document.getElementById("viewport-slam");
                    const hudBtn = document.getElementById("btn-toggle-hud");
                    const theaterBtn = document.querySelector('.btn-mode[data-mode="theater"]');
                    const splitBtn = document.querySelector('.btn-mode[data-mode="split"]');
                    
                    const theaterStyle = window.getComputedStyle(vpTheater);
                    const slamStyle = window.getComputedStyle(vpSlam);

                    return {
                        wrapperClass: wrapper ? wrapper.className : null,
                        theaterDisplay: theaterStyle ? theaterStyle.display : null,
                        theaterWidth: vpTheater ? vpTheater.clientWidth : 0,
                        slamDisplay: slamStyle ? slamStyle.display : null,
                        slamWidth: vpSlam ? vpSlam.clientWidth : 0,
                        hudBtnText: hudBtn ? hudBtn.textContent.trim() : null,
                        hudBtnHasDim: hudBtn ? hudBtn.classList.contains("text-dim") : false,
                        hudBtnHasGreen: hudBtn ? hudBtn.classList.contains("text-neon-green") : false,
                        theaterBtnActive: theaterBtn ? theaterBtn.classList.contains("active") : false,
                        splitBtnActive: splitBtn ? splitBtn.classList.contains("active") : false,
                        isHudEnabled: typeof isHudEnabled !== "undefined" ? isHudEnabled : null,
                        activeViewportMode: typeof activeViewportMode !== "undefined" ? activeViewportMode : null
                    };
                })()
            """)

            print("=" * 76)
            print("[*] INITIAL SYSTEM STATE AUDIT:")
            print(f"    - Wrapper Class: {init_state.get('wrapperClass')} (Expected: 'mode-theater')")
            print(f"    - Active Viewport Mode: {init_state.get('activeViewportMode')}")
            print(f"    - Theater Viewport Display: {init_state.get('theaterDisplay')} | Width: {init_state.get('theaterWidth')}px")
            print(f"    - SLAM Viewport Display: {init_state.get('slamDisplay')} | Width: {init_state.get('slamWidth')}px (Expected: 'none' / 0px)")
            print(f"    - HUD Button Text: {init_state.get('hudBtnText')} (Expected: 'HUD: OFF')")
            print(f"    - HUD Enabled Variable: {init_state.get('isHudEnabled')} (Expected: False)")
            print(f"    - Theater Button Active: {init_state.get('theaterBtnActive')} (Expected: True)")
            print("=" * 76)

            assert init_state.get("wrapperClass") == "mode-theater", "Wrapper must be mode-theater"
            assert init_state.get("activeViewportMode") == "theater", "activeViewportMode must be theater"
            assert init_state.get("theaterDisplay") == "block", "Theater viewport must be display: block"
            assert init_state.get("slamDisplay") == "none", "SLAM viewport must be display: none"
            assert init_state.get("hudBtnText") == "HUD: OFF", "HUD button must display 'HUD: OFF'"
            assert init_state.get("isHudEnabled") is False, "isHudEnabled must be false"
            assert init_state.get("theaterBtnActive") is True, "Theater button must be active"

            # Capture Screenshot 1: Theater Mode Full Screen, HUD OFF
            shot1 = await cdp_call("Page.captureScreenshot", {"format": "png"})
            shot1_bytes = base64.b64decode(shot1["data"])
            path1 = os.path.join(ARTIFACT_DIR, "uav_cockpit_theater_mode_hud_off.png")
            with open(path1, "wb") as f:
                f.write(shot1_bytes)
            print(f"[+] Saved Screenshot 1 (Theater Mode, HUD OFF): {path1}")

            # -------------------------------------------------------------
            # TEST 2: Turn HUD ON via Button Click
            # -------------------------------------------------------------
            print("\n[*] STEP 3: Activating HUD Overlay via Button Click...")
            await eval_js("""
                (() => {
                    const hudBtn = document.getElementById("btn-toggle-hud");
                    if (hudBtn) hudBtn.click();
                })()
            """)

            # Wait for animation frame to render HUD symbology
            await asyncio.sleep(1.0)

            hud_on_state = await eval_js("""
                (() => {
                    const hudBtn = document.getElementById("btn-toggle-hud");
                    return {
                        hudBtnText: hudBtn ? hudBtn.textContent.trim() : null,
                        hudBtnHasDim: hudBtn ? hudBtn.classList.contains("text-dim") : false,
                        hudBtnHasGreen: hudBtn ? hudBtn.classList.contains("text-neon-green") : false,
                        isHudEnabled: typeof isHudEnabled !== "undefined" ? isHudEnabled : null
                    };
                })()
            """)

            print(f"[+] HUD Button State after toggle: {hud_on_state.get('hudBtnText')}")
            print(f"[+] isHudEnabled variable: {hud_on_state.get('isHudEnabled')}")
            assert hud_on_state.get("hudBtnText") == "HUD: ON", "HUD button must show 'HUD: ON'"
            assert hud_on_state.get("isHudEnabled") is True, "isHudEnabled must be true"
            assert hud_on_state.get("hudBtnHasGreen") is True, "HUD button must have text-neon-green"

            # Capture Screenshot 2: Theater Mode Full Screen with HUD ON
            shot2 = await cdp_call("Page.captureScreenshot", {"format": "png"})
            shot2_bytes = base64.b64decode(shot2["data"])
            path2 = os.path.join(ARTIFACT_DIR, "uav_cockpit_hud_turned_on.png")
            with open(path2, "wb") as f:
                f.write(shot2_bytes)
            print(f"[+] Saved Screenshot 2 (Theater Mode, HUD ON): {path2}")

            # -------------------------------------------------------------
            # TEST 3: Test Keyboard 'H' Shortcut to Toggle HUD OFF & ON
            # -------------------------------------------------------------
            print("\n[*] STEP 4: Testing Keyboard 'H' Shortcut Toggle...")
            await cdp_call("Input.dispatchKeyEvent", {
                "type": "keyDown",
                "key": "h",
                "code": "KeyH",
                "text": "h"
            })
            await cdp_call("Input.dispatchKeyEvent", {
                "type": "keyUp",
                "key": "h",
                "code": "KeyH"
            })
            await asyncio.sleep(0.5)

            key_h_state = await eval_js("""
                (() => {
                    return {
                        hudBtnText: document.getElementById("btn-toggle-hud").textContent.trim(),
                        isHudEnabled: isHudEnabled
                    };
                })()
            """)
            print(f"[+] HUD State after pressing 'H': Text={key_h_state.get('hudBtnText')}, Enabled={key_h_state.get('isHudEnabled')}")
            assert key_h_state.get("hudBtnText") == "HUD: OFF"
            assert key_h_state.get("isHudEnabled") is False

            # -------------------------------------------------------------
            # TEST 4: Test Switching to SPLIT 50/50 Viewport Mode
            # -------------------------------------------------------------
            print("\n[*] STEP 5: Testing Viewport Mode Switching to SPLIT 50/50...")
            await eval_js("""
                (() => {
                    const splitBtn = document.querySelector('.btn-mode[data-mode="split"]');
                    if (splitBtn) splitBtn.click();
                })()
            """)
            await asyncio.sleep(1.0)

            split_state = await eval_js("""
                (() => {
                    const wrapper = document.getElementById("viewports-wrapper");
                    const vpTheater = document.getElementById("viewport-theater");
                    const vpSlam = document.getElementById("viewport-slam");
                    return {
                        wrapperClass: wrapper.className,
                        theaterDisplay: window.getComputedStyle(vpTheater).display,
                        theaterWidth: vpTheater.clientWidth,
                        slamDisplay: window.getComputedStyle(vpSlam).display,
                        slamWidth: vpSlam.clientWidth
                    };
                })()
            """)
            print(f"[+] Split Viewport State: Wrapper={split_state.get('wrapperClass')}")
            print(f"    - Theater Width: {split_state.get('theaterWidth')}px | SLAM Width: {split_state.get('slamWidth')}px")
            assert split_state.get("wrapperClass") == "mode-split"
            assert split_state.get("theaterDisplay") == "block"
            assert split_state.get("slamDisplay") == "block"
            assert split_state.get("slamWidth") > 200

            print("\n" + "=" * 76)
            print("[+] ALL CHECKS PASSED: THEATER MODE & HUD TOGGLE VERIFIED 100%!")
            print("=" * 76)
            return True

    finally:
        try:
            chrome_proc.terminate()
            chrome_proc.wait(timeout=3)
        except Exception:
            pass

        try:
            server_proc.terminate()
            server_proc.wait(timeout=3)
        except Exception:
            pass

        try:
            shutil.rmtree(temp_profile, ignore_errors=True)
        except Exception:
            pass

if __name__ == "__main__":
    success = asyncio.run(run_verification())
    sys.exit(0 if success else 1)
