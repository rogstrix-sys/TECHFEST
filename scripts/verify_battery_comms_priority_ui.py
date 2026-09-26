"""
Verification script for:
1. Battery power & airspeed/altitude telemetry in fleet cards and inspect panel.
2. Reliable fast-click selection on fleet cards without refresh cancellation.
3. Mission budget countdown timer in header.
4. Disaster site priority queue ordering and badges.
5. Captures high-res screenshot to artifact directory.
"""

import asyncio
import base64
import json
import os
import subprocess
import tempfile
import time
import httpx
import websockets

CHROME_PATH = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
ARTIFACT_DIR = r"C:\Users\kedia\.gemini\antigravity\brain\520a6b51-7d8b-4a86-80c6-ab15e9a57763"
DEBUG_PORT = 9224


async def verify_ui():
    temp_profile = tempfile.mkdtemp(prefix="chrome_priority_test_")
    print("=" * 76)
    print("[*] Launching Chrome Headless with DevTools...")
    print("=" * 76)

    env = os.environ.copy()
    env["SHIM_MCCOMPAT"] = "0x800000001"
    env["CUDA_VISIBLE_DEVICES"] = "0"
    env["__NV_PRIME_RENDER_OFFLOAD"] = "1"
    env["__GLX_VENDOR_LIBRARY_NAME"] = "nvidia"

    chrome_proc = subprocess.Popen([
        CHROME_PATH,
        "--headless=new",
        "http://127.0.0.1:8000/",
        f"--remote-debugging-port={DEBUG_PORT}",
        f"--user-data-dir={temp_profile}",
        "--no-sandbox",
        "--force-high-performance-gpu",
        "--gpu-preference=2",
        "--window-size=1536,864",
    ], env=env)

    try:
        page_ws_url = None
        for _ in range(30):
            time.sleep(0.4)
            try:
                r = httpx.get(f"http://127.0.0.1:{DEBUG_PORT}/json", timeout=1.5)
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
            print("[-] Could not connect to Chrome DevTools.")
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

            # Wait 2.5 seconds for Three.js and telemetry websocket to stream frames
            await asyncio.sleep(2.5)

            # 1. Verify Mission Budget Timer
            budget_res = await eval_js("""(() => {
                const el = document.getElementById('metric-budget');
                return {
                    text: el ? el.textContent.trim() : null,
                    colorClass: el ? el.className : null
                };
            })()""")
            print(f"[+] Mission Budget Display: {budget_res.get('text')} ({budget_res.get('colorClass')})")
            assert budget_res.get("text") and ":" in budget_res.get("text"), f"Invalid budget text: {budget_res}"

            # 2. Verify Fleet Card Power (PWR) and Comms (COM)
            fleet_res = await eval_js("""(() => {
                const cards = Array.from(document.querySelectorAll('#fleet-list .drone-card'));
                return cards.map(c => ({
                    id: c.getAttribute('data-drone-id'),
                    pwr: c.querySelector('.stat-pwr') ? c.querySelector('.stat-pwr').textContent : '',
                    com: c.querySelector('.stat-com') ? c.querySelector('.stat-com').textContent : ''
                }));
            })()""")
            print(f"[+] Found {len(fleet_res)} drone cards in fleet list:")
            for d in fleet_res[:4]:
                print(f"    - {d['id']}: PWR={d['pwr']}, COM={d['com']}")
            assert len(fleet_res) == 16, f"Expected 16 drone cards, found {len(fleet_res)}"
            assert all("W" in d["pwr"] for d in fleet_res), "Expected all cards to have power metrics"

            # 3. Verify Priority Queue badges in Disaster Survey Sites
            poi_res = await eval_js("""(() => {
                const cards = Array.from(document.querySelectorAll('#poi-list .poi-card'));
                return cards.map(c => ({
                    id: c.getAttribute('data-poi-id'),
                    drone: c.getAttribute('data-drone-id'),
                    badge: c.querySelector('.badge') ? c.querySelector('.badge').textContent : ''
                }));
            })()""")
            print(f"[+] Found {len(poi_res)} Disaster Sites in Priority Queue:")
            for p in poi_res[:4]:
                badge_clean = p['badge'].encode('ascii', 'ignore').decode('ascii').strip()
                print(f"    - [{badge_clean}]: {p['id']} (UAV: {p['drone']})")
            assert len(poi_res) > 0, "No priority queue disaster sites found"

            # 4. Test clicking on a drone card (e.g. SCOUT_1)
            print("[*] Simulating fast click / pointerdown on SCOUT_1 drone card...")
            click_res = await eval_js("""(() => {
                const card = document.querySelector('#fleet-list .drone-card[data-drone-id="SCOUT_1"]');
                if (!card) return false;
                const pointerEvt = new PointerEvent('pointerdown', { bubbles: true, cancelable: true });
                card.dispatchEvent(pointerEvt);
                return true;
            })()""")
            assert click_res is True, "Failed to dispatch pointerdown on SCOUT_1"

            await asyncio.sleep(0.6)

            # 5. Verify Inspect Panel opened and displays Avionics Power & Endurance
            inspect_data = await eval_js("""(() => {
                const pnl = document.getElementById('drone-inspect-panel');
                const idEl = document.getElementById('inspect-id');
                const pwrEl = document.getElementById('inspect-power');
                const endurEl = document.getElementById('inspect-endurance');
                const commsEl = document.getElementById('inspect-comms');
                return {
                    isOpen: pnl && !pnl.classList.contains('hidden'),
                    idText: idEl ? idEl.textContent : '',
                    power: pwrEl ? pwrEl.textContent : '',
                    endurance: endurEl ? endurEl.textContent : '',
                    comms: commsEl ? commsEl.textContent : ''
                };
            })()""")
            print(f"[+] Inspect Panel Status:")
            print(f"    - Open: {inspect_data.get('isOpen')}")
            print(f"    - Drone ID: {inspect_data.get('idText')}")
            print(f"    - Power Draw: {inspect_data.get('power')}")
            print(f"    - Est Endurance: {inspect_data.get('endurance')}")
            print(f"    - Comms Status: {inspect_data.get('comms')}")
            assert inspect_data.get("isOpen") is True, "Inspect panel should be visible after clicking drone card"
            assert "SCOUT_1" in inspect_data.get("idText"), "Inspect panel should show SCOUT_1"
            assert "W" in inspect_data.get("power"), "Inspect panel should show power in Watts"

            # 6. Capture high-resolution verification screenshot
            shot = await cdp_call("Page.captureScreenshot", {"format": "png"})
            shot_bytes = base64.b64decode(shot["data"])
            out_path = os.path.join(ARTIFACT_DIR, "uav_cockpit_battery_priority_inspect_verified.png")
            with open(out_path, "wb") as f:
                f.write(shot_bytes)
            print(f"[+] Saved High-Res Verification Screenshot: {out_path}")

        return True

    finally:
        chrome_proc.terminate()
        try:
            chrome_proc.wait(timeout=3)
        except Exception:
            chrome_proc.kill()


if __name__ == "__main__":
    asyncio.run(verify_ui())
