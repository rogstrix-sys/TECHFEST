import subprocess
import time
import httpx
import json
import websockets
import asyncio
import tempfile
import os
import shutil

async def test_flags(name, flags, env_vars=None):
    temp_p = tempfile.mkdtemp(prefix="gpu_test_")
    chrome_path = r"C:\Program Files\Google\Chrome\Application\chrome.exe"
    env = os.environ.copy()
    if env_vars:
        env.update(env_vars)
    cmd = [
        chrome_path,
        "--headless=new",
        "--remote-debugging-port=9299",
        f"--user-data-dir={temp_p}",
        "--no-sandbox",
        *flags,
        "about:blank"
    ]
    proc = subprocess.Popen(cmd, env=env)
    try:
        time.sleep(1.2)
        r = httpx.get("http://127.0.0.1:9299/json").json()
        ws_url = r[0]["webSocketDebuggerUrl"]
        async with websockets.connect(ws_url) as ws:
            js = """
            (() => {
                const canvas = document.createElement('canvas');
                const gl = canvas.getContext('webgl2', { powerPreference: 'high-performance' }) || canvas.getContext('webgl');
                const dbg = gl.getExtension('WEBGL_debug_renderer_info');
                return {
                    vendor: dbg ? gl.getParameter(dbg.UNMASKED_VENDOR_WEBGL) : 'unknown',
                    renderer: dbg ? gl.getParameter(dbg.UNMASKED_RENDERER_WEBGL) : 'unknown'
                };
            })()
            """
            await ws.send(json.dumps({"id": 1, "method": "Runtime.evaluate", "params": {"expression": js, "returnByValue": True}}))
            res = json.loads(await ws.recv())
            val = res.get("result", {}).get("result", {}).get("value")
            print(f"[{name}] ->", val.get("renderer") if val else "NONE")
    except Exception as e:
        print(f"[{name}] ERROR: {e}")
    finally:
        proc.terminate()
        proc.wait()
        shutil.rmtree(temp_p, ignore_errors=True)

async def main():
    await test_flags("Test 1: Baseline", [])
    await test_flags("Test 2: --force-high-performance-gpu", ["--force-high-performance-gpu"])
    await test_flags("Test 3: --gpu-preference=2", ["--gpu-preference=2"])
    await test_flags("Test 4: SHIM_MCCOMPAT", [], {"SHIM_MCCOMPAT": "0x800000001"})
    await test_flags("Test 5: Combined Flags", [
        "--force-high-performance-gpu",
        "--gpu-preference=2",
        "--use-angle=d3d11",
        "--enable-gpu-rasterization"
    ], {"SHIM_MCCOMPAT": "0x800000001"})

if __name__ == "__main__":
    asyncio.run(main())
