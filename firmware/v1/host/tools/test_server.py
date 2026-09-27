#!/usr/bin/env python3
"""End-to-end check of the host tool's HTTP + WebSocket plumbing, without hardware.

Starts server.py on a free port, fetches the page and assets, opens the
WebSocket, checks the hello handshake, that a bad serial port produces an error
message (not a crash), and that pause/status round-trips.

    python3 host/tools/test_server.py
"""

from __future__ import annotations

import asyncio
import json
import os
import math
import socket
import subprocess
import sys
import time
import urllib.request

HERE = os.path.dirname(os.path.abspath(__file__))
HOST_DIR = os.path.dirname(HERE)
SERVER = os.path.join(HOST_DIR, "server.py")
PYTHON = sys.executable or "python3"

FAILURES: list[str] = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(f"  {'ok  ' if ok else 'FAIL'} {name}" + (f"  [{detail}]" if detail else ""))
    if not ok:
        FAILURES.append(name)


def free_port() -> int:
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


async def sample_checks(port: int, device: str, baud: int, protocol: str) -> None:
    """Drive the real service against a real board: connect, then require live
    samples to arrive over the WebSocket with sane decoded values."""
    import aiohttp

    url = f"http://127.0.0.1:{port}"
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(f"{url}/ws", timeout=10) as ws:
            await asyncio.wait_for(ws.receive_str(), 5)          # hello
            await ws.send_json({"op": "connect", "port": device, "baud": baud,
                                "protocol": protocol, "mode": "sync_read",
                                "read_len": 20, "rate": 100})
            samples = []
            stats = None
            deadline = time.monotonic() + 15
            while time.monotonic() < deadline and (len(samples) < 40 or stats is None):
                message = json.loads(await asyncio.wait_for(ws.receive_str(), 5))
                if message.get("type") == "sample":
                    samples.append(message["s"])
                elif message.get("type") == "stats":
                    stats = message
                elif message.get("type") == "error":
                    check("live samples: no error", False, message.get("message", ""))
                    return
            check("live samples arrive", len(samples) >= 40, f"{len(samples)} samples")
            if not samples:
                return
            last = samples[-1]
            needed = ("quat", "gyro_trunk_dps", "gravity", "euler", "accel_mg", "counter", "latency_ms")
            missing = [k for k in needed if k not in last]
            check("sample has every field the frontend uses", not missing, str(missing))
            norm = math.sqrt(sum(v * v for v in last["quat"]))
            check("live quaternion is unit", abs(norm - 1.0) < 5e-3, f"|q|={norm:.5f}")
            gnorm = math.sqrt(sum(v * v for v in last["gravity"]))
            check("live gravity is unit", abs(gnorm - 1.0) < 5e-3, f"|g|={gnorm:.5f}")
            lat = last.get("latency_ms", 0.0)
            check("live latency is sane", 0.0 < lat < 50.0, f"{lat:.2f} ms")
            if stats is not None:
                check("stats message carries counters", "stats" in stats and "achieved_hz" in stats,
                      json.dumps(stats)[:120])
                check("achieved rate is plausible", stats.get("achieved_hz", 0) > 20.0,
                      f"{stats.get('achieved_hz', 0):.1f} Hz")
            await ws.send_json({"op": "disconnect"})


async def ws_checks(port: int) -> None:
    import aiohttp

    url = f"http://127.0.0.1:{port}"
    async with aiohttp.ClientSession() as session:
        async with session.ws_connect(f"{url}/ws", timeout=5) as ws:
            hello = json.loads(await asyncio.wait_for(ws.receive_str(), 5))
            check("ws hello", hello.get("type") == "hello", json.dumps(hello)[:120])
            check("hello advertises no connection", hello.get("connected") is False)

            await ws.send_json({"op": "connect", "port": "/dev/ttyDOESNOTEXIST", "baud": 1000000})
            saw_error = False
            for _ in range(10):
                message = json.loads(await asyncio.wait_for(ws.receive_str(), 5))
                if message.get("type") == "error":
                    saw_error = True
                    break
                if message.get("type") == "status":
                    break
            check("bad port reports an error", saw_error)

            await ws.send_json({"op": "pause", "paused": True})
            saw_status = False
            for _ in range(10):
                message = json.loads(await asyncio.wait_for(ws.receive_str(), 5))
                if message.get("type") == "status":
                    saw_status = True
                    break
            check("pause round-trips as a status message", saw_status)


def http_checks(port: int) -> None:
    base = f"http://127.0.0.1:{port}"
    for path, needle in (("/", "ws"), ("/app.js", "WebSocket"), ("/style.css", "{")):
        try:
            with urllib.request.urlopen(f"{base}{path}", timeout=5) as response:
                body = response.read().decode("utf-8", "replace")
            check(f"GET {path}", response.status == 200 and needle in body,
                  f"{response.status}, {len(body)} bytes")
        except Exception as exc:
            check(f"GET {path}", False, str(exc))
    try:
        with urllib.request.urlopen(f"{base}/api/ports", timeout=5) as response:
            data = json.loads(response.read())
        check("GET /api/ports", isinstance(data.get("ports"), list), str(data)[:120])
    except Exception as exc:
        check("GET /api/ports", False, str(exc))


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--device", default="", help="serial port (enables the live sample test)")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--protocol", choices=["dxl", "fee"], default="dxl")
    args = parser.parse_args()

    port = free_port()
    print(f"server integration test on 127.0.0.1:{port}")
    process = subprocess.Popen(
        [PYTHON, SERVER, "--host", "127.0.0.1", "--http-port", str(port)],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    try:
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{port}/api/ports", timeout=1):
                    break
            except Exception:
                time.sleep(0.2)
        else:
            check("server started", False, "no HTTP answer within 10 s")
            return 1
        check("server started", True)

        http_checks(port)
        asyncio.run(ws_checks(port))
        if args.device:
            asyncio.run(sample_checks(port, args.device, args.baud, args.protocol))
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILURE(S): {', '.join(FAILURES)}")
        return 1
    print("PASS: host tool HTTP + WebSocket plumbing works")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
