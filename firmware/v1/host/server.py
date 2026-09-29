#!/usr/bin/env python3
"""Local web host tool for the imu_to_dxl v1 node.

Serves a single-page instrument (web/) on http://127.0.0.1:8081 and talks to the
board over the bench serial link:

    ./build.sh host
    python3 host/server.py --port /dev/ttyACM0 --baud 1000000 --protocol dxl

The browser gets a WebSocket at /ws and receives decoded samples at the poll
rate, a statistics block twice a second, and status/config updates whenever they
change.  The 3D orientation view and the waterfall plots are drawn entirely in
the browser (hand-written WebGL/canvas, no CDN), so the tool works offline.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import queue
import statistics
import sys
import threading
import time
from typing import Any, Optional

from aiohttp import WSMsgType, web

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from bus import PROTO_DXL, PROTO_FEE, SIM_MODE_IDS, Link  # noqa: E402

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")

SIM_MODE_BY_ID = {v: k for k, v in SIM_MODE_IDS.items()}


class Session:
    """Owns the serial link and the poller thread; the aiohttp handlers only
    ever touch it through the lock."""

    def __init__(self) -> None:
        self.lock = threading.RLock()
        self.link: Optional[Link] = None
        self.port = ""
        self.baud = 1_000_000
        self.protocol = PROTO_DXL
        self.mode = "sync_read"
        self.read_len = 20
        self.imu_id = 200
        self.rate = 100.0
        self.paused = False
        self.error = ""
        self.config: dict = {}
        self.device: dict = {}
        self.samples = 0
        self._stamps: list[float] = []
        self.out: "queue.Queue[dict]" = queue.Queue(maxsize=4000)
        self._thread: Optional[threading.Thread] = None
        self._running = False

    # -- lifecycle

    def connect(self, port: str, baud: int, protocol: str, mode: str,
                read_len: int, imu_id: int, rate: float) -> None:
        with self.lock:
            self.disconnect()
            link = Link(port=port, baud=baud, protocol=protocol, imu_id=imu_id,
                        read_len=read_len, mode=mode)
            link.open()
            self.link = link
            self.port, self.baud, self.protocol = port, baud, protocol
            self.mode, self.read_len, self.imu_id, self.rate = mode, read_len, imu_id, rate
            self.error = ""
            self.samples = 0
            self.paused = False
            try:
                self.device = link.device_info()
            except Exception as exc:
                self.device = {"error": str(exc)}
            try:
                self.config = link.get_config()
            except Exception as exc:
                self.config = {"error": str(exc)}
            self._running = True
            self._thread = threading.Thread(target=self._poll_loop, name="poll", daemon=True)
            self._thread.start()

    def disconnect(self) -> None:
        with self.lock:
            self._running = False
            thread, self._thread = self._thread, None
        if thread is not None:
            thread.join(timeout=1.0)
        with self.lock:
            if self.link is not None:
                try:
                    self.link.close()
                except Exception:
                    pass
                self.link = None
            self.config = {}
            self.device = {}
            self._stamps.clear()
            while True:
                try:
                    self.out.get_nowait()
                except queue.Empty:
                    break

    @property
    def connected(self) -> bool:
        return self.link is not None

    # -- polling

    def _poll_loop(self) -> None:
        next_t = time.monotonic()
        while True:
            with self.lock:
                if not self._running:
                    return
                link = self.link
                paused = self.paused
                rate = max(1.0, self.rate)
            if link is None:
                return
            if paused:
                time.sleep(0.05)
                next_t = time.monotonic()
                continue
            try:
                with self.lock:
                    sample = link.poll()
            except Exception as exc:                      # link died
                with self.lock:
                    self.error = str(exc)
                time.sleep(0.2)
                continue
            now = time.monotonic()
            if sample is not None:
                self._stamps.append(now)
                if len(self._stamps) > 1000:
                    del self._stamps[:500]
                self.samples += 1
                self._push({"type": "sample", "s": sample})
            next_t += 1.0 / rate
            sleep = next_t - time.monotonic()
            if sleep > 0.0005:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()

    def achieved_hz(self) -> float:
        now = time.monotonic()
        recent = [s for s in self._stamps if now - s <= 2.0]
        if len(recent) < 2:
            return 0.0
        span = recent[-1] - recent[0]
        return (len(recent) - 1) / span if span > 0 else 0.0

    def _push(self, message: dict) -> None:
        try:
            self.out.put_nowait(message)
        except queue.Full:
            pass

    def status(self) -> dict:
        with self.lock:
            link = self.link
            stats = {}
            latency = {}
            if link is not None:
                stats = {
                    "polls": link.stats.polls,
                    "ok": link.stats.ok,
                    "timeouts": link.stats.timeouts,
                    "bad_frames": link.stats.bad_frames,
                    "extra_bytes": link.stats.extra_bytes,
                    "last_error": link.stats.last_error,
                }
                latency = link.stats.latency_summary()
            return {
                "type": "status",
                "connected": self.connected,
                "paused": self.paused,
                "protocol": self.protocol,
                "link": {
                    "port": self.port,
                    "baud": self.baud,
                    "mode": self.mode,
                    "read_len": self.read_len,
                    "id": self.imu_id,
                    "rate": self.rate,
                },
                "config": self.config,
                "device": self.device,
                "stats": {**stats, "latency": latency},
                "achieved_hz": self.achieved_hz(),
                "samples": self.samples,
                "error": self.error,
            }

    # -- commands from the browser (run outside the event loop)

    def apply_config(self, fields: dict) -> None:
        with self.lock:
            if self.link is None:
                raise RuntimeError("not connected")
            self.link.set_config(**fields)
            self.config = self.link.get_config()

    def command(self, cmd: str) -> None:
        from bus import VCMD_DEFAULTS, VCMD_REBOOT, VCMD_RESET_SIM, VCMD_SAVE

        mapping = {
            "save": VCMD_SAVE,
            "defaults": VCMD_DEFAULTS,
            "reboot": VCMD_REBOOT,
            "reset_sim": VCMD_RESET_SIM,
        }
        if cmd not in mapping:
            raise ValueError(f"unknown command {cmd!r}")
        with self.lock:
            if self.link is None:
                raise RuntimeError("not connected")
            self.link.set_config(cmd=mapping[cmd])


SESSION = Session()
CLIENTS: "set[web.WebSocketResponse]" = set()


def serial_ports() -> list[dict]:
    try:
        from serial.tools import list_ports

        return [
            {"device": p.device, "description": p.description or ""}
            for p in sorted(list_ports.comports(), key=lambda p: p.device)
        ]
    except Exception:
        return []


async def broadcast(message: dict) -> None:
    if not CLIENTS:
        return
    text = json.dumps(message)
    dead = []
    for ws in list(CLIENTS):
        try:
            await ws.send_str(text)
        except Exception:
            dead.append(ws)
    for ws in dead:
        CLIENTS.discard(ws)


async def ws_handler(request: web.Request) -> web.WebSocketResponse:
    ws = web.WebSocketResponse(heartbeat=20, max_msg_size=1 << 20)
    await ws.prepare(request)
    CLIENTS.add(ws)
    await ws.send_json({
        "type": "hello",
        "protocol": SESSION.protocol,
        "config": SESSION.config,
        "device": SESSION.device,
        "link": {"port": SESSION.port, "baud": SESSION.baud, "mode": SESSION.mode,
                 "rate": SESSION.rate, "id": SESSION.imu_id, "read_len": SESSION.read_len},
        "connected": SESSION.connected,
        "ports": serial_ports(),
    })
    try:
        async for msg in ws:
            if msg.type != WSMsgType.TEXT:
                continue
            try:
                data = json.loads(msg.data)
            except Exception:
                continue
            op = data.get("op")
            loop = asyncio.get_running_loop()
            try:
                if op == "connect":
                    await loop.run_in_executor(None, lambda: SESSION.connect(
                        data.get("port") or SESSION.port or "/dev/ttyACM0",
                        int(data.get("baud", 1_000_000)),
                        data.get("protocol", PROTO_DXL),
                        data.get("mode", "sync_read"),
                        int(data.get("read_len", 20)),
                        int(data.get("id", 200)),
                        float(data.get("rate", 100)),
                    ))
                elif op == "disconnect":
                    await loop.run_in_executor(None, SESSION.disconnect)
                elif op == "pause":
                    SESSION.paused = bool(data.get("paused", True))
                elif op == "config":
                    fields = {k: v for k, v in data.items() if k != "op"}
                    if fields:
                        await loop.run_in_executor(None, lambda: SESSION.apply_config(fields))
                elif op == "command":
                    await loop.run_in_executor(None, lambda: SESSION.command(str(data.get("cmd"))))
                else:
                    continue
            except Exception as exc:
                await ws.send_json({"type": "error", "message": f"{op}: {exc}"})
                continue
            await broadcast(SESSION.status())
    finally:
        CLIENTS.discard(ws)
    return ws


def _no_store(resp: web.StreamResponse) -> web.StreamResponse:
    """Mark a static file as never reusable from cache.

    aiohttp's FileResponse sends Last-Modified/ETag but no Cache-Control, so a
    browser falls back to heuristic freshness (10% of the file's age).  A copy of
    `app.js` fetched while the file was a week old therefore stays "fresh" for
    hours, and an edit followed by a plain reload silently keeps running the old
    script -- which looks exactly like "my change did nothing".  This is a local
    bench tool serving a few KB of static text, so re-reading it every load is
    free and removes the whole class of confusion.
    """
    resp.headers["Cache-Control"] = "no-store, must-revalidate"
    return resp


async def index_handler(request: web.Request) -> web.StreamResponse:
    return _no_store(web.FileResponse(os.path.join(WEB_DIR, "index.html")))


async def static_handler(request: web.Request) -> web.StreamResponse:
    name = request.match_info["name"]
    if "/" in name or ".." in name:
        raise web.HTTPNotFound()
    path = os.path.join(WEB_DIR, name)
    if not os.path.isfile(path):
        raise web.HTTPNotFound()
    return _no_store(web.FileResponse(path))


async def ports_handler(request: web.Request) -> web.StreamResponse:
    return web.json_response({"ports": serial_ports()})


async def state_handler(request: web.Request) -> web.StreamResponse:
    return web.json_response(SESSION.status())


async def broadcaster(app: web.Application) -> None:
    """Drain the poller queue into the browsers and push stats twice a second."""
    last_stats = 0.0
    while True:
        sent = 0
        for _ in range(50):
            try:
                message = SESSION.out.get_nowait()
            except queue.Empty:
                break
            await broadcast(message)
            sent += 1
        now = time.monotonic()
        if now - last_stats >= 0.5:
            last_stats = now
            status = SESSION.status()
            await broadcast({
                "type": "stats",
                "stats": status["stats"],
                "achieved_hz": status["achieved_hz"],
                "config": status["config"],
                "connected": status["connected"],
                "paused": status["paused"],
                "error": status["error"],
            })
        await asyncio.sleep(0.002 if sent else 0.02)


async def on_startup(app: web.Application) -> None:
    app["broadcaster"] = asyncio.create_task(broadcaster(app))


async def on_cleanup(app: web.Application) -> None:
    app["broadcaster"].cancel()
    SESSION.disconnect()


def build_app() -> web.Application:
    app = web.Application()
    app.router.add_get("/", index_handler)
    app.router.add_get("/api/ports", ports_handler)
    app.router.add_get("/api/state", state_handler)
    app.router.add_get("/ws", ws_handler)
    app.router.add_get("/{name}", static_handler)
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    return app


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--http-port", type=int, default=8081)
    parser.add_argument("--port", default="", help="serial port (default: none until the UI connects)")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--protocol", choices=[PROTO_DXL, PROTO_FEE], default=PROTO_DXL)
    parser.add_argument("--id", type=int, default=200)
    parser.add_argument("--rate", type=float, default=100.0)
    args = parser.parse_args()

    if args.port:
        try:
            SESSION.connect(args.port, args.baud, args.protocol, "sync_read", 20, args.id, args.rate)
            print(f"connected: {args.port} @ {args.baud} ({args.protocol})")
        except Exception as exc:
            print(f"could not open {args.port}: {exc}", file=sys.stderr)

    print(f"host tool on http://{args.host}:{args.http_port}  (web dir: {WEB_DIR})")
    web.run_app(build_app(), host=args.host, port=args.http_port, print=None)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
