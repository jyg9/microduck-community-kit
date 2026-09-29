#!/usr/bin/env python3
"""Read-only probe for a real FeeTech HLS/SCS servo on the bench.

Purpose (see docs/bus_timing_borrow_plan.md): settle the questions that decide
this node's bus contract, using a real servo instead of guesses:

  * the real 56..70 block layout and whether a 20-byte read at 56 is accepted,
  * the register ledger (version / END / model / ID / baud / mode / lock ...),
  * reply timing and arrival order for READ / PING / SYNC_READ, including a
    SYNC_READ whose ID list puts an *absent* device first (the runtime case:
    IMU id 200 is first, then the servos),
  * whether a bad checksum is answered at all.

SAFETY: this script never writes. It only sends PING, READ and SYNC_READ frames
(none of which change servo state), plus one deliberately corrupt frame to see
whether it is answered. No WRITE / REG_WRITE / ACTION / RESET / CAL / torque /
motion traffic of any kind.

Usage:
    host/tools/py.sh host/tools/servo_probe.py --port /dev/ttyACM0
    ... --port /dev/ttyACM0 --baud 1000000 --id 1 --repeat 15 --json out.json
"""

from __future__ import annotations

import argparse
import json
import os
import statistics
import sys
import time
from typing import Dict, List, Optional, Sequence, Tuple

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
if HOST not in sys.path:
    sys.path.insert(0, HOST)

import bus  # noqa: E402  (our own reference codec: /host/bus.py)

try:
    import serial  # type: ignore
except Exception:  # pragma: no cover
    serial = None  # type: ignore

FEE_BROADCAST = 0xFE
FEE_INST_PING = 0x01
FEE_INST_READ = 0x02
FEE_INST_SYNC_READ = 0x82

# HLS memory table fields worth naming in the ledger dump (from the vendor
# manual / 飞特通讯协议说明.md §5; page 0..86 of the map).
LEDGER = [
    (0, 2, "firmware version"),
    (2, 1, "END byte order"),
    (3, 2, "servo version/model"),
    (5, 1, "ID"),
    (6, 1, "baud code"),
    (7, 1, "secondary ID"),
    (8, 1, "ACK status level"),
    (9, 2, "min angle limit"),
    (11, 2, "max angle limit"),
    (15, 1, "min input voltage"),
    (16, 2, "max torque"),
    (19, 1, "unload condition"),
    (20, 1, "LED alarm"),
    (21, 1, "P (EEPROM)"),
    (22, 1, "D (EEPROM)"),
    (23, 1, "I (EEPROM)"),
    (26, 1, "CW dead band"),
    (27, 1, "CCW dead band"),
    (28, 1, "protection current"),
    (30, 1, "angle resolution"),
    (31, 2, "OFS (mid offset)"),
    (33, 1, "running mode"),
    (34, 1, "protection torque"),
    (35, 1, "protection time"),
    (36, 1, "overload torque"),
    (37, 1, "current loop P"),
    (38, 1, "current loop I"),
    (39, 1, "speed loop P"),
    (40, 1, "torque enable"),
    (41, 1, "acceleration"),
    (42, 2, "goal position"),
    (44, 2, "goal current"),
    (46, 2, "goal speed"),
    (48, 2, "torque limit"),
    (50, 1, "Kp (RAM)"),
    (51, 1, "Kd (RAM)"),
    (52, 1, "Ki (RAM)"),
    (55, 1, "lock"),
    (56, 2, "present position"),
    (58, 2, "present speed"),
    (60, 2, "present load"),
    (62, 1, "present voltage (0.1 V)"),
    (63, 1, "present temperature (C)"),
    (64, 1, "reg 64 (torque/status area)"),
    (65, 1, "status"),
    (66, 1, "moving"),
    (67, 2, "reg 67..68"),
    (69, 2, "present current (6.5 mA)"),
    (71, 2, "reg 71..72"),
    (73, 2, "reg 73..74"),
    (75, 1, "reg 75"),
    (76, 1, "reg 76"),
]


class Timeout(Exception):
    pass


class BadFrame(Exception):
    pass


class Port:
    """Minimal SCS framing on top of pyserial, with timestamps."""

    def __init__(self, port: str, baud: int, timeout: float):
        if serial is None:
            raise SystemExit("pyserial is required (host/tools/py.sh picks the venv)")
        self.ser = serial.Serial(port, baud, timeout=0.002, write_timeout=0.5)
        self.timeout = timeout

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    # ── low level ───────────────────────────────────────────────────────────

    def drain(self) -> int:
        n = self.ser.in_waiting
        if n:
            self.ser.read(n)
        self.ser.reset_input_buffer()
        return n

    def write(self, frame: bytes) -> float:
        self.drain()
        t0 = time.monotonic()
        self.ser.write(frame)
        self.ser.flush()
        return t0

    def read_frame(self, deadline: float) -> Tuple[int, int, bytes, bytes, float]:
        """Return (id, status, data, raw, t_first_byte)."""
        state = 0
        buf = bytearray()
        t_first = 0.0
        while time.monotonic() < deadline:
            chunk = self.ser.read(1)
            if not chunk:
                continue
            b = chunk[0]
            if t_first == 0.0:
                t_first = time.monotonic()
            if state == 0:
                state = 1 if b == 0xFF else 0
                if state == 0:
                    t_first = 0.0
            elif state == 1:
                if b == 0xFF:
                    state = 2
                else:
                    state = 0
                    t_first = 0.0
            elif state == 2:
                buf = bytearray([b])
                state = 3
            elif state == 3:
                if b < 2:
                    state = 0
                    buf = bytearray()
                    t_first = 0.0
                    continue
                buf.append(b)
                state = 4
            else:
                buf.append(b)
                if len(buf) == buf[1] + 2:
                    want = (~sum(buf[:-1])) & 0xFF
                    if want != buf[-1]:
                        raise BadFrame(bytes(buf))
                    # buf = [ID, LEN, STATUS, data..., SUM]
                    return buf[0], buf[2], bytes(buf[3:-1]), bytes(buf), t_first
        raise Timeout()

    # ── transactions ────────────────────────────────────────────────────────

    def ping(self, sid: int, timeout: Optional[float] = None):
        return self.expect_one(bus.fee_ping(sid), sid, 0, timeout)

    def read(self, sid: int, addr: int, length: int, timeout: Optional[float] = None):
        return self.expect_one(bus.fee_read(sid, addr, length), sid, length, timeout)

    def expect_one(self, frame: bytes, sid: int, want_len: int, timeout=None):
        to = self.timeout if timeout is None else timeout
        t0 = self.write(frame)
        deadline = time.monotonic() + to
        try:
            rid, status, data, raw, t_first = self.read_frame(deadline)
        except Timeout:
            return {"ok": False, "t_total_ms": (time.monotonic() - t0) * 1e3}
        except BadFrame as exc:
            return {"ok": False, "bad_frame": exc.args[0].hex(),
                    "t_total_ms": (time.monotonic() - t0) * 1e3}
        return {
            "ok": (rid == sid and len(data) == want_len),
            "id": rid,
            "status": status,
            "data": data,
            "raw": raw,
            "len": len(data),
            "t_first_ms": (t_first - t0) * 1e3,
            "t_total_ms": (time.monotonic() - t0) * 1e3,
        }

    def sync_read(self, ids: Sequence[int], addr: int, length: int, timeout: Optional[float] = None,
                  sid: int = 0):
        frame = bus.fee_sync_read(ids, addr, length)
        to = self.timeout if timeout is None else timeout
        t0 = self.write(frame)
        deadline = time.monotonic() + to
        got: List[dict] = []
        while time.monotonic() < deadline:
            try:
                rid, status, data, raw, t_first = self.read_frame(deadline)
            except Timeout:
                break
            except BadFrame as exc:
                got.append({"bad_frame": exc.args[0].hex()})
                continue
            got.append({
                "id": rid,
                "status": status,
                "len": len(data),
                "t_first_ms": (t_first - t0) * 1e3,
                "t_end_ms": (time.monotonic() - t0) * 1e3,
                "data": data.hex(),
            })
        answered = [g["id"] for g in got if "id" in g]
        return {
            "ok": answered == list(ids),
            "answered": answered,
            "missing": [i for i in ids if i not in answered],
            "servo": next((g for g in got if g.get("id") == sid), None),
            "replies": got,
        }


def summarize(name: str, samples: List[dict], key: str) -> dict:
    vals = [s[key] for s in samples if s.get(key) is not None]
    if not vals:
        return {"case": name, "n": 0}
    return {
        "case": name,
        "n": len(vals),
        "median_ms": round(statistics.median(vals), 3),
        "min_ms": round(min(vals), 3),
        "max_ms": round(max(vals), 3),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--id", type=int, default=1)
    ap.add_argument("--repeat", type=int, default=15)
    ap.add_argument("--timeout", type=float, default=0.05)
    ap.add_argument("--json", default=None, help="also write the raw report here")
    args = ap.parse_args()

    if not os.path.exists(args.port):
        print("[!] %s does not exist (in a sandboxed shell /dev is masked - run with "
              "full access)" % args.port)
        return 2

    sid = args.id
    report: Dict[str, object] = {"port": args.port, "baud": args.baud, "id": sid,
                                 "when": time.strftime("%Y-%m-%d %H:%M:%S")}
    p = Port(args.port, args.baud, args.timeout)
    try:
        print("=" * 72)
        print("servo probe (READ-ONLY)  %s @ %d  id=%d  %s"
              % (args.port, args.baud, sid, report["when"]))
        print("=" * 72)

        # 1 ── identity -------------------------------------------------------
        print("\n1. identity (READ 0..8)")
        ids: Dict[str, object] = {}
        for addr, ln, what in LEDGER[:7]:
            r = p.read(sid, addr, ln)
            if r["ok"]:
                val = int.from_bytes(r["data"], "little")
                ids[addr] = val
                print("   %-4d %-22s len=%d  = %s  (0x%s)  status=%d"
                      % (addr, what, ln, val, r["data"][::-1].hex().upper(), r["status"]))
            else:
                print("   %-4d %-22s FAILED %s" % (addr, what, r))
        report["identity"] = ids
        fw = None
        if 0 in ids:
            v = int(ids[0])
            fw = "%d.%d" % (v & 0xFF, (v >> 8) & 0xFF)
        print("   -> firmware %s, END byte order %s, version/model word %s"
              % (fw, ids.get(2), ids.get(3)))

        # 2 ── the block at 56: 12 / 15 / 20 bytes ----------------------------
        print("\n2. block at address 56, three requested lengths")
        blocks: Dict[str, object] = {}
        for ln in (12, 15, 20, 21):
            r = p.read(sid, 56, ln)
            blocks[str(ln)] = {"ok": r["ok"], "status": r.get("status"), "len": r.get("len"),
                               "data": r.get("data", b"").hex()}
            if r["ok"]:
                print("   len=%-3d OK   status=%d  data=%s" % (ln, r["status"], r["data"].hex()))
            else:
                print("   len=%-3d refused/timeout  status=%s  (waited %.2f ms)"
                      % (ln, r.get("status"), r.get("t_total_ms", 0.0)))
        report["block_at_56"] = blocks

        # 3 ── ledger dump 0..90 ---------------------------------------------
        print("\n3. ledger dump 0..90 (one READ per row, read-only)")
        ledger: Dict[str, object] = {}
        for base in range(0, 91, 6):
            ln = min(6, 91 - base)
            r = p.read(sid, base, ln)
            if r["ok"]:
                ledger[str(base)] = r["data"].hex()
        for addr, ln, what in LEDGER:
            key = str(addr - addr % 6)
            raw = ledger.get(key)
            if raw is None:
                continue
            off = addr - (addr - addr % 6)
            chunk = bytes.fromhex(raw)[off:off + ln]
            if len(chunk) == ln:
                print("   %-3d %-28s = %-6s (0x%s)"
                      % (addr, what, int.from_bytes(chunk, "little"), chunk[::-1].hex().upper()))
        report["ledger"] = ledger

        # 4 ── timing / ordering ---------------------------------------------
        print("\n4. reply timing (median of %d, wall clock incl. USB adapter)" % args.repeat)
        cases: Dict[str, list] = {}

        cases["PING"] = []
        cases["READ 56/15"] = []
        cases["SYNC_READ [1]"] = []
        cases["SYNC_READ [7,1] absent first"] = []
        cases["SYNC_READ [1,7] absent last"] = []
        cases["SYNC_READ [200,1] runtime layout"] = []
        cases["SYNC_READ [7] absent only"] = []
        cases["SYNC_READ [1] len20"] = []
        cases["SYNC_READ [7,8,1] two absent first"] = []
        cases["SYNC_READ [7,8,9,1] three absent first"] = []
        parts: Dict[str, list] = {k: [] for k in cases}

        def take(name, r):
            """Record the servo's own reply; also record which ids were missing."""
            srv = r.get("servo")
            if srv is None and r.get("ok") and "t_total_ms" in r:
                srv = {"id": r.get("id"), "t_first_ms": r.get("t_first_ms"),
                       "t_end_ms": r.get("t_total_ms")}
            if srv is not None:
                cases[name].append(srv)
            parts[name].append({"answered": r.get("answered"), "missing": r.get("missing")})

        for _ in range(args.repeat):
            take("PING", p.ping(sid))
            take("READ 56/15", p.read(sid, 56, 15))
            take("SYNC_READ [1]", p.sync_read([sid], 56, 15, sid=sid))
            take("SYNC_READ [7,1] absent first", p.sync_read([7, sid], 56, 15, sid=sid))
            take("SYNC_READ [1,7] absent last", p.sync_read([sid, 7], 56, 15, sid=sid))
            take("SYNC_READ [200,1] runtime layout", p.sync_read([200, sid], 56, 15, sid=sid))
            take("SYNC_READ [7] absent only", p.sync_read([7], 56, 15, sid=sid))
            take("SYNC_READ [1] len20", p.sync_read([sid], 56, 20, sid=sid))
            take("SYNC_READ [7,8,1] two absent first", p.sync_read([7, 8, sid], 56, 15, sid=sid))
            take("SYNC_READ [7,8,9,1] three absent first",
                 p.sync_read([7, 8, 9, sid], 56, 15, sid=sid))

        timing = []
        for name, samples in cases.items():
            if not samples:
                print("   %-38s no successful sample" % name)
                timing.append({"case": name, "n": 0})
                continue
            row = summarize(name, samples, "t_end_ms" if "t_end_ms" in samples[0] else "t_total_ms")
            first = summarize(name, samples, "t_first_ms")
            row["first_median_ms"] = first.get("median_ms")
            row["first_min_ms"] = first.get("min_ms")
            row["first_max_ms"] = first.get("max_ms")
            seen = sorted({tuple(x["answered"] or []) for x in parts[name]})
            row["answered_seen"] = [list(x) for x in seen]
            print("   %-38s n=%-3d first byte med %6.3f ms (min %6.3f)  full med %6.3f ms  answered=%s"
                  % (name, row["n"], row.get("first_median_ms") or -1,
                     row.get("first_min_ms") or -1, row["median_ms"],
                     [list(x) for x in seen]))
            timing.append(row)
        report["timing"] = timing

        # 5 ── arrival order inside a multi-reply burst ----------------------
        print("\n5. arrival order (SYNC_READ [1,7]: does 1 wait for the absent 7?)")
        orders = []
        for _ in range(5):
            r = p.sync_read([sid, 7], 56, 15, sid=sid)
            orders.append([x.get("id") for x in r["replies"]])
        print("   reply id order per run: %s" % orders)
        report["order_1_7"] = orders

        # 6 ── bad checksum: answered or not? --------------------------------
        print("\n6. corrupt frame (bad checksum) - answered?")
        frame = bytearray(bus.fee_read(sid, 56, 15))
        frame[-1] ^= 0xFF
        r = p.expect_one(bytes(frame), sid, 15, timeout=0.05)
        print("   answered=%s  %s" % (r["ok"], "" if not r["ok"] else r))
        report["bad_checksum_answered"] = bool(r["ok"])

    finally:
        p.close()

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2, ensure_ascii=False)
        print("\nraw report -> %s" % args.json)
    return 0


if __name__ == "__main__":
    sys.exit(main())
