#!/usr/bin/env python3
"""Repeatable, trigger-friendly bus traffic for an oscilloscope.

Why this exists: `servo_probe.py` walks a fixed sequence once, `bus_smoke.py` is
a pass/fail test, and `robotd` is a 50 Hz mix of reads and writes whose frames
change every tick. None of them is what you want on a scope, where the useful
thing is *one known frame, repeating at a steady rate, forever*.

Modes (all read-only except `write`, which is gated):

  ping       unicast PING to one id. The smallest frame on this bus: 6 bytes.
  sync       the robot's real tick: one broadcast SYNC_READ of all 16 devices.
  sync-pos   the startup read: SYNC_READ of the 15 joints, 2 bytes each.
  write      broadcast SYNC_WRITE of goal positions, each servo's *own current
             position* echoed back, so the frame is real and nothing moves.
             Needs `--confirm-motion-safe` because it is the only mode that
             writes; it also reads positions first, so it is not a pure burst.
  idle       send nothing. For measuring the idle level and the pull-ups.

Timing cheatsheet, printed at startup too (1 Mbps, 8N1 = 10 bits/byte = 10 us
per byte, line idle high):

  PING request        6 bytes   60 us
  PING reply          6 bytes   60 us, typically ~0.5 ms after the request ends
  SYNC_READ 16 ids   24 bytes  240 us request
  a 15-byte reply    21 bytes  210 us per device
  reply slot                   ~295 us measured, devices answer in id order
  whole 16-device burst        ~4.9 ms from request start to last reply

SAFETY: only `write` touches the bus in a way that could move a servo, and it
commands each servo to where it already is. Everything else is PING / SYNC_READ.
Run it alone: `robotd` and the debugger GUI open the tty with TIOCEXCL, and
TIOCEXCL does not reject a port that is *already* open, so a second process can
silently steal half of every transaction. Check with `fuser /dev/ttyACM0` first.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Dict, List, Optional, Sequence

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
if HOST not in sys.path:
    sys.path.insert(0, HOST)

import bus  # noqa: E402

try:
    import serial  # noqa: E402
except ImportError:  # pragma: no cover - reported in main()
    serial = None

IMU_ID = 200
JOINT_IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]
ALL_IDS = [IMU_ID] + JOINT_IDS

#: The IMU node and the servos both answer the servo status address; on the node
#: the first 12 of the 15 bytes are the SFLP control block.
CONTRACT_ADDR = 56

#: Vendor HLS addresses this tool names directly (飞特通讯协议/HLS系列舵机内存表).
#: `bus.py` keeps the protocol-level names (`FEE_TELEM_ADDR` is the same 56), and
#: the goal-position write is a raw address nothing else in that module needs.
PRESENT_POSITION_L = 56
GOAL_POSITION_L = 42


def encode_signed_magnitude(value: int, sign_bit: int = 15) -> int:
    """The vendor's encoding for every signed feedback/target field.

    Bit `sign_bit` is a direction flag and the rest is the magnitude — not two's
    complement. `bus.py` only needs the decoder (for telemetry); this tool also
    builds a goal-position payload, so it needs the encoder too.
    """
    magnitude = abs(int(value)) & ((1 << sign_bit) - 1)
    return magnitude | (1 << sign_bit) if value < 0 else magnitude

#: A USB-serial adapter of this class drops the first transaction after a
#: close/open cycle. Measured on the bench (2026-09-30): four cold starts in a
#: row lost the ping to the first servo in the census. `host/bus.py` carries the
#: same constant for the same reason.
OPEN_SETTLE = 0.4

#: Measured reply slot on this bus: what one device occupies with its status
#: block, spent even when the addressed device is absent.
SLOT_US = 295


def _banner(port: str, baud: int, mode: str, hz: float) -> None:
    print("=" * 78)
    print("scope traffic  port=%s  baud=%d  1 bit = %.2f us  1 byte = %.1f us"
          % (port, baud, 1e6 / baud, 10e6 / baud))
    print("  mode=%s  repeat at %.1f Hz (%.1f ms per period)"
          % (mode, hz, 1000.0 / hz if hz else 0.0))
    print("-" * 78)
    print("  PING request        6 bytes    60 us")
    print("  PING reply          6 bytes    60 us, ~0.5 ms after the request ends")
    print("  SYNC_READ 16 ids   24 bytes   240 us request")
    print("  SYNC_READ 15 ids   23 bytes   230 us (len 2, the startup read)")
    print("  one 15-byte reply  21 bytes   210 us   (len-2 reply: 8 bytes / 80 us)")
    print("  reply slot                   ~%d us; devices answer in id order" % SLOT_US)
    print("  16-device burst              ~4.9 ms, request start to last reply")
    print("  SYNC_WRITE 15 svos 53 bytes   530 us, broadcast: no reply at all")
    print("-" * 78)
    print("  probe: bus data line + GND at any servo connector (half duplex,")
    print("         line idle high). Trigger on the falling edge of the first")
    print("         start bit. 20 us/div shows two bytes; 500 us/div shows a")
    print("         whole 16-device burst. Measure request-end -> first reply")
    print("         for the turnaround, and reply-to-reply for the slot.")
    print("=" * 78)


class Session(object):
    """One serial port, one request/response pattern, in a loop."""

    def __init__(self, port: str, baud: int, timeout: float = 0.05):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.ser = serial.Serial(port, baud, timeout=0.0, write_timeout=0.2)
        time.sleep(OPEN_SETTLE)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self) -> None:
        try:
            self.ser.close()
        except Exception:
            pass

    def send(self, request: bytes, expected: Sequence[int], budget: float):
        """Write one request, collect replies until `budget` has passed.

        Returns `(frames, latency_ms)` where `frames` is a list of
        `(id, status, data)` in arrival order. Timestamps are taken when the
        chunk carrying the frame arrives, so they are the host's view (chunked
        by USB), not the wire's — good to ~0.2 ms, which is enough to see the
        slot structure but not the bit timing. Use the scope for bit timing.
        """
        expected = set(int(i) for i in expected)
        t0 = time.perf_counter()
        self.ser.reset_input_buffer()
        self.ser.write(request)
        self.ser.flush()
        buf = bytearray()
        frames: List[tuple] = []
        deadline = t0 + budget
        while time.perf_counter() < deadline:
            chunk = self.ser.read(512)
            now = time.perf_counter()
            if not chunk:
                time.sleep(0.0002)
                continue
            buf += chunk
            while len(buf) >= 6:
                if buf[0] != 0xFF or buf[1] != 0xFF:
                    del buf[0]
                    continue
                total = buf[3] + 4
                if len(buf) < total:
                    break
                raw = bytes(buf[:total])
                del buf[:total]
                try:
                    pid, status, data = bus.fee_parse_ack(raw)
                except Exception:
                    continue
                frames.append((pid, status, data, (now - t0) * 1000.0))
        return frames, (time.perf_counter() - t0) * 1000.0


def _summary(frames, total_ms, expected: Sequence[int]) -> str:
    got = {}
    for pid, status, data, ms in frames:
        got.setdefault(pid, (status, len(data), ms))
    missing = [i for i in expected if i not in got]
    late = max((v[2] for v in got.values()), default=0.0)
    parts = ["%d frames" % len(frames), "last reply %.2f ms" % late]
    if missing:
        parts.append("missing %s" % ",".join(str(i) for i in missing))
    else:
        parts.append("all %d answered" % len(expected))
    return "  ".join(parts)


def mode_request(mode: str, args) -> tuple:
    """`(request_bytes, expected_reply_ids, budget_seconds)` for one iteration."""
    if mode == "ping":
        return bus.fee_ping(args.id), [args.id], args.timeout
    if mode == "sync":
        return bus.fee_sync_read(ALL_IDS, CONTRACT_ADDR, 15), ALL_IDS, args.timeout
    if mode == "sync-pos":
        return (bus.fee_sync_read(JOINT_IDS, PRESENT_POSITION_L, 2),
                JOINT_IDS, args.timeout)
    raise ValueError("mode %s has no single request" % mode)


def run_write_mode(sess: Session, args) -> None:
    """SYNC_WRITE each servo to where it already is.

    Read the 15-byte block, then send one broadcast write of goal positions
    built from the positions just read. The frame is the real one `robotd`
    sends every tick; the servos have nothing to do because the target is the
    position they are already holding.
    """
    frames, _ = sess.send(bus.fee_sync_read(JOINT_IDS, CONTRACT_ADDR, 15),
                          JOINT_IDS, args.timeout)
    positions: Dict[int, int] = {}
    for pid, status, data, _ms in frames:
        if len(data) >= 2:
            raw = data[0] | (data[1] << 8)
            magnitude = raw & 0x7FFF
            positions[pid] = -magnitude if raw & 0x8000 else magnitude
    missing = [i for i in JOINT_IDS if i not in positions]
    if missing:
        print("  no position for %s; not writing (would command a servo to 0)"
              % ",".join(str(i) for i in missing))
        return
    payloads = [encode_signed_magnitude(positions[i], 15).to_bytes(2, "little")
                for i in JOINT_IDS]
    request = bus.fee_sync_write(JOINT_IDS, GOAL_POSITION_L, payloads)
    sess.ser.reset_input_buffer()
    sess.ser.write(request)
    sess.ser.flush()
    time.sleep(0.002)
    trailing = sess.ser.read(64)
    print("  wrote %d bytes to %d servos; %d bytes came back (broadcast writes "
          "are unacknowledged, so this should be 0)"
          % (len(request), len(JOINT_IDS), len(trailing)))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--mode", default="ping",
                    choices=["ping", "sync", "sync-pos", "write", "idle"])
    ap.add_argument("--id", type=int, default=JOINT_IDS[0],
                    help="ping mode: which device to address (default 20)")
    ap.add_argument("--hz", type=float, default=20.0,
                    help="repeats per second (0 = as fast as the bus allows)")
    ap.add_argument("--count", type=int, default=0,
                    help="stop after N iterations (0 = forever, Ctrl-C to stop)")
    ap.add_argument("--timeout", type=float, default=0.05,
                    help="seconds to collect replies after each request")
    ap.add_argument("--json", default=None, help="also append per-iteration JSON here")
    ap.add_argument("--confirm-motion-safe", action="store_true",
                    help="required by --mode write; it is the only mode that writes")
    args = ap.parse_args()

    if serial is None:
        print("[!] pyserial is not installed (pip install pyserial)", file=sys.stderr)
        return 2
    if not os.path.exists(args.port):
        print("[!] %s does not exist (a sandboxed shell may have /dev masked)"
              % args.port, file=sys.stderr)
        return 2
    if args.mode == "write" and not args.confirm_motion_safe:
        print("[!] --mode write needs --confirm-motion-safe. It commands every "
              "servo to the position it is already holding, which is still a "
              "write: with torque on, a servo that moved between the read and "
              "the write would be driven back.", file=sys.stderr)
        return 2

    _banner(args.port, args.baud, args.mode, args.hz)

    if args.mode == "idle":
        print("  sending nothing. The line should sit high (UART idle) with the")
        print("  transceivers released; probe it and watch for anything toggling.")
        time.sleep(1.0 if not args.count else 0.0)
        return 0

    try:
        sess = Session(args.port, args.baud)
    except Exception as exc:
        print("[!] cannot open %s: %s" % (args.port, exc), file=sys.stderr)
        return 2

    period = 1.0 / args.hz if args.hz > 0 else 0.0
    jsonl = open(args.json, "a", encoding="utf-8") if args.json else None
    iteration = 0
    try:
        while args.count == 0 or iteration < args.count:
            started = time.perf_counter()
            if args.mode == "write":
                run_write_mode(sess, args)
                line = "  iteration %d: SYNC_WRITE (goal = measured position)" % iteration
            else:
                request, expected, budget = mode_request(args.mode, args)
                frames, total_ms = sess.send(request, expected, budget)
                line = ("  iteration %d: sent %d bytes, %s"
                        % (iteration, len(request), _summary(frames, total_ms, expected)))
            print(line)
            if jsonl:
                jsonl.write(json.dumps({"t": time.time(), "mode": args.mode,
                                        "iteration": iteration,
                                        "report": line.strip()}) + "\n")
                jsonl.flush()
            iteration += 1
            if period:
                slack = period - (time.perf_counter() - started)
                if slack > 0:
                    time.sleep(slack)
    except KeyboardInterrupt:
        print("\n  stopped after %d iterations" % iteration)
    finally:
        sess.close()
        if jsonl:
            jsonl.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
