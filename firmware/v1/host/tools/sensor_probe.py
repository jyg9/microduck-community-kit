#!/usr/bin/env python3
"""Read-only characterisation probe for the LSM6DSV16X IMU node on the bench.

Where `bus_smoke.py` answers "does the node still speak both protocols", this
answers "is the real sensor behaving like a real sensor".  It samples the
20-byte telemetry block and reports:

  * identity: model / firmware registers, the telemetry address in use, and the
    block's status flags decoded by name;
  * per-axis gyro (dps) and accelerometer (mg) statistics over N samples:
    mean / std / min / max;
  * quaternion norm statistics and the angle between the accelerometer's
    specific-force direction and the gravity direction projected from the
    quaternion (the two independent measurements of "which way is down");
  * the effective sample rate and counter continuity, i.e. the fraction of the
    samples the node produced that the host actually saw;
  * the static gyro offset in raw counts - the number a user would write into
    the `gyro_bias` vendor register;
  * an SFLP restart check through the `VCMD_RESET_SIM` vendor command, with the
    measured convergence time;
  * a PASS/FAIL summary using the same tolerances as the smoke test.

SAFETY: every transaction here is a READ except the SFLP restart, which is the
documented `VCMD_RESET_SIM` vendor command (a read-modify-write of the 20-byte
vendor window at DXL 148 / FeeTech 160) plus the `imu_restart()` it triggers.
No servo traffic, no torque, no motion, nothing persistent.

Usage:
    host/tools/py.sh host/tools/sensor_probe.py --port /dev/ttyACM0
    ... --port /dev/ttyACM0 --protocol fee --samples 500 --json probe.json
"""

from __future__ import annotations

import argparse
import json
import math
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

# Same tolerances as host/tools/bus_smoke.py.
ACCEL_MG_MIN = 900.0
ACCEL_MG_MAX = 1100.0
QUAT_NORM_MIN = 0.99
QUAT_NORM_MAX = 1.01
GRAVITY_MAX_DEG = 5.0
GYRO_STD_MAX_DPS = 3.0
RESTART_TIMEOUT_S = 2.5

GYRO_MDPS_PER_LSB = bus.GYRO_MDPS_PER_LSB

FLAG_SFLP_VALID = bus.TELEM_FLAG_BITS["sflp_valid"]
FLAG_FUSION_OK = bus.TELEM_FLAG_BITS["fusion_ok"]
FLAG_SENSOR_ERR = bus.TELEM_FLAG_BITS["sensor_err"]
FLAG_SIMULATED = bus.TELEM_FLAG_BITS["simulated"]


def read_block(link: bus.Link) -> bus.ImuBlock:
    raw = link.read_registers(link.telemetry_addr, bus.TELEM_LEN)
    return bus.ImuBlock.decode(raw, link.protocol)


def stats(values: Sequence[float], ndigits: int = 3) -> dict:
    if not values:
        return {"n": 0}
    return {
        "n": len(values),
        "mean": round(statistics.fmean(values), ndigits),
        "std": round(statistics.pstdev(values), ndigits) if len(values) > 1 else 0.0,
        "min": round(min(values), ndigits),
        "max": round(max(values), ndigits),
    }


class Report:
    """Collects PASS/FAIL lines so the JSON and the console agree."""

    def __init__(self) -> None:
        self.checks: List[dict] = []

    def check(self, name: str, ok: bool, detail: str = "") -> None:
        self.checks.append({"check": name, "ok": bool(ok), "detail": detail})
        mark = "ok  " if ok else "FAIL"
        print(f"   {mark} {name}" + (f"  [{detail}]" if detail else ""))

    def passed(self) -> bool:
        return bool(self.checks) and all(c["ok"] for c in self.checks)


def collect(link: bus.Link, want: int, seconds: float) -> List[Tuple[float, bus.ImuBlock]]:
    """Timestamped blocks, stopping at `want` unique samples or `seconds`."""
    out: List[Tuple[float, bus.ImuBlock]] = []
    unique = 0
    last: Optional[int] = None
    deadline = time.monotonic() + seconds
    while unique < want and time.monotonic() < deadline:
        try:
            block = read_block(link)
        except Exception:
            continue
        out.append((time.monotonic(), block))
        if block.counter != last:
            unique += 1
            last = block.counter
    return out


def counter_continuity(
    samples: List[Tuple[float, bus.ImuBlock]],
) -> dict:
    """Effective rate and how much of the node's output the host saw.

    `counter` advances once per published 100 Hz sample, so the sum of the
    deltas is the number of samples the node produced while we watched, and the
    number of unique counters we saw is the fraction of that we received.
    """
    if len(samples) < 2:
        return {"n": len(samples)}
    span = samples[-1][0] - samples[0][0]
    counters = [b.counter for _t, b in samples]
    deltas = [((b - a) & 0xFF) for a, b in zip(counters, counters[1:])]
    # `counter` is a u8 that wraps every 2.56 s, so count transitions rather
    # than distinct values, and include the first sample in what the node
    # produced across the window.
    produced = 1 + sum(deltas)
    unique = 1 + sum(1 for d in deltas if d != 0)
    stale = sum(1 for d in deltas if d == 0)
    return {
        "n_polls": len(samples),
        "span_s": round(span, 3),
        "poll_hz": round((len(samples) - 1) / span, 1) if span > 0 else 0.0,
        "effective_hz": round((unique - 1) / span, 1) if span > 0 else 0.0,
        "unique_samples": unique,
        "node_samples_produced": produced,
        "seen_fraction": round(unique / produced, 4) if produced else 0.0,
        "max_counter_delta": max(deltas) if deltas else 0,
        "gaps_over_3": sum(1 for d in deltas if d > 3),
        # What microduck's SflpDecoder calls a "stale read": the block came back
        # with the counter it already had, i.e. the node had not produced a new
        # sample between two polls.
        "stale_reads": stale,
        "stale_ratio": round(stale / len(deltas), 4) if deltas else 0.0,
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1_000_000)
    ap.add_argument("--id", type=int, default=200, help="IMU node id (both protocols)")
    ap.add_argument("--protocol", choices=["dxl", "fee"], default="dxl")
    ap.add_argument("--samples", type=int, default=300, help="unique samples to characterise")
    ap.add_argument("--seconds", type=float, default=6.0, help="hard time cap for the sample run")
    ap.add_argument("--timeout", type=float, default=0.1)
    ap.add_argument("--json", default=None, help="also write the raw report here")
    args = ap.parse_args()

    if not os.path.exists(args.port):
        print(f"[!] {args.port} does not exist (in a sandboxed shell /dev is masked - "
              "run with full access)")
        return 2

    report: Dict[str, object] = {
        "port": args.port,
        "baud": args.baud,
        "id": args.id,
        "protocol": args.protocol,
        "when": time.strftime("%Y-%m-%d %H:%M:%S"),
    }
    checks = Report()
    link = bus.Link(
        port=args.port,
        baud=args.baud,
        protocol=args.protocol,
        imu_id=args.id,
        read_len=20,
        mode="sync_read",
        timeout=args.timeout,
    )
    try:
        link.open()
        print("=" * 72)
        print("IMU sensor probe  %s @ %d  id=%d proto=%s  %s"
              % (args.port, args.baud, args.id, args.protocol, report["when"]))
        print("=" * 72)

        # 1 ── identity ------------------------------------------------------
        print("\n1. identity")
        info = None
        last_exc: Optional[Exception] = None
        for _ in range(5):
            try:
                info = link.device_info()
                break
            except Exception as exc:
                last_exc = exc
                time.sleep(0.2)
        if info is None:
            print(f"   identity FAILED: {last_exc}")
            report["identity"] = {"error": str(last_exc)}
        else:
            report["identity"] = info
            print("   model=%s firmware=%s (%s) raw=%s"
                  % (info["model"], info["firmware"], info["firmware_str"],
                     [hex(b) for b in info["raw"]]))

        block = None
        last_exc = None
        for _ in range(10):
            try:
                block = read_block(link)
                break
            except Exception as exc:
                last_exc = exc
                time.sleep(0.2)
        if block is None:
            print(f"   telemetry block FAILED: {last_exc}")
            report["error"] = f"no telemetry block: {last_exc}"
            return 1
        flags = block.flags
        report["telemetry_addr"] = link.telemetry_addr
        report["diagnostic_len"] = bus.TELEM_LEN
        report["flags"] = {"byte": flags, "names": block.flag_names()}
        print("   telemetry block: %d bytes at %d (contract block at %d)"
              % (bus.TELEM_LEN, link.telemetry_addr, link.contract_addr))
        print("   flags = 0x%02X  %s" % (flags, ", ".join(block.flag_names()) or "none"))
        checks.check(
            "sensor is real (no TELEM_FLAG_SIMULATED)",
            not (flags & FLAG_SIMULATED),
            "flags=0x%02X" % flags,
        )
        checks.check(
            "sflp_valid + fusion_ok set, sensor_err clear",
            bool(flags & FLAG_SFLP_VALID) and bool(flags & FLAG_FUSION_OK)
            and not (flags & FLAG_SENSOR_ERR),
            "flags=0x%02X" % flags,
        )

        # 2 ── N samples -----------------------------------------------------
        print(f"\n2. sampling (up to {args.samples} unique blocks / {args.seconds:.0f} s)")
        samples = collect(link, args.samples, args.seconds)
        if len(samples) < 5:
            print(f"   only {len(samples)} samples - is the node answering?")
            report["error"] = f"only {len(samples)} samples"
            return 1
        # De-duplicate repeated polls of the same 100 Hz sample: the node
        # refreshes its block at 100 Hz and the host polls faster, so raw poll
        # statistics would weight slow samples more heavily than fast ones.
        unique: List[bus.ImuBlock] = []
        last_counter: Optional[int] = None
        stale_polls = 0
        for _t, block in samples:
            if block.counter == last_counter:
                stale_polls += 1
                continue
            unique.append(block)
            last_counter = block.counter
        print(f"   {len(samples)} polls, {len(unique)} unique samples, "
              f"{samples[-1][0] - samples[0][0]:.2f} s window, "
              f"{stale_polls} stale reads (same counter as the previous poll)")

        gyro_dps: List[List[float]] = [[], [], []]
        gyro_raw: List[List[float]] = [[], [], []]
        accel_mg: List[List[float]] = [[], [], []]
        accel_mag: List[float] = []
        norms: List[float] = []
        angles: List[float] = []
        valid = 0
        for block in unique:
            if block.quat_half != (0, 0, 0):
                valid += 1
            q = block.quat_raw
            norms.append(math.sqrt(sum(c * c for c in q)))
            for axis in range(3):
                gyro_dps[axis].append(block.gyro_dps[axis])
                gyro_raw[axis].append(float(block.gyro_raw[axis]))
                accel_mg[axis].append(block.accel_mg[axis])
            mag = math.sqrt(sum(a * a for a in block.accel_mg))
            accel_mag.append(mag)
            if mag > 1e-6:
                accel_n = tuple(a / mag for a in block.accel_mg)
                # specific force = -gravity; gravity is the world down direction
                # projected into the chip frame by the quaternion.
                gravity = bus.q_normalize(bus.q_rotate_inv(q, (0.0, 0.0, -1.0)))
                dot = sum(a * -g for a, g in zip(accel_n, gravity))
                angles.append(math.degrees(math.acos(max(-1.0, min(1.0, dot)))))

        report["gyro_dps"] = {a: stats(gyro_dps[i]) for i, a in enumerate("xyz")}
        report["gyro_raw"] = {a: stats(gyro_raw[i], 2) for i, a in enumerate("xyz")}
        report["accel_mg"] = {a: stats(accel_mg[i], 2) for i, a in enumerate("xyz")}
        report["accel_magnitude_mg"] = stats(accel_mag, 2)
        report["quat_norm"] = stats(norms, 5)
        report["quat_valid_samples"] = {"valid": valid, "total": len(unique)}
        report["accel_vs_quat_gravity_deg"] = stats(angles, 3)

        print("\n3. gyro (dps) per axis")
        for i, axis in enumerate("xyz"):
            s = report["gyro_dps"][axis]  # type: ignore[index]
            print("   %s  mean %+8.3f  std %6.3f  min %+8.3f  max %+8.3f"
                  % (axis, s["mean"], s["std"], s["min"], s["max"]))
        print("\n4. accelerometer (mg) per axis")
        for i, axis in enumerate("xyz"):
            s = report["accel_mg"][axis]  # type: ignore[index]
            print("   %s  mean %+9.2f  std %7.3f  min %+9.2f  max %+9.2f"
                  % (axis, s["mean"], s["std"], s["min"], s["max"]))
        mag_s = report["accel_magnitude_mg"]  # type: ignore[assignment]
        print("   |a| mean %.2f mg  (%.0f..%.0f mg), std %.3f"
              % (mag_s["mean"], mag_s["min"], mag_s["max"], mag_s["std"]))
        print("\n5. attitude")
        n_s = report["quat_norm"]  # type: ignore[assignment]
        print("   |q| mean %.5f  std %.5f  (%.5f..%.5f)"
              % (n_s["mean"], n_s["std"], n_s["min"], n_s["max"]))
        print("   quaternion non-zero in %d/%d samples" % (valid, len(unique)))
        a_s = report["accel_vs_quat_gravity_deg"]  # type: ignore[assignment]
        print("   accel vs quaternion gravity: mean %.3f deg  max %.3f deg (n=%d)"
              % (a_s["mean"], a_s["max"], a_s["n"]))

        # 6 ── rate / continuity --------------------------------------------
        cont = counter_continuity(samples)
        report["counter_continuity"] = cont
        print("\n6. sample rate and counter continuity")
        print("   effective %.1f Hz of the node's 100 Hz, host polled %.1f Hz; "
              "node produced %d samples, %.1f%% seen"
              % (cont["effective_hz"], cont["poll_hz"],
                 cont["node_samples_produced"], 100.0 * cont["seen_fraction"]))
        print("   max counter delta %d, gaps >3: %d, stale reads: %d (%.2f%% of polls)"
              % (cont["max_counter_delta"], cont["gaps_over_3"],
                 cont["stale_reads"], 100.0 * cont["stale_ratio"]))

        # 7 ── gyro offset in raw counts -------------------------------------
        bias_raw = [int(round(-statistics.fmean(gyro_raw[i]))) for i in range(3)]
        report["gyro_bias_raw_counts"] = bias_raw
        print("\n7. static gyro offset (the value for the gyro_bias vendor register)")
        print("   measured mean:  x %+.2f  y %+.2f  z %+.2f raw counts"
              % tuple(statistics.fmean(gyro_raw[i]) for i in range(3)))
        print("   -> write gyro_bias = %s  (vendor window +%d, i16 LE each)"
              % (tuple(bias_raw), bus.VOFF["bias_x_l"]))
        print("   the firmware ADDS this to the chip's counts, hence the negation")

        # 8 ── SFLP restart --------------------------------------------------
        print("\n8. SFLP restart (VCMD_RESET_SIM)")
        restart: Dict[str, object] = {}
        try:
            t0 = time.monotonic()
            link.set_config(cmd=bus.VCMD_RESET_SIM)
            restart["command_accepted"] = True
            zero_at: Optional[float] = None
            first_nonzero: Optional[float] = None
            recover_at: Optional[float] = None
            while time.monotonic() - t0 < RESTART_TIMEOUT_S:
                try:
                    b = read_block(link)
                except Exception:
                    continue
                dt = time.monotonic() - t0
                if b.quat_half == (0, 0, 0):
                    if zero_at is None:
                        zero_at = dt
                else:
                    if first_nonzero is None:
                        first_nonzero = dt
                    if zero_at is not None and recover_at is None:
                        recover_at = dt
                time.sleep(0.002)   # the block refreshes at 100 Hz
            restart.update({
                "zero_seen": zero_at is not None,
                "zero_at_s": round(zero_at, 4) if zero_at is not None else None,
                "first_nonzero_s": round(first_nonzero, 4) if first_nonzero is not None else None,
                "recover_s": round(recover_at, 4) if recover_at is not None else None,
                "timeout_s": RESTART_TIMEOUT_S,
            })
            print("   command accepted; zero window: %s; first non-zero %.0f ms after; "
                  "convergence %s"
                  % ("yes at %.0f ms" % (zero_at * 1000.0) if zero_at is not None else "NONE",
                     (first_nonzero * 1000.0) if first_nonzero is not None else -1.0,
                     ("%.2f s" % recover_at) if recover_at is not None else "not measured"))
        except Exception as exc:
            restart["error"] = str(exc)
            print(f"   restart FAILED: {exc}")
        report["restart"] = restart

        # 9 ── summary -------------------------------------------------------
        print("\n9. summary (same tolerances as the smoke test)")
        checks.check(
            "quat norm %.4f in %.2f-%.2f" % (n_s["mean"], QUAT_NORM_MIN, QUAT_NORM_MAX),
            all(QUAT_NORM_MIN <= v <= QUAT_NORM_MAX for v in norms),
            "n=%d" % len(norms),
        )
        checks.check(
            "quaternion non-zero in some sample",
            valid > 0,
            "%d/%d" % (valid, len(unique)),
        )
        checks.check(
            "accel magnitude %.0f mg in %.0f-%.0f mg"
            % (mag_s["mean"], ACCEL_MG_MIN, ACCEL_MG_MAX),
            all(ACCEL_MG_MIN <= v <= ACCEL_MG_MAX for v in accel_mag),
            "n=%d" % len(accel_mag),
        )
        checks.check(
            "quat-gravity vs accel %.2f deg mean, %.2f max (< %.0f deg)"
            % (a_s["mean"], a_s["max"], GRAVITY_MAX_DEG) if angles else
            "quat-gravity vs accel (no samples)",
            bool(angles) and a_s["mean"] < GRAVITY_MAX_DEG and a_s["max"] < GRAVITY_MAX_DEG,
            "n=%d" % len(angles),
        )
        worst_std = max((report["gyro_dps"][a]["std"] for a in "xyz"), default=float("inf"))
        checks.check(
            "gyro noise std %.3f dps (< %.0f dps)" % (worst_std, GYRO_STD_MAX_DPS),
            worst_std < GYRO_STD_MAX_DPS,
            "mean x=%+.2f y=%+.2f z=%+.2f dps (offset is expected)"
            % tuple(report["gyro_dps"][a]["mean"] for a in "xyz"),
        )
        checks.check(
            "sample counter continuity (no gaps >3)",
            cont["gaps_over_3"] == 0,
            "max delta %d, %.1f%% seen, %d stale reads"
            % (cont["max_counter_delta"], 100.0 * cont["seen_fraction"],
               cont["stale_reads"]),
        )
        if restart.get("command_accepted"):
            checks.check(
                "VCMD_RESET_SIM zeroes the quaternion",
                bool(restart.get("zero_seen")),
                "zero at %s ms" % round((restart.get("zero_at_s") or 0) * 1000.0)
                if restart.get("zero_seen") else "never zero in %.1f s" % RESTART_TIMEOUT_S,
            )
            checks.check(
                "SFLP non-zero again within %.1f s of restart" % RESTART_TIMEOUT_S,
                restart.get("recover_s") is not None
                and float(restart["recover_s"]) < RESTART_TIMEOUT_S,
                ("%.2f s" % restart["recover_s"]) if restart.get("recover_s") is not None
                else "not measured",
            )

        report["checks"] = checks.checks
        report["pass"] = checks.passed()
        print()
        if checks.passed():
            print("PASS: the LSM6DSV16X node looks healthy")
        else:
            bad = [c["check"] for c in checks.checks if not c["ok"]]
            print("FAIL: %d check(s) failed: %s" % (len(bad), "; ".join(bad)))
        report["stats"] = {
            "samples": len(unique),
            "window_s": round(samples[-1][0] - samples[0][0], 3),
        }
    finally:
        link.close()

    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=2, ensure_ascii=False)
        print("\nraw report -> %s" % args.json)

    return 0 if report.get("pass") else 1


if __name__ == "__main__":
    sys.exit(main())
