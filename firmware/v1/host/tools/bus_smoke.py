#!/usr/bin/env python3
"""Hardware smoke test for the imu_to_dxl v1 node over the motor bus.

The node shares the single-wire motor bus with the servos, so the port below is
that bus - never the debug console (USART0), which only prints DBG_* lines and
answers nothing:

    ./build.sh smoke
    python3 host/tools/bus_smoke.py --port /dev/ttyACM0 --baud 1000000 --protocol both
    python3 host/tools/bus_smoke.py --self-test        # no hardware needed

(A bench board with no motor bus wired yet can be driven over the console by
building with -DBUS_MIRROR_ENABLE=1; the upgrade session stays motor-bus only
even then.)

It checks the two protocol slaves end to end: identity registers, telemetry
block, both read modes, latency and sample-rate health, quaternion/gyro
consistency, and a sensor section that follows whatever the node is running:

  * IMU_USE_SPI=0 - the simulator is compiled in, so every simulation mode is
    exercised (frozen and sflp-wait are the negative cases microduck's health
    logic depends on);
  * IMU_USE_SPI=1 - the block comes from a real LSM6DSV16X, so the simulation
    checks are meaningless: the block does not change when V_SIM_MODE is
    written.  The real-sensor section instead verifies what the chip actually
    reports (unit quaternion, ~1 g at rest, quaternion gravity agreeing with
    the accelerometer, a quiet gyro) and the SFLP restart through
    VCMD_RESET_SIM.

The node says which of the two it is in the telemetry block's flags byte
(TELEM_FLAG_SIMULATED), which is what `sensor_is_simulated()` reads.
"""

from __future__ import annotations

import argparse
import math
import os
import statistics
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import bus  # noqa: E402  (path juggling above)

FAILURES: list[str] = []


def report(name: str, ok: bool, detail: str = "") -> None:
    mark = "ok  " if ok else "FAIL"
    print(f"  {mark} {name}" + (f"  [{detail}]" if detail else ""))
    if not ok:
        FAILURES.append(name)


def collect(link: bus.Link, seconds: float) -> list[dict]:
    samples: list[dict] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        sample = link.poll()
        if sample is not None:
            samples.append(sample)
    return samples


SAMPLE_PERIOD_S = 0.01   # the node refreshes its block at 100 Hz

# ── real-sensor expectations, measured on the bench LSM6DSV16X ──────────────
# gyro +-500 dps @ 17.5 mdps/LSB, accel +-4 g @ 0.122 mg/LSB, both at 120 Hz.
REAL_ACCEL_MG_MIN = 900.0        # at rest the magnitude is 1 g
REAL_ACCEL_MG_MAX = 1100.0
REAL_QUAT_NORM_MIN = 0.99        # |q| = 1.0000 measured
REAL_QUAT_NORM_MAX = 1.01
REAL_GRAVITY_MAX_DEG = 5.0       # measured 0.03 deg mean on the bench
REAL_GYRO_STD_MAX_DPS = 3.0      # measured 0.02-0.12 dps per axis at rest
REAL_RESTART_TIMEOUT_S = 2.5     # SFLP needs ~1.5 s after a restart

FLAG_SFLP_VALID = bus.TELEM_FLAG_BITS["sflp_valid"]
FLAG_FUSION_OK = bus.TELEM_FLAG_BITS["fusion_ok"]
FLAG_SENSOR_ERR = bus.TELEM_FLAG_BITS["sensor_err"]
FLAG_SIMULATED = bus.TELEM_FLAG_BITS["simulated"]


def read_block(link: bus.Link) -> bus.ImuBlock:
    """One 20-byte diagnostic block, decoded but still in the chip frame."""
    raw = link.read_registers(link.telemetry_addr, bus.TELEM_LEN)
    return bus.ImuBlock.decode(raw, link.protocol)


def sensor_is_simulated(link: bus.Link) -> bool:
    """True when the node runs the simulator rather than the real chip.

    TELEM_FLAG_SIMULATED (block byte 19, bit 4) is set by src/imu_sim.c and
    never by the real LSM6DSV16X driver (src/imu_spi.c), so the flags byte is
    the node's own statement of which source produced the block.
    """
    return bool(read_block(link).flags & FLAG_SIMULATED)


def collect_blocks(link: bus.Link, seconds: float) -> list[tuple[float, bus.ImuBlock]]:
    """Timestamped raw blocks over `seconds` (chip frame, mounts not applied).

    Polling faster than the node's 100 Hz refresh returns the same sample twice,
    so callers that want per-sample statistics should de-duplicate on
    `ImuBlock.counter`; the timestamped list is what makes the effective rate
    and the counter continuity measurable.
    """
    out: list[tuple[float, bus.ImuBlock]] = []
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        try:
            out.append((time.monotonic(), read_block(link)))
        except Exception:
            pass
    return out


def unique_blocks(blocks: list[tuple[float, bus.ImuBlock]]) -> list[bus.ImuBlock]:
    """Drop repeated polls of the same 100 Hz sample (compare the counter)."""
    out: list[bus.ImuBlock] = []
    last: Optional[int] = None
    for _t, block in blocks:
        if block.counter != last:
            out.append(block)
            last = block.counter
    return out


def settle(seconds: float = 0.4) -> None:
    """Let the node's protocol stickiness (BUS_PROTO_STICKY_MS) expire before a
    protocol switch, and drain anything still in flight."""
    time.sleep(seconds)


def quat_consistency(samples: list[dict], gyro_only_motion: bool = False) -> Optional[float]:
    """Median angle (deg) between the gyro and the quaternion derivative.

    Both are in the trunk frame, so dq = conj(q_prev) * q_cur must be a rotation
    about the measured rate vector, with |angle| = |w| * dt.

    `gyro_only_motion` decides what "nothing is moving" means.  The simulator
    integrates its own gyro, so both the rate and the quaternion delta are
    smooth and either can gate the comparison.  A real SFLP quaternion arrives
    as binary16 components, so at rest it still steps by ~0.0005 per sample -
    about 0.1 rad/s of apparent rotation at 100 Hz - while the gyro is quiet;
    the real sensor therefore gates on the gyro alone and skips the quiet case.
    """
    errors: list[float] = []
    for prev, cur in zip(samples, samples[1:]):
        # Polling faster than the node's 100 Hz refresh returns the same sensor
        # sample twice; those pairs carry no rate information at all.  Use the
        # node's own counter for the interval, not the host clock.
        steps = (cur["counter"] - prev["counter"]) & 0xFF
        if steps == 0 or steps > 5:
            continue
        dt = steps * SAMPLE_PERIOD_S
        qa, qb = prev["quat"], cur["quat"]
        dq = bus.q_mul(bus.q_conj(qa), qb)
        if dq[0] < 0:
            dq = tuple(-c for c in dq)
        w_quat = (2.0 * dq[1] / dt, 2.0 * dq[2] / dt, 2.0 * dq[3] / dt)
        w_gyro = tuple(math.radians(v) for v in cur["gyro_trunk_dps"])
        na = math.sqrt(sum(v * v for v in w_quat))
        nb = math.sqrt(sum(v * v for v in w_gyro))
        if nb < 0.05 and (gyro_only_motion or na < 0.05):
            continue                     # nothing is moving; no direction to compare
        if na < 1e-6 or nb < 1e-6:
            errors.append(180.0)
            continue
        dot = sum(a * b for a, b in zip(w_quat, w_gyro)) / (na * nb)
        errors.append(math.degrees(math.acos(max(-1.0, min(1.0, dot)))))
    if not errors:
        return None
    return statistics.median(errors)


def run_protocol(args: argparse.Namespace, protocol: str) -> None:
    name = "Dynamixel 2.0" if protocol == bus.PROTO_DXL else "FeeTech SCS"
    print(f"\n=== {name} ===")
    cfg: Optional[dict] = None
    link = bus.Link(
        port=args.port,
        baud=args.baud,
        protocol=protocol,
        imu_id=args.id,
        read_len=20,
        mode="sync_read",
        timeout=args.timeout,
    )
    try:
        link.open()
    except Exception as exc:
        report(f"{name}: open {args.port}", False, str(exc))
        return

    try:
        # identity
        try:
            info = link.device_info()
            report(f"{name}: identity registers", info["model"] != 0, str(info))
        except Exception as exc:
            report(f"{name}: identity registers", False, str(exc))

        report(f"{name}: ping id {args.id}", link.ping())

        # configuration window
        try:
            cfg = link.get_config()
            report(
                f"{name}: vendor window",
                cfg["magic"] == bus.VENDOR_MAGIC,
                f"magic={cfg['magic']:#06x} mode={cfg['sim_mode_name']} amp={cfg['sim_amp']} "
                f"freq={cfg['sim_freq']} frame={'trunk' if cfg['report_frame'] else 'chip'}",
            )
        except Exception as exc:
            report(f"{name}: vendor window", False, str(exc))

        # the 20-byte diagnostic block (FeeTech: the 128 alias; Dynamixel: 124)
        simulated: Optional[bool] = None
        try:
            raw = link.read_registers(link.telemetry_addr, 20)
            block = bus.ImuBlock.decode(raw, protocol)
            simulated = bool(block.flags & FLAG_SIMULATED)
            report(
                f"{name}: 20-byte block",
                len(raw) == 20,
                f"gyro={block.gyro_raw} quat_half={tuple(hex(h) for h in block.quat_half)} "
                f"cnt={block.counter} flags={','.join(block.flag_names()) or 'none'} "
                f"sensor={'simulated' if simulated else 'real'}",
            )
            report(f"{name}: telemetry address is 20 bytes", len(raw) == 20)
        except Exception as exc:
            report(f"{name}: 20-byte block", False, str(exc))

        # the block the runtime shares with the servos: FeeTech answers 15 bytes
        # at 56 (counter and status inside that span, byte 14 reserved zero),
        # Dynamixel the 20-byte block at 124
        try:
            ctrl, cnt, flags = link.read_contract()
            report(
                f"{name}: shared block at {link.contract_addr}",
                len(ctrl) == bus.CTRL_LEN and ctrl != bytes(bus.CTRL_LEN),
                f"cnt={cnt} flags={flags:#04x}",
            )
            if protocol == bus.PROTO_FEE:
                raw15 = link.read_registers(bus.FEE_TELEM_ADDR, bus.FEE_BLOCK_LEN)
                report(
                    f"{name}: byte 14 reserved zero",
                    raw15[bus.FEE_BLOCK_RESERVED] == 0,
                    f"block={raw15.hex()}",
                )
                # and the counter the runtime reads must be the same one the
                # diagnostic block carries, just at a different offset
                raw20 = link.read_registers(link.telemetry_addr, bus.TELEM_LEN)
                report(
                    f"{name}: contract and diagnostic agree",
                    abs(raw20[18] - raw15[bus.FEE_BLOCK_CNT]) <= 4,
                    f"cnt15={raw15[bus.FEE_BLOCK_CNT]} cnt20={raw20[18]}",
                )
            # the counter has to advance while the node produces samples
            time.sleep(0.05)
            _ctrl2, cnt2, _flags2 = link.read_contract()
            report(f"{name}: counter advances", cnt2 != cnt, f"{cnt} -> {cnt2}")
        except Exception as exc:
            report(f"{name}: shared block", False, str(exc))

        # both read modes, for both protocols
        for mode in ("read", "sync_read"):
            link.mode = mode
            samples = collect(link, 0.4)
            report(f"{name}: {mode} returns samples", len(samples) >= 5, f"{len(samples)} samples")
        link.mode = "sync_read"
        link.read_len = 12
        samples = collect(link, 0.2)
        report(f"{name}: 12-byte control block", len(samples) >= 2, f"{len(samples)} samples")
        link.read_len = 20

        # sustained polling: rate, latency, health
        samples = collect(link, args.seconds)
        if len(samples) < 20:
            report(f"{name}: sustained polling", False, f"only {len(samples)} samples")
            return
        span = samples[-1]["t"] - samples[0]["t"]
        achieved = (len(samples) - 1) / span if span > 0 else 0.0
        lat = link.stats.latency_summary()
        report(
            f"{name}: sustained polling",
            achieved > 20.0 and lat.get("p95_ms", 99) < args.max_latency_ms,
            f"{achieved:.1f} Hz, latency median {lat.get('median_ms', 0):.2f} ms "
            f"p95 {lat.get('p95_ms', 0):.2f} ms, timeouts {link.stats.timeouts}, "
            f"bad frames {link.stats.bad_frames}",
        )
        report(
            f"{name}: no timeouts / bad frames",
            link.stats.timeouts == 0 and link.stats.bad_frames == 0,
            f"timeouts={link.stats.timeouts} bad={link.stats.bad_frames} extra={link.stats.extra_bytes}",
        )

        gyro_mag = [math.sqrt(sum(v * v for v in s["gyro_trunk_dps"])) for s in samples]
        norms = [math.sqrt(sum(v * v for v in s["quat"])) for s in samples]
        gmag = [math.sqrt(sum(v * v for v in s["gravity"])) for s in samples]
        report(f"{name}: gyro within +-500 dps", max(gyro_mag) < 520.0, f"peak {max(gyro_mag):.1f} dps")
        report(f"{name}: quaternion unit", abs(statistics.mean(norms) - 1.0) < 5e-3,
               f"|q| = {statistics.mean(norms):.5f}")
        report(f"{name}: gravity unit", abs(statistics.mean(gmag) - 1.0) < 5e-3,
               f"|g| = {statistics.mean(gmag):.5f}")

        # Sample counter continuity.  The node advances it at exactly 100 Hz, so
        # a delta of 0 means "polled faster than the node", 1 is ideal, and 2~3
        # happens when the *host* was late (USB scheduling) - none of those are
        # node-side sample loss.  Only a bigger gap is worth failing on.
        counters = [s["counter"] for s in samples]
        deltas = [((b - a) & 0xFF) for a, b in zip(counters, counters[1:])]
        gaps = sum(1 for d in deltas if d > 3)
        produced = sum(deltas)
        report(f"{name}: sample counter continuity", gaps == 0,
               f"{gaps} gaps >3, node produced {produced} samples while we read {len(samples)}")

        # the strongest check: gyro and quaternion must describe the same motion
        err = quat_consistency(samples, gyro_only_motion=(simulated is False))
        if err is None:
            report(f"{name}: gyro/quaternion consistency", True, "nothing moving - skipped")
        else:
            report(f"{name}: gyro/quaternion consistency", err < 8.0, f"median {err:.2f} deg")

        # Force the protocol-2.0 stuffing pattern onto the wire: a gyro of -1
        # (FF FF) followed by a low byte of 0xFD is exactly what rustypot 1.6
        # de-stuffs, and a node that did not stuff would be misparsed.  The
        # simulator is simply told to output those counts; a real chip has to be
        # biased there through the gyro_bias register.
        if protocol == bus.PROTO_DXL and cfg is not None:
            if simulated is False:
                run_dxl_stuffing_real(link, name, cfg)
            else:
                run_dxl_stuffing(link, name, cfg)

        # FeeTech-only device semantics from the vendor memory table
        if protocol == bus.PROTO_FEE:
            run_fee_device_semantics(link, name)

        # Sensor section: the simulator's modes, or the real chip's physics.
        # (the detected type is re-read from the block here through the helper)
        try:
            simulated = sensor_is_simulated(link)
        except Exception as exc:
            report(f"{name}: sensor type", False, f"flags read failed: {exc}")
        if simulated:
            if cfg is not None:
                run_modes(link, name)
        elif simulated is not None:
            run_real_sensor(link, name, args.seconds)

        print(
            f"  -- link stats: polls={link.stats.polls} ok={link.stats.ok} "
            f"timeouts={link.stats.timeouts} bad={link.stats.bad_frames} "
            f"extra_bytes={link.stats.extra_bytes} last_error={link.stats.last_error!r}"
        )
    finally:
        settle(0.2)
        # put the node back the way we found it
        if cfg is not None:
            try:
                link.set_config(
                    sim_mode=cfg["sim_mode"],
                    sim_amp=cfg["sim_amp"],
                    sim_freq=cfg["sim_freq"],
                    gyro_bias=tuple(cfg["gyro_bias"]),
                    report_frame=cfg["report_frame"],
                    proto_lock={
                        0: "none",
                        1: "dxl",
                        2: "fee",
                    }[cfg["proto_lock"]],
                    dbg_level=cfg["dbg_level"],
                )
            except Exception:
                pass
        link.close()


def run_dxl_stuffing(link: bus.Link, name: str, cfg: dict) -> None:
    """Drive the gyro to FF FF FD and check the stuffed frame is parsed."""
    print("  -- protocol 2.0 byte stuffing on the wire")
    try:
        link.set_config(sim_mode="static", gyro_bias=(-1, -3, 0))
        time.sleep(0.25)
        stuffed_seen = 0
        stuffed_ok = 0
        for _ in range(60):
            frame, params = link.read_registers_raw(link.telemetry_addr, 20)
            if b"\xff\xff\xfd" in frame[8:-2]:
                stuffed_seen += 1
                # the payload must come back exactly as the host asked for it
                if params[0:2] == b"\xff\xff" and params[2:3] == b"\xfd":
                    stuffed_ok += 1
        report(f"{name}: stuffed frames parse correctly", stuffed_seen > 0 and stuffed_ok == stuffed_seen,
               f"{stuffed_seen} stuffed frames seen, {stuffed_ok} decoded")
    finally:
        link.set_config(sim_mode=cfg["sim_mode"], gyro_bias=tuple(cfg["gyro_bias"]))
        time.sleep(0.2)


def run_dxl_stuffing_real(link: bus.Link, name: str, cfg: dict) -> None:
    """Same wire check as `run_dxl_stuffing`, but for a real chip.

    The simulator can be told to output any gyro counts; a real LSM6DSV16X
    cannot.  What it *can* take is the gyro_bias injection (src/imu_spi.c adds
    it to the chip's raw counts in chip frame), so the bias is computed from the
    measured counts: put gyro x at -1 (bytes FF FF) and gyro y's low byte at
    0xFD, which is the FF FF FD sequence rustypot 1.6 de-stuffs.  Report frame
    is forced to chip so no mount rotation sits between the bias and the bytes.
    """
    print("  -- protocol 2.0 byte stuffing on the wire (real sensor)")
    try:
        link.set_config(report_frame=0)
        time.sleep(0.25)
        base = unique_blocks(collect_blocks(link, 0.4))
        if len(base) < 5:
            report(f"{name}: stuffed frames parse correctly", False,
                   f"only {len(base)} gyro samples to estimate the bias from")
            return
        mx = int(round(statistics.median(b.gyro_raw[0] for b in base)))
        my = int(round(statistics.median(b.gyro_raw[1] for b in base)))
        bias = (-1 - mx, -3 - my, 0)   # -1 -> FF FF; low byte of -3 is FD
        link.set_config(gyro_bias=bias)
        time.sleep(0.25)
        # The gyro has to land on two exact counts at once (x == -1 and y's low
        # byte == 0xFD), which the chip's noise makes roughly a 4% chance per
        # *sample*, not per read; polling faster than the 100 Hz refresh sees
        # the same sample several times, so keep reading until a few have hit
        # rather than assuming the first handful will.
        stuffed_seen = 0
        stuffed_ok = 0
        reads = 0
        deadline = time.monotonic() + 2.5
        while reads < 2500 and stuffed_seen < 3 and time.monotonic() < deadline:
            frame, params = link.read_registers_raw(link.telemetry_addr, 20)
            reads += 1
            if b"\xff\xff\xfd" in frame[8:-2]:
                stuffed_seen += 1
                # the payload must come back exactly as the host asked for it
                if params[0:2] == b"\xff\xff" and params[2:3] == b"\xfd":
                    stuffed_ok += 1
        report(f"{name}: stuffed frames parse correctly", stuffed_seen > 0 and stuffed_ok == stuffed_seen,
               f"{stuffed_seen} stuffed frames seen, {stuffed_ok} decoded in {reads} reads "
               f"(measured gyro x={mx} y={my} counts -> bias={bias})")
    finally:
        link.set_config(report_frame=cfg["report_frame"], gyro_bias=tuple(cfg["gyro_bias"]))
        time.sleep(0.2)


def run_real_sensor(link: bus.Link, name: str, seconds: float) -> None:
    """The physics a real LSM6DSV16X has to report.

    Identity, the vendor window, both read modes, latency and the block layout
    are already checked above and are sensor-independent.  What is *not*
    sensor-independent is the content: a real chip must report a unit
    quaternion, roughly 1 g of specific force when the bench is at rest, a
    gravity direction consistent with that accelerometer, a quiet gyro with
    only a small zero-rate offset, and a working SFLP restart.
    """
    print("  -- real LSM6DSV16X sensor")
    try:
        block = read_block(link)
    except Exception as exc:
        report(f"{name}: real-sensor block", False, str(exc))
        return

    flags = block.flags
    report(
        f"{name}: sensor is real, flags={flags:#04x} ({','.join(block.flag_names()) or 'none'})",
        not (flags & FLAG_SIMULATED)
        and bool(flags & FLAG_SFLP_VALID)
        and bool(flags & FLAG_FUSION_OK)
        and not (flags & FLAG_SENSOR_ERR),
    )
    report(
        f"{name}: sflp_valid+fusion_ok set, sensor_err clear (flags={flags:#04x})",
        bool(flags & FLAG_SFLP_VALID) and bool(flags & FLAG_FUSION_OK)
        and not (flags & FLAG_SENSOR_ERR),
    )

    blocks = unique_blocks(collect_blocks(link, max(2.0, seconds)))
    if len(blocks) < 5:
        report(f"{name}: real-sensor samples", False, f"only {len(blocks)} unique samples")
        return

    norms: list[float] = []
    mags: list[float] = []
    angles: list[float] = []
    gyro: list[list[float]] = [[], [], []]
    valid = 0
    for block in blocks:
        if block.quat_half != (0, 0, 0):
            valid += 1
        q = block.quat_raw
        norms.append(math.sqrt(sum(c * c for c in q)))
        accel = block.accel_mg
        mag = math.sqrt(sum(a * a for a in accel))
        mags.append(mag)
        for axis in range(3):
            gyro[axis].append(block.gyro_dps[axis])
        if mag > 1e-6:
            accel_n = tuple(a / mag for a in accel)
            # specific force is the *negative* of gravity; gravity here is the
            # world down direction projected into the chip frame (see imu.h).
            gravity = bus.q_normalize(bus.q_rotate_inv(q, (0.0, 0.0, -1.0)))
            dot = sum(a * -g for a, g in zip(accel_n, gravity))
            angles.append(math.degrees(math.acos(max(-1.0, min(1.0, dot)))))

    mean_norm = statistics.fmean(norms)
    report(
        f"{name}: quat norm |q|={mean_norm:.4f} (0.99-1.01, n={len(norms)})",
        all(REAL_QUAT_NORM_MIN <= v <= REAL_QUAT_NORM_MAX for v in norms),
    )
    report(
        f"{name}: quaternion non-zero in {valid}/{len(blocks)} samples",
        valid > 0,
        "zero bytes before the SFLP has converged",
    )

    mean_mag = statistics.fmean(mags)
    report(
        f"{name}: accel magnitude {mean_mag:.0f} mg (900-1100 mg, n={len(mags)})",
        all(REAL_ACCEL_MG_MIN <= v <= REAL_ACCEL_MG_MAX for v in mags),
    )

    if angles:
        mean_angle = statistics.fmean(angles)
        max_angle = max(angles)
        report(
            f"{name}: quat-gravity vs accel {mean_angle:.2f} deg mean, {max_angle:.2f} max "
            f"(< 5 deg, n={len(angles)})",
            mean_angle < REAL_GRAVITY_MAX_DEG and max_angle < REAL_GRAVITY_MAX_DEG,
        )
    else:
        report(f"{name}: quat-gravity vs accel", False, "no usable samples")

    stds = [statistics.pstdev(axis) for axis in gyro if len(axis) > 1]
    means = [statistics.fmean(axis) for axis in gyro]
    worst = max(stds) if stds else float("inf")
    report(
        f"{name}: gyro noise std {worst:.3f} dps (< 3 dps, n={len(gyro[0])})",
        all(s < REAL_GYRO_STD_MAX_DPS for s in stds),
        f"per-axis mean x={means[0]:.2f} y={means[1]:.2f} z={means[2]:.2f} dps "
        f"(zero-rate offset, not a failure; compensation is gyro_bias = "
        f"({-means[0] / 0.0175:+.0f}, {-means[1] / 0.0175:+.0f}, "
        f"{-means[2] / 0.0175:+.0f}) raw counts)",
    )

    run_real_restart(link, name)


def run_real_restart(link: bus.Link, name: str) -> None:
    """VCMD_RESET_SIM restarts the chip's SFLP: the quaternion must go to zero
    and come back non-zero within ~2 s (src/imu_spi.c holds the block at zero
    until the fusion converges again)."""
    print("  -- SFLP restart (VCMD_RESET_SIM)")
    t0 = time.monotonic()
    try:
        link.set_config(cmd=bus.VCMD_RESET_SIM)
    except Exception as exc:
        report(f"{name}: VCMD_RESET_SIM accepted", False, str(exc))
        return
    report(f"{name}: VCMD_RESET_SIM accepted", True)

    zero_at: Optional[float] = None
    recover_at: Optional[float] = None
    first_nonzero: Optional[float] = None
    while time.monotonic() - t0 < REAL_RESTART_TIMEOUT_S:
        try:
            block = read_block(link)
        except Exception:
            continue
        elapsed = time.monotonic() - t0
        if block.quat_half == (0, 0, 0):
            if zero_at is None:
                zero_at = elapsed
        else:
            if first_nonzero is None:
                first_nonzero = elapsed
            if zero_at is not None and recover_at is None:
                recover_at = elapsed
        # The block refreshes at 100 Hz, so a few ms per poll is plenty and
        # keeps this section from adding thousands of transactions to the bus.
        time.sleep(0.002)

    if zero_at is None:
        report(
            f"{name}: VCMD_RESET_SIM zeroes the quaternion",
            False,
            f"never zero in {REAL_RESTART_TIMEOUT_S:.1f} s; first non-zero after "
            f"{first_nonzero * 1000:.0f} ms" if first_nonzero is not None else "never zero",
        )
    else:
        report(
            f"{name}: VCMD_RESET_SIM zeroes the quaternion after {zero_at * 1000:.0f} ms",
            True,
        )

    if recover_at is None:
        report(
            f"{name}: SFLP non-zero again within {REAL_RESTART_TIMEOUT_S:.1f} s of restart",
            False,
            "no zero window, so there was nothing to recover from"
            if zero_at is None else "still zero at the deadline",
        )
    else:
        report(
            f"{name}: SFLP back non-zero {recover_at:.2f} s after restart (< 2.5 s)",
            recover_at < REAL_RESTART_TIMEOUT_S,
        )


def run_fee_device_semantics(link: bus.Link, name: str) -> None:
    """Register 8 (ACK status level) and 55 (EEPROM lock), straight from the
    vendor memory table: 0 = only read/PING answered, and the lock is about
    persistence rather than refusing the write."""
    print("  -- device semantics (vendor table)")
    level = link.read_registers(8, 1)[0]
    report(f"{name}: ACK status level defaults to 1", level == 1, f"level={level}")

    try:
        # level 0: a write is executed but not answered; a read still is
        link.write_registers(8, bytes([0]), expect_ack=True)
        silent = link.write_registers(40, bytes([0]), expect_ack=False)
        read_ok = False
        try:
            link.read_registers(56, 2)
            read_ok = True
        except Exception:
            read_ok = False
        report(f"{name}: level 0 silences writes but not reads", (not silent) and read_ok,
               f"ack={silent} read_answered={read_ok}")
    finally:
        link.write_registers(8, bytes([1]), expect_ack=False)

    # lock on: a write to the EEPROM half is accepted (RAM value changes), it is
    # just not persisted
    try:
        link.write_registers(55, bytes([1]))
        link.write_registers(24, bytes([123]))          # 最小启动力, EEPROM half
        got = link.read_registers(24, 1)[0]
        report(f"{name}: locked EEPROM write still applies in RAM", got == 123,
               f"register 24 = {got}")
    finally:
        link.write_registers(55, bytes([0]))
        link.write_registers(24, bytes([0]))


def run_modes(link: bus.Link, name: str) -> None:
    """Set every simulation mode and check what the block does in each."""
    print("  -- simulation modes")
    expectations = {
        "static": lambda s, e: max(abs(v) for v in s["gyro_trunk_dps"]) < 2.0,
        "sine": lambda s, e: e["gyro_ptp"] > 20.0,
        "drift": lambda s, e: e["gyro_z_mean"] > 8.0,
        # SIM_NOISE injects +-12 raw counts of white noise per axis, i.e. about
        # +-210 mdps: well above the LSB-level static floor, far below a real
        # sensor's bandwidth-limited noise.  Compare against the measured static
        # floor instead of a hand-picked number.
        "noise": lambda s, e: e["gyro_std"] > max(0.02, 2.0 * e["static_std"]),
        "spin": lambda s, e: e["gyro_max"] > 100.0,
        "sflp-wait": lambda s, e: not e["quat_valid"] and e["quat_zero"],
        "frozen": lambda s, e: e["identical"],
    }
    seen_std: dict[str, float] = {}
    for mode, check in expectations.items():
        try:
            link.set_config(sim_mode=mode)
        except Exception as exc:
            report(f"{name}: mode {mode}", False, f"write failed: {exc}")
            continue
        time.sleep(0.25)
        samples = collect(link, 0.5)
        if len(samples) < 5:
            report(f"{name}: mode {mode}", False, "no samples")
            continue
        raw_blocks = []
        for _ in range(6):
            try:
                raw_blocks.append(link.read_registers(link.telemetry_addr, 20))
            except Exception:
                pass
        gyros = [abs(s["gyro_trunk_dps"][2]) for s in samples]
        allg = [abs(v) for s in samples for v in s["gyro_trunk_dps"]]
        ev = {
            "gyro_ptp": (max(allg) - min(allg)) if allg else 0.0,
            "gyro_z_mean": statistics.fmean(gyros) if gyros else 0.0,
            "gyro_std": statistics.pstdev(allg) if len(allg) > 1 else 0.0,
            "gyro_max": max(allg) if allg else 0.0,
            "quat_valid": samples[-1]["quat_valid"],
            "quat_zero": all(b == 0 for b in samples[-1]["quat_raw"][1:]),
            "identical": len(set(raw_blocks)) <= 1 and bool(raw_blocks),
            "static_std": seen_std.get("static", 0.0),
        }
        seen_std[mode] = ev["gyro_std"]
        report(
            f"{name}: mode {mode}",
            check(samples[-1], ev),
            f"gyro_z_mean={ev['gyro_z_mean']:.1f} dps ptp={ev['gyro_ptp']:.1f} "
            f"std={ev['gyro_std']:.2f} valid={ev['quat_valid']} identical={ev['identical']}",
        )
    try:
        link.set_config(sim_mode="sine")
    except Exception:
        pass


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--id", type=int, default=200, help="node id (both protocols)")
    parser.add_argument("--protocol", choices=["dxl", "fee", "both"], default="both")
    parser.add_argument("--seconds", type=float, default=2.0, help="sustained polling per protocol")
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--max-latency-ms", type=float, default=20.0)
    parser.add_argument("--self-test", action="store_true", help="codec vectors, no hardware")
    args = parser.parse_args()

    if args.self_test:
        return bus._self_test()

    print(f"imu_to_dxl smoke test: {args.port} @ {args.baud} baud, id {args.id}")
    protocols = ["dxl", "fee"] if args.protocol == "both" else [args.protocol]
    for index, protocol in enumerate(protocols):
        if index:
            settle()
        run_protocol(args, protocol)

    print()
    if FAILURES:
        print(f"{len(FAILURES)} FAILURE(S):")
        for item in FAILURES:
            print(f"  - {item}")
        return 1
    print("PASS: both protocol slaves answered correctly")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
