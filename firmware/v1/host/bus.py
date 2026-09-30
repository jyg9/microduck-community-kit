#!/usr/bin/env python3
"""Protocol codecs and serial link for the imu_to_dxl v1 node.

This module is the single source of truth on the host side for:

  * the Dynamixel protocol 2.0 framing (instruction/status, ROBOTIS CRC-16),
  * the FeeTech SCS/HLS framing (8-bit complement checksum),
  * the 20-byte IMU block layout and the mounting rotation microduck applies,
  * a serial `Link` that polls the node and decodes samples.

Everything here mirrors `src/*.c`; the constants and the
packet vectors are asserted against the C implementation by
`tools/bus_smoke.py --self-test` and `tools/test_protocols.c`.

Run `python3 bus.py` for the codec self test (no hardware needed).
"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass, field
from typing import Iterable, Optional, Sequence

try:  # pyserial is only needed for the live link, not for the codecs
    import serial  # type: ignore
except Exception:  # pragma: no cover - import guard for self tests
    serial = None  # type: ignore

# ── constants, mirrored from src/board.h ────────────────────────────────────

DXL_BROADCAST = 0xFE
FEE_BROADCAST = 0xFE

PROTO_DXL = "dxl"
PROTO_FEE = "fee"

DXL_TELEM_ADDR = 124
FEE_TELEM_ADDR = 56
FEE_TELEM_ALT_ADDR = 128
DXL_VENDOR_ADDR = 148
FEE_VENDOR_ADDR = 160
DXL_READ_ADDR = DXL_TELEM_ADDR
FEE_READ_ADDR = FEE_TELEM_ADDR
#: Where the *20-byte* diagnostic block lives (accelerometer included).  The
#: FeeTech map answers at 56 with the 15-byte shape a real servo uses there, so
#: the diagnostic tail sits in the 128 alias instead; Dynamixel has room at 124.
DXL_DIAG_ADDR = DXL_TELEM_ADDR
FEE_DIAG_ADDR = FEE_TELEM_ALT_ADDR

TELEM_LEN = 20
CTRL_LEN = 12
#: Longest Dynamixel frame the scanner will believe: one status packet is at most
#: 7 + 256 + 4 bytes, and a Fast Sync Read *aggregate* is longer still - it is
#: `8 + N*(X+4)` for every device in the id list (microduck's 16 devices of 12
#: bytes make 264).  The bound only stops a corrupted LEN from making the scanner
#: wait for bytes that will never come; the CRC is what decides the frame.
DXL_MAX_FRAME = 520
#: FeeTech 56..70: the 12 control bytes plus counter and status, which is the
#: span the runtime reads in the shared `sync_read` (src/board.h FEE_BLOCK_*).
FEE_BLOCK_LEN = 15
FEE_BLOCK_CNT = 12
FEE_BLOCK_STATUS = 13
FEE_BLOCK_RESERVED = 14

# vendor window offsets (src/board.h V_*)
VOFF = {
    "magic_l": 0,
    "magic_h": 1,
    "cmd": 2,
    "sim_mode": 3,
    "amp_l": 4,
    "amp_h": 5,
    "freq_l": 6,
    "freq_h": 7,
    "report_frame": 8,
    "proto_lock": 9,
    "bias_x_l": 10,
    "bias_x_h": 11,
    "bias_y_l": 12,
    "bias_y_h": 13,
    "bias_z_l": 14,
    "bias_z_h": 15,
    "mirror_baud": 16,
    "dbg_level": 17,
    "status": 18,
    # Not reserved: the last protocol error this node refused or clamped
    # (DXL_ERR_*), which is where a refused write says why.  The reply's own
    # status byte is always 0 because FeeTech tooling reads a nonzero ack status
    # as a *servo* fault (src/board.h, V_LAST_ERR; src/fee.c).
    "last_err": 19,
}
VENDOR_MAGIC = 0x4D49

VCMD_NONE = 0
VCMD_SAVE = 1
VCMD_DEFAULTS = 2
VCMD_REBOOT = 3
VCMD_RESET_SIM = 4
VCMD_CONFIRM = 5
VCMD_BOOT = 6
VCMD_SWITCH_SLOT = 7

SIM_MODES = {
    0: "static",
    1: "sine",
    2: "drift",
    3: "noise",
    4: "spin",
    5: "sflp-wait",
    6: "frozen",
}
SIM_MODE_IDS = {v: k for k, v in SIM_MODES.items()}

TELEM_FLAG_BITS = {
    "sflp_valid": 0x01,
    "cnt_wrap": 0x02,
    "gyro_sat": 0x04,
    "sensor_err": 0x08,
    "simulated": 0x10,
    "frozen": 0x20,
    "cfg_dirty": 0x40,
    "fusion_ok": 0x80,
}

DXL_INST = {
    "ping": 0x01,
    "read": 0x02,
    "write": 0x03,
    "reg_write": 0x04,
    "action": 0x05,
    "factory_reset": 0x06,
    "reboot": 0x08,
    "clear": 0x10,
    "sync_read": 0x82,
    "sync_write": 0x83,
    "fast_sync_read": 0x8A,
    "bulk_read": 0x92,
}
FEE_INST = {
    "ping": 0x01,
    "read": 0x02,
    "write": 0x03,
    "reg_write": 0x04,
    "reg_action": 0x05,
    "recovery": 0x06,
    "sync_read": 0x82,
    "sync_write": 0x83,
    "reset": 0x0A,
    "cal": 0x0B,
}

# scales (src/board.h)
GYRO_MDPS_PER_LSB = 17.5
ACCEL_MG_PER_LSB = 0.122

# microduck's SflpDecoder::DEFAULT_MOUNT: trunk = [+raw_z, +raw_y, -raw_x]
DEFAULT_MOUNT = (
    math.sqrt(0.5),
    0.0,
    math.sqrt(0.5),
    0.0,
)


class ProtocolError(Exception):
    """A frame arrived but did not parse (CRC, length, id mismatch, ...)."""


# ── Dynamixel protocol 2.0 ──────────────────────────────────────────────────


def crc16_dxl(data: bytes) -> int:
    """ROBOTIS `update_crc`: poly 0x8005, init 0, MSB first, over the whole packet."""
    crc = 0
    for byte in data:
        crc ^= (byte << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x8005) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def dxl_add_stuffing(data: bytes) -> bytes:
    """Insert protocol 2.0 byte stuffing: an extra 0xFD after any FF FF FD.

    Mirrors the official DynamixelSDK `addStuffing` (and rustypot's port): the
    pattern scan runs over the original bytes only, and `FF FF FF` keeps an
    `FF FF` suffix alive.
    """
    out = bytearray()
    run = 0
    for byte in data:
        out.append(byte)
        if run == 2 and byte == 0xFD:
            out.append(0xFD)          # stuffing byte
            run = 0
        elif byte == 0xFF:
            run = run + 1 if run in (0, 1) else 2
        else:
            run = 0
    return bytes(out)


def dxl_remove_stuffing(data: bytes) -> bytes:
    """Drop the 0xFD inserted after each FF FF FD (received body only)."""
    out = bytearray()
    run = 0
    for byte in data:
        if run == 3:
            run = 0
            if byte == 0xFD:
                continue                  # stuffing byte: drop it
        if run == 2 and byte == 0xFD:
            run = 3
        elif byte == 0xFF:
            run = run + 1 if run in (0, 1) else 2
        else:
            run = 0
        out.append(byte)
    return bytes(out)


def dxl_instruction(packet_id: int, instruction: int, params: bytes = b"") -> bytes:
    """FF FF FD 00 ID LEN_L LEN_H INST PARAM... CRC_L CRC_H (LEN = params + 3).

    The parameter region is byte-stuffed and LEN counts the stuffed bytes, which
    is what rustypot's `InstructionPacketV2::to_bytes` does.
    """
    params = dxl_add_stuffing(params)
    body = bytes([packet_id]) + struct.pack("<H", len(params) + 3) + bytes([instruction]) + params
    frame = b"\xFF\xFF\xFD\x00" + body
    return frame + struct.pack("<H", crc16_dxl(frame))


def dxl_ping(packet_id: int) -> bytes:
    return dxl_instruction(packet_id, DXL_INST["ping"])


def dxl_read(packet_id: int, addr: int, length: int) -> bytes:
    return dxl_instruction(packet_id, DXL_INST["read"], struct.pack("<HH", addr, length))


def dxl_write(packet_id: int, addr: int, data: bytes) -> bytes:
    return dxl_instruction(packet_id, DXL_INST["write"], struct.pack("<H", addr) + data)


def dxl_sync_read(ids: Sequence[int], addr: int, length: int) -> bytes:
    params = struct.pack("<HH", addr, length) + bytes(ids)
    return dxl_instruction(DXL_BROADCAST, DXL_INST["sync_read"], params)


def dxl_sync_write(ids: Sequence[int], addr: int, payloads: Sequence[bytes]) -> bytes:
    if not ids:
        raise ValueError("sync_write needs at least one id")
    width = len(payloads[0])
    params = struct.pack("<HH", addr, width)
    for pid, payload in zip(ids, payloads):
        if len(payload) != width:
            raise ValueError("all sync_write payloads must have the same width")
        params += bytes([pid]) + payload
    return dxl_instruction(DXL_BROADCAST, DXL_INST["sync_write"], params)


def dxl_fast_sync_read(ids: Sequence[int], addr: int, length: int) -> bytes:
    """The 0x8A instruction: byte for byte a SYNC_READ with a different opcode.

    The *answer* is not: the devices build one status packet between them (id
    `0xFE`), each appending `ERROR ID DATA CRC`, the CRC covering everything up to
    its own block - see [`dxl_parse_fast_sync_read`].  Nothing is byte-stuffed.
    """
    params = struct.pack("<HH", addr, length) + bytes(ids)
    return dxl_instruction(DXL_BROADCAST, DXL_INST["fast_sync_read"], params)


def dxl_fast_sync_block(pid: int, data: bytes, error: int = 0, upto: bytes = b"") -> bytes:
    """One device's contribution to a Fast Sync Read packet.

    `upto` is everything already on the wire (the prefix and the blocks ahead of
    this one); the CRC that ends the block covers it plus this block.
    """
    block = bytes([error, pid]) + data
    return block + struct.pack("<H", crc16_dxl(upto + block))


def dxl_parse_fast_sync_read(frame: bytes, ids: Sequence[int],
                             length: int) -> list[tuple[bytes, int]]:
    """Split an aggregate Fast Sync Read packet into `[(data, error), ...]`.

    Transcribed from rustypot 1.8's `parse_fast_sync_read_status_with_error`:
    every block's running CRC is checked (which is what pins the block positions)
    and the id in a block must be the one that was asked for in that slot.  Raises
    `ProtocolError` when anything does not line up.
    """
    block_size = length + 4
    if len(frame) != 8 + len(ids) * block_size:
        raise ProtocolError(f"aggregate length {len(frame)} != {8 + len(ids) * block_size}")
    if frame[4] != DXL_BROADCAST or frame[7] != 0x55:
        raise ProtocolError("aggregate packet must answer as 0xFE with 0x55")
    if struct.unpack_from("<H", frame, 5)[0] != len(frame) - 7:
        raise ProtocolError("aggregate LEN does not match the packet")
    out: list[tuple[bytes, int]] = []
    for i, pid in enumerate(ids):
        block = 8 + i * block_size
        crc_at = block + 2 + length
        if struct.unpack_from("<H", frame, crc_at)[0] != crc16_dxl(frame[:crc_at]):
            raise ProtocolError(f"block {i} CRC mismatch")
        if frame[block + 1] != pid:
            raise ProtocolError(f"block {i} is id {frame[block + 1]}, expected {pid}")
        out.append((bytes(frame[block + 2:crc_at]), frame[block]))
    return out


def dxl_parse_status(frame: bytes) -> tuple[int, int, bytes]:
    """Returns (id, error_byte, params).  Raises ProtocolError on a bad frame."""
    if len(frame) < 11:
        raise ProtocolError(f"status too short: {len(frame)}")
    if frame[0:4] != b"\xFF\xFF\xFD\x00":
        raise ProtocolError("bad header")
    if frame[7] != 0x55:
        raise ProtocolError(f"missing 0x55 status marker (got {frame[7]:#04x})")
    length = struct.unpack_from("<H", frame, 5)[0]
    if length != len(frame) - 7:
        raise ProtocolError(f"length mismatch: LEN={length} frame={len(frame)}")
    want = struct.unpack_from("<H", frame, len(frame) - 2)[0]
    got = crc16_dxl(frame[:-2])
    if want != got:
        raise ProtocolError(f"crc mismatch: got {got:#06x} want {want:#06x}")
    # body = error byte + parameters, byte-stuffed; de-stuff only after the CRC
    # (rustypot 1.6: `remove_stuffing(&data[8..msg_length - 2])`)
    body = dxl_remove_stuffing(bytes(frame[8:-2]))
    if not body:
        raise ProtocolError("empty status body")
    return frame[4], body[0], bytes(body[1:])


# ── FeeTech SCS/HLS ─────────────────────────────────────────────────────────


def fee_instruction(packet_id: int, instruction: int, params: bytes = b"") -> bytes:
    """FF FF ID LEN INST [ADDR DATA...] ~SUM, LEN = INST + params + SUM."""
    body = bytes([packet_id, len(params) + 2, instruction]) + params
    checksum = (~sum(body)) & 0xFF
    return b"\xFF\xFF" + body + bytes([checksum])


def fee_ping(packet_id: int) -> bytes:
    return fee_instruction(packet_id, FEE_INST["ping"])


def fee_read(packet_id: int, addr: int, length: int) -> bytes:
    return fee_instruction(packet_id, FEE_INST["read"], bytes([addr, length]))


def fee_write(packet_id: int, addr: int, data: bytes) -> bytes:
    return fee_instruction(packet_id, FEE_INST["write"], bytes([addr]) + data)


def fee_sync_read(ids: Sequence[int], addr: int, length: int) -> bytes:
    return fee_instruction(
        FEE_BROADCAST, FEE_INST["sync_read"], bytes([addr, length]) + bytes(ids)
    )


def fee_sync_write(ids: Sequence[int], addr: int, payloads: Sequence[bytes]) -> bytes:
    if not ids:
        raise ValueError("sync_write needs at least one id")
    width = len(payloads[0])
    params = bytes([addr, width])
    for pid, payload in zip(ids, payloads):
        if len(payload) != width:
            raise ValueError("all sync_write payloads must have the same width")
        params += bytes([pid]) + payload
    return fee_instruction(FEE_BROADCAST, FEE_INST["sync_write"], params)


def fee_parse_ack(frame: bytes) -> tuple[int, int, bytes]:
    """Returns (id, status, data)."""
    if len(frame) < 6:
        raise ProtocolError(f"ack too short: {len(frame)}")
    if frame[0:2] != b"\xFF\xFF":
        raise ProtocolError("bad header")
    length = frame[3]
    if length != len(frame) - 4:
        raise ProtocolError(f"length mismatch: LEN={length} frame={len(frame)}")
    if ((~sum(frame[2:-1])) & 0xFF) != frame[-1]:
        raise ProtocolError("checksum mismatch")
    return frame[2], frame[4], bytes(frame[5:-1])


# ── quaternion maths (same conventions as src/imu_math.c) ───────────────────


def half_to_float(bits: int) -> float:
    """IEEE 754 binary16 -> float."""
    sign = -1.0 if bits & 0x8000 else 1.0
    exponent = (bits >> 10) & 0x1F
    fraction = bits & 0x3FF
    if exponent == 0:
        return sign * fraction * 2.0 ** -24
    if exponent == 0x1F:
        return sign * math.inf if fraction == 0 else math.nan
    return sign * (1.0 + fraction / 1024.0) * 2.0 ** (exponent - 15)


def float_to_half(value: float) -> int:
    """Round-to-nearest-even float -> binary16 (used by the self tests)."""
    import numpy as np  # only for the self test; not needed at runtime

    return int(np.float16(value).view(np.uint16))


def q_mul(a: Sequence[float], b: Sequence[float]) -> tuple[float, float, float, float]:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


def q_conj(q: Sequence[float]) -> tuple[float, float, float, float]:
    return (q[0], -q[1], -q[2], -q[3])


def q_normalize(q: Sequence[float]) -> tuple[float, float, float, float]:
    n = math.sqrt(sum(c * c for c in q))
    if n <= 0.0:
        return (1.0, 0.0, 0.0, 0.0)
    return tuple(c / n for c in q)  # type: ignore[return-value]


def q_rotate(q: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    """v' = q v q⁻¹"""
    w, x, y, z = q
    t = (
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    )
    c = (
        y * t[2] - z * t[1],
        z * t[0] - x * t[2],
        x * t[1] - y * t[0],
    )
    return (
        v[0] + w * t[0] + c[0],
        v[1] + w * t[1] + c[1],
        v[2] + w * t[2] + c[2],
    )


def q_rotate_inv(q: Sequence[float], v: Sequence[float]) -> tuple[float, float, float]:
    """v' = q⁻¹ v q"""
    w, x, y, z = q
    t = (
        2.0 * (y * v[2] - z * v[1]),
        2.0 * (z * v[0] - x * v[2]),
        2.0 * (x * v[1] - y * v[0]),
    )
    c = (
        y * t[2] - z * t[1],
        z * t[0] - x * t[2],
        x * t[1] - y * t[0],
    )
    return (
        v[0] - w * t[0] + c[0],
        v[1] - w * t[1] + c[1],
        v[2] - w * t[2] + c[2],
    )


def euler_from_quat(q: Sequence[float]) -> tuple[float, float, float]:
    """ZYX (yaw-pitch-roll) in degrees, for display only."""
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))


# ── the block, decoded ──────────────────────────────────────────────────────


@dataclass
class ImuBlock:
    """One 20-byte block, still in the chip frame."""

    gyro_raw: tuple[int, int, int]
    quat_half: tuple[int, int, int]
    accel_raw: tuple[int, int, int]
    counter: int
    flags: int

    @classmethod
    def decode(cls, data: bytes, protocol: str = PROTO_DXL) -> "ImuBlock":
        """Decode one block.

        Two wire shapes exist (src/board.h, docs/bus_timing_borrow_plan.md):

          * the FeeTech 56..70 *contract* block, 15 bytes: the 12 control bytes
            plus the sample counter and the status flags.  That is the span the
            runtime reads from the IMU node and from every servo in one
            `sync_read`, so the counter and status have to live inside it, and
            the raw accelerometer does not fit (it is in the 128 alias);
          * the 20-byte diagnostic block at FeeTech 128 / Dynamixel 124, which
            adds the raw accelerometer.
        """
        if (protocol == PROTO_FEE) and (len(data) == FEE_BLOCK_LEN):
            g = struct.unpack_from("<hhh", data, 0)
            h = struct.unpack_from("<HHH", data, 6)
            return cls(g, h, (0, 0, 0), data[FEE_BLOCK_CNT], data[FEE_BLOCK_STATUS])

        if len(data) < CTRL_LEN:
            raise ProtocolError(f"block too short: {len(data)}")
        padded = bytes(data) + bytes(max(0, TELEM_LEN - len(data)))
        g = struct.unpack_from("<hhh", padded, 0)
        h = struct.unpack_from("<HHH", padded, 6)
        a = struct.unpack_from("<hhh", padded, 12)
        return cls(g, h, a, padded[18], padded[19])

    @property
    def gyro_dps(self) -> tuple[float, float, float]:
        return tuple(g * GYRO_MDPS_PER_LSB / 1000.0 for g in self.gyro_raw)  # type: ignore

    @property
    def gyro_rad(self) -> tuple[float, float, float]:
        return tuple(math.radians(g) for g in self.gyro_dps)  # type: ignore

    @property
    def accel_mg(self) -> tuple[float, float, float]:
        return tuple(a * ACCEL_MG_PER_LSB for a in self.accel_raw)  # type: ignore

    @property
    def quat_xyz(self) -> tuple[float, float, float]:
        return tuple(half_to_float(h) for h in self.quat_half)  # type: ignore

    @property
    def quat_raw(self) -> tuple[float, float, float, float]:
        x, y, z = self.quat_xyz
        return (math.sqrt(max(0.0, 1.0 - (x * x + y * y + z * z))), x, y, z)

    @property
    def quat_valid(self) -> bool:
        return bool(self.flags & TELEM_FLAG_BITS["sflp_valid"])

    def flag_names(self) -> list[str]:
        return [name for name, bit in TELEM_FLAG_BITS.items() if self.flags & bit]


class ImuDecoder:
    """Applies the mounting rotation and the hold-last-good rule, exactly like
    microduck's `SflpDecoder`, and keeps the health counters the host tool shows."""

    def __init__(
        self,
        mount: Sequence[float] = DEFAULT_MOUNT,
        report_frame: str = "chip",
    ) -> None:
        self.mount = tuple(mount)
        self.report_frame = report_frame
        self.last_quat = (1.0, 0.0, 0.0, 0.0)
        self.quat_samples = 0
        self.total = 0
        self.stale_total = 0
        self.stale_run = 0
        self.max_stale_run = 0
        self._last_block: Optional[bytes] = None
        self.counter_jumps = 0
        self._last_counter: Optional[int] = None

    def decode(self, blk: ImuBlock, raw_bytes: bytes = b"") -> dict:
        self.total += 1

        if raw_bytes:
            if self._last_block == raw_bytes[:TELEM_LEN]:
                self.stale_total += 1
                self.stale_run += 1
                self.max_stale_run = max(self.max_stale_run, self.stale_run)
            else:
                self.stale_run = 0
            self._last_block = raw_bytes[:TELEM_LEN]

        if self._last_counter is not None and blk.counter != (self._last_counter + 1) & 0xFF:
            self.counter_jumps += 1
        self._last_counter = blk.counter

        if blk.quat_valid:
            q = blk.quat_raw
            if self.report_frame == "chip":
                q = q_mul(q, q_conj(self.mount))
            self.last_quat = q_normalize(q)
            self.quat_samples += 1

        if self.report_frame == "chip":
            gyro_trunk = q_rotate(self.mount, blk.gyro_rad)
        else:
            gyro_trunk = blk.gyro_rad

        gravity = q_normalize(q_rotate_inv(self.last_quat, (0.0, 0.0, -1.0)))
        accel = blk.accel_mg
        if self.report_frame == "chip":
            accel = q_rotate(self.mount, accel)

        roll, pitch, yaw = euler_from_quat(self.last_quat)

        return {
            "gyro_dps": list(blk.gyro_dps),
            "gyro_trunk_dps": list(math.degrees(g) for g in gyro_trunk),
            "quat_raw": list(blk.quat_raw),
            "quat": list(self.last_quat),
            "gravity": list(gravity),
            "accel_mg": list(accel),
            "euler": [roll, pitch, yaw],
            "counter": blk.counter,
            "flags": blk.flags,
            "flag_names": blk.flag_names(),
            "quat_valid": blk.quat_valid,
            "ready": self.quat_samples >= 25,
            "stale_run": self.stale_run,
            "stale_total": self.stale_total,
            "samples": self.total,
        }


# ── serial link ─────────────────────────────────────────────────────────────


@dataclass
class LinkStats:
    polls: int = 0
    ok: int = 0
    timeouts: int = 0
    bad_frames: int = 0
    extra_bytes: int = 0
    last_error: str = ""
    latencies: list[float] = field(default_factory=list)

    def latency_summary(self) -> dict:
        if not self.latencies:
            return {"n": 0}
        values = sorted(self.latencies)
        n = len(values)
        return {
            "n": n,
            "min_ms": values[0],
            "median_ms": values[n // 2],
            "p95_ms": values[min(n - 1, int(n * 0.95))],
            "max_ms": values[-1],
        }


class Link:
    """One serial link to the node, speaking either protocol."""

    def __init__(
        self,
        port: str,
        baud: int = 1_000_000,
        protocol: str = PROTO_DXL,
        imu_id: int = 200,
        read_len: int = CTRL_LEN,
        mode: str = "sync_read",
        timeout: float = 0.05,
    ) -> None:
        self.port = port
        self.baud = baud
        self.protocol = protocol
        self.imu_id = imu_id
        self.read_len = read_len
        self.mode = mode
        self.timeout = timeout
        self.stats = LinkStats()
        self.decoder = ImuDecoder()
        self._ser = None
        self._rx = bytearray()

    # -- lifecycle

    #: A USB-serial adapter needs a moment after a close -> open cycle: measured
    #: with the CH340 on this bench, a transaction issued immediately after a
    #: reopen times out, while one issued 0.4 s later succeeds.  Cheap insurance
    #: for a tool where a user clicks Disconnect then Connect.
    OPEN_SETTLE_S = 0.35

    def open(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed")
        self._ser = serial.Serial(self.port, self.baud, timeout=0.0, write_timeout=0.2)
        time.sleep(self.OPEN_SETTLE_S)
        self._ser.reset_input_buffer()
        self._ser.reset_output_buffer()
        self._rx.clear()

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self) -> "Link":
        self.open()
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    # -- framing

    @property
    def telemetry_addr(self) -> int:
        """Address of the 20-byte diagnostic block this tool polls."""
        return DXL_DIAG_ADDR if self.protocol == PROTO_DXL else FEE_DIAG_ADDR

    @property
    def contract_addr(self) -> int:
        """Address of the block the runtime shares with the servos."""
        return DXL_TELEM_ADDR if self.protocol == PROTO_DXL else FEE_TELEM_ADDR

    @property
    def vendor_addr(self) -> int:
        return DXL_VENDOR_ADDR if self.protocol == PROTO_DXL else FEE_VENDOR_ADDR

    def _read_frame(self) -> bytes:
        """Read exactly one status/ack frame, tolerating debug text and noise."""
        assert self._ser is not None
        deadline = time.monotonic() + self.timeout
        while True:
            frame, consumed = self._extract()
            if frame is not None:
                del self._rx[:consumed]
                return frame
            if consumed:
                self.stats.extra_bytes += consumed
                del self._rx[:consumed]
            if time.monotonic() >= deadline:
                raise TimeoutError("no frame")
            chunk = self._ser.read(256)
            if chunk:
                self._rx.extend(chunk)

    def _extract(self) -> tuple[Optional[bytes], int]:
        """(frame, bytes_to_drop) - scans the buffer for a complete frame."""
        buf = self._rx
        if self.protocol == PROTO_DXL:
            start = buf.find(b"\xFF\xFF\xFD\x00")
            if start < 0:
                # keep a possible partial header
                drop = max(0, len(buf) - 3)
                return None, drop
            if len(buf) < start + 7:
                return None, start
            length = int.from_bytes(buf[start + 5 : start + 7], "little")
            total = 7 + length
            if length < 3 or total > DXL_MAX_FRAME:
                return None, start + 2
            if len(buf) < start + total:
                return None, start
            frame = bytes(buf[start : start + total])
            try:
                dxl_parse_status(frame)
            except ProtocolError:
                self.stats.bad_frames += 1
                return None, start + 2
            return frame, start + total

        # FeeTech: FF FF ID LEN ...
        idx = 0
        while True:
            start = buf.find(b"\xFF\xFF", idx)
            if start < 0:
                drop = max(0, len(buf) - 1)
                return None, drop
            if len(buf) < start + 4:
                return None, start
            length = buf[start + 3]
            total = length + 4
            if length < 2 or total > 260:
                idx = start + 1
                continue
            if len(buf) < start + total:
                return None, start
            frame = bytes(buf[start : start + total])
            try:
                fee_parse_ack(frame)
            except ProtocolError:
                self.stats.bad_frames += 1
                idx = start + 1
                continue
            return frame, start + total

    def _transaction(self, request: bytes) -> bytes:
        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write(request)
        return self._read_frame()

    # -- transactions

    def ping(self) -> bool:
        request = dxl_ping(self.imu_id) if self.protocol == PROTO_DXL else fee_ping(self.imu_id)
        try:
            frame = self._transaction(request)
            if self.protocol == PROTO_DXL:
                packet_id, err, _ = dxl_parse_status(frame)
            else:
                packet_id, status, _ = fee_parse_ack(frame)
        except (TimeoutError, ProtocolError) as exc:
            self.stats.last_error = str(exc) or "timeout"
            return False
        if self.protocol == PROTO_DXL:
            return packet_id == self.imu_id and err == 0
        return packet_id == self.imu_id and status == 0

    def read_registers(self, addr: int, length: int) -> bytes:
        if self.protocol == PROTO_DXL:
            frame = self._transaction(dxl_read(self.imu_id, addr, length))
            pid, err, params = dxl_parse_status(frame)
        else:
            frame = self._transaction(fee_read(self.imu_id, addr, length))
            pid, err, params = fee_parse_ack(frame)
        if pid != self.imu_id:
            raise ProtocolError(f"reply from id {pid}, expected {self.imu_id}")
        if err:
            raise ProtocolError(f"device error {err:#04x} reading {addr}+{length}")
        if len(params) != length:
            raise ProtocolError(f"short read: {len(params)} != {length}")
        return params

    def read_contract(self) -> tuple[bytes, int, int]:
        """Read the block the runtime shares with the servos.

        Returns `(control_bytes, sample_counter, status_flags)`.  On the FeeTech
        map that is the 15-byte block at 56 - the same span a servo answers with
        position/speed/load/voltage/temperature/status/current - which is why
        the counter and status live inside it; the accelerometer is only in the
        diagnostic block (read [`telemetry_addr`]).  On Dynamixel it is the
        20-byte block at 124, whose first 12 bytes are what microduck reads.
        """
        if self.protocol == PROTO_FEE:
            raw = self.read_registers(FEE_TELEM_ADDR, FEE_BLOCK_LEN)
            return raw[:CTRL_LEN], raw[FEE_BLOCK_CNT], raw[FEE_BLOCK_STATUS]
        raw = self.read_registers(DXL_TELEM_ADDR, TELEM_LEN)
        return raw[:CTRL_LEN], raw[18], raw[19]

    def read_registers_raw(self, addr: int, length: int) -> tuple[bytes, bytes]:
        """Read registers and also return the wire frame.

        The frame is what the stuffing tests need: protocol 2.0 status packets
        are byte-stuffed, so a payload containing `FF FF FD` arrives longer than
        the payload it carries.
        """
        if self.protocol == PROTO_DXL:
            frame = self._transaction(dxl_read(self.imu_id, addr, length))
            _pid, _err, params = dxl_parse_status(frame)
        else:
            frame = self._transaction(fee_read(self.imu_id, addr, length))
            _pid, _err, params = fee_parse_ack(frame)
        return frame, params

    def fast_sync_read(self, ids: Sequence[int], addr: int, length: int,
                       feed: bytes = b"", want: Optional[int] = None) -> bytes:
        """Send a Fast Sync Read (0x8A) and return the node's raw answer.

        A Fast Sync Read status packet is built by every addressed device in turn,
        so what comes back depends on where this node sits in the id list:

        * **first** (index 0, which is where microduck puts the IMU node): the
          node sends the 8-byte prefix and its own block straight away.  The
          default `want=None` reads one whole aggregate packet, which only
          completes if every id in the list answers - on a bench with no servos
          pass `want=length + 12` (prefix + one block) or let a caller append the
          missing blocks;
        * **later**: the node stays silent until it has heard the blocks ahead of
          it, so those have to be on the wire right behind the instruction -
          `feed` carries them (`dxl_fast_sync_block` builds one).  The answer is
          then `length + 4` bytes with no header of its own.
        """
        assert self._ser is not None
        self._ser.reset_input_buffer()
        self._ser.write(dxl_fast_sync_read(ids, addr, length) + feed)
        if want is None:
            return self._read_frame()
        deadline = time.monotonic() + self.timeout
        buf = bytearray()
        # Anything the scanner was still holding belongs to an earlier, abandoned
        # transaction: `_read_frame` leaves a partial frame in `self._rx` when it
        # times out, and seeding this read with those bytes would splice the tail
        # of an old reply onto the front of this one (measured: it decoded as a
        # block with a stale counter and a flag byte that is never set).
        self._rx.clear()
        while len(buf) < want:
            chunk = self._ser.read(want - len(buf))
            if chunk:
                buf.extend(chunk)
            elif time.monotonic() >= deadline:
                raise TimeoutError(f"fast sync read: {len(buf)}/{want} bytes")
        return bytes(buf)

    def write_registers(self, addr: int, data: bytes, expect_ack: bool = True) -> bool:
        """Write registers.  Returns True when the node acknowledged.

        `expect_ack=False` is for the FeeTech ACK-status-level tests: with
        register 8 at 0 the servo (and this node) executes writes silently, so a
        timeout is the correct answer rather than an error.
        """
        try:
            if self.protocol == PROTO_DXL:
                frame = self._transaction(dxl_write(self.imu_id, addr, data))
                pid, err, _ = dxl_parse_status(frame)
            else:
                frame = self._transaction(fee_write(self.imu_id, addr, data))
                pid, err, _ = fee_parse_ack(frame)
        except TimeoutError:
            if not expect_ack:
                return False
            raise
        if pid != self.imu_id:
            raise ProtocolError(f"reply from id {pid}, expected {self.imu_id}")
        if err:
            raise ProtocolError(f"device error {err:#04x} writing {addr}")
        if not expect_ack:
            raise ProtocolError(f"expected no ack for {addr}, but the node answered")
        return True

    def poll(self) -> Optional[dict]:
        """One telemetry transaction.  Returns a decoded sample dict or None."""
        started = time.monotonic()
        if self.protocol == PROTO_DXL:
            if self.mode == "sync_read":
                request = dxl_sync_read([self.imu_id], self.telemetry_addr, self.read_len)
            else:
                request = dxl_read(self.imu_id, self.telemetry_addr, self.read_len)
        else:
            if self.mode == "sync_read":
                request = fee_sync_read([self.imu_id], self.telemetry_addr, self.read_len)
            else:
                request = fee_read(self.imu_id, self.telemetry_addr, self.read_len)

        self.stats.polls += 1
        try:
            frame = self._transaction(request)
            if self.protocol == PROTO_DXL:
                pid, err, params = dxl_parse_status(frame)
            else:
                pid, err, params = fee_parse_ack(frame)
        except TimeoutError:
            self.stats.timeouts += 1
            self.stats.last_error = "timeout"
            return None
        except ProtocolError as exc:
            self.stats.bad_frames += 1
            self.stats.last_error = str(exc)
            return None

        if pid != self.imu_id:
            self.stats.bad_frames += 1
            self.stats.last_error = f"id {pid}"
            return None

        try:
            block = ImuBlock.decode(params, self.protocol)
        except ProtocolError as exc:
            self.stats.bad_frames += 1
            self.stats.last_error = str(exc)
            return None

        elapsed_ms = (time.monotonic() - started) * 1000.0
        self.stats.latencies.append(elapsed_ms)
        if len(self.stats.latencies) > 2000:
            del self.stats.latencies[:1000]
        self.stats.ok += 1

        sample = self.decoder.decode(block, params)
        sample["latency_ms"] = elapsed_ms
        sample["t"] = time.time()
        sample["error"] = err
        return sample

    # -- configuration (vendor window)

    def get_config(self) -> dict:
        raw = self.read_registers(self.vendor_addr, TELEM_LEN)
        status = raw[VOFF["status"]]
        return {
            "magic": struct.unpack_from("<H", raw, VOFF["magic_l"])[0],
            "sim_mode": raw[VOFF["sim_mode"]],
            "sim_mode_name": SIM_MODES.get(raw[VOFF["sim_mode"]], "?"),
            "sim_amp": struct.unpack_from("<H", raw, VOFF["amp_l"])[0],
            "sim_freq": struct.unpack_from("<H", raw, VOFF["freq_l"])[0],
            "report_frame": raw[VOFF["report_frame"]],
            "proto_lock": raw[VOFF["proto_lock"]],
            "gyro_bias": [
                struct.unpack_from("<h", raw, VOFF[f"bias_{axis}_l"])[0] for axis in "xyz"
            ],
            "mirror_baud": raw[VOFF["mirror_baud"]],
            "dbg_level": raw[VOFF["dbg_level"]],
            "status": status,
            "cfg_dirty": bool(status & 0x01),
            "cfg_from_flash": bool(status & 0x02),
            "sim_active": bool(status & 0x04),
            "raw": list(raw),
        }

    def set_config(self, **fields: object) -> None:
        """Write individual vendor-window fields (any of sim_mode, sim_amp, ...)."""
        current = bytearray(self.read_registers(self.vendor_addr, TELEM_LEN))
        for name, value in fields.items():
            if name == "sim_mode":
                current[VOFF["sim_mode"]] = SIM_MODE_IDS[str(value)] if isinstance(value, str) else int(value)  # type: ignore[arg-type]
            elif name == "sim_amp":
                struct.pack_into("<H", current, VOFF["amp_l"], int(value))  # type: ignore[arg-type]
            elif name == "sim_freq":
                struct.pack_into("<H", current, VOFF["freq_l"], int(value))  # type: ignore[arg-type]
            elif name == "report_frame":
                current[VOFF["report_frame"]] = int(value)  # type: ignore[arg-type]
            elif name == "proto_lock":
                current[VOFF["proto_lock"]] = {"none": 0, "dxl": 1, "fee": 2}[str(value)]
            elif name == "gyro_bias":
                bx, by, bz = value  # type: ignore[misc]
                struct.pack_into("<h", current, VOFF["bias_x_l"], int(bx))
                struct.pack_into("<h", current, VOFF["bias_y_l"], int(by))
                struct.pack_into("<h", current, VOFF["bias_z_l"], int(bz))
            elif name == "dbg_level":
                current[VOFF["dbg_level"]] = int(value)  # type: ignore[arg-type]
            elif name == "cmd":
                current[VOFF["cmd"]] = int(value)  # type: ignore[arg-type]
            else:
                raise ValueError(f"unknown configuration field {name!r}")
        self.write_registers(self.vendor_addr, bytes(current))

    def device_info(self) -> dict:
        """Model / firmware version, read through each protocol's own registers.

        `firmware` is always the nibble-packed byte the tools already understand
        (`0x10` = 1.0): the Dynamixel control table stores exactly that, and the
        FeeTech map stores major/minor in registers 0/1, so it is packed the same
        way here.  A real HD-1910 answers 3.46 there; this node answers its own
        version (`firmware_str` has the readable form).
        """
        if self.protocol == PROTO_DXL:
            info = self.read_registers(0, 7)
            return {
                "model": struct.unpack_from("<H", info, 0)[0],
                "firmware": info[6],
                "firmware_str": f"{(info[6] >> 4) & 0x0F}.{info[6] & 0x0F}",
                "raw": list(info),
            }
        info = self.read_registers(0, 6)
        return {
            "model": struct.unpack_from("<H", info, 3)[0],
            "firmware": ((info[0] & 0x0F) << 4) | (info[1] & 0x0F),
            "firmware_str": f"{info[0]}.{info[1]}",
            "raw": list(info),
        }


# ── self test ───────────────────────────────────────────────────────────────


def _self_test() -> int:
    """Codec vectors: identical to the C tests, so both sides agree byte for byte."""
    failures = 0

    def check(name: str, got: object, want: object) -> None:
        nonlocal failures
        if got != want:
            print(f"  FAIL {name}: got {got!r} want {want!r}")
            failures += 1

    check("crc ping", f"{crc16_dxl(bytes([0xFF,0xFF,0xFD,0x00,0x02,0x03,0x00,0x01])):#06x}", "0x7219")
    check("ping id=2", dxl_ping(2).hex(" ").upper(),
          "FF FF FD 00 02 03 00 01 19 72")
    check("read id=1 addr=0x2B", dxl_read(1, 0x2B, 2).hex(" ").upper(),
          "FF FF FD 00 01 07 00 02 2B 00 02 00 2E CD")
    check("write id=1 addr=116", dxl_write(1, 116, struct.pack("<I", 512)).hex(" ").upper(),
          "FF FF FD 00 01 09 00 03 74 00 00 02 00 00 CA 89")
    check("sync_read [1,2] addr=132", dxl_sync_read([1, 2], 132, 4).hex(" ").upper(),
          "FF FF FD 00 FE 09 00 82 84 00 04 00 01 02 CE FA")
    check("sync_write [1,2] addr=116",
          dxl_sync_write([1, 2], 116, [struct.pack("<I", 150), struct.pack("<I", 170)]).hex(" ").upper(),
          "FF FF FD 00 FE 11 00 83 74 00 04 00 01 96 00 00 00 02 AA 00 00 00 82 87")
    check("status parse", dxl_parse_status(bytes.fromhex("FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C0")),
          (1, 0, bytes([0xA6, 0, 0, 0])))
    check("status parse rejects bad crc",
          _raises(lambda: dxl_parse_status(bytes.fromhex("FF FF FD 00 01 08 00 55 00 A6 00 00 00 8C C1"))),
          True)

    # Fast Sync Read: one aggregate packet, built by all the devices in turn.
    # The vector is the protocol 2.0 e-manual's (and rustypot's) example: ids
    # 3/7/4 answer a read of present position (addr 132, 4 bytes).
    fast_example = bytes.fromhex(
        "FF FF FD 00 FE 19 00 55 00 03 A6 00 00 00 84 08"
        " 00 07 1F 08 00 00 16 CA 00 04 FF 03 00 00 D1 9E")
    check("fast_sync_read [1,2] addr=132", dxl_fast_sync_read([1, 2], 132, 4).hex(" ").upper(),
          "FF FF FD 00 FE 09 00 8A 84 00 04 00 01 02 4D 72")
    check("fast_sync_read parse (e-manual)",
          dxl_parse_fast_sync_read(fast_example, [3, 7, 4], 4),
          [(bytes([0xA6, 0, 0, 0]), 0), (bytes([0x1F, 8, 0, 0]), 0), (bytes([0xFF, 3, 0, 0]), 0)])
    flipped = fast_example[:10] + bytes([fast_example[10] ^ 0x01]) + fast_example[11:]
    check("fast_sync_read rejects a flipped data byte",
          _raises(lambda: dxl_parse_fast_sync_read(flipped, [3, 7, 4], 4)), True)
    check("fast_sync_read rejects ids in the wrong order",
          _raises(lambda: dxl_parse_fast_sync_read(fast_example, [3, 4, 7], 4)), True)
    check("fast_sync_read rejects a packet one block short",
          _raises(lambda: dxl_parse_fast_sync_read(fast_example[:24], [3, 7, 4], 4)), True)
    check("fast_sync_read block builder (id 3, first)",
          dxl_fast_sync_block(3, bytes([0xA6, 0, 0, 0]), 0,
                              fast_example[:8]).hex(" ").upper(),
          fast_example[8:16].hex(" ").upper())

    # byte stuffing (rustypot 1.6 de-stuffs status bodies, so the node must stuff)
    check("stuffing: no pattern", dxl_add_stuffing(bytes([1, 2, 0xFF, 0xFD, 3])).hex(" ").upper(),
          "01 02 FF FD 03")
    check("stuffing: FF FF FD", dxl_add_stuffing(bytes([0xFF, 0xFF, 0xFD, 7])).hex(" ").upper(),
          "FF FF FD FD 07")
    check("stuffing: scan restarts", dxl_add_stuffing(bytes([0xFF, 0xFF, 0xFD, 0xFD])).hex(" ").upper(),
          "FF FF FD FD FD")
    check("stuffing: suffix survives FFs",
          dxl_add_stuffing(bytes([0xFF, 0xFF, 0xFF, 0xFD])).hex(" ").upper(),
          "FF FF FF FD FD")
    payload = bytes([0xFF, 0xFF, 0xFD, 0x7F]) + bytes(range(4, 12))
    check("stuffing: round trip", dxl_remove_stuffing(dxl_add_stuffing(payload)), payload)

    # FeeTech: the node's own answers (id 200, block 0..11)
    check("fee ping id=200", fee_ping(200).hex(" ").upper(), "FF FF C8 02 01 34")
    check("fee read 56/12", fee_read(200, 56, 12).hex(" ").upper(), "FF FF C8 04 02 38 0C ED")
    check("fee ack parse", fee_parse_ack(bytes.fromhex("FF FF C8 02 00 35")), (200, 0, b""))

    # half precision, matching the firmware's f32_to_half
    check("half 1.0", "%04X" % float_to_half(1.0), "3C00")
    check("half -1.0", "%04X" % float_to_half(-1.0), "BC00")
    check("half 0.5", "%04X" % float_to_half(0.5), "3800")
    check("half -> float 0x3555", round(half_to_float(0x3555), 3), 0.333)

    # mount round trip: a chip-frame vector rotated by mount must give the trunk vector
    q = DEFAULT_MOUNT
    check("quat norm", round(math.sqrt(sum(c * c for c in q)), 6), 1.0)
    v = (1.0, 2.0, 3.0)
    back = q_rotate_inv(q, q_rotate(q, v))
    check("rotate round trip", tuple(round(c, 9) for c in back), v)

    if failures:
        print(f"{failures} FAILURE(S)")
        return 1
    print("host codecs: all vectors match the firmware")
    return 0


def _raises(fn) -> bool:  # type: ignore[no-untyped-def]
    try:
        fn()
    except Exception:
        return True
    return False


if __name__ == "__main__":
    raise SystemExit(_self_test())
