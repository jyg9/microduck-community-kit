#!/usr/bin/env python3
"""REBOOT probe for a real FeeTech HLS/SCS servo (instruction 0x08).

Question it answers (docs/bus_timing_borrow_plan.md §6 item 4, and the peer
project's `飞特适配架构.md` §3.4): does this servo's firmware actually implement
the 0x08 "reboot" a 2026-06 manual documents, does it answer, how long does it
take to come back, and what survives the reboot?

The servo is on the bench with nothing attached to its output, so a reboot is
harmless.  Writes are limited to the two *RAM* gain registers (50/51, which do
not persist by design) so the "RAM gains revert to EEPROM on reboot" part of the
`RobotIo::reboot` contract can be checked; they are restored afterwards.  No
torque, no motion, no EEPROM write, no RESET/CAL.

Usage:
    host/tools/py.sh host/tools/servo_reboot_probe.py --port /dev/ttyACM0 --id 1
"""

from __future__ import annotations

import argparse
import os
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
if HOST not in sys.path:
    sys.path.insert(0, HOST)

import bus  # noqa: E402

FEE_INST_REBOOT = 0x08
ADDR_P_EEPROM, ADDR_D_EEPROM = 21, 22
ADDR_TORQUE, ADDR_MODE, ADDR_LOCK = 40, 33, 55
ADDR_KP_RAM, ADDR_KD_RAM = 50, 51
ADDR_ID, ADDR_BAUD, ADDR_OFS = 5, 6, 31

REGS = [
    (ADDR_ID, 1, "ID"),
    (ADDR_BAUD, 1, "baud code"),
    (ADDR_MODE, 1, "mode"),
    (ADDR_TORQUE, 1, "torque"),
    (ADDR_LOCK, 1, "lock"),
    (ADDR_P_EEPROM, 1, "P (EEPROM 21)"),
    (ADDR_D_EEPROM, 1, "D (EEPROM 22)"),
    (ADDR_KP_RAM, 1, "Kp (RAM 50)"),
    (ADDR_KD_RAM, 1, "Kd (RAM 51)"),
    (ADDR_OFS, 2, "OFS (31)"),
]


def snapshot(link: "bus.Link") -> dict:
    out = {}
    for addr, ln, what in REGS:
        try:
            out[what] = int.from_bytes(link.read_registers(addr, ln), "little")
        except Exception as exc:  # noqa: BLE001
            out[what] = f"err:{exc}"
    return out


def diff(before: dict, after: dict) -> None:
    for key in before:
        mark = "same" if before[key] == after[key] else "CHANGED"
        print(f"   {key:16s} {str(before[key]):>8s} -> {str(after[key]):<8s} {mark}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyACM0")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--id", type=int, default=1)
    args = ap.parse_args()

    if not os.path.exists(args.port):
        print(f"[!] {args.port} does not exist (sandbox masks /dev - run with full access)")
        return 2

    link = bus.Link(port=args.port, protocol=bus.PROTO_FEE, imu_id=args.id,
                    read_len=15, timeout=0.05)
    link.open()
    try:
        print("=" * 72)
        print(f"REBOOT probe (0x08) on {args.port} id={args.id}")
        print("=" * 72)
        print("\n1. before")
        before = snapshot(link)
        for key, val in before.items():
            print(f"   {key:16s} {val}")

        # RAM gains: distinct values so a revert is unmistakable.  50/51 are the
        # RAM mirrors of the EEPROM position gains (21/22), not persisted.
        print("\n2. write RAM gains Kp=99 Kd=88 (RAM only, no torque, no motion)")
        link.write_registers(ADDR_KP_RAM, bytes([99]))
        link.write_registers(ADDR_KD_RAM, bytes([88]))
        print("   now:", link.read_registers(ADDR_KP_RAM, 1)[0],
              link.read_registers(ADDR_KD_RAM, 1)[0])

        # 0x08: no parameters.  A real servo answers nothing (manual: reboot takes
        # ~800 ms and you must switch torque off first).
        print("\n3. send REBOOT (0x08), unicast, no parameters")
        frame = bus.fee_instruction(args.id, FEE_INST_REBOOT)
        t0 = time.monotonic()
        answered = False
        try:
            link._transaction(frame)          # noqa: SLF001 - deliberate raw probe
            answered = True
        except TimeoutError:
            answered = False
        print(f"   answered: {answered}  (expected False)")

        print("\n4. when does it come back?  (PING every 20 ms)")
        back_ms = None
        while (time.monotonic() - t0) < 3.0:
            if link.ping():
                back_ms = (time.monotonic() - t0) * 1000.0
                break
            time.sleep(0.02)
        if back_ms is None:
            print("   no answer within 3 s; scanning ids 0..20 at 1 Mbps")
            found = None
            for sid in range(21):
                l2 = bus.Link(port=args.port, protocol=bus.PROTO_FEE, imu_id=sid,
                              read_len=15, timeout=0.03)
                l2.open()
                try:
                    if l2.ping():
                        found = sid
                        break
                finally:
                    l2.close()
            print(f"   found: {found}")
            return 1
        print(f"   answered again after {back_ms:.0f} ms  "
              f"(manual says ~800 ms; peer doc quotes the same)")

        print("\n5. after")
        after = snapshot(link)
        diff(before, after)

        print("\n6. restore RAM gains to the EEPROM values")
        link.write_registers(ADDR_KP_RAM, bytes([after["P (EEPROM 21)"]]))
        link.write_registers(ADDR_KD_RAM, bytes([after["D (EEPROM 22)"]]))
        print("   now:", link.read_registers(ADDR_KP_RAM, 1)[0],
              link.read_registers(ADDR_KD_RAM, 1)[0])

        print("\n7. this node's own FeeTech personality for 0x08")
        print("   (src/fee.c has no 0x08 case, so it is a silent no-op there)")
    finally:
        link.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
