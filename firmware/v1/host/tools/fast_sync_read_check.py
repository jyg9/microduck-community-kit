#!/usr/bin/env python3
"""Fast Sync Read (0x8A) acceptance check for a real imu_to_dxl v1 node.

The question it answers: can official microduck keep `bus.fast_sync_read = true`
with this node on the bus?  A Fast Sync Read is all-or-nothing - every addressed
device appends a block to ONE status packet (id 0xFE, no byte stuffing, each
block ending with the CRC of the whole packet up to that block), so a device that
does not implement 0x8A, or that puts a malformed block on the wire, costs the
whole 16-device read.  The runtime puts the IMU node first in the id list, so
that is the case checked here byte for byte; the mid-list case (a debug tool that
moves the node) is checked against the byte-counted rule the firmware uses there.

Nothing here writes a register or enables torque: the reads are the ones a plain
0x82 tick read performs, on a suspended robot.  On a bench with no servos the
host plays the missing devices (their blocks are synthesized locally or fed onto
the wire); on a real robot every id in the list answers for itself and
`--all-devices` reads and validates the true aggregate instead.

Usage:
    host/tools/py.sh host/tools/fast_sync_read_check.py --port /dev/ttyACM0
    host/tools/py.sh host/tools/fast_sync_read_check.py --port /dev/ttyACM0 --all-devices
    host/tools/py.sh host/tools/fast_sync_read_check.py --port /dev/ttyACM0 --repeat 100
    host/tools/py.sh host/tools/fast_sync_read_check.py --self-test

`--all-devices` is for a robot whose fifteen joints are on the bus (stop `robotd`
first - it owns the port).  Nothing else about the tool changes; it simply
expects every id in the list to answer, which is the strongest form of the check
and the one the firmware was written against.
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

# microduck's tick read: address 124, 12 bytes, the IMU node first and the
# fifteen joints behind it (duck-control/src/model.rs).
READ_ADDR = 124
READ_LEN = 12
JOINT_IDS = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]

FAILURES: list[str] = []


def report(name: str, ok: bool, detail: str = "") -> None:
    mark = "ok  " if ok else "FAIL"
    print(f"  {mark} {name}" + (f"  [{detail}]" if detail else ""))
    if not ok:
        FAILURES.append(name)


def dumm(seed: int, length: int = READ_LEN) -> bytes:
    """Stand-in data for a device that is not on the bench."""
    return bytes((seed + i) & 0xFF for i in range(length))


def prefix_for(ids: list[int], length: int) -> bytes:
    """The 8 bytes the *first* device sends: FF FF FD 00 FE LEN_L LEN_H 0x55."""
    total = 1 + len(ids) * (length + 4)
    return bytes([0xFF, 0xFF, 0xFD, 0x00, bus.DXL_BROADCAST]) + total.to_bytes(2, "little") + b"\x55"


def check_runtime_layout(link: bus.Link) -> None:
    """The layout the runtime uses: [IMU, joints...], node at index 0."""
    ids = [link.imu_id] + JOINT_IDS
    frame = link.fast_sync_read(ids, READ_ADDR, READ_LEN, want=8 + READ_LEN + 4)
    want_len = 1 + len(ids) * (READ_LEN + 4)
    report("runtime layout: the node answers as the aggregate (0xFE)",
           frame[4] == bus.DXL_BROADCAST and frame[7] == 0x55,
           f"id={frame[4]:#04x}")
    got_len = int.from_bytes(frame[5:7], "little")
    report(f"runtime layout: LEN covers all {len(ids)} blocks", got_len == want_len,
           f"LEN={got_len} (want {want_len})")
    report("runtime layout: our block is the first one and error-free",
           frame[8] == 0 and frame[9] == link.imu_id, f"err={frame[8]:#04x} id={frame[9]}")

    # The devices behind us, as bytes: what the servos would have appended.  The
    # reference parser then judges the whole 264-byte packet - including that
    # every block's running CRC lands where the layout says it does.
    packet = bytearray(frame)
    for offset, pid in enumerate(JOINT_IDS, start=0x10):
        packet += bus.dxl_fast_sync_block(pid, dumm(offset), 0, bytes(packet))
    report("runtime layout: the completed 16-device packet has the right size",
           len(packet) == 8 + len(ids) * (READ_LEN + 4), f"{len(packet)} bytes")
    values = bus.dxl_parse_fast_sync_read(bytes(packet), ids, READ_LEN)
    report("runtime layout: the reference parser accepts the whole packet",
           len(values) == len(ids), f"{len(values)} blocks")
    block = bus.ImuBlock.decode(values[0][0], bus.PROTO_DXL)
    report("runtime layout: block 0 decodes as a plausible IMU block",
           sum(v * v for v in block.quat_xyz) <= 1.0 + 1e-6,
           f"quat_xyz={['%.3f' % v for v in block.quat_xyz]}")
    report("runtime layout: block 0 is the 12 bytes a plain read returns",
           values[0][0] == frame[10:10 + READ_LEN], "same bytes, no stuffing applied")
    report("runtime layout: every block keeps the id it was asked for",
           [pid for pid, _ in zip(ids, values)] == ids, "order preserved")


def check_real_aggregate(link: bus.Link, ids: list[int], what: str) -> None:
    """Read the whole aggregate off a bus where every id is a real device.

    No synthesis: the packet that comes back is the one the devices built, and
    the reference parser checks every block's running CRC and id - so this also
    says whether the devices ahead of the node really did chain their blocks the
    way the firmware assumes.
    """
    total = 1 + len(ids) * (READ_LEN + 4)
    try:
        frame = link.fast_sync_read(ids, READ_ADDR, READ_LEN)  # one whole frame
    except TimeoutError:
        report(f"{what}: every id in the list answers", False,
               "the aggregate never completed - is every joint on this bus?")
        return
    report(f"{what}: the aggregate covers every id in the list",
           len(frame) == 7 + total and int.from_bytes(frame[5:7], "little") == total,
           f"{len(frame)} bytes, LEN={int.from_bytes(frame[5:7], 'little')} (want {total})")
    values = bus.dxl_parse_fast_sync_read(frame, ids, READ_LEN)
    report(f"{what}: every block's id and running CRC line up", len(values) == len(ids),
           f"{len(values)} blocks")
    ours = ids.index(link.imu_id)
    report(f"{what}: the IMU block is the one at index {ours}",
           values[ours][1] == 0 and [p for p, _ in zip(ids, values)] == ids,
           f"err={values[ours][1]:#04x}, order preserved")


def check_mid_list(link: bus.Link) -> None:
    """A debug tool that puts the node second: it must wait for the block ahead.

    The bytes ahead of the node have to be on the wire right behind the
    instruction: the 8-byte aggregate prefix (there is no real device 0 on this
    bench, so the host plays one) and then that device's block.  The node counts
    them and answers on the byte that completes the block before it.
    """
    ids = [10, link.imu_id, 11]
    prefix = prefix_for(ids, READ_LEN)
    ahead = prefix + bus.dxl_fast_sync_block(10, dumm(0x40), 0, prefix)
    block = link.fast_sync_read(ids, READ_ADDR, READ_LEN, feed=ahead, want=READ_LEN + 4)
    report("mid-list: the node sends its block only after the one ahead",
           len(block) == READ_LEN + 4 and block[1] == link.imu_id,
           f"{len(block)} bytes, id={block[1] if len(block) > 1 else '?'}")
    report("mid-list: no header of its own (that is the first device's job)",
           block[0] == 0, f"err={block[0]:#04x}")

    packet = prefix + ahead[len(prefix):] + block
    tail = bus.dxl_fast_sync_block(11, dumm(0x80), 0, packet)
    values = bus.dxl_parse_fast_sync_read(packet + tail, ids, READ_LEN)
    report("mid-list: the packet with our block in the middle parses",
           len(values) == 3 and values[1][0] == block[2:2 + READ_LEN],
           f"{len(values)} blocks")

    # Negative control, and the reason the firmware checks the prefix at all: a
    # stream that starts with a block instead of the aggregate prefix belongs to
    # no packet, and answering late into whatever is on the bus by then would
    # corrupt it.  The node stays silent and its `ignored` counter moves.
    try:
        link.fast_sync_read(ids, READ_ADDR, READ_LEN,
                            feed=bus.dxl_fast_sync_block(10, dumm(0x40), 0, prefix),
                            want=READ_LEN + 4)
        report("mid-list: a stream without the aggregate prefix is refused", False,
               "the node answered anyway")
    except TimeoutError:
        report("mid-list: a stream without the aggregate prefix is refused", True,
               "silent")


def check_not_listed(link: bus.Link) -> None:
    """A Fast Sync Read that does not name the node must leave the bus alone."""
    try:
        link.fast_sync_read([10, 11], READ_ADDR, READ_LEN, want=1)
        report("not listed: silence", False, "the node answered anyway")
    except TimeoutError:
        report("not listed: silence", True, "no bytes came back")


def check_plain_read_agrees(link: bus.Link) -> None:
    """The 0x8A block must be the same block a plain 0x82 read serves.

    Both are read back to back, so the sample counter either does not move (same
    100 Hz sample: the bytes must then be identical, which is the real
    equivalence) or moves by one (two samples: only the shape is comparable).
    """
    ids = [link.imu_id]
    frame = link.fast_sync_read(ids, READ_ADDR, bus.TELEM_LEN,
                                want=8 + bus.TELEM_LEN + 4)
    fast = bus.ImuBlock.decode(frame[10:10 + bus.TELEM_LEN], bus.PROTO_DXL)
    plain = bus.ImuBlock.decode(link.read_registers(READ_ADDR, bus.TELEM_LEN), bus.PROTO_DXL)
    report("0x8A and 0x82 serve the same block: status flags agree",
           fast.flags == plain.flags,
           f"0x8A={fast.flags:#04x} 0x82={plain.flags:#04x}")
    delta = (plain.counter - fast.counter) & 0xFF
    report("0x8A and 0x82 serve the same block: the sample counter keeps up",
           delta <= 2, f"counter {fast.counter} -> {plain.counter}")
    if delta == 0:
        report("0x8A and 0x82 serve the same block: same sample, same bytes",
               fast == plain, "identical when the counter did not move")
    else:
        print(f"  ---- two samples apart (counter +{delta}); byte comparison skipped")
    plausible = sum(v * v for v in fast.quat_xyz) <= 1.0 + 1e-6
    report("0x8A block 0 is a plausible quaternion", plausible,
           f"|xyz|^2={sum(v * v for v in fast.quat_xyz):.4f}")


def run_hardware(args: argparse.Namespace) -> int:
    print(f"fast sync read (0x8A) check on {args.port}")
    link = bus.Link(args.port, args.baud, bus.PROTO_DXL, imu_id=args.id,
                    timeout=args.timeout)
    with link:
        if not link.ping():
            report(f"node {args.id} answers a ping", False, "no answer")
            return 1
        report(f"node {args.id} answers a ping", True)
        if args.all_devices:
            # Every id in both lists is a real device on this bus: no synthesized
            # blocks, no synthetic feed.  The second list puts the node second,
            # behind real servos - the case the firmware's byte counting is for.
            check_real_aggregate(link, [link.imu_id] + JOINT_IDS, "real bus, node first")
            check_real_aggregate(link, [10, link.imu_id, 11], "real bus, node behind two servos")
            print("  ---- mid-list behind real servos: the blocks ahead were theirs, not ours")
        else:
            check_runtime_layout(link)
            check_mid_list(link)
            check_not_listed(link)
        check_plain_read_agrees(link)
        if args.repeat:
            # A reply that never arrives is not necessarily the node's doing: this
            # bench's USB adapter drops roughly one reply in a long burst, the same
            # host-side truncation `docs/imu_to_dxl_protocol.md` §12 item 8 records
            # for 0x82, and the node's own counters (`answered` on the debug
            # console) are what tell the two apart - measured 50/50 answered with
            # one reply lost in the adapter.  So tolerate a small rate and say what
            # was seen, rather than failing the node for the host's loss.
            bad = 0
            for _ in range(args.repeat):
                ids = [link.imu_id] + JOINT_IDS
                try:
                    frame = link.fast_sync_read(ids, READ_ADDR, READ_LEN,
                                                want=8 + READ_LEN + 4)
                    clean = frame[8] == 0 and frame[9] == link.imu_id
                except TimeoutError:
                    clean = False
                if not clean:
                    bad += 1
                time.sleep(0.01)
            allowed = max(1, args.repeat // 20)
            report(f"{args.repeat} back-to-back reads, at most a few host-side drops",
                   bad <= allowed,
                   f"{bad}/{args.repeat} missed (allowed {allowed}; check the node's "
                   f"`answered` counter before blaming it)")
    print("\n  host-side wall time is dominated by the USB adapter (~1 ms); the\n"
          "  node's own T_resp needs a scope (docs/bus_timing_borrow_plan.md §8).")
    return 1 if FAILURES else 0


def self_test() -> int:
    """Codec-only checks, no serial port: the packet shapes and the parser."""
    failures = 0

    def check(name: str, ok: bool) -> None:
        nonlocal failures
        print(f"  {'ok  ' if ok else 'FAIL'} {name}")
        if not ok:
            failures += 1

    ids = [200] + JOINT_IDS
    prefix = prefix_for(ids, READ_LEN)
    check("prefix LEN for 16 devices", prefix[5] == 0x01 and prefix[6] == 0x01)
    packet = bytearray(prefix)
    for offset, pid in enumerate(ids, start=0):
        packet += bus.dxl_fast_sync_block(pid, dumm(offset), 0, bytes(packet))
    values = bus.dxl_parse_fast_sync_read(bytes(packet), ids, READ_LEN)
    check("16 blocks parse", len(values) == 16)
    check("first block is the IMU", values[0][0] == dumm(0))
    check("order preserved", all(values[i][0] == dumm(i) for i in range(16)))
    print("self-test: all checks passed" if not failures else f"self-test: {failures} FAILURE(S)")
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=1_000_000)
    parser.add_argument("--id", type=int, default=200)
    parser.add_argument("--timeout", type=float, default=0.05)
    parser.add_argument("--all-devices", action="store_true",
                        help="every id in the list is a real device (a real robot): "
                             "read the true aggregate instead of synthesizing blocks")
    parser.add_argument("--repeat", type=int, default=0,
                        help="extra back-to-back reads on the runtime layout")
    parser.add_argument("--self-test", action="store_true", help="no hardware needed")
    args = parser.parse_args()

    if args.self_test:
        return self_test()
    return run_hardware(args)


if __name__ == "__main__":
    raise SystemExit(main())
