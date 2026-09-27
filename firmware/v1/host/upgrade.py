#!/usr/bin/env python3
"""Linux upgrade tool for the imu_to_dxl v1 node (A/B slots over the motor bus).

Three things this tool is for, in the order a user meets them:

1. **Check the package** (`verify`, or automatically at the start of `upgrade`).
   The 32-byte container, the embedded 64-byte application header and both
   CRC-32 layers are validated before a single byte is sent, so a truncated or
   corrupted download never reaches the node.

2. **Detect the version and say whether an upgrade is needed** (`status`).
   The node publishes its mode (application / bootloader), slot, version and
   image CRC-32 in a read-only window at register 180, and the version a second
   time in the Dynamixel firmware register.  The tool compares that with the
   package and explains the decision - including "you are already on this
   version" and "this would be a downgrade".

3. **Do the upgrade** (`upgrade`).  The application is asked to reset into its
   bootloader (`VCMD_BOOT`), the image is written in 16-byte blocks with a
   per-block CRC-16 and read-back, an end-to-end CRC-32, and the new image is
   left on *trial*: it must run and confirm itself or the bootloader rolls back
   to the previous slot.  The same code path drives both protocols, so
   `--protocol auto` works on a Dynamixel bus and a FeeTech bus alike.

Everything is driven with ordinary register reads/writes over `host/bus.py`
(the same codecs the smoke test and the web tool use); there is no second
protocol implementation here.

Examples:

    ./host/upgrade.py verify build/gd32f303cc_imu_to_dxl_slot_b.ipkg
    ./host/upgrade.py status --port /dev/ttyUSB0
    ./host/upgrade.py upgrade build/gd32f303cc_imu_to_dxl_slot_b.ipkg \\
        --port /dev/ttyUSB0 --protocol auto --yes
"""

from __future__ import annotations

import argparse
import dataclasses
import os
import struct
import sys
import time
from typing import Optional

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import package as pkgmod  # noqa: E402
from bus import Link, ProtocolError, crc16_dxl, PROTO_DXL, PROTO_FEE  # noqa: E402

# ── constants mirrored from src/board.h ────────────────────────────────────

UID_WIN_ADDR = 180
UID_WIN_LEN = 16
UID_MAGIC = 0x5055
UID_MODE_APP = 0
UID_MODE_BOOT = 1
UID_FLAG_NO_HEADER = 0x80
#: offset 11 of the window: 1 means the four bytes after the image CRC carry the
#: configuration page's boot state.  A node that predates them leaves it 0, and
#: the tool then reports "unknown" instead of reading whatever happens to be
#: mapped there.
UID_INFO_VERSION = 11
UID_BOOT_SLOT = 12
UID_TRIAL_SLOT = 13
UID_ATTEMPTS = 14
UID_BOOT_STAY = 15

UPGRADE_WIN_ADDR = 208
UPGRADE_WIN_LEN = 48
UPGRADE_DATA_ADDR = 240
UPGRADE_DATA_LEN = 16
UPGRADE_MAGIC = 0x4247

(U_MAGIC_L, U_MAGIC_H, U_CMD, U_STATUS, U_TARGET, U_SEQ_L, U_SEQ_H, U_OFF_0,
 U_OFF_1, U_OFF_2, U_OFF_3, U_BLKLEN, U_BLKCRC_L, U_BLKCRC_H, U_ACK_SEQ_L,
 U_ACK_SEQ_H, U_ERRCODE, U_VER_L, U_VER_H, U_NODE_CRC_0, U_NODE_CRC_1,
 U_NODE_CRC_2, U_NODE_CRC_3, U_RESERVED, U_IMGLEN_0, U_IMGLEN_1, U_IMGLEN_2,
 U_IMGLEN_3, U_IMGCRC_0, U_IMGCRC_1, U_IMGCRC_2, U_IMGCRC_3, U_DATA_OFF) = range(33)

UPG_CMD_BEGIN = 1
UPG_CMD_DATA = 2
UPG_CMD_END = 3
UPG_CMD_ABORT = 4
UPG_CMD_CONFIRM = 5
UPG_CMD_REBOOT = 6
UPG_CMD_READBACK = 7

UPG_ST_IDLE = 0
UPG_ST_ERASED = 1
UPG_ST_RECEIVING = 2
UPG_ST_BLOCK_OK = 3
UPG_ST_VERIFIED = 4
UPG_ST_COMMITTED = 5
UPG_ST_ERROR = 0x80

UPG_ERR_NAMES = {
    0: "none",
    1: "block CRC mismatch",
    2: "offset out of range",
    3: "flash program failed",
    4: "state machine error (wrong command order)",
    5: "target slot refused",
    6: "session magic missing",
    7: "image verification failed",
    8: "bad length",
}

# vendor window (src/board.h V_*): how the application is asked for boot mode
V_CMD = 2
VENDOR_MAGIC = 0x4D49
VCMD_BOOT = 6
VCMD_CONFIRM = 5
VENDOR_ADDR = {PROTO_DXL: 148, PROTO_FEE: 160}
TELEM_LEN = 20

SLOT_NAMES = {0: "A", 1: "B", 255: "-"}
PROTOCOLS = {PROTO_DXL: "dynamixel", PROTO_FEE: "feetech"}


class UpgradeError(Exception):
    """The upgrade cannot continue (protocol error, refusal, timeout)."""


# ── device side ────────────────────────────────────────────────────────────


@dataclasses.dataclass
class DeviceInfo:
    protocol: str
    mode: str  # "app" | "boot" | "unknown"
    slot: int
    flags: int
    version: int
    imgcrc: int
    model: Optional[int] = None
    firmware: Optional[int] = None
    boot_slot: Optional[int] = None       # last confirmed slot (configuration)
    trial_slot: Optional[int] = None      # image still to confirm itself
    attempts: Optional[int] = None        # trial boots so far
    boot_stay: Optional[int] = None
    raw: bytes = b""

    @property
    def version_str(self) -> str:
        return pkgmod.format_version(self.version)

    @property
    def slot_name(self) -> str:
        return SLOT_NAMES.get(self.slot, "?")

    @property
    def has_header(self) -> bool:
        return not (self.flags & UID_FLAG_NO_HEADER)

    @property
    def on_trial(self) -> bool:
        """True while an image still has to confirm itself.

        In the application that means the running image; in the bootloader it
        means the image the next boot will start (the one just written), which is
        not the slot the identity window describes until the bootloader leaves
        boot mode.
        """
        if self.trial_slot is not None:
            if self.mode == "boot":
                return self.trial_slot in (0, 1)
            if self.slot in (0, 1):
                return self.trial_slot == self.slot
        return bool(self.flags & 1)      # fall back to the frozen header flag

    def describe(self) -> str:
        lines = [
            f"protocol    : {PROTOCOLS.get(self.protocol, self.protocol)}",
            f"mode        : {'bootloader' if self.mode == 'boot' else 'application'}",
            f"slot        : {self.slot_name}"
            + (" (would run)" if self.mode == "boot" else " (running)"),
        ]
        if self.model is not None:
            lines.append(f"model       : {self.model:#06x}")
        if self.firmware is not None:
            lines.append(f"fw register : {self.firmware:#04x} "
                         f"({(self.firmware >> 4) & 0xF}.{self.firmware & 0xF})")
        lines.append(f"image       : v{self.version_str}"
                     + ("" if self.has_header else "  (no valid header!)"))
        lines.append(f"image crc   : {self.imgcrc:#010x}")
        lines.append(f"flags       : {self.flags:#04x}"
                     + ("  (on trial)" if self.on_trial else ""))
        if self.trial_slot is None:
            lines.append("boot state  : unknown (older firmware)")
        else:
            lines.append(
                f"boot state  : confirmed slot {SLOT_NAMES.get(self.boot_slot, '?')}, "
                f"trial {SLOT_NAMES.get(self.trial_slot, '-')} "
                f"({self.attempts} attempt(s)), "
                f"{'holding' if self.boot_stay else 'normal'}")
        return "\n".join(lines)


def detect_protocol(port: str, baud: int, imu_id: int, timeout: float,
                    explicit: str) -> Link:
    """Open the link and find out which protocol the node answers."""
    order = [PROTO_DXL, PROTO_FEE]
    if explicit == "dxl":
        order = [PROTO_DXL]
    elif explicit == "fee":
        order = [PROTO_FEE]

    last_error = "no protocol answered"
    for protocol in order:
        link = Link(port, baud=baud, protocol=protocol, imu_id=imu_id,
                    timeout=timeout)
        try:
            link.open()
        except Exception as exc:  # pyserial errors
            raise UpgradeError(f"cannot open {port}: {exc}") from exc
        if link.ping():
            return link
        last_error = f"{PROTOCOLS[protocol]} did not answer id {imu_id}"
        link.close()
    raise UpgradeError(last_error)


def read_identity(link: Link) -> DeviceInfo:
    """Mode / slot / version of whatever is on the other end."""
    raw = link.read_registers(UID_WIN_ADDR, UID_WIN_LEN)
    magic = struct.unpack_from("<H", raw, 0)[0]
    if magic != UID_MAGIC:
        # no identity window: very old firmware, or something else on the bus
        info = DeviceInfo(link.protocol, "unknown", 255, 0, 0, 0, raw=raw)
        try:
            dev = link.device_info()
            info.model = dev.get("model")
            info.firmware = dev.get("firmware")
        except ProtocolError:
            pass
        return info
    mode = raw[2]
    info = DeviceInfo(
        protocol=link.protocol,
        mode="boot" if mode == UID_MODE_BOOT else "app",
        slot=raw[3],
        flags=raw[4],
        version=struct.unpack_from("<H", raw, 5)[0],
        imgcrc=struct.unpack_from("<I", raw, 7)[0],
        raw=raw,
    )
    if len(raw) > UID_INFO_VERSION and raw[UID_INFO_VERSION] == 1:
        info.boot_slot = raw[UID_BOOT_SLOT]
        info.trial_slot = raw[UID_TRIAL_SLOT]
        info.attempts = raw[UID_ATTEMPTS]
        info.boot_stay = raw[UID_BOOT_STAY]
    try:
        dev = link.device_info()
        info.model = dev.get("model")
        info.firmware = dev.get("firmware")
    except ProtocolError:
        pass
    return info


def ask_for_boot(link: Link, timeout_s: float = 5.0) -> DeviceInfo:
    """Ask the application to reset into the bootloader and wait for it."""
    vendor = VENDOR_ADDR[link.protocol]
    saved = link.timeout
    link.timeout = 0.5  # the node erases a flash page before it answers
    try:
        window = bytearray(link.read_registers(vendor, TELEM_LEN))
        struct.pack_into("<H", window, 0, VENDOR_MAGIC)
        window[V_CMD] = VCMD_BOOT
        link.write_registers(vendor, bytes(window))
    finally:
        link.timeout = saved

    deadline = time.monotonic() + timeout_s
    last = None
    while time.monotonic() < deadline:
        time.sleep(0.15)
        try:
            info = read_identity(link)
        except (TimeoutError, ProtocolError) as exc:
            last = exc
            continue
        if info.mode == "boot":
            return info
    raise UpgradeError(
        f"the node did not enter the bootloader within {timeout_s:.0f} s "
        f"(last error: {last})"
    )


def confirm_image(link: Link) -> None:
    """Tell the application to accept the running (trial) image."""
    vendor = VENDOR_ADDR[link.protocol]
    saved = link.timeout
    link.timeout = 0.5
    try:
        window = bytearray(link.read_registers(vendor, TELEM_LEN))
        struct.pack_into("<H", window, 0, VENDOR_MAGIC)
        window[V_CMD] = VCMD_CONFIRM
        link.write_registers(vendor, bytes(window))
    finally:
        link.timeout = saved


# ── the upgrade session ────────────────────────────────────────────────────


@dataclasses.dataclass
class SessionResult:
    slot: int
    blocks: int
    retransmits: int
    bytes_written: int
    seconds: float
    node_crc: int
    host_crc: int
    erase_pages: int = 0

    @property
    def rate_kib_s(self) -> float:
        return (self.bytes_written / 1024.0 / self.seconds) if self.seconds else 0.0


class Upgrader:
    #: A block is sent as ONE frame that spans U_SEQ_L..the end of the block's
    #: bytes, with the session fields the frame passes over (version, image
    #: length, image CRC) filled in again so it cannot clobber them.  Two frames
    #: per block (a control frame and a data frame) work just as well and the
    #: firmware tests cover both, but each frame costs a full USB round trip on a
    #: bench link.  The frame is exactly as long as the block needs, because the
    #: node takes the data length from the bytes that were written.

    def __init__(self, link: Link, target: int, progress=None, retries: int = 3,
                 quiet: bool = False) -> None:
        self.link = link
        self.target = target
        self.retries = retries
        self.progress = progress
        self.quiet = quiet
        self.retransmits = 0
        self.commands = 0
        self.image_size = 0
        self.image_crc = 0
        self.version = 0

    # -- register window helpers

    def write_window(self, offset: int, data: bytes) -> None:
        self.link.write_registers(UPGRADE_WIN_ADDR + offset, data)
        self.commands += 1

    def read_window(self, offset: int, length: int) -> bytes:
        data = self.link.read_registers(UPGRADE_WIN_ADDR + offset, length)
        self.commands += 1
        return data

    #: one read covers status .. node CRC (offsets 3..22)
    STATUS_BLOCK = 24

    def status_block(self) -> bytes:
        return self.read_window(U_STATUS, self.STATUS_BLOCK)

    def status(self) -> tuple[int, int, int]:
        """(status, errcode, ack_seq) in one read."""
        raw = self.status_block()
        return (raw[0],
                raw[U_ERRCODE - U_STATUS],
                struct.unpack_from("<H", raw, U_ACK_SEQ_L - U_STATUS)[0])

    def command(self, cmd: int) -> None:
        self.write_window(U_CMD, bytes([cmd]))

    # -- session

    def begin(self, image_size: int, image_crc: int, version: int,
              timeout_s: float = 4.0) -> None:
        self.image_size = image_size
        self.image_crc = image_crc
        self.version = version
        payload = struct.pack("<H", UPGRADE_MAGIC)
        self.write_window(U_MAGIC_L, payload)
        self.write_window(U_TARGET, bytes([self.target]))
        self.write_window(U_IMGLEN_0, struct.pack("<I", image_size))
        self.write_window(U_IMGCRC_0, struct.pack("<I", image_crc))
        self.write_window(U_VER_L, struct.pack("<H", version))
        saved = self.link.timeout
        self.link.timeout = timeout_s      # the erase is ~25 ms per 2 KB page
        try:
            self.command(UPG_CMD_BEGIN)
        finally:
            self.link.timeout = saved

        status, errcode, _ = self.status()
        if status & UPG_ST_ERROR:
            raise UpgradeError(
                f"UPG_BEGIN refused: status 0x{status:02X} ({UPG_ERR_NAMES.get(errcode, '?')})"
            )
        if status != UPG_ST_ERASED:
            raise UpgradeError(f"UPG_BEGIN left status 0x{status:02X}, expected erased")

    def block_frame(self, seq: int, offset: int, data: bytes) -> bytes:
        """One frame carrying seq/offset/length/CRC *and* the block's bytes."""
        frame = bytearray(U_DATA_OFF - U_SEQ_L + len(data))
        struct.pack_into("<H", frame, U_SEQ_L - U_SEQ_L, seq)
        struct.pack_into("<I", frame, U_OFF_0 - U_SEQ_L, offset)
        frame[U_BLKLEN - U_SEQ_L] = len(data)
        struct.pack_into("<H", frame, U_BLKCRC_L - U_SEQ_L, self.link_crc(data))
        struct.pack_into("<H", frame, U_VER_L - U_SEQ_L, self.version)
        struct.pack_into("<I", frame, U_IMGLEN_0 - U_SEQ_L, self.image_size)
        struct.pack_into("<I", frame, U_IMGCRC_0 - U_SEQ_L, self.image_crc)
        base = U_DATA_OFF - U_SEQ_L
        frame[base:base + len(data)] = data
        return bytes(frame)

    def send_block(self, seq: int, offset: int, data: bytes) -> None:
        control = self.block_frame(seq, offset, data)
        attempt = 0
        while True:
            attempt += 1
            try:
                self.write_window(U_SEQ_L, control)
                status, errcode, ack_seq = self.status()
            except (TimeoutError, ProtocolError) as exc:
                if attempt > self.retries:
                    raise UpgradeError(
                        f"block {seq} at {offset:#06x}: no answer after "
                        f"{self.retries} retries ({exc})"
                    ) from exc
                if not self.quiet:
                    print(f"  retry block {seq}: {exc}")
                continue

            if status & UPG_ST_ERROR or ack_seq != seq:
                if attempt > self.retries:
                    raise UpgradeError(
                        f"block {seq} at {offset:#06x} rejected: status "
                        f"0x{status:02X} ({UPG_ERR_NAMES.get(errcode, '?')}), "
                        f"ack_seq={ack_seq}"
                    )
                if not self.quiet:
                    print(f"  retry block {seq}: status 0x{status:02X} "
                          f"({UPG_ERR_NAMES.get(errcode, '?')}) ack={ack_seq}")
                self.retransmits += 1
                continue
            break

    def end(self, timeout_s: float = 4.0) -> tuple[int, int]:
        saved = self.link.timeout
        self.link.timeout = timeout_s
        try:
            self.command(UPG_CMD_END)
        finally:
            self.link.timeout = saved
        raw = self.status_block()
        status = raw[0]
        errcode = raw[U_ERRCODE - U_STATUS]
        node_crc = struct.unpack_from("<I", raw, U_NODE_CRC_0 - U_STATUS)[0]
        if status & UPG_ST_ERROR or status != UPG_ST_COMMITTED:
            raise UpgradeError(
                f"UPG_END failed: status 0x{status:02X} "
                f"({UPG_ERR_NAMES.get(errcode, '?')}), node CRC {node_crc:#010x}"
            )
        return status, node_crc

    def abort(self) -> None:
        try:
            self.command(UPG_CMD_ABORT)
        except (TimeoutError, ProtocolError, UpgradeError):
            pass

    def reboot(self) -> None:
        try:
            self.command(UPG_CMD_REBOOT)
        except (TimeoutError, ProtocolError, UpgradeError):
            pass

    def arm(self) -> None:
        """Open a session without erasing anything (needed for read-back)."""
        self.write_window(U_MAGIC_L, struct.pack("<H", UPGRADE_MAGIC))
        self.write_window(U_TARGET, bytes([self.target]))

    def readback(self, offset: int, length: int) -> bytes:
        """Read flash back through the data window, 16 bytes at a time.

        Each chunk needs its own UPG_READBACK command with an advancing offset:
        the data window is only refreshed when the command runs, so re-reading it
        without re-issuing would return the first chunk over and over.
        """
        self.arm()
        out = bytearray()
        while len(out) < length:
            chunk = min(UPGRADE_DATA_LEN, length - len(out))
            self.write_window(U_OFF_0,
                              struct.pack("<I", offset + len(out)) + bytes([chunk]))
            self.command(UPG_CMD_READBACK)
            data = self.read_window(U_DATA_OFF, chunk)
            if len(data) < chunk:
                raise UpgradeError(
                    f"read-back at {offset + len(out):#06x} returned "
                    f"{len(data)} of {chunk} bytes"
                )
            out.extend(data[:chunk])
        return bytes(out)

    @staticmethod
    def link_crc(data: bytes) -> int:
        return crc16_dxl(data)

    def run(self, image: bytes, version: int, progress_label: str = "") -> SessionResult:
        size = len(image)
        host_crc = pkgmod.image_crc_combined(image)
        started = time.monotonic()

        self.begin(size, host_crc, version)
        offset = 0
        seq = 0
        while offset < size:
            chunk = min(UPGRADE_DATA_LEN, size - offset)
            if chunk % 4:
                raise UpgradeError(
                    f"image size {size} leaves a {chunk}-byte tail; the "
                    "packaging tool pads to a whole number of words"
                )
            self.send_block(seq, offset, image[offset:offset + chunk])
            offset += chunk
            seq += 1
            if self.progress is not None:
                self.progress(offset, size, seq, progress_label)

        _status, node_crc = self.end()
        elapsed = time.monotonic() - started
        if node_crc != host_crc:
            raise UpgradeError(
                f"the node reports CRC {node_crc:#010x}, the package says "
                f"{host_crc:#010x}"
            )
        return SessionResult(
            slot=self.target,
            blocks=seq,
            retransmits=self.retransmits,
            bytes_written=size,
            seconds=elapsed,
            node_crc=node_crc,
            host_crc=host_crc,
        )


# ── the decision the user asked for: is an upgrade needed? ─────────────────


def decide(pkg: pkgmod.Package, dev: DeviceInfo, allow_downgrade: bool) -> tuple[str, str]:
    """Returns (verdict, explanation).

    verdict is one of "upgrade", "same", "downgrade", "blocked".
    """
    if not dev.has_header and dev.mode == "app":
        return ("upgrade",
                "the running image has no valid header (it was flashed with a "
                "debugger, not packaged), so any package is an improvement")
    cmp = pkgmod.compare_versions(pkg.image_version, dev.version)
    if cmp > 0:
        return ("upgrade",
                f"package v{pkg.version} is newer than the running "
                f"v{dev.version_str}")
    if cmp == 0:
        same_crc = dev.imgcrc == pkg.payload_crc32
        if same_crc:
            return ("same",
                    f"the node already runs v{pkg.version} with the same image "
                    f"CRC {dev.imgcrc:#010x}")
        return ("same",
                f"the node already runs v{dev.version_str}, but a different "
                f"build (node CRC {dev.imgcrc:#010x}, package "
                f"{pkg.payload_crc32:#010x})")
    if allow_downgrade:
        return ("downgrade",
                f"package v{pkg.version} is OLDER than the running "
                f"v{dev.version_str}; continuing because --allow-downgrade")
    return ("downgrade",
            f"package v{pkg.version} is older than the running "
            f"v{dev.version_str}; refusing without --allow-downgrade")


def choose_target(pkg: pkgmod.Package, dev: DeviceInfo, override: str | None) -> int:
    if override:
        return pkgmod.SLOT_INDEX[override]
    if pkg.slot != 255:
        return pkg.slot
    # the package does not say: use the slot the node is not running from
    if dev.slot in (0, 1):
        return 1 - dev.slot
    return pkgmod.SLOT_INDEX["b"]


# ── commands ───────────────────────────────────────────────────────────────


def cmd_info(args: argparse.Namespace) -> int:
    pkg = pkgmod.read_package(args.package)
    print(pkg.describe())
    problems = pkgmod.verify_package(pkg)
    if problems:
        print("\nNOT USABLE:")
        for problem in problems:
            print(f"  - {problem}")
        return 1
    print("\npackage is valid")
    return 0


def cmd_status(args: argparse.Namespace) -> int:
    link = detect_protocol(args.port, args.baud, args.id, args.timeout, args.protocol)
    try:
        info = read_identity(link)
        print(info.describe())
        if args.package:
            pkg = pkgmod.read_package(args.package)
            problems = pkgmod.verify_package(pkg)
            if problems:
                print("\npackage NOT USABLE:")
                for problem in problems:
                    print(f"  - {problem}")
                return 1
            verdict, why = decide(pkg, info, args.allow_downgrade)
            print(f"\npackage     : v{pkg.version} for slot {pkg.slot_name.upper()} "
                  f"({pkg.image_size} bytes)")
            print(f"verdict     : {verdict.upper()} - {why}")
            target = choose_target(pkg, info, args.target)
            if info.mode == "app" and target == info.slot:
                print(f"note        : slot {SLOT_NAMES[target]} is the running one; "
                      "the upgrade will be written to the other slot unless "
                      "--slot is given")
            return 0 if verdict in ("upgrade", "same") else 3
        return 0
    finally:
        link.close()


def _progress_printer(quiet: bool):
    if quiet:
        return None
    state = {"next": 0.0}

    def show(done: int, total: int, seq: int, label: str) -> None:
        now = time.monotonic()
        if done < total and now < state["next"]:
            return
        state["next"] = now + 0.5
        pct = 100.0 * done / total if total else 100.0
        sys.stdout.write(f"\r  {label}writing {done}/{total} bytes ({pct:5.1f}%) ")
        sys.stdout.flush()
        if done >= total:
            sys.stdout.write("\n")
            sys.stdout.flush()

    return show


def cmd_upgrade(args: argparse.Namespace) -> int:
    # 1. the package must be correct before anything else happens
    pkg = pkgmod.read_package(args.package)
    problems = pkgmod.verify_package(pkg)
    if problems:
        print(f"{args.package}: package is NOT usable:", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    print(f"package     : {pkg.source}")
    print(f"            : v{pkg.version}, slot {pkg.slot_name.upper()}, "
          f"{pkg.image_size} bytes, crc {pkg.payload_crc32:#010x}")

    # 2. version check against the node
    link = detect_protocol(args.port, args.baud, args.id, args.timeout, args.protocol)
    try:
        info = read_identity(link)
        print(f"node        : {PROTOCOLS.get(info.protocol)} id {args.id}, "
              f"{'bootloader' if info.mode == 'boot' else 'application'}, "
              f"slot {info.slot_name}, v{info.version_str}")
        verdict, why = decide(pkg, info, args.allow_downgrade)
        print(f"version     : {why}")
        if verdict == "same" and not args.force:
            print("nothing to do: the node is already up to date "
                  "(use --force to reflash anyway)")
            return 0
        if verdict == "downgrade" and not args.allow_downgrade:
            return 3

        target = choose_target(pkg, info, args.target)
        if target < 0 or target > 1:
            print("no target slot could be determined; use --slot a|b", file=sys.stderr)
            return 2
        # A slot image is linked for one specific slot (every absolute address in
        # it assumes the slot base), so it can only ever be written there.
        if pkg.slot != 255 and target != pkg.slot:
            print(f"refusing: this package is linked for slot "
                  f"{pkg.slot_name.upper()}, not {SLOT_NAMES[target]}", file=sys.stderr)
            return 2
        if info.mode == "app" and target == info.slot:
            other = SLOT_NAMES[1 - info.slot]
            print(f"refusing: the node is running from slot {info.slot_name}, and a "
                  f"node cannot erase the slot it is executing from.", file=sys.stderr)
            print(f"          Build the package for slot {other} "
                  f"(APP_SLOT={'1' if other == 'A' else '2'}) and flash that one; "
                  "the running image stays as the rollback copy.", file=sys.stderr)
            return 2
        print(f"target      : slot {SLOT_NAMES[target]}")

        if args.dry_run:
            print("--dry-run: stopping before anything is written")
            return 0

        # 3. make sure we are in the bootloader
        if info.mode != "boot":
            if args.no_enter_boot:
                print("the node is not in its bootloader and --no-enter-boot was "
                      "given", file=sys.stderr)
                return 2
            print("entering    : asking the application for boot mode ...")
            info = ask_for_boot(link, timeout_s=args.boot_timeout)
            print(f"            : node is in the bootloader (slot {info.slot_name})")
        if info.mode == "boot" and target == info.slot and not args.force_target:
            # the bootloader refuses to erase the only bootable image
            print(f"refusing: slot {SLOT_NAMES[target]} holds the only bootable "
                  f"image, and the bootloader will not erase it", file=sys.stderr)
            print("          Flash the package built for the other slot instead "
                  "(--slot picks the target only when the package does not say).",
                  file=sys.stderr)
            return 2

        upgrader = Upgrader(link, target, progress=_progress_printer(args.quiet),
                            retries=args.retries, quiet=args.quiet)
        try:
            result = upgrader.run(pkg.payload, pkg.image_version,
                                 progress_label=f"slot {SLOT_NAMES[target]} ")
        except UpgradeError:
            upgrader.abort()
            raise

        print(f"verified    : {result.bytes_written} bytes in "
              f"{result.blocks} blocks ({result.retransmits} retransmits)")
        print(f"image crc   : node {result.node_crc:#010x} == package "
              f"{result.host_crc:#010x}")
        print(f"throughput  : {result.rate_kib_s:.1f} KiB/s over "
              f"{result.seconds:.2f} s")

        # 4. hand the new image back to the application
        if args.reboot:
            print("rebooting   : leaving the bootloader")
            upgrader.reboot()
            info = wait_for_app(link, args.id, timeout_s=args.reboot_timeout)
            if info is None:
                print("warning: the node did not come back as an application "
                      "within the timeout; it stays on trial and will roll back "
                      "after 3 boot attempts if it cannot confirm itself",
                      file=sys.stderr)
                return 4
            print(f"running     : v{info.version_str} in slot {info.slot_name} "
                  f"({'on trial' if info.on_trial else 'confirmed'})")
            if args.confirm:
                confirm_image(link)
                print("confirmed   : the running image was accepted explicitly")
        else:
            print("note        : the image is on trial; it confirms itself after "
                  "30 s of healthy running, or rolls back after 3 boot attempts")
        print("done")
        return 0
    finally:
        link.close()


def wait_for_app(link: Link, imu_id: int, timeout_s: float) -> Optional[DeviceInfo]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        time.sleep(0.2)
        try:
            if not link.ping():
                continue
            info = read_identity(link)
        except (TimeoutError, ProtocolError):
            continue
        if info.mode == "app":
            return info
    return None


def cmd_confirm(args: argparse.Namespace) -> int:
    link = detect_protocol(args.port, args.baud, args.id, args.timeout, args.protocol)
    try:
        info = read_identity(link)
        if info.mode != "app":
            print("the node is in its bootloader; nothing to confirm", file=sys.stderr)
            return 2
        confirm_image(link)
        time.sleep(0.3)
        after = read_identity(link)
        print(f"confirmed: slot {after.slot_name} v{after.version_str}, "
              f"boot state {after.boot_slot} -> trial {after.trial_slot}")
        return 0
    finally:
        link.close()


def cmd_boot(args: argparse.Namespace) -> int:
    """Put the node into its bootloader and leave it there."""
    link = detect_protocol(args.port, args.baud, args.id, args.timeout, args.protocol)
    try:
        info = read_identity(link)
        if info.mode == "boot":
            print(f"already in the bootloader (slot {info.slot_name})")
            return 0
        info = ask_for_boot(link, timeout_s=args.boot_timeout)
        print(f"bootloader  : slot {info.slot_name}, mode {info.mode}")
        return 0
    finally:
        link.close()


def cmd_abort(args: argparse.Namespace) -> int:
    link = detect_protocol(args.port, args.baud, args.id, args.timeout, args.protocol)
    try:
        info = read_identity(link)
        if info.mode != "boot":
            print("the node is not in its bootloader", file=sys.stderr)
            return 2
        upgrader = Upgrader(link, 0)
        upgrader.abort()
        print("session aborted")
        return 0
    finally:
        link.close()


def cmd_readback(args: argparse.Namespace) -> int:
    link = detect_protocol(args.port, args.baud, args.id, args.timeout, args.protocol)
    try:
        info = read_identity(link)
        if info.mode != "boot":
            print("read-back needs the bootloader (use `boot` first)", file=sys.stderr)
            return 2
        target = pkgmod.SLOT_INDEX[args.slot]
        upgrader = Upgrader(link, target, quiet=True)
        upgrader.write_window(U_TARGET, bytes([target]))
        data = upgrader.readback(args.offset, args.length)
        if args.output:
            with open(args.output, "wb") as handle:
                handle.write(data)
            print(f"wrote {len(data)} bytes to {args.output}")
        else:
            for i in range(0, len(data), 16):
                print(f"{args.offset + i:06x}  " + data[i:i + 16].hex(" "))
        return 0
    finally:
        link.close()


# ── argument parsing ───────────────────────────────────────────────────────


def add_link_args(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--port", default="/dev/ttyUSB0",
                        help="serial device (default /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=1_000_000,
                        help="bus baud rate (default 1000000)")
    parser.add_argument("--id", type=int, default=200,
                        help="node id (default 200)")
    parser.add_argument("--protocol", default="auto", choices=["auto", "dxl", "fee"],
                        help="bus protocol (default: try both)")
    parser.add_argument("--timeout", type=float, default=0.05,
                        help="per-transaction timeout in seconds")
    parser.add_argument("--quiet", action="store_true", help="no progress output")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("info", help="describe and check a package (offline)")
    p.add_argument("package")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("verify", help="exit 1 when a package must not be flashed")
    p.add_argument("package")
    p.set_defaults(func=cmd_verify)

    p = sub.add_parser("status", help="what the node runs and whether it needs the package")
    p.add_argument("package", nargs="?", help="optional package to compare against")
    p.add_argument("--target", choices=["a", "b"])
    p.add_argument("--allow-downgrade", action="store_true")
    add_link_args(p)
    p.set_defaults(func=cmd_status)

    p = sub.add_parser("upgrade", help="write a package into the other slot")
    p.add_argument("package")
    p.add_argument("--slot", dest="target", choices=["a", "b"],
                   help="force the target slot")
    p.add_argument("--allow-downgrade", action="store_true",
                   help="allow flashing an older version")
    p.add_argument("--force", action="store_true",
                   help="flash even when the node already runs this version")
    p.add_argument("--force-target", action="store_true",
                   help="do not steer away from the slot the bootloader protects")
    p.add_argument("--no-enter-boot", action="store_true",
                   help="fail instead of asking the application for boot mode")
    p.add_argument("--reboot", dest="reboot", action="store_true", default=True,
                   help="reboot into the new image when done (default)")
    p.add_argument("--no-reboot", dest="reboot", action="store_false",
                   help="stay in the bootloader after committing")
    p.add_argument("--confirm", action="store_true",
                   help="after rebooting, confirm the new image explicitly")
    p.add_argument("--reboot-timeout", type=float, default=8.0)
    p.add_argument("--boot-timeout", type=float, default=6.0)
    p.add_argument("--retries", type=int, default=3,
                   help="retries per block (default 3)")
    p.add_argument("--dry-run", action="store_true",
                   help="do everything except writing the image")
    p.add_argument("--yes", "-y", action="store_true",
                   help="do not ask anything (the tool never prompts today)")
    add_link_args(p)
    p.set_defaults(func=cmd_upgrade)

    p = sub.add_parser("boot", help="ask the application to reset into the bootloader")
    p.add_argument("--boot-timeout", type=float, default=6.0)
    add_link_args(p)
    p.set_defaults(func=cmd_boot)

    p = sub.add_parser("confirm", help="accept the running trial image")
    add_link_args(p)
    p.set_defaults(func=cmd_confirm)

    p = sub.add_parser("abort", help="drop an upgrade session in progress")
    add_link_args(p)
    p.set_defaults(func=cmd_abort)

    p = sub.add_parser("readback", help="read flash back through the bootloader")
    p.add_argument("--slot", default="b", choices=["a", "b"])
    p.add_argument("--offset", type=lambda v: int(v, 0), default=0)
    p.add_argument("--length", type=lambda v: int(v, 0), default=64)
    p.add_argument("--output", help="write the bytes to a file instead of a hex dump")
    add_link_args(p)
    p.set_defaults(func=cmd_readback)

    args = parser.parse_args(argv)
    try:
        return int(args.func(args))
    except UpgradeError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 5
    except KeyboardInterrupt:
        print("interrupted", file=sys.stderr)
        return 130


def cmd_verify(args: argparse.Namespace) -> int:
    try:
        pkg = pkgmod.read_package(args.package)
    except (OSError, pkgmod.PackageError) as exc:
        print(f"{args.package}: {exc}")
        return 1
    problems = pkgmod.verify_package(pkg)
    if problems:
        print(f"{args.package}: NOT USABLE")
        for problem in problems:
            print(f"  - {problem}")
        return 1
    print(f"{args.package}: OK - v{pkg.version}, slot {pkg.slot_name.upper()}, "
          f"{pkg.image_size} bytes, crc {pkg.payload_crc32:#010x}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
