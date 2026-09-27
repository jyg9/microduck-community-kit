#!/usr/bin/env python3
"""Build and check upgrade packages for the imu_to_dxl v1 node.

A package carries one complete slot image (vector table + app header + code) in
a 32-byte container that records everything needed to decide whether it is
usable *before* anything is sent over the motor bus:

    0   4  magic            b"IUP1"
    4   2  format_version   1
    6   2  header_len       32 (payload offset)
    8   4  payload_len      == the image's image_size
    12  4  payload_crc32    CRC-32 of the two header-free segments, i.e. exactly
                            what the node recomputes in UPG_END
    16  4  full_crc32       CRC-32 of the whole payload, header included
    20  2  board_id         must match APP_BOARD_ID
    22  2  image_version    (major<<12)|(minor<<8)|patch
    24  1  slot             0 = A, 1 = B, 255 = "either"
    25  1  flags            bit0: packaged as a trial image
    26  2  reserved         0
    28  4  build_unix       build time (informational)

The same constants live in `src/board.h` / `src/app_header.h`; `selftest`
cross-checks the C side through `host/tools/test_boot.c`.

Used three ways:
  * `host/package.py pack ...` from the CMake POST_BUILD step,
  * `host/upgrade.py` to load and validate what it is about to flash,
  * `host/package.py selftest` (and `host/tools/test_upgrade.py`) as a test.
"""

from __future__ import annotations

import argparse
import dataclasses
import hashlib
import json
import os
import struct
import subprocess
import sys
import time
import zlib

# ── constants mirrored from src/board.h / src/app_header.h ──────────────────

APP_HDR_OFF = 0x200
APP_HDR_SIZE = 64
APP_HDR_MAGIC = 0x314D4841  # 'A','H','M','1'
APP_HDR_VERSION = 1
APP_HDR_FLAG_TRIAL = 0x0001
APP_HDR_FLAG_CONFIRMED = 0x0002
APP_HDR_FLAG_NO_BOOT = 0x0004

APP_BOARD_ID = 0x0001
APP_PROTO_VERSION = 1
UPG_MIN_IMGLEN = APP_HDR_OFF + APP_HDR_SIZE  # 0x240

SLOT_SIZE = 96 * 1024
SLOT_BASE = {"a": 0x08008000, "b": 0x08020000}
SLOT_INDEX = {"a": 0, "b": 1}
SLOT_BY_BASE = {0x08008000: "a", 0x08020000: "b"}
FLASH_PAGE_SIZE = 2048

PKG_MAGIC = b"IUP1"
PKG_FORMAT_VERSION = 1
PKG_HDR_LEN = 32
PKG_HDR_FMT = "<4sHHII IHHBBHI".replace(" ", "")
# magic, format, hdr_len, payload_len, payload_crc32, full_crc32,
# board_id, image_version, slot, flags, reserved, build_unix
PKG_STRUCT = struct.Struct(PKG_HDR_FMT)
assert PKG_STRUCT.size == PKG_HDR_LEN, PKG_STRUCT.size

HDR_STRUCT = struct.Struct("<IHHIIII IHH H30s".replace(" ", ""))
assert HDR_STRUCT.size == APP_HDR_SIZE, HDR_STRUCT.size

# field offsets inside the 64-byte application header
H_MAGIC = 0
H_HEADER_VERSION = 4
H_IMAGE_VERSION = 6
H_IMAGE_SIZE = 8
H_CRC_LOW = 12
H_CRC_HIGH = 16
H_BUILD_UNIX = 20
H_ENTRY = 24
H_BOARD_ID = 28
H_PROTO_VERSION = 30
H_FLAGS = 32
H_RESERVED = 34


class PackageError(Exception):
    """The file is not a package this tool can use."""


# ── versions ────────────────────────────────────────────────────────────────


def parse_version(text: str) -> int:
    """'1.2.3' -> (1 << 12) | (2 << 8) | 3, the wire format."""
    parts = str(text).strip().split(".")
    if len(parts) > 3 or not parts:
        raise ValueError(f"version must be major[.minor[.patch]], got {text!r}")
    nums = []
    for i, part in enumerate(parts):
        try:
            nums.append(int(part, 0))
        except ValueError as exc:
            raise ValueError(f"version component {part!r} is not a number") from exc
    major = nums[0]
    minor = nums[1] if len(nums) > 1 else 0
    patch = nums[2] if len(nums) > 2 else 0
    if not (0 <= major <= 15 and 0 <= minor <= 15 and 0 <= patch <= 255):
        raise ValueError(
            f"version {text!r} out of range (major/minor 0..15, patch 0..255)"
        )
    return (major << 12) | (minor << 8) | patch


def format_version(value: int) -> str:
    return f"{(value >> 12) & 0xF}.{(value >> 8) & 0xF}.{value & 0xFF}"


def compare_versions(a: int, b: int) -> int:
    return (a > b) - (a < b)


# ── image level helpers (the C side: src/app_header.c) ──────────────────────


def image_crc_low(image: bytes) -> int:
    return zlib.crc32(image[:APP_HDR_OFF])


def image_crc_high(image: bytes) -> int:
    return zlib.crc32(image[APP_HDR_OFF + APP_HDR_SIZE :])


def image_crc_combined(image: bytes) -> int:
    """CRC-32 of the concatenation of the two header-free segments.

    Identical to `app_image_crc_combined()` in the firmware and to what the node
    recomputes in `UPG_END`; `zlib.crc32(second, first)` continues a running CRC.
    """
    crc = image_crc_low(image)
    return zlib.crc32(image[APP_HDR_OFF + APP_HDR_SIZE :], crc)


def parse_header(image: bytes) -> dict:
    if len(image) < UPG_MIN_IMGLEN:
        raise PackageError(
            f"image is {len(image)} bytes, shorter than the {UPG_MIN_IMGLEN}-byte minimum"
        )
    raw = image[APP_HDR_OFF : APP_HDR_OFF + APP_HDR_SIZE]
    (
        magic,
        header_version,
        image_version,
        image_size,
        crc_low,
        crc_high,
        build_unix,
        entry,
        board_id,
        proto_version,
        flags,
        reserved,
    ) = HDR_STRUCT.unpack(raw)
    return {
        "magic": magic,
        "header_version": header_version,
        "image_version": image_version,
        "version": format_version(image_version),
        "image_size": image_size,
        "crc_low": crc_low,
        "crc_high": crc_high,
        "build_unix": build_unix,
        "entry": entry,
        "board_id": board_id,
        "proto_version": proto_version,
        "flags": flags,
        "reserved": reserved[:],
    }


def build_header(image: bytes, version: int, build_unix: int, flags: int) -> bytes:
    """Compute the 64-byte header for a raw slot image (header area reserved)."""
    if len(image) < UPG_MIN_IMGLEN:
        raise PackageError("image is too short to carry a header")
    if len(image) % 4:
        raise PackageError("image size must be a multiple of 4")
    entry = struct.unpack_from("<I", image, 4)[0]
    if not (entry & 1):
        raise PackageError(
            f"word at offset 4 ({entry:#010x}) is not a Thumb vector-table entry: "
            "the linker script did not put .vectors first"
        )
    return HDR_STRUCT.pack(
        APP_HDR_MAGIC,
        APP_HDR_VERSION,
        version,
        len(image),
        image_crc_low(image),
        image_crc_high(image),
        build_unix,
        entry,
        APP_BOARD_ID,
        APP_PROTO_VERSION,
        flags,
        bytes(H_RESERVED and (APP_HDR_SIZE - H_RESERVED)),
    )


def slot_of_entry(entry: int) -> str | None:
    """Infer the slot a linked image belongs to from its absolute entry address."""
    for name, base in SLOT_BASE.items():
        if base + UPG_MIN_IMGLEN <= entry < base + SLOT_SIZE:
            return name
    return None


# ── packages ────────────────────────────────────────────────────────────────


@dataclasses.dataclass
class Package:
    payload: bytes
    format_version: int
    payload_crc32: int
    full_crc32: int
    board_id: int
    image_version: int
    slot: int  # 0 = A, 1 = B, 255 = either
    flags: int
    build_unix: int
    header: dict
    source: str = ""

    @property
    def version(self) -> str:
        return format_version(self.image_version)

    @property
    def slot_name(self) -> str:
        return {0: "a", 1: "b", 255: "either"}[self.slot]

    @property
    def image_size(self) -> int:
        return self.header["image_size"]

    @property
    def sha256(self) -> str:
        return hashlib.sha256(self.payload).hexdigest()

    def describe(self) -> str:
        stamp = (
            time.strftime("%Y-%m-%d %H:%M:%S UTC", time.gmtime(self.build_unix))
            if self.build_unix
            else "unknown"
        )
        return (
            f"package     : {self.source or '<memory>'}\n"
            f"format      : v{self.format_version}\n"
            f"board id    : {self.board_id}\n"
            f"image       : v{self.version} ({self.image_size} bytes) "
            f"for slot {self.slot_name.upper()}\n"
            f"built       : {stamp}\n"
            f"flags       : 0x{self.flags:02X}"
            f"{' trial' if self.flags & APP_HDR_FLAG_TRIAL else ''}"
            f"{' confirmed' if self.flags & APP_HDR_FLAG_CONFIRMED else ''}"
            f"{' no-boot' if self.flags & APP_HDR_FLAG_NO_BOOT else ''}\n"
            f"entry       : {self.header['entry']:#010x}\n"
            f"crc_low     : {self.header['crc_low']:#010x}\n"
            f"crc_high    : {self.header['crc_high']:#010x}\n"
            f"payload crc : {self.payload_crc32:#010x}  (node recomputes this)\n"
            f"full crc    : {self.full_crc32:#010x}\n"
            f"sha256      : {self.sha256}"
        )

    def to_json(self) -> dict:
        return {
            "source": self.source,
            "format_version": self.format_version,
            "board_id": self.board_id,
            "image_version": self.image_version,
            "version": self.version,
            "slot": self.slot_name,
            "flags": self.flags,
            "image_size": self.image_size,
            "build_unix": self.build_unix,
            "entry": self.header["entry"],
            "crc_low": self.header["crc_low"],
            "crc_high": self.header["crc_high"],
            "payload_crc32": self.payload_crc32,
            "full_crc32": self.full_crc32,
            "sha256": self.sha256,
        }


def make_package(payload: bytes, slot: int, flags: int, source: str = "") -> bytes:
    """Wrap an already-header-filled slot image in the container."""
    if len(payload) < UPG_MIN_IMGLEN:
        raise PackageError("payload too short")
    header = parse_header(payload)
    meta = PKG_STRUCT.pack(
        PKG_MAGIC,
        PKG_FORMAT_VERSION,
        PKG_HDR_LEN,
        len(payload),
        image_crc_combined(payload),
        zlib.crc32(payload),
        header["board_id"],
        header["image_version"],
        slot,
        flags,
        0,
        header["build_unix"],
    )
    return meta + payload


def read_package(path: str) -> Package:
    """Load a .ipkg container *or* a bare slot image (.bin).

    A bare image is accepted because that is what a debugger writes; its slot is
    then inferred from the entry address, and a missing header is reported by
    `verify()` instead of being a hard error here.
    """
    with open(path, "rb") as handle:
        data = handle.read()
    if data[:4] == PKG_MAGIC:
        if len(data) < PKG_HDR_LEN:
            raise PackageError("truncated package header")
        (
            _magic,
            fmt,
            hdr_len,
            payload_len,
            payload_crc32,
            full_crc32,
            board_id,
            image_version,
            slot,
            flags,
            _reserved,
            build_unix,
        ) = PKG_STRUCT.unpack(data[:PKG_HDR_LEN])
        if hdr_len != PKG_HDR_LEN:
            raise PackageError(f"unexpected payload offset {hdr_len}")
        payload = data[hdr_len:]
        if payload_len != len(payload):
            raise PackageError(
                f"package says {payload_len} payload bytes, file has {len(payload)}"
            )
        return Package(
            payload=payload,
            format_version=fmt,
            payload_crc32=payload_crc32,
            full_crc32=full_crc32,
            board_id=board_id,
            image_version=image_version,
            slot=slot,
            flags=flags,
            build_unix=build_unix,
            header=parse_header(payload),
            source=path,
        )

    # bare slot image
    header = parse_header(data)
    slot_name = slot_of_entry(header["entry"])
    return Package(
        payload=data,
        format_version=PKG_FORMAT_VERSION,
        payload_crc32=image_crc_combined(data),
        full_crc32=zlib.crc32(data),
        board_id=header["board_id"],
        image_version=header["image_version"],
        slot=(255 if slot_name is None else SLOT_INDEX[slot_name]),
        flags=header["flags"],
        build_unix=header["build_unix"],
        header=header,
        source=path,
    )


def verify_package(pkg: Package) -> list[str]:
    """Every reason this package must not be flashed.  Empty list = good."""
    problems: list[str] = []
    payload = pkg.payload

    if pkg.format_version != PKG_FORMAT_VERSION:
        problems.append(
            f"container format v{pkg.format_version}, this tool speaks v{PKG_FORMAT_VERSION}"
        )
    if len(payload) < UPG_MIN_IMGLEN:
        problems.append(f"payload is only {len(payload)} bytes")
        return problems
    if len(payload) > SLOT_SIZE:
        problems.append(f"payload {len(payload)} bytes does not fit a {SLOT_SIZE}-byte slot")
    if len(payload) % 4:
        problems.append("payload length is not a multiple of 4 (the FMC programs words)")

    got = zlib.crc32(payload)
    if got != pkg.full_crc32:
        problems.append(
            f"file CRC-32 mismatch: {got:#010x} != {pkg.full_crc32:#010x} (corrupted transfer?)"
        )
    got = image_crc_combined(payload)
    if got != pkg.payload_crc32:
        problems.append(
            f"image CRC-32 mismatch: {got:#010x} != {pkg.payload_crc32:#010x}"
        )

    header = pkg.header
    if header["magic"] != APP_HDR_MAGIC:
        problems.append(
            f"bad app header magic {header['magic']:#010x} - the image was never packaged"
        )
        return problems
    if header["header_version"] != APP_HDR_VERSION:
        problems.append(f"unsupported app header version {header['header_version']}")
    if header["image_size"] != len(payload):
        problems.append(
            f"header says image_size={header['image_size']}, payload is {len(payload)}"
        )
    if header["image_size"] > SLOT_SIZE:
        problems.append(f"image_size {header['image_size']} exceeds the slot")
    if header["crc_low"] != image_crc_low(payload):
        problems.append(
            f"crc_low mismatch: header {header['crc_low']:#010x}, "
            f"recomputed {image_crc_low(payload):#010x}"
        )
    if header["crc_high"] != image_crc_high(payload):
        problems.append(
            f"crc_high mismatch: header {header['crc_high']:#010x}, "
            f"recomputed {image_crc_high(payload):#010x}"
        )
    entry = struct.unpack_from("<I", payload, 4)[0]
    if header["entry"] != entry:
        problems.append(
            f"header entry {header['entry']:#010x} != vector table {entry:#010x}"
        )
    if not (entry & 1):
        problems.append("entry address is not a Thumb address")
    if header["board_id"] != APP_BOARD_ID:
        problems.append(
            f"board id {header['board_id']} is not this board ({APP_BOARD_ID})"
        )
    if pkg.board_id != APP_BOARD_ID:
        problems.append(f"container board id {pkg.board_id} is not this board")
    if header["proto_version"] != APP_PROTO_VERSION:
        problems.append(
            f"upgrade protocol v{header['proto_version']} unsupported "
            f"(this node speaks v{APP_PROTO_VERSION})"
        )
    if pkg.image_version != header["image_version"]:
        problems.append(
            f"container version {pkg.version} != header version {header['version']}"
        )
    if header["flags"] & APP_HDR_FLAG_NO_BOOT:
        problems.append("the image is marked no-boot")

    image_slot = slot_of_entry(entry)
    if image_slot is None:
        problems.append(
            f"entry {entry:#010x} is not inside slot A or B: "
            "was this image linked with APP_SLOT=1 or 2?"
        )
    elif pkg.slot != 255 and pkg.slot != SLOT_INDEX[image_slot]:
        problems.append(
            f"package claims slot {pkg.slot_name.upper()} but the image is linked for "
            f"slot {image_slot.upper()}"
        )
    return problems


# ── packing ─────────────────────────────────────────────────────────────────


def pad_to_word(data: bytes) -> bytes:
    """Flash programs 32-bit words; the tail is padded with 0xFF (erased state).

    The padding is inside `image_size`, so it is covered by the CRCs - the node
    sees exactly the bytes this tool measured.
    """
    if len(data) % 4:
        data = data + b"\xFF" * (4 - len(data) % 4)
    return data


def pack_image(
    image: bytes, slot: str, version: int, build_unix: int, flags: int
) -> tuple[bytes, bytes]:
    """Fill the app header, then wrap the image in a container.

    Returns (patched_image, package_bytes).
    """
    if slot not in SLOT_INDEX:
        raise PackageError(f"slot must be a or b, got {slot!r}")
    image = pad_to_word(bytearray(image) and bytes(image))
    if len(image) > SLOT_SIZE:
        raise PackageError(f"image is {len(image)} bytes, slot holds {SLOT_SIZE}")
    header = build_header(image, version, build_unix, flags)
    patched = bytearray(image)
    patched[APP_HDR_OFF : APP_HDR_OFF + APP_HDR_SIZE] = header
    patched = bytes(patched)
    return patched, make_package(patched, SLOT_INDEX[slot], flags)


def cmd_pack(args: argparse.Namespace) -> int:
    bin_path = args.bin or (args.prefix + ".bin")
    with open(bin_path, "rb") as handle:
        raw = handle.read()
    version = parse_version(args.version)
    build_unix = args.build_unix
    if build_unix is None:
        build_unix = int(os.environ.get("SOURCE_DATE_EPOCH") or 0)
    if build_unix == 0:
        try:
            build_unix = int(os.path.getmtime(bin_path))
        except OSError:
            build_unix = 0
    flags = APP_HDR_FLAG_TRIAL
    if args.confirmed:
        flags = APP_HDR_FLAG_CONFIRMED
    if args.no_boot:
        flags |= APP_HDR_FLAG_NO_BOOT

    patched, pkg = pack_image(raw, args.slot, version, build_unix, flags)

    with open(bin_path, "wb") as handle:
        handle.write(patched)
    prefix = args.prefix or os.path.splitext(bin_path)[0]
    pkg_path = prefix + ".ipkg"
    with open(pkg_path, "wb") as handle:
        handle.write(pkg)
    loaded = read_package(pkg_path)
    problems = verify_package(loaded)
    if problems:
        for problem in problems:
            print(f"ERROR: {problem}", file=sys.stderr)
        return 1

    if not args.no_hex:
        with open(prefix + ".json", "w") as handle:
            json.dump(loaded.to_json(), handle, indent=2, sort_keys=True)
            handle.write("\n")
        cmd = [
            args.objcopy,
            "-I",
            "binary",
            "-O",
            "ihex",
            "--change-addresses",
            hex(SLOT_BASE[args.slot]),
            bin_path,
            prefix + ".hex",
        ]
        try:
            subprocess.run(cmd, check=True, capture_output=True)
        except (OSError, subprocess.CalledProcessError) as exc:
            print(f"ERROR: {args.objcopy} failed: {exc}", file=sys.stderr)
            return 1

    print(
        f"packaged {os.path.basename(pkg_path)}: v{loaded.version} "
        f"slot {loaded.slot_name.upper()} {loaded.image_size} bytes "
        f"crc {loaded.payload_crc32:#010x}"
    )
    return 0


def cmd_inspect(args: argparse.Namespace) -> int:
    pkg = read_package(args.file)
    print(pkg.describe())
    return 0


def cmd_verify(args: argparse.Namespace) -> int:
    try:
        pkg = read_package(args.file)
    except (OSError, PackageError) as exc:
        print(f"{args.file}: {exc}")
        return 1
    problems = verify_package(pkg)
    if problems:
        print(f"{args.file}: NOT USABLE")
        for problem in problems:
            print(f"  - {problem}")
        return 1
    print(f"{args.file}: OK - v{pkg.version}, slot {pkg.slot_name.upper()}, "
          f"{pkg.image_size} bytes, crc {pkg.payload_crc32:#010x}")
    return 0


def cmd_selftest(args: argparse.Namespace) -> int:
    del args
    failures = 0

    def check(name: str, got: object, want: object) -> None:
        nonlocal failures
        if got != want:
            print(f"  FAIL {name}: got {got!r} want {want!r}")
            failures += 1

    # version codec
    check("version 1.2.3", parse_version("1.2.3"), (1 << 12) | (2 << 8) | 3)
    check("version 1.0", parse_version("1.0"), (1 << 12))
    check("version format", format_version(parse_version("2.15.255")), "2.15.255")
    check("version compare", compare_versions(parse_version("1.0.1"), parse_version("1.0.0")), 1)

    # a synthetic slot A image: vector table + header placeholder + payload
    for slot in ("a", "b"):
        image = bytearray(b"\xFF" * (APP_HDR_OFF + APP_HDR_SIZE + 0x400))
        struct.pack_into("<I", image, 0, 0x2000BF00)          # initial MSP
        struct.pack_into("<I", image, 4, SLOT_BASE[slot] + 0x241)  # Thumb entry
        image[APP_HDR_OFF + APP_HDR_SIZE :] = bytes(range(256)) * 4
        patched, pkg_bytes = pack_image(bytes(image), slot, parse_version("1.2.3"), 1700000000,
                                        APP_HDR_FLAG_TRIAL)
        pkg = read_package_bytes(pkg_bytes)
        check(f"slot {slot} verifies", verify_package(pkg), [])
        check(f"slot {slot} version", pkg.version, "1.2.3")
        check(f"slot {slot} crc_low", pkg.header["crc_low"], image_crc_low(patched))
        check(f"slot {slot} crc_high", pkg.header["crc_high"], image_crc_high(patched))
        check(f"slot {slot} combined", pkg.payload_crc32, image_crc_combined(patched))
        # slot mismatch must be caught
        broken = dataclasses.replace(pkg, slot=1 - SLOT_INDEX[slot])
        check(f"slot {slot} mismatch detected", bool(verify_package(broken)), True)
        # one flipped payload byte must be caught by the file CRC
        bad = bytearray(pkg.payload)
        bad[APP_HDR_OFF + APP_HDR_SIZE + 3] ^= 0x01
        broken = dataclasses.replace(pkg, payload=bytes(bad))
        check(f"slot {slot} corruption detected", bool(verify_package(broken)), True)

    # an unpackaged image (header still the placeholder) must be rejected
    raw = bytearray(b"\xFF" * (APP_HDR_OFF + APP_HDR_SIZE + 0x40))
    struct.pack_into("<I", raw, 4, SLOT_BASE["a"] + 0x241)
    problems = verify_package(read_package_bytes(make_package(bytes(raw), 0, 0)))
    check("unpackaged image rejected", bool(problems), True)

    if failures:
        print(f"{failures} FAILURE(S)")
        return 1
    print("package.py selftest: all checks passed")
    return 0


def read_package_bytes(data: bytes) -> Package:
    """read_package() for a buffer (used by the tests and selftest)."""
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".ipkg", delete=False) as handle:
        handle.write(data)
        path = handle.name
    try:
        return read_package(path)
    finally:
        os.unlink(path)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("pack", help="fill the app header and write .ipkg/.hex/.json")
    p.add_argument("--prefix", help="input/output path prefix: <prefix>.bin is patched")
    p.add_argument("--bin", help="explicit input .bin (default <prefix>.bin)")
    p.add_argument("--slot", required=True, choices=["a", "b"],
                   help="slot the image was linked for")
    p.add_argument("--version", required=True, help="image version, e.g. 1.2.3")
    p.add_argument("--build-unix", type=int, default=None,
                   help="build timestamp (default: SOURCE_DATE_EPOCH or the .bin mtime)")
    p.add_argument("--confirmed", action="store_true",
                   help="mark the image confirmed instead of trial")
    p.add_argument("--no-boot", action="store_true", help="mark the image no-boot")
    p.add_argument("--no-hex", action="store_true", help="do not run objcopy")
    p.add_argument("--objcopy", default="arm-none-eabi-objcopy")
    p.set_defaults(func=cmd_pack)

    p = sub.add_parser("inspect", help="print what a package contains")
    p.add_argument("file")
    p.set_defaults(func=cmd_inspect)

    p = sub.add_parser("verify", help="exit 1 when a package must not be flashed")
    p.add_argument("file")
    p.set_defaults(func=cmd_verify)

    p = sub.add_parser("selftest", help="check the format without any hardware")
    p.set_defaults(func=cmd_selftest)

    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
