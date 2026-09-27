#!/usr/bin/env python3
"""Tests for the upgrade package format and the Linux upgrade tool.

Two halves:

* **offline** - package packing/validation, the version comparison and the
  "is an upgrade needed?" decision, using `host/package.py` and `host/upgrade.py`
  as libraries.  No serial port is involved.
* **end to end** - a real serial link (a pty pair) to `boot_node_sim`, which is
  the *actual* bootloader firmware (`boot_regs.c`, `boot_upgrade.c`,
  `boot_flash.c`, `cfg_blob.c`, `dxl2.c`, `fee.c`) compiled for the PC with the
  FMC replaced by a 256 KB RAM image.  The tool is driven through its CLI entry
  point (`upgrade.main(argv)`), so argument handling is covered too, and the
  test then inspects the "flash" the node dumped to check that the image landed
  byte for byte and that the trial state was recorded.

Run:  ../.venv/bin/python host/tools/test_upgrade.py     (or python3)
"""

from __future__ import annotations

import io
import os
import pty
import shutil
import struct
import subprocess
import sys
import tempfile
import contextlib

HERE = os.path.dirname(os.path.abspath(__file__))
HOST = os.path.dirname(HERE)
SRC = os.path.join(os.path.dirname(HOST), "src")
sys.path.insert(0, HOST)

import package as pkgmod  # noqa: E402
import upgrade as upgmod  # noqa: E402
from bus import crc16_dxl, PROTO_DXL, PROTO_FEE  # noqa: E402

FAILURES = 0
CHECKS = 0


def check(ok: bool, what: str) -> None:
    global FAILURES, CHECKS

    CHECKS += 1
    if not ok:
        FAILURES += 1
        print(f"  FAIL {what}")


def check_eq(got, want, what: str) -> None:
    if isinstance(got, (bytes, bytearray)) and isinstance(want, (bytes, bytearray)):
        check_bytes(got, want, what)
        return
    check(got == want, f"{what}: got {got!r}, want {want!r}")


def check_bytes(got: bytes, want: bytes, what: str) -> None:
    """Byte comparison that never dumps a whole image into the log."""
    if got == want:
        check(True, what)
        return
    where = next((i for i, (a, b) in enumerate(zip(got, want)) if a != b),
                 min(len(got), len(want)))
    check(False, f"{what}: {len(got)} vs {len(want)} bytes, first difference at "
                 f"{where:#x} ({got[where:where + 4].hex()} vs "
                 f"{want[where:where + 4].hex()})")


# ── synthetic slot images ──────────────────────────────────────────────────

SLOT_BASE = {"a": 0x08008000, "b": 0x08020000}
APP_HDR_OFF = pkgmod.APP_HDR_OFF


def make_image(slot: str, size: int, seed: int = 0x5A) -> bytes:
    image = bytearray(b"\xFF" * size)
    image[0:16] = b"\x00" * 16
    struct.pack_into("<I", image, 0, 0x2000BF00)                 # initial MSP
    struct.pack_into("<I", image, 4, SLOT_BASE[slot] + 0x241)    # Thumb entry
    for i in range(APP_HDR_OFF + pkgmod.APP_HDR_SIZE, size):
        image[i] = (i * 31 + seed) & 0xFF
    return bytes(image)


def pack(slot: str, size: int, version: str, seed: int = 0x5A,
         flags: int = pkgmod.APP_HDR_FLAG_TRIAL, directory: str = "") -> tuple[bytes, str]:
    """Returns (package bytes, package path)."""
    image = make_image(slot, size, seed)
    patched, pkg_bytes = pkgmod.pack_image(image, slot, pkgmod.parse_version(version),
                                           1_700_000_000, flags)
    directory = directory or tempfile.mkdtemp(prefix="ipkg")
    path = os.path.join(directory, f"slot_{slot}.ipkg")
    with open(path, "wb") as handle:
        handle.write(pkg_bytes)
    return pkg_bytes, path


def parse_cfg_blob(data: bytes) -> dict:
    """Mirror of `cfg_blob_t` (src/cfg_blob.h) for the cfg page dump."""
    magic, version, length = struct.unpack_from("<IHH", data, 0)
    boot = struct.unpack_from("<BBBBBBBB", data, 28)
    (boot_slot, trial_slot, attempts, boot_stay, pingpong, *_reserved) = boot
    crc = struct.unpack_from("<H", data, 36)[0]
    return {
        "magic": magic,
        "version": version,
        "len": length,
        "boot_slot": boot_slot,
        "trial_slot": trial_slot,
        "attempts": attempts,
        "boot_stay": boot_stay,
        "pingpong": pingpong,
        "crc_ok": crc16_dxl(data[:36]) == crc,
    }


# ── offline tests ──────────────────────────────────────────────────────────


def test_package_half() -> None:
    print("package format")

    pkg_bytes, path = pack("b", 0x4000, "1.2.3")
    pkg = pkgmod.read_package(path)
    check_eq(pkgmod.verify_package(pkg), [], "a freshly packed image verifies")
    check_eq(pkg.version, "1.2.3", "version round trip")
    check_eq(pkg.slot_name, "b", "slot round trip")
    check_eq(pkg.image_size, len(pkg.payload), "image_size")

    # the combined CRC the node recomputes must be what the tool calls payload_crc32
    check_eq(pkg.payload_crc32, pkgmod.image_crc_combined(pkg.payload),
             "payload crc == image_crc_combined")
    check_eq(pkg.payload_crc32, pkg.header["crc_low"] and pkg.payload_crc32,
             "payload crc present")

    # container CRC catches a corrupted byte anywhere in the payload
    bad = bytearray(pkg_bytes)
    bad[40] ^= 0x01
    broken = pkgmod.read_package_bytes(bytes(bad))
    check(bool(pkgmod.verify_package(broken)), "a flipped payload byte is detected")

    # ... and in the metadata
    bad = bytearray(pkg_bytes)
    bad[8] ^= 0x01           # payload_len (container offset 8)
    try:
        pkgmod.read_package_bytes(bytes(bad))
        check(False, "a corrupted length is detected")
    except pkgmod.PackageError:
        check(True, "a corrupted length is detected")

    # a bare image (as a debugger would flash) is accepted, and its slot is
    # inferred from the entry address
    image = make_image("a", 0x1000)
    patched, _ = pkgmod.pack_image(image, "a", pkgmod.parse_version("2.0.0"), 0, 0)
    with tempfile.NamedTemporaryFile(suffix=".bin", delete=False) as handle:
        handle.write(patched)
        bare = handle.name
    bare_pkg = pkgmod.read_package(bare)
    check_eq(pkgmod.verify_package(bare_pkg), [], "a bare slot image verifies")
    check_eq(bare_pkg.slot_name, "a", "slot inferred from the entry address")
    os.unlink(bare)

    # the wrong board id
    wrong = bytearray(pkg.payload)
    struct.pack_into("<H", wrong, APP_HDR_OFF + 28, 0x0042)
    broken = pkgmod.read_package_bytes(pkgmod.make_package(bytes(wrong), 1, 0))
    check(any("board" in p for p in pkgmod.verify_package(broken)),
          "wrong board id is reported")

    # slot/entry mismatch (a slot A image in a slot B package)
    broken = pkgmod.read_package_bytes(pkg_bytes)
    broken = type(broken)(**{**broken.__dict__, "slot": 0})
    check(any("slot" in p for p in pkgmod.verify_package(broken)),
          "slot mismatch is reported")

    # version helpers
    check_eq(pkgmod.format_version(pkgmod.parse_version("0.15.255")), "0.15.255",
             "version formatting")
    for bad_version in ("16.0.0", "1.16.0", "1.0.256", "x.y.z"):
        try:
            pkgmod.parse_version(bad_version)
            check(False, f"version {bad_version} is rejected")
        except ValueError:
            check(True, f"version {bad_version} is rejected")


def test_decision() -> None:
    print("version decision")

    _pkg_bytes, path = pack("b", 0x1000, "1.2.3")
    pkg = pkgmod.read_package(path)

    def dev(version: str, imgcrc: int, mode: str = "app", flags: int = 0):
        return upgmod.DeviceInfo(protocol=PROTO_DXL, mode=mode, slot=0, flags=flags,
                                 version=pkgmod.parse_version(version), imgcrc=imgcrc)

    verdict, _ = upgmod.decide(pkg, dev("1.0.0", 0), False)
    check_eq(verdict, "upgrade", "newer package -> upgrade")

    verdict, why = upgmod.decide(pkg, dev("1.2.3", pkg.payload_crc32), False)
    check_eq(verdict, "same", "same version and CRC -> nothing to do")
    check("same image" in why, "the reason mentions the identical image")

    verdict, why = upgmod.decide(pkg, dev("1.2.3", 0xDEADBEEF), False)
    check_eq(verdict, "same", "same version, different build -> still 'same'")
    check("different" in why, "the reason mentions the different build")

    verdict, _ = upgmod.decide(pkg, dev("2.0.0", 0), False)
    check_eq(verdict, "downgrade", "older package -> downgrade")
    verdict, _ = upgmod.decide(pkg, dev("2.0.0", 0), True)
    check_eq(verdict, "downgrade", "the verdict stands, --allow-downgrade decides")

    verdict, _ = upgmod.decide(pkg, dev("0.0.0", 0, flags=upgmod.UID_FLAG_NO_HEADER),
                               False)
    check_eq(verdict, "upgrade", "an unpackaged running image can always be replaced")

    # target selection
    check_eq(upgmod.choose_target(pkg, dev("1.0.0", 0), None), 1,
             "the package's own slot wins")
    check_eq(upgmod.choose_target(pkg, dev("1.0.0", 0), "a"), 0, "--slot overrides")
    either = pkgmod.read_package_bytes(pkgmod.make_package(pkg.payload, 255, 0))
    check_eq(upgmod.choose_target(either, dev("1.0.0", 0), None), 1,
             "a slot-less package goes to the slot the node is not running from")


# ── end-to-end over a pty ──────────────────────────────────────────────────


class Node:
    """A `boot_node_sim` process on one end of a pty pair."""

    def __init__(self, sim: str, run_slot: str | None = None, preload: str | None = None,
                 other_bootable: bool = False) -> None:
        self.dir = tempfile.mkdtemp(prefix="sim")
        self.master, self.slave = pty.openpty()
        self.slave_name = os.ttyname(self.slave)
        argv = [sim, "--fd", str(self.master), "--dump-dir", self.dir,
                "--seconds", "180"]
        if run_slot:
            argv += ["--run-slot", run_slot]
        if preload:
            argv += ["--preload", preload]
        if other_bootable:
            argv += ["--other-bootable"]
        self.proc = subprocess.Popen(argv, pass_fds=(self.master,),
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True)
        # The parent deliberately keeps its slave descriptor open.  Closing the
        # last slave fd hangs the pty up (the master then returns EIO forever),
        # and re-opening the same /dev/pts/N afterwards does not reliably clear
        # that - the tool would talk into a dead pipe.
        # wait for the "SIM: ready" line (the preload banner comes first)
        banner = []
        for _ in range(10):
            line = self.proc.stdout.readline()
            if not line:
                break
            banner.append(line)
            if "ready" in line:
                break
        else:
            raise RuntimeError(f"simulator did not start: {banner}")
        if not any("ready" in line for line in banner):
            raise RuntimeError(f"simulator did not start: {banner}")
        self.banner = "".join(banner)

    def stop(self) -> str:
        """Close the link so the simulator dumps its flash and exits.

        The caller has already closed the tool's serial port; closing the
        parent's remaining slave descriptor now hangs the pty up, which makes the
        simulator's read() return EIO and its main loop finish.
        """
        if self.slave >= 0:
            os.close(self.slave)
            self.slave = -1
        rest = ""
        try:
            rest = self.proc.stdout.read() or ""
        except Exception:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()
            rest += "\n(simulator killed after the timeout)"
        return rest

    def dumped(self, name: str) -> bytes:
        with open(os.path.join(self.dir, name), "rb") as handle:
            return handle.read()

    def cleanup(self) -> None:
        shutil.rmtree(self.dir, ignore_errors=True)


def build_sim(outdir: str) -> str:
    binary = os.path.join(outdir, "boot_node_sim")
    sources = [
        os.path.join(HERE, "boot_node_sim.c"),
        os.path.join(SRC, "boot_regs.c"),
        os.path.join(SRC, "boot_upgrade.c"),
        os.path.join(SRC, "boot_decide.c"),
        os.path.join(SRC, "boot_flash.c"),
        os.path.join(SRC, "boot_crc32.c"),
        os.path.join(SRC, "boot_ram.c"),
        os.path.join(SRC, "cfg_blob.c"),
        os.path.join(SRC, "app_header.c"),
        os.path.join(SRC, "dev_cfg.c"),
        os.path.join(SRC, "crc16.c"),
        os.path.join(SRC, "stuffing.c"),
        os.path.join(SRC, "dxl2.c"),
        os.path.join(SRC, "fee.c"),
    ]
    cmd = ["gcc", "-std=gnu99", "-Wall", "-Wextra", "-Wno-unused-parameter", "-O1",
           "-DIMU_TO_DXL_HOST_TEST=1", f"-I{SRC}", "-o", binary] + sources
    subprocess.run(cmd, check=True, capture_output=True)
    return binary


def prepare_link(port: str, protocol: str):
    """An open Link on the simulator (the caller closes it)."""
    from bus import Link

    link = Link(port, baud=1_000_000, protocol=protocol, imu_id=200, timeout=0.05)
    link.open()
    return link


def run_tool(argv: list[str], expect: int | None = None) -> int:
    """Call the upgrade tool's CLI entry point, keeping its output."""
    out = io.StringIO()
    with contextlib.redirect_stdout(out), contextlib.redirect_stderr(out):
        rc = upgmod.main(argv)
    if expect is not None:
        check_eq(rc, expect, f"{' '.join(argv[:3])} ... exit code\n{out.getvalue()}")
    if rc != 0:
        print("    --- tool output ---")
        for line in out.getvalue().splitlines():
            print(f"    {line}")
    return rc


def test_end_to_end(sim: str) -> None:
    for protocol in (PROTO_DXL, PROTO_FEE):
        # 0x4004 leaves a 4-byte final block on purpose: the node takes the block
        # length from the frame, so a short tail is the case that catches an
        # off-by-one in the single-frame block write
        for size, label in ((0x4000, "16 KB"), (0x4004, "16 KB + 4-byte tail"),
                            (0x18000, "96 KB")):
            if protocol == PROTO_FEE and size > 0x4004:
                continue        # one full-size run is enough; FeeTech is slower to drive
            print(f"end-to-end {protocol}, {label} image")
            pkg_bytes, pkg_path = pack("b", size, "1.2.3")
            node = Node(sim, run_slot="a", preload="a:1.0.0", other_bootable=True)
            port = node.slave_name
            try:
                # the node is a bootloader that would run slot A (v1.0.0)
                rc = run_tool(["status", pkg_path, "--port", port, "--protocol", protocol],
                              expect=0)
                check(rc == 0, "status succeeded")
                rc = run_tool(["upgrade", pkg_path, "--port", port,
                               "--protocol", protocol, "--quiet", "--no-reboot",
                               "--yes"], expect=0)
                check(rc == 0, "upgrade succeeded")
                # the tool must see the image as being on trial afterwards
                probe = prepare_link(port, protocol)
                try:
                    info = upgmod.read_identity(probe)
                finally:
                    probe.close()
                check_eq(info.trial_slot, 1, "the node reports slot B on trial")
                check_eq(info.boot_slot, 255, "no slot confirmed yet")
                check(info.on_trial, "the tool calls the new image 'on trial'")
                # the identity window still describes slot A (the slot the
                # bootloader would run); slot B becomes current only after the
                # bootloader leaves boot mode, which the real board does on reboot
                check_eq(info.slot, 0, "slot A is still the one it would run")
                check_eq(info.attempts, 0, "trial attempts reset")
            finally:
                summary = node.stop()
            print(f"    {summary.strip().splitlines()[-2:]}")
            check("commits=1" in summary, f"the node committed one image ({protocol})")
            check("commit_slot=1" in summary, "the commit was for slot B")
            check(f"blocks={(size + 15) // 16}" in summary,
                  "one block per 16 bytes (last one short)")
            check("status=0x05" in summary, "the session ended committed")
            check("retransmits=0" in summary, "no retransmissions were needed")

            # what landed in "flash" must be the package payload, byte for byte
            written = node.dumped("slot_b.bin")
            check_eq(written[:size], pkgmod.read_package(pkg_path).payload,
                     "slot B holds exactly the package payload")
            check(all(b == 0xFF for b in written[size:]), "the rest of slot B is erased")

            # ... and the boot state must put the new image on trial
            cfg = parse_cfg_blob(node.dumped("cfg.bin"))
            check_eq(cfg["magic"], 0x314D4941, "configuration blob magic")
            check_eq(cfg["version"], 2, "configuration blob version")
            check(cfg["crc_ok"], "configuration blob CRC")
            check_eq(cfg["trial_slot"], 1, "slot B is on trial")
            check_eq(cfg["attempts"], 0, "trial attempts reset")
            check_eq(cfg["pingpong"], 1, "pingpong advanced")
            node.cleanup()

    # the "already up to date" path: same version, different build
    print("end-to-end: nothing to do")
    pkg_bytes, pkg_path = pack("b", 0x2000, "1.0.0")
    node = Node(sim, run_slot="a", preload="a:1.0.0", other_bootable=True)
    try:
        rc = run_tool(["upgrade", pkg_path, "--port", node.slave_name,
                       "--protocol", "dxl", "--quiet", "--yes"], expect=0)
        check(rc == 0, "same version -> exit 0 without writing")
    finally:
        summary = node.stop()
    check("commits=0" in summary, "nothing was committed for an identical version")
    check(all(b == 0xFF for b in node.dumped("slot_b.bin")), "slot B was left alone")

    # a downgrade is refused unless asked for
    print("end-to-end: downgrade refusal")
    pkg_bytes, pkg_path = pack("b", 0x2000, "0.9.0")
    node = Node(sim, run_slot="a", preload="a:1.0.0", other_bootable=True)
    try:
        rc = run_tool(["upgrade", pkg_path, "--port", node.slave_name,
                       "--protocol", "dxl", "--quiet", "--yes"], expect=3)
        check(rc == 3, "an older package is refused with exit 3")
        rc = run_tool(["upgrade", pkg_path, "--port", node.slave_name,
                       "--protocol", "dxl", "--quiet", "--yes",
                       "--allow-downgrade", "--no-reboot"], expect=0)
        check(rc == 0, "--allow-downgrade proceeds")
    finally:
        summary = node.stop()
    check("commits=1" in summary, "the downgrade was committed when allowed")

    # the bootloader protects the only bootable slot
    print("end-to-end: protected slot refusal")
    pkg_bytes, pkg_path = pack("a", 0x2000, "9.9.9")
    node = Node(sim, run_slot="a", preload="a:1.0.0", other_bootable=False)
    try:
        rc = run_tool(["upgrade", pkg_path, "--port", node.slave_name,
                       "--protocol", "dxl", "--quiet", "--yes"], expect=2)
        check(rc == 2, "a package for the last bootable slot is refused")
    finally:
        summary = node.stop()
    check("commits=0" in summary, "nothing was committed")
    check("status=0x00" in summary, "no session was opened either")

    # a corrupted package must be rejected before any traffic
    print("end-to-end: corrupted package")
    pkg_bytes, pkg_path = pack("b", 0x2000, "1.2.3")
    with open(pkg_path, "r+b") as handle:
        handle.seek(64)
        handle.write(b"\x00")
    node = Node(sim, run_slot="a", preload="a:1.0.0", other_bootable=True)
    try:
        rc = run_tool(["upgrade", pkg_path, "--port", node.slave_name,
                       "--protocol", "dxl", "--quiet", "--yes"], expect=1)
        check(rc == 1, "a corrupted package is refused with exit 1")
        rc = run_tool(["verify", pkg_path], expect=1)
        check(rc == 1, "verify exits 1 for a corrupted package")
    finally:
        summary = node.stop()
    check("blocks=0" in summary, "the node saw no blocks")

    # read-back through the bootloader
    print("end-to-end: read-back")
    pkg_bytes, pkg_path = pack("b", 0x4000, "1.2.3")
    node = Node(sim, run_slot="a", preload="a:1.0.0", other_bootable=True)
    try:
        run_tool(["upgrade", pkg_path, "--port", node.slave_name,
                  "--protocol", "dxl", "--quiet", "--no-reboot", "--yes"], expect=0)
        out = os.path.join(node.dir, "readback.bin")
        rc = run_tool(["readback", "--port", node.slave_name, "--protocol", "dxl",
                       "--slot", "b", "--offset", hex(APP_HDR_OFF), "--length", "64",
                       "--output", out], expect=0)
        check(rc == 0, "readback succeeded")
        with open(out, "rb") as handle:
            back = handle.read()
        expected = pkgmod.read_package(pkg_path).payload[APP_HDR_OFF:APP_HDR_OFF + 64]
        check_eq(back, expected, "readback returns the committed header")
    finally:
        node.stop()
    node.cleanup()

    # entering boot mode from the application, and confirming an image
    print("end-to-end: application -> bootloader -> confirm")
    node = Node(sim, run_slot="a", preload="a:1.0.0", other_bootable=True)
    try:
        rc = run_tool(["boot", "--port", node.slave_name, "--protocol", "dxl"],
                      expect=None)
        # the simulator is *always* a bootloader, so this must be a no-op success
        check_eq(rc, 0, "boot on a node already in the bootloader is a no-op")
        rc = run_tool(["confirm", "--port", node.slave_name, "--protocol", "dxl"],
                      expect=2)
        check_eq(rc, 2, "confirm is refused while in the bootloader")
    finally:
        node.stop()
    node.cleanup()


# ── built artifacts ────────────────────────────────────────────────────────


def test_built_packages() -> None:
    """If the firmware was built, cross-check the real .ipkg files."""
    build = os.path.join(os.path.dirname(HOST), "build")
    found = 0
    for name in ("gd32f303cc_imu_to_dxl_slot_a.ipkg",
                 "gd32f303cc_imu_to_dxl_slot_b.ipkg"):
        path = os.path.join(build, name)
        if not os.path.exists(path):
            continue
        found += 1
        pkg = pkgmod.read_package(path)
        check_eq(pkgmod.verify_package(pkg), [], f"{name} verifies")
        # the image must be linked for the slot the package claims
        check_eq(pkgmod.slot_of_entry(pkg.header["entry"]), pkg.slot_name,
                 f"{name} entry address matches its slot")
        # and the JSON next to it must agree
        json_path = path[:-len(".ipkg")] + ".json"
        if os.path.exists(json_path):
            import json

            with open(json_path) as handle:
                meta = json.load(handle)
            check_eq(meta["payload_crc32"], pkg.payload_crc32,
                     f"{name} json payload CRC")
            check_eq(meta["image_version"], pkg.image_version, f"{name} json version")
            check_eq(meta["slot"], pkg.slot_name, f"{name} json slot")
    if not found:
        print("  (no built packages in build/ - skipping; run ./build.sh first)")


def main() -> int:
    print("upgrade tool tests")

    test_package_half()
    test_decision()
    test_built_packages()

    with tempfile.TemporaryDirectory(prefix="upgtest") as tmp:
        print(f"building the node simulator in {tmp}")
        sim = build_sim(tmp)
        test_end_to_end(sim)

    print(f"{CHECKS} checks, {FAILURES} failure(s)")
    if FAILURES:
        print("UPGRADE TESTS FAILED")
        return 1
    print("all upgrade tool tests pass")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
