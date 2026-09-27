#!/usr/bin/env bash
# Host-side C tests: CRC vectors, both protocol slaves, simulator conventions,
# the bootloader's image header / boot decision / bus upgrade state machine.
# No hardware and no cross toolchain needed - plain gcc on the real sources.
set -euo pipefail
cd "$(dirname "$0")"

OUT="${TMPDIR:-/tmp}/imu_to_dxl_tests"
mkdir -p "${OUT}"

CFLAGS="-std=gnu99 -Wall -Wextra -Wno-unused-parameter -O1 -g"
INCLUDES="-I../../src"

echo "== CRC-16 against rustypot/ROBOTIS vectors =="
gcc ${CFLAGS} -o "${OUT}/test_crc16" test_crc16.c ../../src/crc16.c -lm
"${OUT}/test_crc16"

echo
echo "== CRC-32 against Python zlib.crc32 vectors =="
gcc ${CFLAGS} ${INCLUDES} -o "${OUT}/test_crc32" test_crc32.c ../../src/boot_crc32.c
"${OUT}/test_crc32"

echo
echo "== protocol slaves + simulator =="
gcc ${CFLAGS} -DIMU_TO_DXL_HOST_TEST=1 \
    -o "${OUT}/test_protocols" \
    test_protocols.c \
    ../../src/crc16.c \
    ../../src/imu_math.c \
    ../../src/imu_sim.c \
    ../../src/dxl2.c \
    ../../src/stuffing.c \
    ../../src/fee.c \
    ../../src/telem_pack.c \
    ../../src/bus_arb.c \
    -lm
"${OUT}/test_protocols"

echo
echo "== bootloader: header, boot decision, A/B upgrade over the bus =="
# The real bootloader sources, with the FMC replaced by a 256 KB RAM image that
# behaves like flash (see src/boot_flash.h).  dxl2.c/fee.c are linked so the
# protocol tests drive real frames through the state machine.
gcc ${CFLAGS} -DIMU_TO_DXL_HOST_TEST=1 ${INCLUDES} \
    -o "${OUT}/test_boot" \
    test_boot.c \
    ../../src/boot_regs.c \
    ../../src/boot_upgrade.c \
    ../../src/boot_decide.c \
    ../../src/boot_flash.c \
    ../../src/boot_crc32.c \
    ../../src/boot_ram.c \
    ../../src/cfg_blob.c \
    ../../src/app_header.c \
    ../../src/dev_cfg.c \
    ../../src/crc16.c \
    ../../src/stuffing.c \
    ../../src/dxl2.c \
    ../../src/fee.c \
    -lm
"${OUT}/test_boot"

echo
echo "== upgrade package format (host/package.py) =="
./py.sh ../package.py selftest
