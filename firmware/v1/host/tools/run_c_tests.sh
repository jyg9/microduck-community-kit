#!/usr/bin/env bash
# Host-side C tests: CRC vectors, both protocol slaves, simulator conventions,
# the bootloader's image header / boot decision / bus upgrade state machine, and
# the register images of both FeeTech personalities.
# No hardware and no cross toolchain needed - plain gcc on the real sources.
#
#   ./run_c_tests.sh                      # everything
#   ./run_c_tests.sh --regen-regmap-golden
#                                         # re-capture test_regmap.c's golden
#                                         # from the current sources, then stop
set -euo pipefail
cd "$(dirname "$0")"

OUT="${TMPDIR:-/tmp}/imu_to_dxl_tests"
mkdir -p "${OUT}"

CFLAGS="-std=gnu99 -Wall -Wextra -Wno-unused-parameter -O1 -g"
INCLUDES="-I../../src"

# The register-image test runs the real src/dev.c.  It is the same set of
# sources the application links, minus the parts a PC cannot provide: the
# temperature ADC is compiled out, the flash is the RAM-backed host image
# (src/boot_flash.c under IMU_TO_DXL_HOST_TEST), and critical.h's CMSIS
# intrinsics are stubbed to their no-op meaning on a single-threaded host.
REGMAP_DEFS="-DIMU_TO_DXL_HOST_TEST=1 -DIMU_USE_SPI=0 -DTEMP_SENSOR_ENABLE=0
    -DCFG_FLASH_ENABLE=1 -D__get_PRIMASK()=0U -D__disable_irq()=((void)0)
    -D__enable_irq()=((void)0)"
REGMAP_SOURCES="../../src/dev.c
    ../../src/dev_cfg.c
    ../../src/cfg_blob.c
    ../../src/app_header.c
    ../../src/boot_ram.c
    ../../src/boot_flash.c
    ../../src/boot_crc32.c
    ../../src/crc16.c
    ../../src/imu.c
    ../../src/imu_sim.c
    ../../src/imu_math.c
    ../../src/telem_pack.c"

# build_regmap <personality> <output> [extra defines]
build_regmap() {
    # shellcheck disable=SC2086
    gcc ${CFLAGS} ${REGMAP_DEFS} -DFEE_IMU_PERSONALITY_FT6="$1" ${3:-} ${INCLUDES} \
        -o "$2" test_regmap.c ${REGMAP_SOURCES} -lm
}

if [ "${1:-}" = "--regen-regmap-golden" ]; then
    build_regmap 0 "${OUT}/test_regmap_native"
    "${OUT}/test_regmap_native" --dump > "${OUT}/regmap_golden.txt"
    python3 - "${OUT}/regmap_golden.txt" test_regmap.c <<'PY'
import pathlib, sys

golden = dict(zip(*[iter(pathlib.Path(sys.argv[1]).read_text().split())] * 2))
path = pathlib.Path(sys.argv[2])
text = path.read_text()

def literal(hexstr):
    return "\n".join('    "%s"' % hexstr[i:i + 64] for i in range(0, len(hexstr), 64))

for macro, tag in (("GOLDEN_DXL", "DXL"), ("GOLDEN_FEE_NATIVE", "FEE")):
    head = "static const char %s[] =\n" % macro
    start = text.index(head) + len(head)
    end = text.index('";', start) + 2
    text = text[:start] + literal(golden[tag]) + ";" + text[end:]

path.write_text(text)
print("golden rewritten from the current sources; review the diff before committing")
PY
    exit 0
fi

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
echo "== register images: native golden + FT6 personality (Dynamixel must not move) =="
build_regmap 0 "${OUT}/test_regmap_native"
"${OUT}/test_regmap_native"
build_regmap 1 "${OUT}/test_regmap_ft6"
"${OUT}/test_regmap_ft6"
# and once with the pre-rotation explicitly switched off (identity), which is what
# a board mounted like the original FT6 node uses; the default ft6 build above
# already carries this board's 180 deg about Z
build_regmap 1 "${OUT}/test_regmap_ft6_flat" \
    "-DFEE_IMU_PREMOUNT_QW=1 -DFEE_IMU_PREMOUNT_QX=0 -DFEE_IMU_PREMOUNT_QY=0 -DFEE_IMU_PREMOUNT_QZ=0"
"${OUT}/test_regmap_ft6_flat"

echo
echo "== upgrade package format (host/package.py) =="
./py.sh ../package.py selftest
