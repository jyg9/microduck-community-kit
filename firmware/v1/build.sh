#!/usr/bin/env bash
# Build / flash / test the imu_to_dxl v1 firmware and the A/B upgrade tooling.
#
# Usage:
#   ./build.sh              # build everything (app, bootloader, both slots)
#   ./build.sh flash        # build, erase, program the APP_SLOT image, run
#   ./build.sh flash-boot   # J-Link: program the bootloader (0x08000000)
#   ./build.sh flash-slot a # J-Link: program the slot A image (0x08008000)
#   ./build.sh flash-pair a # J-Link: bootloader + slot A (fresh board bring-up)
#   ./build.sh erase        # J-Link: erase the whole chip
#   ./build.sh reset        # J-Link: reset & run whatever is in flash
#   ./build.sh clean        # remove build/
#   ./build.sh smoke        # host-side protocol smoke test over the serial port
#   ./build.sh host         # start the web-based host test tool
#   ./build.sh test         # all host tests: C units + upgrade E2E + package/self tests
#   ./build.sh status       # ask the node what it runs and whether a package is needed
#   ./build.sh boot         # put the node into its bootloader (stays there)
#   ./build.sh upgrade PKG  # upgrade the node over the motor bus with a package
#
# Extra CMake options can be passed through, e.g.
#   EXTRA_CMAKE_ARGS="-DAPP_SLOT=1 -DDBG_ENABLE=0" ./build.sh
#   EXTRA_CMAKE_ARGS="-DSYS_CLK_SOURCE=IRC8M" ./build.sh
#   EXTRA_CMAKE_ARGS="-DAPP_VERSION_STR=1.2.3" ./build.sh
#
# Serial port / baud for the motor bus (this node and the servos share it):
#   BUS_PORT=/dev/ttyACM0 BUS_BAUD=1000000 ./build.sh smoke    # bench / PCBA
#   BUS_PORT=/dev/ttyS2   BUS_BAUD=1000000 ./build.sh status   # on the robot
#
# The upgrade path (`status`/`boot`/`upgrade`) defaults to BUS_PORT=auto, which
# probes ttyACM0-1, ttyUSB0-5 and ttyS2 and takes the first device the node
# answers on, so it works on a bench and on the robot without being told which
# (docs/upgrade.md §4.5).  The smoke test and the web UI want a concrete device:
# they use BUS_PORT when it was given, else /dev/ttyACM0.
#
# A USB-TTL adapter is a legitimate bus, so ttyUSB* is probed rather than
# excluded; a *debug console* on USART0 only prints DBG_* lines and answers
# nothing, so probing it costs one timeout and nothing else (docs/upgrade.md §1).
# On the robot the bus is the SoC UART /dev/ttyS2 and robotd owns it while it
# runs: stop it first (`sudo systemctl stop robotd`), then start it again
# afterwards.  Device numbers follow enumeration, so check `ls /dev/ttyACM*`
# after plugging anything else in.
set -euo pipefail
cd "$(dirname "$0")"

BUILD_DIR=build
TOOLCHAIN=cmake/gd32_toolchain.cmake
ELF=${BUILD_DIR}/gd32f303cc_imu_to_dxl
BOOT_ELF=${BUILD_DIR}/gd32f303cc_imu_to_dxl_boot
SLOT_ELF=${BUILD_DIR}/gd32f303cc_imu_to_dxl_slot_

JLINK_DEVICE=GD32F303CC
JLINK_IF=SWD
JLINK_SPEED=4000
EXTRA_CMAKE_ARGS="${EXTRA_CMAKE_ARGS:-}"

# `auto` lets host/upgrade.py pick the motor bus and say which one it took:
# ttyACM0 -> ttyACM1 -> ttyS2 (docs/upgrade.md §4.5).  The smoke test and the
# web tool want a concrete device: they take BUS_PORT when it was given
# explicitly, and ttyACM0 otherwise.
BUS_PORT="${BUS_PORT:-auto}"
if [ "${BUS_PORT}" = "auto" ]; then
    SMOKE_PORT="${SMOKE_PORT:-/dev/ttyACM0}"
else
    SMOKE_PORT="${SMOKE_PORT:-${BUS_PORT}}"
fi
BUS_BAUD="${BUS_BAUD:-1000000}"
VENV_PY="${VENV_PY:-}"

py() {
    VENV_PY="${VENV_PY}" host/tools/py.sh "$@"
}

action="${1:-build}"

if [ "${action}" = "clean" ]; then
    rm -rf "${BUILD_DIR}"
    echo "Removed ${BUILD_DIR}."
    exit 0
fi

configure_build() {
    # Only pass the toolchain file on the first configure: re-passing it on an
    # already-configured tree makes CMake warn that the variable is unused.
    if [ -f "${BUILD_DIR}/CMakeCache.txt" ]; then
        # shellcheck disable=SC2086
        cmake -S . -B "${BUILD_DIR}" -DCMAKE_BUILD_TYPE=Release ${EXTRA_CMAKE_ARGS}
    else
        # shellcheck disable=SC2086
        cmake -S . -B "${BUILD_DIR}" \
            -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN}" \
            -DCMAKE_BUILD_TYPE=Release ${EXTRA_CMAKE_ARGS}
    fi

    # Print the options that change what goes on the wire.  CMake keeps cached
    # values across a source update, so a build/ directory from before
    # BUS_MIRROR_ENABLE defaulted to 0 would still be a bench (mirror) build
    # without saying so.
    local k
    for k in APP_SLOT APP_VERSION_STR BOOT_ENABLE DBG_ENABLE BUS_MIRROR_ENABLE IMU_USE_SPI; do
        printf '  %-18s %s\n' "${k}" \
            "$(sed -n "s/^${k}:[^=]*=//p" "${BUILD_DIR}/CMakeCache.txt")"
    done
    if grep -q '^BUS_MIRROR_ENABLE:[^=]*=1' "${BUILD_DIR}/CMakeCache.txt"; then
        echo "  NOTE bench build: USART0 also answers protocol frames."
        echo "       The upgrade session is still refused there (motor bus only)."
    fi

    cmake --build "${BUILD_DIR}" -j"$(nproc)"
}

# jlink_run <script> [required-success-regex]
#
# -ExitOnError makes JLinkExe fail properly, the log scan catches the case where
# it exits 0 with every command failed, and the regex asserts it really worked.
jlink_run() {
    local script="$1" expect="${2:-}" log rc
    log="${BUILD_DIR}/jlink_last.log"

    set +e
    JLinkExe -device "${JLINK_DEVICE}" -if "${JLINK_IF}" -speed "${JLINK_SPEED}" \
             -autoconnect 1 -ExitOnError 1 -CommanderScript "${script}" 2>&1 | tee "${log}"
    rc="${PIPESTATUS[0]}"
    set -e

    if [ "${rc}" -ne 0 ] || grep -qE \
        'FAILED:|Unknown device|Cannot connect to J-Link|Failed to open|Could not' "${log}"; then
        echo "ERROR: J-Link command failed (exit ${rc}); target NOT modified." >&2
        echo "       Full log: ${log}" >&2
        return 1
    fi
    if [ -n "${expect}" ] && ! grep -qE "${expect}" "${log}"; then
        echo "ERROR: J-Link never reported '${expect}'; target NOT modified." >&2
        echo "       Full log: ${log}" >&2
        return 1
    fi
    return 0
}

# flash_files <erase 0|1> <hex> <label> [<hex> <label> ...]
#
# The J-Link script's `erase` wipes the whole chip, so flashing two images in a
# row with a per-image script would erase the first one again - which is exactly
# how a board ends up with an empty bootloader region and a HardFault.  One
# script, one optional erase, then every loadfile.
flash_files() {
    local erase="$1"; shift
    local script="${BUILD_DIR}/jlink_flash_run.jlink"
    local hex label spec

    {
        echo "si SWD"
        echo "speed ${JLINK_SPEED}"
        echo "Device ${JLINK_DEVICE}"
        echo "Connect"
        echo "h"
        if [ "${erase}" = "1" ]; then
            echo "erase"
        fi
        while [ "$#" -gt 0 ]; do
            hex="$1"; label="$2"; shift 2
            if [ ! -f "${hex}" ]; then
                echo "ERROR: ${hex} not found - run ./build.sh first." >&2
                return 1
            fi
            echo "loadfile $(pwd)/${hex}"
            echo "# ${label}"
        done
        echo "r"
        echo "g"
        echo "q"
    } > "${script}"

    jlink_run "${script}" 'Program & Verify|Programming|O\.K\.'
    echo "Flash OK: ${script##*/} (erase=${erase})"
}

case "${action}" in
    build)
        configure_build
        echo
        echo "Artifacts:"
        for f in "${ELF}.hex" "${BOOT_ELF}.hex" "${SLOT_ELF}a.hex" "${SLOT_ELF}b.hex"; do
            [ -f "$f" ] && printf '  %-48s %s bytes\n' "$f" "$(stat -c%s "$f")"
        done
        for f in "${BUILD_DIR}"/*.ipkg; do
            [ -f "$f" ] && printf '  %-48s %s bytes\n' "$f" "$(stat -c%s "$f")"
        done
        ;;
    flash)
        configure_build
        flash_files 1 "${ELF}.hex" "application image (APP_SLOT from CMake)"
        ;;
    flash-boot)
        configure_build
        flash_files 1 "${BOOT_ELF}.hex" "bootloader (32 KB @ 0x08000000)"
        ;;
    flash-slot)
        slot="${2:-}"
        case "${slot}" in
            a|A) slot=a ;;
            b|B) slot=b ;;
            *) echo "usage: ./build.sh flash-slot a|b" >&2; exit 2 ;;
        esac
        configure_build
        # no erase: this only replaces one slot of an already-bootable board
        flash_files 0 "${SLOT_ELF}${slot}.hex" "slot ${slot} image"
        echo "Note: the bootloader was left alone.  Use ./build.sh flash-pair a on a"
        echo "      blank board, or ./build.sh flash-boot to (re)install it."
        ;;
    flash-pair)
        slot="${2:-a}"
        case "${slot}" in
            a|A) slot=a ;;
            b|B) slot=b ;;
            *) echo "usage: ./build.sh flash-pair a|b" >&2; exit 2 ;;
        esac
        configure_build
        flash_files 1 \
            "${BOOT_ELF}.hex" "bootloader (32 KB @ 0x08000000)" \
            "${SLOT_ELF}${slot}.hex" "slot ${slot} image"
        echo "Note: an application image links for one slot only; use the upgrade"
        echo "      tool to fill the other one in the field."
        ;;
    erase)
        jlink_run scripts/jlink_erase.jlink 'Erasing done'
        echo "Chip erased."
        ;;
    reset)
        jlink_run scripts/jlink_reset.jlink
        echo "Target reset & running."
        ;;
    smoke)
        py host/tools/bus_smoke.py --port "${SMOKE_PORT}" --baud "${BUS_BAUD}"
        ;;
    host)
        exec host/tools/py.sh host/server.py --port "${SMOKE_PORT}" --baud "${BUS_BAUD}"
        ;;
    test)
        bash host/tools/run_c_tests.sh
        echo
        echo "== upgrade tool end-to-end (pty + firmware simulator) =="
        py host/tools/test_upgrade.py
        echo
        echo "== host codecs =="
        py host/bus.py
        echo
        echo "== web server (no hardware) =="
        py host/tools/test_server.py
        ;;
    status)
        shift || true
        py host/upgrade.py status --port "${BUS_PORT}" --baud "${BUS_BAUD}" "$@"
        ;;
    boot)
        py host/upgrade.py boot --port "${BUS_PORT}" --baud "${BUS_BAUD}"
        ;;
    upgrade)
        pkgfile="${2:-}"
        if [ -z "${pkgfile}" ]; then
            echo "usage: ./build.sh upgrade <package.ipkg> [extra options]" >&2
            exit 2
        fi
        shift 2
        py host/upgrade.py upgrade "${pkgfile}" --port "${BUS_PORT}" \
            --baud "${BUS_BAUD}" "$@"
        ;;
    *)
        echo "Unknown action: ${action}" >&2
        echo "use build|flash|flash-boot|flash-slot|flash-pair|erase|reset|smoke|host|test|status|boot|upgrade|clean" >&2
        exit 2
        ;;
esac
