cmake_minimum_required(VERSION 3.10)

# ---------------------------------------------------------------------------
# imu_to_dxl v1  -  GD32F303CC (LQFP48, 256 KB flash / 48 KB SRAM, 12 MHz xtal)
#
# A Dynamixel 2.0 + FeeTech node that publishes the LSM6DSV16X block as a
# register area, with a simulator standing in for the missing SPI sensor.
#
# Targets:
#   gd32f303cc_imu_to_dxl            the application, linked for APP_SLOT
#   gd32f303cc_imu_to_dxl_boot       the 32 KB bootloader at 0x08000000
#   gd32f303cc_imu_to_dxl_slot_a     application image for slot A (+ package)
#   gd32f303cc_imu_to_dxl_slot_b     application image for slot B (+ package)
#
# Build:
#   ./build.sh                 (or)
#   cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=cmake/gd32_toolchain.cmake
#   cmake --build build -j
#
# Flash over J-Link/SWD:
#   ./build.sh flash           (the APP_SLOT image, development board)
#   ./build.sh flash-boot      (the bootloader)
#   ./build.sh flash-slot a    (a slot image)
#
# Options (all cached, override with -D...):
#   APP_SLOT          0 = link at 0x08000000 (development board, default)
#                     1 = slot A, 2 = slot B (see docs/flash_layout.md)
#   BOOT_ENABLE       1 = also build the bootloader and both slot images
#   APP_VERSION_STR   "major.minor.patch" stamped into the slot headers
#   DBG_ENABLE        1 = debug console on USART0 (0 removes the code)
#   BUS_MIRROR_ENABLE 1 = USART0 also answers protocol frames (bench only;
#                         0, the default, leaves it a debug console)
#   STATUS_LED_ENABLE 1 = PB5 heartbeat LED (development board)
#   CFG_FLASH_ENABLE  1 = persist id/baud/sim settings in the last flash page
#   TEMP_SENSOR_ENABLE 1 = read the on-chip temperature sensor
#   IMU_USE_SPI       1 = LSM6DSV16X on SPI0, 0 = the simulator
#   FEE_IMU_PERSONALITY
#                     ft6    = byte-compatible with the FT6 node contract
#                              (default: what an R17 robot needs),
#                     native = this repository's own FeeTech contract.
#                     FeeTech map only; the Dynamixel map is identical in both.
#                     Use a separate BUILD_DIR per personality: the two images
#                     are not interchangeable on the bus.
#   FEE_IMU_PREMOUNT  ft6 only: chip-frame rotation "w,x,y,z" for the FeeTech
#                     blocks.  Defaults to this board's mounting (0,0,0,1: the
#                     IMU chip turned 180 deg about its package normal, relative
#                     to the FT6 node's chip).  [1,0,0,0] publishes the raw chip
#                     frame, for a board mounted like the original node.
# ---------------------------------------------------------------------------

project(gd32f303cc_imu_to_dxl C ASM)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# J-Link's device database knows this part as "GD32F303CC"; appending the
# package code (T6) makes JLinkExe pop up an interactive selection dialog.
set(MCU_DEVICE "GD32F303CC")

# ---- board clock -----------------------------------------------------------
# 12 MHz crystal -> (HXTAL / 2) * 20 = 120 MHz.  The vendor system file
# hard-selects an 8 MHz HXTAL / MUL30 combination, so this project ships its
# own copy with a selectable source.
set(SYS_CLK_SOURCE "HXTAL" CACHE STRING "System clock source: HXTAL (crystal) or IRC8M (internal RC)")
set_property(CACHE SYS_CLK_SOURCE PROPERTY STRINGS HXTAL IRC8M)
set(SYS_CLK_HXTAL_HZ "12000000" CACHE STRING "HXTAL crystal frequency in Hz (board: 12 MHz)")

if(SYS_CLK_SOURCE STREQUAL "HXTAL")
    set(SYS_CLK_SOURCE_VAL 1)
elseif(SYS_CLK_SOURCE STREQUAL "IRC8M")
    set(SYS_CLK_SOURCE_VAL 0)
else()
    message(FATAL_ERROR "SYS_CLK_SOURCE must be HXTAL or IRC8M, got '${SYS_CLK_SOURCE}'")
endif()

# ---- feature switches ------------------------------------------------------
set(APP_SLOT "0" CACHE STRING "0 = plain image, 1 = slot A, 2 = slot B")
set_property(CACHE APP_SLOT PROPERTY STRINGS 0 1 2)
set(BOOT_ENABLE "1" CACHE STRING "also build the bootloader and the two slot images")
set(DBG_ENABLE "0" CACHE STRING "debug console on USART0")
set(BUS_MIRROR_ENABLE "0" CACHE STRING "USART0 doubles as a bus port (bench only)")
set(STATUS_LED_ENABLE "1" CACHE STRING "PB5 heartbeat LED")
set(CFG_FLASH_ENABLE "1" CACHE STRING "persist configuration in the last flash page")
set(TEMP_SENSOR_ENABLE "1" CACHE STRING "read the on-chip temperature sensor")
set(IMU_USE_SPI "1" CACHE STRING "1 = LSM6DSV16X on SPI0, 0 = the simulator")

# ---- FeeTech personality ---------------------------------------------------
# Two build-time personalities for the *FeeTech* map only.  The Dynamixel map is
# the same in both: the node's protocol 2.0 answers must not depend on a choice
# the motor bus never sees.  See docs/imu_to_dxl_protocol.md §16.
#
# Decoded before APP_VERSION_STR because the two personalities need different
# image versions by default - see below.
set(FEE_IMU_PERSONALITY "native" CACHE STRING
    "FeeTech side personality: native = this repository's own contract, ft6 = FT6-compatible node")
set_property(CACHE FEE_IMU_PERSONALITY PROPERTY STRINGS native ft6)

if(FEE_IMU_PERSONALITY STREQUAL "native")
    set(FEE_IMU_PERSONALITY_FT6_VAL 0)
elseif(FEE_IMU_PERSONALITY STREQUAL "ft6")
    set(FEE_IMU_PERSONALITY_FT6_VAL 1)
else()
    message(FATAL_ERROR
        "FEE_IMU_PERSONALITY must be native or ft6, got '${FEE_IMU_PERSONALITY}'")
endif()

# The image version is a number end to end - it is packed into a u16 in the app
# header (major 4 bit, minor 4 bit, patch 8 bit) and host/upgrade.py compares it
# to decide update vs downgrade - so it cannot carry a text label.  What it can
# do is differ per personality, and it must: two different images sharing one
# version make upgrade.py refuse the flash, or worse, adopt it as an ordinary
# update and silently replace the personality.  Native keeps the historical
# 1.5.1; ft6 gets its own.  Override with -DAPP_VERSION_STR=... if a release
# line needs its own numbering.
if(FEE_IMU_PERSONALITY STREQUAL "ft6")
    set(_app_version_default "1.6.0")
else()
    set(_app_version_default "1.5.1")
endif()
set(APP_VERSION_STR "${_app_version_default}" CACHE STRING
    "image version stamped into the slot header (defaults per FeeTech personality)")

# ---- FeeTech-side chip-frame rotation (ft6 only) ---------------------------
# The FT6 host owns the mounting rotation: it applies the robot's stored
# `imu_mount_quat` to whatever the node reports, and this node reports chip
# frame.  A board whose IMU chip sits in a different orientation than the
# original FT6 node's chip must therefore compensate here, or the host's
# (correct, robot-specific) calibration lands the attitude half a turn out.
#
# Default: this board's mounting.  The IMU chip is rotated 180 degrees about its
# package normal relative to the FT6 node's chip (the marking dot ends up on the
# other side), so the in-plane X and Y axes flip and Z stays - "0,0,0,1".  That
# restores the chip frame the original node produced, which is why an unmodified
# robot needs no recalibration.  "1,0,0,0" is the identity (raw chip frame), for
# a board mounted like the original node; any other mounting can be passed the
# same way.  See docs/imu_to_dxl_protocol.md §16.7.
if(FEE_IMU_PERSONALITY STREQUAL "ft6")
    set(_pm_default "0,0,0,1")
else()
    set(_pm_default "")
endif()
set(FEE_IMU_PREMOUNT "${_pm_default}" CACHE STRING
    "ft6 personality: chip-frame rotation 'w,x,y,z' for the FeeTech blocks (default 0,0,0,1 = this board's mounting)")

if(FEE_IMU_PREMOUNT)
    string(REPLACE "," ";" _pm_list "${FEE_IMU_PREMOUNT}")
    list(LENGTH _pm_list _pm_n)
    if(NOT _pm_n EQUAL 4)
        message(FATAL_ERROR
            "FEE_IMU_PREMOUNT must be four comma-separated numbers 'w,x,y,z', got '${FEE_IMU_PREMOUNT}'")
    endif()
    foreach(_i RANGE 0 3)
        list(GET _pm_list ${_i} _pm_${_i})
        string(STRIP "${_pm_${_i}}" _pm_${_i})
        if("${_pm_${_i}}" STREQUAL "")
            message(FATAL_ERROR "FEE_IMU_PREMOUNT has an empty component: '${FEE_IMU_PREMOUNT}'")
        endif()
    endforeach()
else()
    set(_pm_0 1)
    set(_pm_1 0)
    set(_pm_2 0)
    set(_pm_3 0)
endif()

function(ldscript_for_slot slot out)
    if(slot STREQUAL "0")
        set(${out} ${CMAKE_CURRENT_SOURCE_DIR}/linker/gd32f303cc_app.ld PARENT_SCOPE)
    elseif(slot STREQUAL "1")
        set(${out} ${CMAKE_CURRENT_SOURCE_DIR}/linker/gd32f303cc_slot_a.ld PARENT_SCOPE)
    elseif(slot STREQUAL "2")
        set(${out} ${CMAKE_CURRENT_SOURCE_DIR}/linker/gd32f303cc_slot_b.ld PARENT_SCOPE)
    elseif(slot STREQUAL "boot")
        set(${out} ${CMAKE_CURRENT_SOURCE_DIR}/linker/gd32f303cc_boot.ld PARENT_SCOPE)
    else()
        message(FATAL_ERROR "slot must be 0, 1, 2 or boot, got '${slot}'")
    endif()
endfunction()

if(NOT APP_SLOT MATCHES "^[012]$")
    message(FATAL_ERROR "APP_SLOT must be 0, 1 or 2")
endif()

# ---- GD32 firmware library -------------------------------------------------
set(FW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../GD32F30x_Firmware_Library)
if(NOT EXISTS ${FW_DIR})
    message(FATAL_ERROR "GD32F30x_Firmware_Library not found at ${FW_DIR}")
endif()

set(STARTUP_FILE ${FW_DIR}/Firmware/CMSIS/GD/GD32F30x/Source/GCC/startup_gd32f30x_hd.S)

# Only the peripheral drivers this project uses.
set(FW_SOURCES
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_gpio.c
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_rcu.c
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_misc.c
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_usart.c
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_adc.c
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_dma.c
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_spi.c
    ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Source/gd32f30x_fmc.c
)

# The two USART slaves and the protocol auto-detection are compiled into *both*
# images from the same sources, which is what makes boot mode speak exactly the
# same protocol as the application.
set(PROTO_SOURCES
    src/uart_port.c
    src/us_time.c
    src/bus_arb.c
    src/dbg.c
    src/crc16.c
    src/stuffing.c
    src/dxl2.c
    src/fee.c
    src/bus.c
    src/gd32f30x_it.c
    src/systick.c
    src/system_gd32f30x.c
)

# Shared by the application and the bootloader: the image header, the flash
# abstraction, the configuration page and the SRAM handshake word.
set(BOOT_SUPPORT_SOURCES
    src/app_header.c
    src/dev_cfg.c
    src/boot_crc32.c
    src/boot_flash.c
    src/boot_ram.c
    src/cfg_blob.c
)

set(APP_SOURCES
    src/main.c
    src/imu_math.c
    src/imu.c
    src/imu_sim.c
    src/imu_spi.c
    src/dev.c
    src/telem_pack.c
    ${BOOT_SUPPORT_SOURCES}
)

set(BOOT_SOURCES
    src/boot_main.c
    src/boot_regs.c
    src/boot_decide.c
    src/boot_upgrade.c
    ${BOOT_SUPPORT_SOURCES}
)

# Cortex-M4F with single-precision FPU: the simulator runs on it
set(MCU_ARCH_FLAGS -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16)

# The packaging tool writes the 64-byte app header (and the .ipkg upgrade
# package) after objcopy has produced the raw image.
find_program(PYTHON3_EXE NAMES python3)

# add_firmware(<target> <slot> <sources...>)
#
# slot 0 = development image at 0x08000000 (no header, no package)
# slot 1/2 = application image for slot A/B, header patched in afterwards
function(add_firmware target slot)
    set(sources ${ARGN})
    ldscript_for_slot(${slot} LDSCRIPT)
    # the bootloader links at 0x08000000 like the development image, but with a
    # 32 KB window and its own script
    if(slot STREQUAL "boot")
        set(app_slot_def 0)
    else()
        set(app_slot_def ${slot})
    endif()

    add_executable(${target} ${sources} ${STARTUP_FILE} ${FW_SOURCES})
    set_target_properties(${target} PROPERTIES SUFFIX ".elf" LINK_DEPENDS ${LDSCRIPT})

    target_include_directories(${target} PRIVATE
        src
        ${FW_DIR}/Firmware/CMSIS
        ${FW_DIR}/Firmware/CMSIS/GD/GD32F30x/Include
        ${FW_DIR}/Firmware/GD32F30x_standard_peripheral/Include
    )

    target_compile_definitions(${target} PRIVATE
        GD32F30X_HD
        SYS_CLK_SOURCE=${SYS_CLK_SOURCE_VAL}
        SYS_CLK_HXTAL_HZ=${SYS_CLK_HXTAL_HZ}
        HXTAL_VALUE=${SYS_CLK_HXTAL_HZ}
        APP_SLOT=${app_slot_def}
        DBG_ENABLE=${DBG_ENABLE}
        BUS_MIRROR_ENABLE=${BUS_MIRROR_ENABLE}
        STATUS_LED_ENABLE=${STATUS_LED_ENABLE}
        CFG_FLASH_ENABLE=${CFG_FLASH_ENABLE}
        TEMP_SENSOR_ENABLE=${TEMP_SENSOR_ENABLE}
        IMU_USE_SPI=${IMU_USE_SPI}
        FEE_IMU_PERSONALITY_FT6=${FEE_IMU_PERSONALITY_FT6_VAL}
        FEE_IMU_PREMOUNT_QW=${_pm_0}
        FEE_IMU_PREMOUNT_QX=${_pm_1}
        FEE_IMU_PREMOUNT_QY=${_pm_2}
        FEE_IMU_PREMOUNT_QZ=${_pm_3}
    )

    target_compile_options(${target} PRIVATE
        ${MCU_ARCH_FLAGS}
        -Wall -Wextra -Wno-unused-parameter
        -ffunction-sections -fdata-sections
        $<$<CONFIG:Release>:-Os>
        $<$<CONFIG:Debug>:-Og -g3>
    )

    target_link_options(${target} PRIVATE
        ${MCU_ARCH_FLAGS}
        -specs=nano.specs
        -specs=nosys.specs
        -T ${LDSCRIPT}
        -Wl,--gc-sections
        -Wl,-Map=${target}.map
        -Wl,--print-memory-usage
    )

    target_link_libraries(${target} PRIVATE m)

    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND arm-none-eabi-objcopy -O ihex   $<TARGET_FILE:${target}> ${target}.hex
        COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:${target}> ${target}.bin
        COMMENT "Generating hex/bin for ${target}"
    )

    if(slot STREQUAL "1")
        set(slot_name a)
    elseif(slot STREQUAL "2")
        set(slot_name b)
    else()
        set(slot_name "")
    endif()

    if(NOT slot_name STREQUAL "" AND PYTHON3_EXE)
        # Patch the app header into the image and emit the bus upgrade package.
        # No-op for the development image (slot 0), which has no header.
        add_custom_command(TARGET ${target} POST_BUILD
            COMMAND ${PYTHON3_EXE} ${CMAKE_CURRENT_SOURCE_DIR}/host/package.py pack
                    --prefix ${CMAKE_CURRENT_BINARY_DIR}/${target}
                    --slot ${slot_name}
                    --version ${APP_VERSION_STR}
            COMMENT "Packaging ${target} (app header + .ipkg)"
        )
    elseif(NOT slot_name STREQUAL "")
        message(WARNING "python3 not found: ${target} will not get an app header")
    endif()

    add_custom_command(TARGET ${target} POST_BUILD
        COMMAND arm-none-eabi-size -t $<TARGET_FILE:${target}>
    )
endfunction()

add_firmware(${PROJECT_NAME} ${APP_SLOT} ${APP_SOURCES} ${PROTO_SOURCES})

if(BOOT_ENABLE)
    add_firmware(${PROJECT_NAME}_boot boot ${BOOT_SOURCES} ${PROTO_SOURCES})
    add_firmware(${PROJECT_NAME}_slot_a 1 ${APP_SOURCES} ${PROTO_SOURCES})
    add_firmware(${PROJECT_NAME}_slot_b 2 ${APP_SOURCES} ${PROTO_SOURCES})
endif()
