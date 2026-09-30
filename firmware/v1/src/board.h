/*!
    \file    board.h
    \brief   imu_to_dxl v1 - board wiring, clocking and build-time options

    Target: GD32F303CC (LQFP48, 256 KB flash / 48 KB SRAM), 12 MHz HXTAL,
            system clock 120 MHz - see src/system_gd32f30x.c.

    The board is the reverse-engineered `imu_to_dxl` node: an LSM6DSV16X on SPI0
    that publishes a 12-byte gyro+SFLP-quaternion block as a Dynamixel node
    (ID 200, control-table address 124) on the shared motor bus.

    v1 runs on a bare GD32F303CC development board, so:
      * the SPI IMU is not fitted - src/imu_sim.c synthesises the sensor block;
      * USART1 (PA2/PA3) is the real motor bus and stays wired as such;
      * USART0 (PB6/PB7, on the host as /dev/ttyUSB0) doubles as the debug
        console AND as a bus mirror, so the PC test tool can exercise the
        protocol before the real PCBA comes back.  See BUS_MIRROR_ENABLE.

    Pin map (final hardware, `elec_imu_to_dxl/imu_to_dxl/imu_to_dxl.kicad_sch`):

      PA2  USART1_TX   motor bus, half duplex through the auto-direction
                       buffer (TXD_EN is generated from TX, no GPIO involved)
      PA3  USART1_RX   motor bus receive
      PA4  SPI0_NSS    IMU chip select (active low)
      PA5  SPI0_SCK    IMU clock
      PA6  SPI0_MISO   IMU data out
      PA7  SPI0_MOSI   IMU data in
      PB0  INT1        IMU interrupt 1
      PB1  INT2        IMU interrupt 2
      PB6  USART0_TX   debug / bus mirror (host: /dev/ttyUSB0)
      PB7  USART0_RX   debug / bus mirror
      PB5  LED         development-board indicator (STATUS_LED_ENABLE)
*/

#ifndef BOARD_H
#define BOARD_H

#ifdef IMU_TO_DXL_HOST_TEST
/* Host-side protocol tests (host/tools/test_protocols.c) compile dxl2.c, fee.c
   and imu_sim.c with gcc, where the GD32 headers do not exist.  Only plain
   integer types are needed there. */
#include <stdint.h>
#else
#include "gd32f30x.h"
#endif

/* ── firmware identity ───────────────────────────────────────────────────── */

#define FW_VERSION_MAJOR    1
#define FW_VERSION_MINOR    0
/* one byte, high nibble = major, low nibble = minor - the Dynamixel
   "firmware version" register format */
#define FW_VERSION_BYTE     ((FW_VERSION_MAJOR << 4) | (FW_VERSION_MINOR & 0x0F))
#define FW_BUILD_NAME       "imu_to_dxl v1"

/* ── build-time options (all overridable from CMake) ─────────────────────── */

/* Debug console on USART0.  0 compiles the whole dbg layer out. */
#ifndef DBG_ENABLE
#define DBG_ENABLE              1
#endif

/* USART0 (PB6/PB7) is the *debug console* and nothing else: it prints DBG_*
   lines so a problem can be found.  Every bus transaction - the register
   protocol and the field upgrade - belongs to USART1 (PA2/PA3), the single
   wire motor bus this node shares with the servos (`docs/upgrade.md` §1,
   `docs/imu_to_dxl_protocol.md` §2).

   1 -> bench only.  USART0 also answers protocol frames, so a development
        board with no motor bus wired yet can be driven over /dev/ttyUSB0.
        A host must then still point the upgrade tool at the bus it means to
        use: everything the protocol can do, this port can do too. */
#ifndef BUS_MIRROR_ENABLE
#define BUS_MIRROR_ENABLE       0
#endif

/* PB5 status LED, active high.  Only the development board has it. */
#ifndef STATUS_LED_ENABLE
#define STATUS_LED_ENABLE       1
#endif

/* Persist ID / baud / simulation settings into the last flash page. */
#ifndef CFG_FLASH_ENABLE
#define CFG_FLASH_ENABLE        1
#endif

/* Read the MCU internal temperature sensor (ADC0 channel 16) for the
   `present temperature` register. */
#ifndef TEMP_SENSOR_ENABLE
#define TEMP_SENSOR_ENABLE      1
#endif

/* Sensor source.  1 = the LSM6DSV16X on SPI0 (the real PCBA, since 2026-09-25);
   0 = src/imu_sim.c, which keeps working with no chip fitted and is what the
   bench smoke test's simulation modes exercise.  The block, the units and the
   flags are identical either way - src/imu.c is the only file that knows. */
#ifndef IMU_USE_SPI
#define IMU_USE_SPI             1
#endif

/* 0 = link at 0x08000000 (development board, no bootloader yet)
   1 = link at slot A, 2 = link at slot B (see linker/, docs/flash_layout.md) */
#ifndef APP_SLOT
#define APP_SLOT                0
#endif

/* ── flash layout (see docs/flash_layout.md) ────────────────────────────── */

#define BOOT_BASE               0x08000000U
#define BOOT_SIZE               (32U * 1024U)
#define SLOT_A_BASE             0x08008000U
#define SLOT_B_BASE             0x08020000U
#define SLOT_SIZE               (96U * 1024U)
#define CFG_FLASH_ADDR          0x0803F800U   /* last 2 KB page */
#define CFG_FLASH_SIZE          2048U
#define FLASH_PAGE_SIZE         2048U
#define FLASH_BASE_ADDR         0x08000000U
#define FLASH_TOTAL_SIZE        (256U * 1024U)

/* slot indices used by the boot decision, the upgrade state machine and the
   host tools (`SLOT_NONE` means "no slot") */
#define SLOT_A                  0U
#define SLOT_B                  1U
#define SLOT_NONE               0xFFU

/* ── application image header ───────────────────────────────────────────── */

/* The header lives at slot_base + APP_HDR_OFF.  The vector table stays at
   slot_base itself: the Cortex-M4 fetches the initial MSP and the reset vector
   from the first two words, so nothing may be placed in front of it.  See
   docs/flash_layout.md §2 for why 0x200 and not 0x150/0x400. */
#define APP_HDR_OFF             0x200U
#define APP_HDR_SIZE            64U
#define APP_HDR_MAGIC           0x314D4841UL   /* 'A','H','M','1' */
#define APP_HDR_VERSION         1U
#define APP_HDR_FLAG_TRIAL      0x0001U        /* confirms before it is trusted */
#define APP_HDR_FLAG_CONFIRMED  0x0002U        /* ran long enough to be trusted */
#define APP_HDR_FLAG_NO_BOOT    0x0004U        /* slot deliberately disabled */

#define APP_BOARD_ID            0x0001U        /* imu_to_dxl v1 */
#define APP_PROTO_VERSION       1U             /* bus upgrade protocol version */

/* image_version = (major << 12) | (minor << 8) | patch, so the same build is
   shown as `1.2.3` on the wire and in the host tool.  The Dynamixel firmware
   register (6) carries the major/minor nibbles of exactly this value. */
#define APP_VERSION_MAKE(maj, min, pat) \
    ((uint16_t)((((uint16_t)(maj) & 0x0FU) << 12) | \
                (((uint16_t)(min) & 0x0FU) << 8) | \
                ((uint16_t)(pat) & 0xFFU)))
#define APP_VERSION_MAJOR(v)    ((uint8_t)(((v) >> 12) & 0x0FU))
#define APP_VERSION_MINOR(v)    ((uint8_t)(((v) >> 8) & 0x0FU))
#define APP_VERSION_PATCH(v)    ((uint8_t)((v) & 0xFFU))
#define APP_VERSION             APP_VERSION_MAKE(FW_VERSION_MAJOR, FW_VERSION_MINOR, 0)

/* ── bootloader identity / handshake ────────────────────────────────────── */

/* Different model numbers from the application's (board.h DXL_MODEL_NUMBER /
   FEE_MODEL_NUMBER), so a host can tell "in the bootloader" from "running the
   application" with a single PING/read. */
#define BOOT_MODEL_NUMBER       0xB007U
#define BOOT_FEE_MODEL_NUMBER   0x00B7U

/* Handshake word in the last 256 bytes of SRAM.  It is linked into its own
   NOINIT section (`.boot_ram`, see linker/gd32f303cc_*.ld) that the startup code does not
   clear, so it survives a soft reset - which is how the application asks the
   bootloader for a boot-mode entry without touching flash.  SRAM has no VBAT
   domain on this part, so this does NOT survive power loss (that is what the
   persisted `boot_stay` flag is for). */
#define BOOT_RAM_ADDR           0x2000BF00U
#define BOOT_RAM_MAGIC          0x544F4F42UL   /* 'B','O','O','T' */

/* ── boot state persisted in the configuration page ─────────────────────── */

#define BOOT_ATTEMPTS_MAX       3U      /* trial boots before rolling back */
#define BOOT_CONFIRM_MS         30000U  /* healthy runtime before self-confirm */
#define BOOT_CONFIRM_LONE_MS    60000U  /* ... when no host ever talks to us */
#define BOOT_IDLE_EXIT_MS       30000U  /* boot mode leaves if the host vanishes */

/* ── upgrade register windows (identical in both protocol maps) ─────────── */

/* Identity window 180..195: read-only in both the application and the
   bootloader.  This is how the host learns which mode/slot/version it is
   talking to before it decides to upgrade. */
#define UID_WIN_ADDR            180U
#define UID_WIN_LEN             16U
#define UID_MAGIC               0x5055U        /* 'U','P' */

#define UID_MAGIC_L             0
#define UID_MAGIC_H             1
#define UID_MODE                2   /* UID_MODE_*                            */
#define UID_SLOT_RUN            3   /* SLOT_A / SLOT_B / SLOT_NONE           */
#define UID_FLAGS               4   /* APP_HDR_FLAG_* of the running image   */
#define UID_VER_L               5   /* image_version of the running image    */
#define UID_VER_H               6
#define UID_IMGCRC_0            7   /* combined CRC32 of the running image   */
#define UID_IMGCRC_1            8
#define UID_IMGCRC_2            9
#define UID_IMGCRC_3            10
#define UID_INFO_VERSION        11  /* 1 = the four bytes below are meaningful */
#define UID_BOOT_SLOT           12  /* configuration: last confirmed slot     */
#define UID_TRIAL_SLOT          13  /* configuration: image on trial          */
#define UID_ATTEMPTS            14  /* configuration: trial boots so far      */
#define UID_BOOT_STAY           15  /* configuration: bootloader is holding   */

/* The header's own TRIAL/CONFIRMED flags can never change after the image was
   packaged (flash bits can only be cleared), so the *effective* bit 0 of
   UID_FLAGS is derived from the configuration page instead: it says whether the
   running image still has to confirm itself.  An older node leaves
   UID_INFO_VERSION at 0 and the host shows "unknown" rather than a guess. */
#define UID_INFO_VERSION_1      1U

#define UID_MODE_APP            0U
#define UID_MODE_BOOT           1U

/* UID_FLAGS bit 7 is set by the application when its own header is missing or
   invalid (an image flashed with a debugger instead of packaged/upgraded).  The
   low bits carry the APP_HDR_FLAG_* of the running image. */
#define UID_FLAG_NO_HEADER      0x80U

/* Session window 208..255: the upgrade state machine.  Writing `UPG_MAGIC`
   opens a session; the bootloader consumes `U_CMD` and the block write. */
#define UPGRADE_WIN_ADDR        208U
#define UPGRADE_WIN_LEN         48U
#define UPGRADE_DATA_ADDR       240U
#define UPGRADE_DATA_LEN        16U
#define UPGRADE_MAGIC           0x4247U        /* 'G','B' */

#define U_MAGIC_L               0
#define U_MAGIC_H               1
#define U_CMD                   2   /* UPG_CMD_*; consumed, always reads 0   */
#define U_STATUS                3   /* UPG_ST_*                              */
#define U_TARGET                4   /* SLOT_A / SLOT_B                       */
#define U_SEQ_L                 5
#define U_SEQ_H                 6
#define U_OFF_0                 7   /* byte offset inside the target slot    */
#define U_OFF_1                 8
#define U_OFF_2                 9
#define U_OFF_3                 10
#define U_BLKLEN                11  /* 4..16, multiple of 4                  */
#define U_BLKCRC_L              12  /* crc16_dxl over the block              */
#define U_BLKCRC_H              13
#define U_ACK_SEQ_L             14  /* last block the node accepted          */
#define U_ACK_SEQ_H             15
#define U_ERRCODE               16  /* UPG_ERR_*                             */
#define U_VER_L                 17  /* image_version to write into the header*/
#define U_VER_H                 18
#define U_NODE_CRC_0            19  /* CRC32 the node computed in UPG_END    */
#define U_NODE_CRC_1            20
#define U_NODE_CRC_2            21
#define U_NODE_CRC_3            22
#define U_RESERVED              23
#define U_IMGLEN_0              24  /* whole image length                    */
#define U_IMGLEN_1              25
#define U_IMGLEN_2              26
#define U_IMGLEN_3              27
#define U_IMGCRC_0              28  /* CRC32 the host computed               */
#define U_IMGCRC_1              29
#define U_IMGCRC_2              30
#define U_IMGCRC_3              31
#define U_DATA_OFF              (UPGRADE_DATA_ADDR - UPGRADE_WIN_ADDR)  /* 32 */
/* 208 + 31 = 239; U_DATA covers 240..255 */

/* block writes must be whole 32-bit words: the FMC programs one word per
   operation and the target region is erased, so a partial word cannot be
   written without a read-modify-write cycle.  The packaging tool pads the
   image to a multiple of 4 and the host sends 4/8/12/16 byte blocks. */
#define UPG_BLK_MIN             4U
#define UPG_BLK_MAX             UPGRADE_DATA_LEN
#define UPG_MIN_IMGLEN          (APP_HDR_OFF + APP_HDR_SIZE)

#define UPG_CMD_NONE            0
#define UPG_CMD_BEGIN           1
#define UPG_CMD_DATA            2
#define UPG_CMD_END             3
#define UPG_CMD_ABORT           4
#define UPG_CMD_CONFIRM         5   /* application only; boot mode errors */
#define UPG_CMD_REBOOT          6
#define UPG_CMD_READBACK        7

#define UPG_ST_IDLE             0U
#define UPG_ST_ERASED           1U
#define UPG_ST_RECEIVING        2U
#define UPG_ST_BLOCK_OK         3U
#define UPG_ST_VERIFIED         4U
#define UPG_ST_COMMITTED        5U
#define UPG_ST_ERROR            0x80U   /* OR-ed with UPG_ERR_* below */

#define UPG_ERR_NONE            0U
#define UPG_ERR_CRC             1U
#define UPG_ERR_RANGE           2U
#define UPG_ERR_FLASH           3U
#define UPG_ERR_STATE           4U
#define UPG_ERR_TARGET          5U
#define UPG_ERR_MAGIC           6U
#define UPG_ERR_VERIFY          7U
#define UPG_ERR_LENGTH          8U

/* ── device defaults ────────────────────────────────────────────────────── */

/* Dynamixel ID of the IMU node on the shared bus (microduck: `IMU_DXL_ID`). */
#define DXL_ID_DEFAULT          200
/* FeeTech (SCS/HLS) node ID.  FeeTech IDs run 0..253, so 253 (0xFD) is
   forbidden - it is the Dynamixel protocol-2.0 header byte and the whole
   protocol auto-detection leans on that. */
#define FEE_ID_DEFAULT          200
#define FEE_ID_MAX              252

/* 1 Mbps on both buses, matching microduck's BAUD_RATE. */
#define BUS_BAUD_DEFAULT        1000000U
#define BUS_MIRROR_BAUD_DEFAULT 1000000U
#define DBG_BAUD_DEFAULT        115200U

/* Dynamixel baud-rate register code for 1 Mbps (XL330 table: 0=9600 ...
   3=1 Mbps). */
#define DXL_BAUD_CODE_1M        3
/* FeeTech baud-rate register code for 1 Mbps (INST.h: 0=1M). */
#define FEE_BAUD_CODE_1M        0

/* Dynamixel model number reported by this node.  1200 is the XL330 model, so
   off-the-shelf tools pick up the right unit conversions for the register
   layout the node mimics.  The IMU block itself is vendor-defined. */
#define DXL_MODEL_NUMBER        1200
/* FeeTech model number (HLS memory table 3..4, little endian).  Deliberately
   NOT a servo model code: a real HD-1910 reports 0x1F0A, and FeeTech tooling
   keys its behaviour off the model, so an unmistakable value ('I','M') says
   "not a servo" instead of making a tool guess which frame layout this is.
   The bootloader reports BOOT_FEE_MODEL_NUMBER instead, which is how a single
   PING tells "in the bootloader" from "running the application". */
#define FEE_MODEL_NUMBER        0x4D49

/* ── mounting ───────────────────────────────────────────────────────────── */

/* The board sits on the trunk so that trunk = [+raw_z, +raw_y, -raw_x]:
   a +90 degree rotation about Y, scalar-first.  Same constant as microduck's
   `SflpDecoder::DEFAULT_MOUNT`. */
#define MOUNT_QW                0.70710678f
#define MOUNT_QX                0.0f
#define MOUNT_QY                0.70710678f
#define MOUNT_QZ                0.0f

/* ── IMU ────────────────────────────────────────────────────────────────── */

#define IMU_SAMPLE_HZ           100U
#define IMU_SAMPLE_PERIOD_MS    (1000U / IMU_SAMPLE_HZ)
/* ±500 dps at 17.5 mdps/LSB */
#define IMU_GYRO_MDPS_PER_LSB   17.5f
/* ±4 g at 0.122 mg/LSB */
#define IMU_ACCEL_MG_PER_LSB    0.122f

/* ── pins ───────────────────────────────────────────────────────────────── */

#define BUS_USART               USART1
#define BUS_GPIO                GPIOA
#define BUS_TX_PIN              GPIO_PIN_2
#define BUS_RX_PIN              GPIO_PIN_3

#define DBG_USART               USART0
#define DBG_GPIO                GPIOB
#define DBG_TX_PIN              GPIO_PIN_6
#define DBG_RX_PIN              GPIO_PIN_7

#if STATUS_LED_ENABLE
#define LED_GPIO                GPIOB
#define LED_PIN                 GPIO_PIN_5
#endif

#define IMU_SPI_GPIO            GPIOA
#define IMU_SPI                 SPI0
#define IMU_CS_PIN              GPIO_PIN_4
#define IMU_SCK_PIN             GPIO_PIN_5
#define IMU_MISO_PIN            GPIO_PIN_6
#define IMU_MOSI_PIN            GPIO_PIN_7
#define IMU_INT1_GPIO           GPIOB
#define IMU_INT1_PIN            GPIO_PIN_0
#define IMU_INT2_GPIO           GPIOB
#define IMU_INT2_PIN            GPIO_PIN_1

/* ── protocol constants shared by the two slaves ────────────────────────── */

#define DXL_BROADCAST_ID        0xFE
#define DXL_HEADER_0            0xFF
#define DXL_HEADER_1            0xFF
#define DXL_HEADER_2            0xFD
#define DXL_HEADER_3            0x00
#define DXL_STATUS_MARKER       0x55   /* protocol 2.0 status packets carry 0x55
                                          where an instruction packet has INST */

/* Dynamixel 2.0 error bits, as decoded by rustypot's `DynamixelErrorV2`. */
#define DXL_ERR_RESULT_FAIL     0x80
#define DXL_ERR_INSTRUCTION     0x40
#define DXL_ERR_CRC             0x20
#define DXL_ERR_RANGE           0x10
#define DXL_ERR_LENGTH          0x08
#define DXL_ERR_LIMIT           0x04
#define DXL_ERR_ACCESS          0x02

#define DXL_INST_PING           0x01
#define DXL_INST_READ           0x02
#define DXL_INST_WRITE          0x03
#define DXL_INST_REG_WRITE      0x04
#define DXL_INST_ACTION         0x05
#define DXL_INST_FACTORY_RESET  0x06
#define DXL_INST_REBOOT         0x08
#define DXL_INST_CLEAR          0x10
#define DXL_INST_SYNC_READ      0x82
#define DXL_INST_SYNC_WRITE     0x83
/* Fast Sync Read: the *instruction* is a SYNC_READ byte for byte with a
   different opcode, but the answer is a different packet shape - every
   addressed device appends one block (ERR + ID + DATA + CRC) to a single status
   packet whose id is 0xFE, the CRC of a block covers the whole packet up to
   that block, and nothing is byte-stuffed.  So the first device in the id list
   sends the 8-byte prefix and the others only add their own block, and a device
   that is not first has to hear the blocks ahead of it before it can answer at
   all (src/dxl2.c, "fast sync read").  microduck's tick read puts this node at
   index 0, which is the cheap case. */
#define DXL_INST_FAST_SYNC_READ 0x8A
#define DXL_INST_BULK_READ      0x92
#define DXL_INST_BULK_WRITE     0x93

#define FEE_HEADER_0            0xFF
#define FEE_HEADER_1            0xFF
#define FEE_BROADCAST_ID        0xFE
#define FEE_INST_PING           0x01
#define FEE_INST_READ           0x02
#define FEE_INST_WRITE          0x03
#define FEE_INST_REG_WRITE      0x04
#define FEE_INST_REG_ACTION     0x05
#define FEE_INST_RECOVERY       0x06
/* 0x08 REBOOT: exists on the real HD-1910 (fw 3.46), which answers nothing and
   is alive again after 823 ms measured (manual: ~800 ms); EEPROM settings
   survive, RAM gains reload from EEPROM.  It is missing from the older vendor
   instruction table this project's 飞特通讯协议说明.md was built from. */
#define FEE_INST_REBOOT         0x08
#define FEE_INST_SYNC_READ      0x82
#define FEE_INST_SYNC_WRITE     0x83
#define FEE_INST_RESET          0x0A
#define FEE_INST_CAL            0x0B

/* ── register image sizes ───────────────────────────────────────────────── */

#define REG_SPACE_SIZE          256
/* telemetry block: gyro(6) + quat xyz half(6) + accel(6) + counter(1) + flags(1) */
#define TELEM_LEN               20
#define TELEM_CTRL_LEN          12   /* what microduck consumes per tick */
#define TELEM_CNT               18   /* sample counter inside the 20-byte block */
#define TELEM_STATUS            19   /* status flags inside the 20-byte block  */

/* Where the block lives in each register map */
#define DXL_TELEM_ADDR          124  /* present_pwm .. position_trajectory */
#define FEE_TELEM_ADDR          56   /* present_position .. present_current */
#define FEE_TELEM_ALT_ADDR      128  /* clean vendor window */

/* The FeeTech map answers at 56 with the 15-byte shape a servo uses there, so
   the counter and status fit inside the span the runtime reads in the shared
   `sync_read`; the raw accelerometer moves to the 128 alias, which is not part
   of that transaction.  See src/telem_pack.h, docs/bus_timing_borrow_plan.md. */
#define FEE_BLOCK_LEN           15
#define FEE_BLOCK_CNT           12
#define FEE_BLOCK_STATUS        13
#define FEE_BLOCK_RESERVED      14

/* One reply slot.  A real HD-1910 answers one slot later for every id that
   precedes it in a sync_read list, present or absent: measured +0.302 ms for
   one absent id ahead, +0.595 for two, +0.882 for three (fw 3.46, 1 Mbps).
   300 us is that number rounded up - and it is also the whole window the
   devices behind us leave for our own reply, which is what makes the response
   path hard-real-time instead of "usually fast enough". */
#define BUS_SLOT_US             300U

/* Vendor/config window: 20 bytes, same layout in both maps. */
#define DXL_VENDOR_ADDR         148
#define FEE_VENDOR_ADDR         160

#define VENDOR_MAGIC            0x4D49u   /* 'I','M' little endian */

/* vendor window offsets (relative to the window base) */
#define V_MAGIC_L               0
#define V_MAGIC_H               1
#define V_CMD                   2
#define V_SIM_MODE              3
#define V_SIM_AMP_L             4
#define V_SIM_AMP_H             5
#define V_SIM_FREQ_L            6
#define V_SIM_FREQ_H            7
#define V_REPORT_FRAME          8
#define V_PROTO_LOCK            9
#define V_BIAS_X_L              10
#define V_BIAS_X_H              11
#define V_BIAS_Y_L              12
#define V_BIAS_Y_H              13
#define V_BIAS_Z_L              14
#define V_BIAS_Z_H              15
#define V_MIRROR_BAUD           16
#define V_DBG_LEVEL             17
#define V_STATUS                18
/* Last protocol error this node refused or clamped (DXL_ERR_*), published for
   hosts that ask on purpose.  The reply's own status byte stays 0: FeeTech
   tooling reads a nonzero ack status as a servo fault, and this node's own host
   treats it as one. */
#define V_LAST_ERR              19

/* vendor commands (write to V_CMD; the node clears it after running) */
#define VCMD_NONE               0
#define VCMD_SAVE               1   /* persist configuration to flash */
#define VCMD_DEFAULTS           2   /* restore factory defaults */
#define VCMD_REBOOT             3   /* software reset */
#define VCMD_RESET_SIM          4   /* restart the motion model / fusion */
#define VCMD_CONFIRM            5   /* accept the running trial image */
#define VCMD_BOOT               6   /* soft reset into the bootloader (stay) */
#define VCMD_SWITCH_SLOT        7   /* reboot into the other slot (trial) */

/* telemetry status flag bits (block byte 19) */
#define TELEM_FLAG_SFLP_VALID   0x01
#define TELEM_FLAG_CNT_WRAP     0x02
#define TELEM_FLAG_GYRO_SAT     0x04
#define TELEM_FLAG_SENSOR_ERR   0x08
#define TELEM_FLAG_SIMULATED    0x10
#define TELEM_FLAG_FROZEN       0x20
#define TELEM_FLAG_CFG_DIRTY    0x40
#define TELEM_FLAG_FUSION_OK    0x80

/* register-image status bits (vendor V_STATUS) */
#define VSTAT_CFG_DIRTY         0x01
#define VSTAT_CFG_FROM_FLASH    0x02
#define VSTAT_SIM_ACTIVE        0x04

/* protocol tags used across bus/dev/proto */
#define PROTO_NONE              0
#define PROTO_DXL               1
#define PROTO_FEE               2

#endif /* BOARD_H */
