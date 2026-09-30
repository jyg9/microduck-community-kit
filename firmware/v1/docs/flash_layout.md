# imu_to_dxl v1 Flash 布局与总线升级设计

目标 MCU：**GD32F303CC**，LQFP48，256 KB code flash @ `0x08000000`，48 KB SRAM @ `0x20000000`，
12 MHz HXTAL，120 MHz 内核（`src/system_gd32f30x.c`，`SYS_CLK_SOURCE=HXTAL`）。
Flash 页大小 **2 KB，共 128 页**（GD32F30x 用户手册 Rev3.4：容量不超过 512 KB 的
GD32F30x_CL/HD 只使用 bank0，bank0 页大小为 2 KB；256 KB / 2 KB = 128 页）。FMC 以
32 位整字编程，因此所有写入必须 4 字节对齐、按页擦除。

本文档区分三种状态：

| 标记 | 含义 |
|---|---|
| **已实现** | 代码已在 `v1/src/` 中，且被 `v1/host/tools/` 的测试或 `./build.sh` 覆盖 |
| **已预留** | 常量/链接脚本已存在，但没有任何代码使用 |
| **设计** | 本文档提出的方案，尚未实现 |

**一句话总结现状（2026-09-19）**：本文档描述的方案**已经全部实现并在真机验证**。
代码在 `v1/src/boot_*.c`、`v1/src/app_header.c`、`v1/src/cfg_blob.c`；
链接脚本 `v1/linker/gd32f303cc_boot.ld` 与两个 slot 脚本；主机构建/烧录见
`v1/build.sh`，升级工具 `v1/host/upgrade.py`，打包工具 `v1/host/package.py`。
台面实测记录、与本文档的**实现偏差**、以及两个由真机暴露出来的 bug 见 §9 与
`docs/upgrade.md` §8。

---

## 1. Flash 映射

### 1.1 区域表

| 区域 | 地址范围 | 大小 | 内容 | 谁写 |
|---|---|---|---|---|
| Bootloader | `0x08000000` – `0x08007FFF` | 32 KB | 复位入口、极简 USART1 协议解析、flash 驱动、header 校验、跳转、升级状态机 | 仅 SWD/J-Link（**设计**：是否支持自升级见 §8） |
| Slot A | `0x08008000` – `0x0801FFFF` | 96 KB | 应用镜像 A：`[0x0000]` Cortex-M 向量表，`[0x0200]` `app_header_t`，`[0x0240]` 起 `.text/.rodata/.data LMA` | 生产：J-Link；现场：bootloader 总线升级（**设计**） |
| Slot B | `0x08020000` – `0x08037FFF` | 96 KB | 应用镜像 B，同上 | 同上 |
| 自由区（未分配） | `0x08038000` – `0x0803F7FF` | 30 KB | 当前无用途。可留给：双份日志页、第二配置页、9 轴/标定页、未来的遥测黑匣子 | 无人写（**设计**：若采用"boot 状态独立成页"的替代方案则占用此区，见 §4.4） |
| 配置页 | `0x0803F800` – `0x0803FFFF` | 2 KB（最后 1 页） | `cfg_blob_t`（magic `'AIM1'` = `0x314D4941`，`dev_cfg_t` + CRC16） | 应用：`dev_cfg_save_now()`；**设计**：bootloader 也要写其中的 boot 状态字段 |

常量来源：`v1/src/board.h` 的 `BOOT_BASE` / `BOOT_SIZE` / `SLOT_A_BASE` / `SLOT_B_BASE` /
`SLOT_SIZE` / `CFG_FLASH_ADDR` / `CFG_FLASH_SIZE`；链接脚本 `v1/linker/gd32f303cc_slot_a.ld`
（`ORIGIN = 0x08008000, LENGTH = 96K`）与 `gd32f303cc_slot_b.ld`（`ORIGIN = 0x08020000, LENGTH = 96K`）。

校验地址算术：

```
BOOT_BASE     0x08000000 + 32K = 0x08008000 = SLOT_A_BASE
SLOT_A_BASE   0x08008000 + 96K = 0x08020000 = SLOT_B_BASE
SLOT_B_BASE   0x08020000 + 96K = 0x08038000
CFG_FLASH     0x0803F800，配置页 2 KB，末尾到 0x08040000 = 256 KB
0x08038000 ~ 0x0803F800 = 0x7800 = 30 KB 自由区
```

`v1/linker/gd32f303cc_app.ld` 的 `ORIGIN = 0x08000000, LENGTH = 254K` 是
**开发板/无 bootloader** 构型（`APP_SLOT=0`），它覆盖了 bootloader 区；量产必须有
bootloader，因此量产一律用 `APP_SLOT=1|2`。两者不能共存于同一颗芯片。

### 1.2 slot 内部布局

| slot 内偏移 | 绝对地址（slot A） | 大小 | 内容 |
|---|---|---|---|
| `0x0000` | `0x08008000` | 4 | 初始 MSP（`_sp`，来自 startup 文件 + 链接脚本 `.heap_stack`） |
| `0x0004` | `0x08008004` | 4 | `Reset_Handler` 地址 |
| `0x0008` | `0x08008008` | — | 其余异常/中断向量（见 §2） |
| `0x0200` | `0x08008200` | 64 | `app_header_t`（**设计**，本文 §3 定义） |
| `0x0240` | `0x08008240` | ≤ 96 KB − `0x240` | `.text` / `.rodata` / `.ARM*` / `.data` 的 LMA（**注**：现有链接脚本没有为 header 留洞，必须改，见 §7） |

当前 `APP_SLOT=1|2` 时 `src/main.c` 只做一件事：

```c
#if (APP_SLOT == 1)
    SCB->VTOR = SLOT_A_BASE;
#elif (APP_SLOT == 2)
    SCB->VTOR = SLOT_B_BASE;
#endif
```

（在 `main()` 开头、`systick_config()` 之前）。只要向量表仍在 slot base，
这段代码在加入 header 之后**不需要改动**。

---

## 2. 为什么 app header 不能和向量表重叠

### 2.1 向量表有多大

| 口径 | 条目数 | 字节数 | 说明 |
|---|---|---|---|
| Cortex-M4 架构上限（16 系统 + 240 IRQ） | 256 | 1024 | 芯片实际不实现这么多 |
| GD32F303 **HD** 满配（16 系统 + 68 IRQ） | 84 | **336** (`0x150`) | 数据手册给出的中断通道上限，`GD32F30X_HD` 宏即此档 |
| 本工程 `startup_gd32f30x_hd.S` 实际定义的 `.vectors` | 76 | **304** (`0x130`) | 交叉验证：`arm-none-eabi-objdump -h build/gd32f303cc_imu_to_dxl.elf` 中 `.vectors` 段大小 `0x130`；`nm` 中 `__gVectors = 0x08000000`、`__Vectors_End = 0x08000130` |

前 4 字节是初始 MSP，第 5 个 4 字节是 `Reset_Handler`，这是 CPU 硬件约定，任何
抢占它们的结构都会让芯片无法启动：

* 上电/复位时 M4 从 `0x00000000`（映射到 `0x08000000`）取 MSP，再从 `+4` 取入口；
* 异常发生时从 `SCB->VTOR + 4*n` 取 handler；
* `main()`（或 bootloader 的跳转代码）会写 `SCB->VTOR`，**不会重排向量内容**。

### 2.2 推荐偏移：`0x200`（512 字节）

| 候选偏移 | 相对 336 B 满配表 | 评价 |
|---|---|---|
| `0x150` | 0 余量 | 不可用，表一长就覆盖 header |
| `0x180` (384) | +48 B | 余量太小，且不是 2 的幂 |
| **`0x200` (512)** | **+176 B** | **推荐**：2 的幂、半页对齐、易记；给未来 IRQ 增加和 `.vectors` 段对齐留足空间；崩溃 dump/CRC 区间边界干净 |
| `0x400` (1024) | +688 B | 浪费 512 B，无收益 |

### 2.3 两种"header 在 slot base"的诱惑，以及为什么不行

1. **把 magic/长度放在 slot base**：会覆盖初始 MSP 或 `Reset_Handler` 指针，芯片复位即
   HardFault，**绝对不可行**。
2. **把 header 放在 slot base、向量表后移**：则 `SCB->VTOR = SLOT_x_BASE` 不再有效，
   `main.c` 要改成 `VTOR = SLOT_x_BASE + 0x200`，且 bootloader 的跳转、`__Vectors_Size`
   、调试器符号全部要跟着偏移，破坏"向量表固定在 slot base"这一条最省心的约定。
   本设计**选择让向量表继续留在 slot base**。

### 2.4 两个"表位置"必须同时记录

因为分开表达，所以文档里必须同时写明：

* **向量表位置**：`slot_base + 0x0000`。bootloader 从 `slot_base + 0` 取 MSP，
  `slot_base + 4` 取入口（见 §4 的跳转流程）。
* **header 位置**：`slot_base + APP_HDR_OFF`（`0x200`）。bootloader 从这里读
  长度和 CRC，用于"这个 slot 是否可信"。header 里也存一份 `entry` 副本，
  便于 host 工具与日志显示；**跳转以向量表为准，header 里的 `entry` 仅作交叉校验**
  （不一致则视为镜像损坏，拒绝启动）。

---

## 3. `app_header_t` 定义（设计）

### 3.1 C 结构

```c
/* 新增：src/app_header.h —— 必须能被主机 gcc 编译（升级/打包工具与 bootloader 共用） */

#define APP_HDR_MAGIC    0x314D4841UL   /* 'A','H','M','1' -> "AHM1"，与配置页的 'AIM1' 区分 */
#define APP_HDR_VERSION  1U
#define APP_HDR_OFF      0x200U         /* 相对 slot base；见 §2 */
#define APP_HDR_SIZE     64U            /* 4 字节对齐，余下字节为 0 */

#define APP_HDR_FLAG_TRIAL      0x0001U /* 试验镜像：确认前可回滚 */
#define APP_HDR_FLAG_CONFIRMED  0x0002U /* 已被运行中的应用/主机确认 */
#define APP_HDR_FLAG_NO_BOOT    0x0004U /* 禁止启动（用于"作废"一个 slot） */

#define APP_BOARD_ID_IMU_TO_DXL_V1  0x0001U  /* board.h 里没有对应常量，新增 */

typedef struct {
    uint32_t magic;            /* APP_HDR_MAGIC                      */
    uint16_t header_version;   /* APP_HDR_VERSION                    */
    uint16_t image_version;    /* (major<<12)|(minor<<8)|patch       */
    uint32_t image_size;       /* 从 slot base 起的有效镜像字节数     */
    uint32_t crc_low;          /* CRC32( slot_base .. slot_base+APP_HDR_OFF-1 )            */
    uint32_t crc_high;         /* CRC32( slot_base+APP_HDR_OFF+APP_HDR_SIZE .. 镜像末尾 )  */
    uint32_t build_unix;       /* 构建时刻（Unix 秒，UTC）；仅供人看      */
    uint32_t entry;            /* == 向量表[1]，交叉校验用               */
    uint16_t board_id;         /* APP_BOARD_ID_*                     */
    uint16_t proto_version;    /* 该镜像支持的总线升级协议版本（当前 1） */
    uint16_t flags;            /* APP_HDR_FLAG_*                     */
    uint8_t  reserved[30];     /* 0；34+30 = 64，且自然 4 字节对齐      */
} app_header_t;

typedef char app_header_must_be_64[(sizeof(app_header_t) == APP_HDR_SIZE) ? 1 : -1];
```

字段表：

| 偏移 | 字段 | 类型 | 作用 | 不通过时的处理 |
|---|---|---|---|---|
| 0 | `magic` | u32 | 区分"擦干净的页（全 `0xFF`）"和"真镜像" | 视为空 slot |
| 4 | `header_version` | u16 | header 结构自身的版本，将来加字段用 | != 1 则拒绝 |
| 6 | `image_version` | u16 | `major.minor.patch = (v>>12).((v>>8)&0xF).(v&0xFF)`；低 8 位与 Dynamixel 寄存器 6 的 `FW_VERSION_BYTE` 语义一致（`board.h` 定义 `FW_VERSION_BYTE = (MAJOR<<4)|MINOR`），便于主机显示 | 仅显示用，不拒绝 |
| 8 | `image_size` | u32 | CRC 覆盖范围、跳转合法性 | 必须 `>= APP_HDR_OFF+APP_HDR_SIZE` 且 `<= SLOT_SIZE` |
| 12 | `crc_low` | u32 | 向量表 + header 之前的所有字节 | 不等则拒绝 |
| 16 | `crc_high` | u32 | header 之后到 `image_size` 的所有字节 | 不等则拒绝 |
| 20 | `build_unix` | u32 | 可追溯性（`SOURCE_DATE_EPOCH`） | 仅显示 |
| 24 | `entry` | u32 | 与 `*(uint32_t*)(slot_base+4)` 比对 | 不一致则拒绝 |
| 28 | `board_id` | u16 | 防止把别的板子的镜像刷进来 | != 本板则拒绝 |
| 30 | `proto_version` | u16 | 主机/bootloader 协议兼容性 | 不匹配则拒绝 |
| 32 | `flags` | u16 | trial / confirmed / no-boot | 见 §4 |
| 34 | `reserved[30]` | — | 0（34 + 30 = 64） | — |

### 3.2 CRC 为什么用两段 CRC32，而不是一段或 CRC16

* **两段**：header 自身不能参与"覆盖 header 的 CRC"（自指）。把镜像切成
  `[base, base+0x200)` 与 `[base+0x240, base+image_size)`，两个 CRC 都由 host
  打包工具算好写进 header，bootloader 只读不算写，逻辑最简单。
* **CRC32 而不是 CRC16**：配置页用 CRC16 是因为 payload 只有 ~40 字节、且要和
  Dynamixel 的 CRC 实现共用 `crc16_dxl()`；镜像长度是 96 KB 量级，CRC16 的
  漏检率（~1/65536 随机错误、对突发错误只能保证 ≤16 bit）不足以支撑"刷错了就变砖"
  的场景。CRC32 的 1 KB 查表放不进 bootloader 的预算，因此**bootloader 里用按位
  CRC32（poly `0x04C11DB7`，init `0xFFFFFFFF`，refin/refout，xorout `0xFFFFFFFF`）**，
  约 8 条指令/字节；96 KB 在 120 MHz、flash 3 等待周期下约 **10~20 ms**，相对 2 s
  的启动窗口可以忽略。主机侧（Python）用 `zlib.crc32` 即可（同一多项式/参数）。
* **不覆盖擦除态全 `0xFF` 的尾部**：`image_size` 是真实长度，CRC 只算到
  `image_size`，这样"烧录后尾部残留旧数据"不会造成误判；同时 `image_size` 本身
  参与 CRC（在 header 里，但被两个 CRC 分别排除……因此**打包工具必须保证
  `image_size` 与两个 CRC 同时被 host 校验**，bootloader 侧则要求
  `image_size` 落在合法区间）。这个细节在 §7 的测试里必须有对应用例。

### 3.3 镜像起点仍是 slot base

应用镜像（含向量表）从 slot base 开始，**VTOR 语义完全不变**。bootloader 只做：

```c
SCB->VTOR = slot_base;
__set_MSP(*(volatile uint32_t *)slot_base);
((void (*)(void))(*(volatile uint32_t *)(slot_base + 4)))();
```

代价是 slot 里有 176 字节（`0x150`~`0x200`）padding，以及 64 字节 header。相对
96 KB / 22 KB 的现有镜像体积，可以忽略。

**另一种做法（不推荐）**：把 header 做成一个"变长"的头，紧跟在向量表之后
（`. = 0x150` 或 `. = ALIGN(0x100)`），用链接脚本导出的 `__Vectors_Size` 决定
header 起点。它能省掉 100~400 字节，但 header 位置变成"随编译产物变化"，bootloader
必须先把向量表字段读出来才能算出 header 在哪（鸡生蛋问题：header 的 magic 在
header 里），而且 A/B 两个镜像的 header 偏移可能不同。**固定 `0x200` 换来的是
"bootloader 只需要知道一个常量"，值这个价。**

---

## 4. 启动决策算法

### 4.1 数据来源

| 数据 | 位置 | 现状 |
|---|---|---|
| 应用配置 `cfg_blob_t` | `0x0803F800`，magic `0x314D4941`，`crc16_dxl` | **已实现**（`dev.c`：`dev_cfg_from_flash()` / `dev_cfg_save_now()`） |
| boot 状态（slot 选择、ping-pong 计数、trial 标志、尝试次数） | **设计**：写进同一个 `cfg_blob_t`（`version` 从 1 → 2），或独立成页 | 未实现 |
| 两个 slot 的 `app_header_t` | `SLOT_A_BASE+0x200` / `SLOT_B_BASE+0x200` | 未实现 |
| RAM magic（软件请求进 boot） | SRAM 固定地址，见 §4.5 | 未实现 |

### 4.2 流程图

```
复位
 │
 ├─ 1. 时钟/GPIO/USART1 初始化（1 Mbps，与 board.h BUS_BAUD_DEFAULT 一致）
 │     不启动 IMU、不初始化寄存器镜像、不开镜像端口
 │
 ├─ 2. 读 RAM magic (BOOT_RAM_MAGIC_ADDR)
 │     └─ == BOOT_RAM_MAGIC  -> 进 BOOT 状态（清 magic，先清后跳，避免死循环）
 │
 ├─ 3. 读配置页 cfg_blob_t
 │     ├─ CRC16 通过 -> 取 boot 状态
 │     └─ 失败/version 不识别 -> boot 状态取默认：slot=A, pingpong=0, trial=none,
 │                               boot_attempts=0
 │
 ├─ 4. 读 header A、header B（绝对地址读，不需要 VTOR）
 │     对每个 slot 做：magic / header_version / board_id / proto_version /
 │     image_size 区间 / entry==vector[1] / crc_low / crc_high
 │     -> 得到 valid_a, valid_b
 │
 ├─ 5. 决策（见 4.3）
 │
 ├─ 6. 若决定启动某 slot：
 │     ├─ 若该 slot 带 TRIAL 标志：先把 boot_attempts+1 与本次 slot 写回配置页
 │     │  （写失败不致命：先启动，应用侧的 watchdog 会兜底）
 │     └─ 设 MSP / 设 VTOR / 跳到 vector[1]
 │
 └─ 7. 若不启动任何 slot：
       进 BOOT 状态：BUS 上只应答本节点 ID，PING 返回"boot 模式"特征值（见 §6）
```

### 4.3 选择规则（ping-pong + trial）

状态字段（**设计**，并入 `cfg_blob_t`，`CFG_VERSION` 升到 2）：

| 字段 | 类型 | 含义 |
|---|---|---|
| `boot_slot` | u8 | 最近一次**已确认**启动的 slot：`0`=A，`1`=B，`0xFF`=无 |
| `boot_pingpong` | u8 | 每次成功 commit 一个新镜像 +1。用于打破"两个都合法时先启动谁"的平局 |
| `boot_trial_slot` | u8 | 当前处于 trial 的 slot；`0xFF`=无 |
| `boot_attempts` | u8 | 当前 trial 镜像已经尝试启动的次数 |
| `boot_magic` | u16 | boot 状态自身的校验（可与整 blob 的 CRC16 合并） |

`boot_pingpong` 的用法：**"另一个" slot 的定义不靠固定 A→B 方向，而靠
`boot_slot`**——候选 = 不是 `boot_slot` 的那个合法 slot。这样即使 A、B 都合法且
版本号写错，也不会在两次复位间来回横跳。`boot_pingpong` 提供单调递增的
"新鲜度"，host 每次 commit 时自增，供日志与主机判断谁更新。

决策表：

| `valid_a` | `valid_b` | `boot_trial_slot` | 决定 |
|---|---|---|---|
| 0 | 0 | — | 进 BOOT 模式（§6，两个都坏） |
| 1 | 0 | — | 启动 A |
| 0 | 1 | — | 启动 B |
| 1 | 1 | 无 trial | 启动 `boot_slot`；`boot_slot==0xFF` 时启动**版本号较大**者，版本相同则启动 ping-pong 较大的候选（即 `boot_pingpong & 1` 对应的 slot） |
| 1 | 1 | 有 trial 且 `boot_attempts < 3` | 启动 trial slot，`boot_attempts+1` |
| 1 | 1 | 有 trial 且 `boot_attempts >= 3` | **放弃 trial**：清 trial 标志、清 `boot_trial_slot`、`boot_attempts=0`，启动"上一个" slot（`boot_slot`，若 `boot_slot==0xFF` 则另一个合法 slot） |

### 4.4 回滚规则（trial 确认）

* host 或 bootloader 在 commit 时把新镜像的 header `flags` 写为 `TRIAL`，并把
  `boot_trial_slot` 指向它、`boot_attempts = 0`。
* 新应用启动后，必须在一个时限内"自我确认"。**建议 N = 30 秒，M = 3 次启动尝试**：

| 条件 | 动作 |
|---|---|
| 应用连续正常运行 ≥ 30 s（由应用内的 `boot_confirm_tick()` 判断） | 擦写配置页：header flags 清 `TRIAL` 置 `CONFIRMED`，`boot_slot = 自己`，`boot_trial_slot = 0xFF`，`boot_attempts = 0` |
| 30 s 内复位（看门狗/掉电/主动重启） | 计数已 +1，下次 bootloader 再给一次机会；到第 3 次仍不确认 → 回滚到旧 slot |
| host 主动要求回滚（新 vendor 命令） | 清 trial，重启进旧 slot |
| trial 镜像连 CRC 都不对 | 不进入 trial 流程，直接按"另一个" slot 启动 |

实现约束：应用要**尽早**在启动后开始计时，但确认写 flash 要**推迟到总线正常、
有主机通信之后**（避免"一上电就自我确认"导致坏镜像被镀金）。建议：应用启动后
每 1 s 累加，若中途收到过任意合法总线帧或总线静默超过 5 s 都算"活着"，到 30 s 写确认。

### 4.5 强制进入 boot 状态的三种手段

| 手段 | 机制 | 覆盖场景 |
|---|---|---|
| 配置页 `boot_stay` 标志 | 应用收到 `VCMD_BOOT`（新增）后：置 `boot_stay=1`、写 `BOOT_RAM_MAGIC`，然后 `NVIC_SystemReset()`；bootloader 见 `boot_stay` 就停在 boot 状态等主机 | **常规路径**：主机要升级时用 |
| RAM magic | bootloader 启动时先看 `BOOT_RAM_MAGIC_ADDR`。应用置位后软复位；bootloader 读到即进 boot 状态，并在进入前清零 | 不依赖 flash 写入、不需要配置页可写；但**掉电后 RAM 内容不确定**（见下行） |
| 启动窗口 + 总线探针 | 无确认镜像时 bootloader 永久停在 boot 状态 | 兜底 |

**诚实说明**：GD32F303 的 SRAM 没有 VBAT 备份域，RAM magic 只能跨"软复位/看门狗
复位"存活，**不能跨掉电**。掉电后要进 boot 状态，只能依靠配置页标志或"没有合法
镜像"。因此 RAM magic 是快捷方式，不是恢复手段；`boot_stay` 标志（掉电保持）才是。

---

## 5. 总线升级协议

### 5.1 载体：复用现有节点 ID 与帧格式，不新造物理层

| 项目 | 现状（**已实现**） | 升级（**设计**） |
|---|---|---|
| 物理层 | USART1 PA2/PA3，半双工，硬件 TXD_EN 自动方向，1 Mbps | 不变 |
| Dynamixel 2.0 从机 | `src/dxl2.c`，ID = `dev_cfg()->dxl_id`（默认 200），CRC16 覆盖整帧 | **bootloader 里编译同一份 `dxl2.c`**，只把 `dev_read`/`dev_write` 换成 boot 版实现 |
| FeeTech SCS/HLS 从机 | `src/fee.c`，ID = `dev_cfg()->fee_id`（默认 200），`~SUM` 校验，地址字段是 **1 字节**（0..255） | 同上，编译同一份 `fee.c` |
| 协议自动识别 | `src/bus.c`：`FF FF FD` → DXL；`FF FF <id≤0xFC>` → Fee，并按 `BUS_PROTO_STICKY_MS` 粘滞 | 不变 |
| 应用内 vendor 窗口 20 字节 | DXL `148`（`board.h DXL_VENDOR_ADDR`）、Fee `160`（`FEE_VENDOR_ADDR`）；偏移常量 `V_MAGIC_L`…`V_RESERVED`；命令 `VCMD_SAVE/DEFAULTS/REBOOT/RESET_SIM` | **保留不动**；升级寄存器另开一段 |
| 每协议 256 字节寄存器镜像 | `REG_SPACE_SIZE = 256`；最高已用地址是 `DXL_VENDOR_ADDR+19 = 167`（`0xA7`），`168`（`0xA8`）以上目前**完全未使用**（`dev.c` 的 `ro_dxl`/`ro_fee` 只覆盖到 `166`/`178`） | **升级窗口 = 十进制 `208..255`（`0xD0..0xFF`，48 字节）** |

关键结论：**升级帧就是普通的 `INST_WRITE` / `INST_READ` / `PING`，没有新的帧类型
或校验层**。这带来两个直接好处：

1. 现有的 `dxl2.c`（含 CRC16 校验、地址/长度检查、`DXL_ERR_*` 错误字节）与 `fee.c`
   （含 `~SUM` 校验）原样复用，host 侧 `v1/host/bus.py` 的 `read_registers()` /
   `write_registers()` / `ping()` 也原样复用，只需加一个循环。
2. 总线上其他舵机看到的仍然只是"发给 ID 200 的读写"，**不需要给舵机加任何特殊处理**。

### 5.2 新增寄存器映射（`board.h` 新增，boot 专用）

同一个地址在两种协议下含义相同（两边的寄存器镜像都是 256 字节，`REG_SPACE_SIZE`）。
**地址 208..255 目前完全未被使用**：现有实现里 DXL 最高用到 `167`（vendor 窗口
`148..167`），Fee 最高用到 `179`（vendor 窗口 `160..179`），`180..207` 与
`208..255` 都没有任何内容。因此升级窗口落在 `208` 起是安全的。

| 名称 | 地址（两种协议相同） | 字节 | R/W | 说明 | 新旧 |
|---|---|---|---|---|---|
| `UPG_MAGIC_L/H` | 208, 209 | 2 | r/w | 固定 `0x4247`（`'G','B'`）；写入即表示"开始/继续一次升级会话" | **新** |
| `UPG_CMD` | 210 | 1 | w | `UPG_CMD_*`（§5.3）；写后由节点立即清零，与现有 `V_CMD` 同风格 | **新** |
| `UPG_STATUS` | 211 | 1 | r | `0`=idle，`1`=erased，`2`=receiving，`3`=block ok，`4`=verify ok，`5`=committed；`0x80` 位表示错误 | **新** |
| `UPG_TARGET` | 212 | 1 | r/w | `0`=A，`1`=B。只允许"非当前运行"的 slot | **新** |
| `UPG_SEQ_L/H` | 213, 214 | 2 | r/w | 块序号，从 0 单调 +1；节点接受 `seq == ack+1` 或重发 `seq == ack` | **新** |
| `UPG_OFF_0..3` | 215..218 | 4 | r/w | 本块在 slot 内的字节偏移（32 位 LE，4 字节对齐） | **新** |
| `UPG_BLKLEN` | 219 | 1 | w | 本帧 `UPG_DATA` 的有效字节数，4..16 且为 4 的倍数（FMC 按 32 位字编程） | **新** |
| `UPG_BLKCRC_L/H` | 220, 221 | 2 | w | 本块 CRC16（`crc16_dxl()`，覆盖 `UPG_DATA[0..BLKLEN-1]`） | **新** |
| `UPG_ACK_SEQ_L/H` | 222, 223 | 2 | r | 最近一次成功写入的块序号 | **已实现** |
| `UPG_ERRCODE` | 224 | 1 | r | `1`=块 CRC，`2`=越界，`3`=flash 编程失败，`4`=状态机，`5`=目标槽非法，`6`=缺 magic，`7`=镜像校验失败，`8`=长度 | **已实现** |
| `UPG_VER_L/H` | 225, 226 | 2 | r/w | 待写镜像的 `image_version`（节点与镜像头交叉核对） | **已实现** |
| `UPG_NODE_CRC_0..3` | 227..230 | 4 | r | 节点在 `UPG_END` 重算出的整镜像 CRC32 | **已实现** |
| `UPG_RESERVED` | 231 | 1 | r | 0 | **已实现** |
| `UPG_IMGLEN_0..3` | 232..235 | 4 | r/w | 整镜像长度 `image_size` | **已实现** |
| `UPG_IMGCRC_0..3` | 236..239 | 4 | r/w | host 计算的整镜像 CRC32（两段拼接后的值，见 §5.5） | **已实现** |
| `UPG_DATA` | **240..255** | **16** | r/w | 数据窗口：本帧的块内容；`UPG_READBACK` 也回填到这里 | **已实现** |

> 与本文档早期的草案相比，`UPG_ERRCODE` 从 226 移到 224，去掉了 `UPG_ACK_CRC_L/H`
> （只有 16 bit，装不下节点复算的 CRC32），改为 227..230 的 `UPG_NODE_CRC_0..3`。
> 另外 180..191 被**身份窗**占用（见下），两级窗口一起构成主机可读的全部升级状态。

### 5.2.1 身份窗（180..195，只读，两种协议相同）

主机在决定升级之前先要知道“对面是谁”：模式、槽、版本、镜像 CRC、以及配置页里的启动状态。
这 16 字节在应用和 bootloader 里都可读、写则被忽略（应用态对 180..255 的写返回
`DXL_ERR_ACCESS`，提示主机“先进入 boot 模式”）：

| 偏移 | 含义 |
|---|---|
| 0..1 | 魔数 `0x5055`（`'U','P'`） |
| 2 | 模式：0=应用，1=bootloader |
| 3 | 槽：0=A，1=B，255=无 |
| 4 | 标志：bit0=有效 trial，bit1=已确认，bit7=镜像头无效/未打包 |
| 5..6 | 运行镜像的 `image_version`（boot mode 下是“将要运行”的槽） |
| 7..10 | 该镜像的两段拼接 CRC32 |
| 11 | 扩展版本（1 = 12..15 有效；老固件为 0，主机显示 unknown 而不是瞎猜） |
| 12 | 配置页 `boot_slot` |
| 13 | 配置页 `trial_slot` |
| 14 | 已尝试的 trial 启动次数 |
| 15 | `boot_stay` |

bit0 是**有效** trial 而不是镜像头里的位：镜像头打包后不可改（flash 只能 1→0），
“已确认”记录在配置页，由固件折算成这一位。

数据窗口定为 **16 字节**而不是 32：FeeTech 的 `LEN` 是 1 字节、数据窗口 `UPG_DATA`
必须是最后一段，16 字节刚好把 `208..255` 用满；且"16 B = 4 次
`fmc_word_program`"让块 CRC 与重传代价都小。若要 32 字节窗口，就必须压缩
ACK/版本/长度/CRC 字段，收益（升级时间从 ~5 s 降到 ~3.5 s）不值得复杂度。

新常量建议直接写进 `board.h`，与现有 `V_*` 并列：

```c
#define UPGRADE_WIN_ADDR     208U   /* 0xD0，两种协议相同，都在 REG_SPACE_SIZE=256 内 */
#define UPGRADE_DATA_ADDR    240U   /* 0xF0 */
#define UPGRADE_DATA_LEN     16U
/* offsets 相对 UPGRADE_WIN_ADDR */
#define U_MAGIC_L 0
#define U_MAGIC_H 1
#define U_CMD     2
#define U_STATUS  3
#define U_TARGET  4
#define U_SEQ_L   5
#define U_SEQ_H   6
#define U_OFF_0   7
#define U_OFF_1   8
#define U_OFF_2   9
#define U_OFF_3   10
#define U_BLKLEN  11
#define U_BLKCRC_L 12
#define U_BLKCRC_H 13
#define U_ACK_SEQ_L 14
#define U_ACK_SEQ_H 15
#define U_ACK_CRC_L 16
#define U_ACK_CRC_H 17
#define U_ERRCODE 18
#define U_VER_L   19
#define U_VER_H   20
#define U_IMGLEN_0 24
#define U_IMGLEN_1 25
#define U_IMGLEN_2 26
#define U_IMGLEN_3 27
#define U_IMGCRC_0 28
#define U_IMGCRC_1 29
#define U_IMGCRC_2 30
#define U_IMGCRC_3 31
/* 208 + 31 = 239；240..255 = UPG_DATA */
```

注意 `ro_dxl()` / `ro_fee()`（`dev.c`）现在把 `166..167`、`178..179` 之外的高地址
视为可写；新增窗口 `208..255` 在 bootloader 里必须显式标记为"只允许升级状态机
写入"，**不能让通用的 `dev_write` 直接落进镜像**（bootloader 里根本没有 256 字节
镜像，`dev_write` 是另一份实现，见 §7）。

### 5.3 命令序列

| `UPG_CMD` | 名称 | 参数（先写好的寄存器） | 节点动作 | 应答 |
|---|---|---|---|---|
| `0x01` | `UPG_BEGIN` | `UPG_TARGET`, `UPG_IMGLEN_*`, `UPG_IMGCRC_*`, `UPG_VER_*`, `UPG_MAGIC` | 校验 target 是"非当前运行"的合法 slot、`imglen` 在 `[0x240, 96K]` 内；**擦除目标 slot 的 `ceil(imglen/2K)` 页**（最多 48 页，约 1.2 s，期间不响应总线——见 §6 的静默处理）；`UPG_STATUS=1`，`UPG_SEQ=0` | ACK |
| `0x02` | `UPG_DATA` | `UPG_SEQ_*`, `UPG_OFF_*`, `UPG_BLKLEN`, `UPG_BLKCRC_*`，随后写 `UPG_DATA[0..BLKLEN-1]` | 状态机检查：会话已 begin、`target` 合法、`off+blklen <= imglen`、偏移 4 字节对齐、`seq == ack_seq+1`（或重发 `seq == ack_seq`）。复算块 CRC16；通过则 `fmc_word_program` 写入目标 slot；更新 `UPG_ACK_SEQ/UPG_ACK_CRC`，`UPG_STATUS=3` | **每块 ACK**（DXL：带 4 字节参数 `ack_seq, status, ack_crc_l, ack_crc_h` 的 status packet；Fee：带 2 字节参数的 ACK） |
| `0x03` | `UPG_END` | 无 | 复算整镜像 CRC32（两段式，见 §5.5），与 `UPG_IMGCRC_*` 比对；通过则**写 `app_header_t`**（`magic/len/ver/crc/entry/flags=TRIAL`）到 `slot_base+0x200`；更新配置页 boot 状态；`UPG_STATUS=5` | ACK（含最终 CRC32 复算值） |
| `0x04` | `UPG_ABORT` | 无 | 放弃会话（不擦已完成的内容；`UPG_STATUS=0`） | ACK |
| `0x05` | `UPG_CONFIRM` | 无 | 把**当前 slot** 的 trial 标志清掉、置 confirmed（应用态命令，boot 态返回错误） | ACK |
| `0x06` | `UPG_REBOOT` | 无 | 应答（含状态包）发完之后软复位 | ACK 后复位 |
| `0x07` | `UPG_READBACK` | `UPG_OFF_*`, `UPG_BLKLEN` | 从目标 slot 读回 `UPG_BLKLEN` 字节到 `UPG_DATA` 供 `INST_READ` 读回 | 通过后续 `READ` 取 |

**`UPG_CMD` 写入语义**：与现有 `V_CMD` 一致——`dev_write` 落进窗口后，升级状态机
消费命令并立刻清零 `UPG_CMD`（`src/dev.c` 的 `vendor_apply()` 就是这么做的），
这样 `READ` 窗口永远读到 0。

### 5.4 重试 / ACK 语义

* **Dynamixel**：`INST_WRITE` 到 `UPG_CMD` 会返回 status packet，`ERR` 字节沿用现有
  `DXL_ERR_*`（`dxl2.c` 已实现 CRC/长度/范围错误）。块写用 `INST_WRITE` 到
  `UPG_DATA`；节点在"块非法"时返回 `DXL_ERR_RANGE` 或 `DXL_ERR_ACCESS`，合法时
  返回 `ERR=0`。**host 必须检查 `UPG_STATUS` 与 `UPG_ACK_SEQ`，不能只看帧 CRC**。
* **FeeTech**：SCS 协议**没有 checksum-error 应答**（`fee.c` 里 `bad_sum` 直接
  `return 0`），所以"帧丢了"只能靠 host 超时 + 重发判断。`build_ack()` 的
  `FEE_ST_*` 状态字节用于表达块级错误。**这一点必须显式写进 host 脚本逻辑**：
  对 Fee 每块重试至少 3 次，并且每次重发都用 `UPG_ACK_SEQ` 读回确认。
* 幂等：`UPG_DATA` 允许 `seq == ack_seq` 的重发（节点复算 CRC 后重新写同一偏移，
  内容相同，无害）。**不允许** `seq < ack_seq` 或跳跃。
* 超时：节点在 `UPG_DATA` 会话中若 500 ms 收不到任何完整帧，回到 `UPG_STATUS=2`
  但保留 `ack_seq`（主机可从断点续传）；若 30 s 无会话，bootloader 按 §4 决策继续
  启动（若目标 slot 还没 commit，则它仍然是旧的/无效的，不会影响启动）。
* 升级期间**目标 slot 必须是"非当前运行"的 slot**。bootloader 会拒绝 `target ==`
  当前正在跑的 slot，避免"擦除自己"。

### 5.5 整镜像 CRC 的两段式（与 §3.2 一致）

host 打包工具在把 `.bin` 交给升级脚本前，先做：

1. 读 `image_size`（`.bin` 长度）；若 `< 0x240` 直接失败。
2. 在原 `.bin` 的 `0x200` 处插入 64 字节 header（其余字节原样后移），得到最终
   `slot image`。
3. `crc_low = crc32(slot_image[0x000 .. 0x1FF])`；
   `crc_high = crc32(slot_image[0x240 .. image_size-1])`。
   把 `crc_low`/`crc_high` 写进 header（`crc_low` 只覆盖向量表 + padding，
   **不覆盖 header 自身**；`crc_high` 覆盖 `.text` 起的所有内容）。
4. `UPG_IMGLEN` = `image_size`；`UPG_IMGCRC` 用**一个** 32 位值：推荐
   `crc32(slot_image[0x000..0x1FF] || slot_image[0x240..image_size-1])`，
   即"两段按顺序拼接后的 CRC"。节点在 `UPG_END` 用同一顺序重算。**不要**用
   `crc_low ^ crc_high` 之类的组合，那没有纠错意义。
5. `UPG_IMGCRC` 是 host 与节点的独立校验；`crc_low`/`crc_high` 是 bootloader
   每次启动都要算的校验。两套都要有。

### 5.6 带宽与时间估算（16 B/帧）

Dynamixel `INST_WRITE` 写 `UPG_DATA` 一帧：

```
FF FF FD 00 (4) | ID (1) | LEN_L LEN_H (2) | INST (1) | ADDR_L ADDR_H (2) | DATA (16) | CRC (2)
= 28 字节 = 280 bit
回程 status packet (ID/LEN/0x55/ERR/CRC) ≈ 11 字节 = 110 bit
合计 390 bit ÷ 1 Mbps = 390 µs（串行时间），加方向切换与主循环处理，取 0.6 ms/块
```

| 项 | 估算 |
|---|---|
| 每块（请求 + 应答 + 余量） | ~0.6 ms |
| 96 KB ÷ 16 B = 6144 块 | **~3.3 s** |
| `UPG_BEGIN` 擦除（48 页 × ~25 ms，未核对数据手册上限） | ~1.2 s |
| `UPG_END` 两段 CRC32（96 KB） | ~10~20 ms |
| 重传预算 5% | +0.2 s |
| **合计（乐观）** | **~4.5~5.5 s** |
| FeeTech | 帧为 `FF FF ID LEN INST ADDR` + 16 B DATA + `~SUM` = 23 字节，应答 ~7 字节，量级相同 |

以上是估算而非实测。真实数字必须由 §7.5 第 8 项的台面计时给出，并据此设置
`host/upgrade.py` 的超时。若以后要缩短，可临时把总线降到 500 kbps
（`dev_dxl_baud()` / `dev_fee_baud()` 都支持）以换取更宽松的时序余量。

### 5.7 不打扰总线上其他舵机

这是设计里最容易被低估的一条，规则如下：

1. **只寻址本节点 ID**（`dev_cfg()->dxl_id` / `fee_id`，默认 200）。**绝不广播**：
   现有 `dxl2.c` 对 `DXL_BROADCAST_ID (0xFE)` 的写是"执行但不回包"，`fee.c` 同理；
   升级状态机必须拒绝来自广播的 `UPG_*` 写（否则总线上所有舵机都会收到扫描帧，
   即使它们不解析也没必要冒风险）。
2. **保持请求-应答节奏**：升级脚本发一帧、等一帧，超时才重发。不要流水线式连发，
   否则节点（单线程主循环 + 无 DMA 的 USART 接收）会丢字节，而舵机侧的报错/回包
   会和升级回包在总线上交错。
3. **回包 ID 过滤**：脚本必须校验回包 ID 是本节点（`host/bus.py` 的
   `_transaction()` 已经检查 `pid != self.imu_id` 并抛 `ProtocolError`）。总线上
   其他舵机的状态包（半双工总线上所有设备都能听到）必须被丢弃而不是当成应答。
4. **擦除窗口要短且有界**：`UPG_BEGIN` 的擦除是 bootloader 唯一长时间关中断的
   操作（48 页 × 25 ms ≈ 1.2 s，见 §6.3）。这期间 bus 上不会有任何回包，主机的
   其他舵机轮询会超时——**必须由主机侧在升级前暂停对舵机的轮询**。这是协议无法
   替代的操作纪律。
5. **失败即静默**：`UPG_ABORT`/超时后节点回到普通 boot 状态，只应答 PING 和
   `UPG_*` 读；不会主动发包，总线上不会出现"疯狂重发"。
6. **协议锁**：升级前可把 `V_PROTO_LOCK` 固定到本次使用的协议（现有功能），
   避免自动识别在升级帧上误判。

---

## 6. 失效模式与处理

### 6.1 掉电 / 断电

| 场景 | 后果 | 处理 |
|---|---|---|
| 擦除中断电 | 目标 slot 部分/全部为 `0xFF`，header `magic` 不是 `APP_HDR_MAGIC` → 无效 | bootloader 用另一个 slot 启动；无合法 slot 则进 boot 模式等主机重刷 |
| 数据传输中断电 | 目标 slot 有旧内容残留 + header 还没写（`0x200` 处是 `0xFFFFFFFF`）→ 无效 | 同上 |
| `UPG_END` 写 header 中断电 | header 可能只写了一部分。**处理：先写 header 的"数据部分"，最后写 `magic`**（`magic` 是 4 字节，`fmc_word_program` 一次原子完成）→ `magic` 存在即 header 完整 | CRC 校验兜底 |
| 配置页写入中断电 | `cfg_blob_t` CRC16 失败 | `dev_cfg_from_flash()` 已实现"失败即用默认值"；bootloader 侧同样按默认 boot 状态走（等价于"没有 trial"） |
| 升级中断电后又上电 | 目标 slot header 的 `TRIAL` 标志可能根本没写进去 | 按 §4.3 用另一个 slot 启动，无需人工干预 |

### 6.2 CRC 失败

| 失败点 | 检测方 | 动作 |
|---|---|---|
| 帧 CRC16（Dynamixel 整帧） | `dxl2.c`（**已实现**） | 回 `DXL_ERR_CRC` status；host 重发该帧 |
| 帧 `~SUM`（FeeTech） | `fee.c`（**已实现**） | **静默丢弃**（协议如此）；host 靠超时重发 |
| 块 CRC16（`UPG_BLKCRC`） | bootloader 升级状态机（**设计**） | `UPG_STATUS=0x80+1`，`UPG_ERRCODE=1`，不写 flash；host 重发同 `seq` |
| 整镜像 CRC32（`UPG_END`） | bootloader（**设计**） | `UPG_STATUS=0x80+1`，**不写 header**，会话保持；host 可 `UPG_READBACK` 定位坏块 |
| 启动时 `crc_low/crc_high` | bootloader（**设计**） | 该 slot 判为无效，走 §4.3 决策 |
| 配置页 CRC16 | `dev_cfg_from_flash()`（**已实现**） | 用默认配置；bootloader 用默认 boot 状态 |

**不会出现"CRC 错了还启动"**：header 的 `magic` 只有在两段 CRC、长度、`entry`、
`board_id`、`proto_version` 全部通过之后才允许跳转。

### 6.3 掉电之外：镜像能启动但永不确认

* 30 s / 3 次尝试（§4.4）后自动回滚，**不需要主机在线**。
* 回滚本身要写配置页（清 trial），若此时又掉电：`boot_attempts` 可能没递增到 3，
  但每次上电都会重试并再递增；因为"确认"永远不发生，最终必然到 3 次并回滚。
  这个收敛性依赖 `boot_attempts` 持久化成功——即使配置页写失败，**上电后
  读到的 `boot_attempts` 会停在旧值，导致无限重试**。因此设计上必须：
  **把 `boot_attempts` 的写放在"跳转之前"，且写失败时仍按 `boot_attempts+1`
  在 RAM 中继续判断**，并在第 3 次时**即使写失败也强制回滚**（RAM 内的判定
  已经足够，写 flash 只是为了跨掉电记忆）。

### 6.4 升级过程中总线静默 / host 消失

* 节点侧：500 ms 无完整帧 → `UPG_STATUS=2`（保留 `ack_seq`）；30 s 无会话 →
  结束会话，按 §4 决策启动。
* host 侧：每块重试 3 次 × 50 ms 超时；连续 5 块失败则 `UPG_ABORT` 并报告。
* 若 host 在 `UPG_BEGIN` 后立即消失：节点已经擦了目标 slot，但 header 仍未写，
  另一个 slot 不受影响，**可恢复**。

### 6.5 两个 slot 都无效

bootloader 必须**仍然能被主机识别**。要求：

| 项 | 值 | 说明 |
|---|---|---|
| `PING`（Dynamixel） | 返回 model number `0xB007`（**设计**，新增 `BOOT_MODEL_NUMBER`），FW version 字节 `0x00` | 与应用的 `DXL_MODEL_NUMBER=1200`（`board.h`）**不同**，主机一看就知道"在 boot 模式" |
| `PING`（FeeTech） | FeeTech 的 `PING` 应答**不带数据**，所以模式靠 `READ` 寄存器 3..4 判断：bootloader 返回 `0x00B7`（`BOOT_FEE_MODEL_NUMBER`），应用返回 `0x0C00`（`FEE_MODEL_NUMBER`） | **已实现**（与早期草案不同，草案里的“PING 回 model”在 SCS 协议下不存在） |
| 其余只读寄存器 | 只实现最小集：`UPG_*` 窗口 + 版本；**不模拟完整 256 字节控制表** | 避免 bootloader 里塞进第二个 `dev.c` |
| 应答 | 只应答本节点 ID | 同 §5.7 |

host 侧判定"节点处于 boot 模式"的规则：`ping()` 返回 true 且 model number 是 boot
特征值。这把"没刷过的新板"和"两个 slot 都坏了的板"统一成同一种可操作状态。

### 6.6 bootloader 永远可再次进入

| 入口 | 依赖 | 掉电后有效？ |
|---|---|---|
| `boot_stay` 标志（配置页） | 配置页可读 | 是 |
| 无合法 slot（§4.3） | header 全部无效 | 是 |
| `BOOT_RAM_MAGIC`（软复位前由应用写） | RAM 内容保持 | **否**（见 §4.5） |
| 启动窗口内的 `UPG_MAGIC` 写 | 主机在线 | 是（主机主动） |

**只要 bootloader 区本身没有被擦，它就一定能被重新进入。** 因此：

* 应用**绝不能**写 `0x08000000`..`0x08007FFF`。现有 `dev_cfg_save_now()` 只写
  `CFG_FLASH_ADDR`，安全；升级状态机只写"非当前运行的 slot"，安全。
* 建议在 bootloader 里对写入地址做一次硬检查：
  `addr >= SLOT_A_BASE && addr + len <= CFG_FLASH_ADDR`，否则 `UPG_ERRCODE=2`。
* 进一步（可选）：把 bootloader 区在 flash 里放两份（`0x08000000` 与
  `0x08002000`，各 8 KB），bootstrap 段跳第一份、校验失败跳第二份。**当前
  32 KB boot 区留了这个可能，但本文档不要求**——bootloader 由 SWD 更新，
  工程上比"在途自升级"安全得多（见 §8）。

---

## 7. 下一里程碑的实现清单

### 7.1 文件与目标（**已全部实现**，实际文件名见右列括号）

| 文件 / 目标 | 动作 | 内容 |
|---|---|---|
| `v1/src/app_header.h` / `.c` | 新增 | §3 的 `app_header_t`、`APP_HDR_*`、`APP_BOARD_ID`、`app_header_fill/check()`、两段 CRC 助手 |
| `v1/src/boot_flash.c/.h` | 新增（含 `IMU_TO_DXL_HOST_TEST` RAM 后备） | `boot_flash_read/erase_range/program_words`，地址合法性硬检查（**不允许写 boot 区**）；带 `#ifdef IMU_TO_DXL_HOST_TEST` 的 RAM 后备实现，使主机测试无需真实 flash |
| `v1/src/boot_crc32.c/.h` | 新增 | 按位 CRC32（poly `0x04C11DB7`）+ 分段 API；主机测试用 `test_crc32.c` 与 `zlib.crc32` 对拍 |
| `v1/src/boot_upgrade.c/.h` | 新增（块写长度取自实写字节数） | §5 的升级状态机；**不依赖 GD32 头文件**（通过 `boot_flash`/`boot_crc32` 间接），因此能被 `test_protocols.c` 编译 |
| `v1/src/boot_regs.c/.h` | 新增（另加 `boot_regs_hooks()` 便于测试串联） | boot 寄存器窗口的 `boot_dev_read/boot_dev_write`（签名与 `dev_read/dev_write` 一致，bootloader 里直接替换掉 `dev.c`） |
| `v1/src/boot_main.c` | 新增（决策在 `boot_decide.c`，纯函数、可主机测试） | §4 的启动决策与跳转；自己的 SysTick/ms 计时（不能复用应用 `systick.c` 的全局状态，见下） |
| `v1/src/cfg_blob.h/.c` + `v1/src/dev_cfg.c` | 新增 | 读/写 `cfg_blob_t` **v2** 的 boot 状态字段（与 `dev.c` 的同名常量保持字面一致，或用共享头 `v1/src/cfg_blob.h` 把 `cfg_blob_t` 抽出来给两者共用——**推荐后者**） |
| `v1/linker/gd32f303cc_boot.ld` | 新增（+ 三个脚本都加 `.boot_ram` NOINIT 段） | `FLASH ORIGIN = 0x08000000, LENGTH = 32K`；`RAM` 同应用；**必须与应用共享 `_sp`/`Reset_Handler` 符号约定** |
| `v1/linker/gd32f303cc_slot_a.ld` / `_slot_b.ld` | 修改 | 新增 `.app_header` 段并放到 `0x200`（§3.1 已给片段）；`FLASH` 仍从 `0x08008000`/`0x08020000` 起——**注意**：链接脚本的 `ORIGIN` 里不能直接写 `.app_header` 的绝对地址，需要在 `SECTIONS` 里用 `* (.app_header)` + 偏移约束 |
| `v1/linker/gd32f303cc_app.ld` | 保持 | 开发板构型（`APP_SLOT=0`），不加 header |
| `v1/CMakeLists.txt` | 修改 | 新增 option `BOOT_ENABLE`（0/1，默认 0）、`BOOT_APP_SLOT`（bootloader 默认启动哪个 slot）、把 `boot_*.c` 加进目标；新增可执行目标 `gd32f303cc_imu_to_dxl_boot`（用 `gd32f303cc_boot.ld`）；`APP_SLOT=1|2` 时自动启用 header 后处理 |
| `v1/CMakeLists.txt` 的 POST_BUILD | 修改 | 应用构建后调用 `v1/host/tools/mkapp_header.py`（下）把 header 打进 `.hex`/`.bin` 并回填 CRC |
| `v1/host/tools/mkapp_header.py` | 新增 | 见 7.2；**实现名 `host/package.py`**（`pack` 子命令） |
| `v1/host/tools/upgrade_bus.py` | 新增 | 见 7.3；**实现名 `host/upgrade.py`** |
| `v1/host/tools/test_protocols.c` | 修改 | 新增 §7.4 的用例；**不要**改动现有 689 行里的既有用例 |
| `v1/host/tools/test_crc32.c` | 新增 | CRC32 对拍（`zlib.crc32` 生成向量，C 实现验证） |
| `v1/host/tools/run_c_tests.sh` | 修改 | 编译并运行新增测试 |
| `v1/src/board.h` | 修改 | 新增 §5.2 的 `UPGRADE_*`/`U_*`/`UPG_CMD_*`/`UPG_ST_*`/`BOOT_MODEL_NUMBER`；应用侧新增 `VCMD_BOOT`（触发软复位进 boot） |
| `v1/src/dev.c` | 修改 | `VCMD_BOOT` 的实现（置 `boot_stay`、写 `BOOT_RAM_MAGIC`、`dev_request_reboot()`）；`cfg_blob_t` 升到 `CFG_VERSION=2` 并加 boot 字段；`UPG_CONFIRM` 在应用态可直接清 trial |

### 7.2 `mkapp_header.py`（实现为 `host/package.py`）流程

```
输入：build/gd32f303cc_imu_to_dxl.bin（APP_SLOT=1|2 构建产物，从 slot base 起）
参数：--slot a|b --version 1.2.3 --board 1 --trial/--confirmed
 1. 检查 len(bin) >= 0x240、len(bin) <= SLOT_SIZE
 2. 把 bin 的 0x200..0x240 区间（由链接脚本 .app_header 段保证是 64 字节占位）替换为 header
 3. crc_low  = zlib.crc32(image[0x000:0x200])
    crc_high = zlib.crc32(image[0x240:image_size])
    imgcrc   = zlib.crc32(image[0x000:0x200] + image[0x240:image_size])   # 与节点 UPG_END 同序
 4. entry = u32le(image[4])，assert entry 落在 slot 内且 bit0 == 1（Thumb）
 5. 写回 header，输出：
      build/gd32f303cc_imu_to_dxl_slot_a.hex   （J-Link 可烧）
      build/gd32f303cc_imu_to_dxl_slot_a.img   （升级脚本输入）
      build/..._slot_a.json                    （长度/CRC/版本，供脚本与 CI 核对）
```

**顺序要求**：先算 CRC，再写 header，再把 `.hex` 交出去；`UPG_END` 里节点复算的
`imgcrc` 必须与 json 里的值一致（CI 里断言）。

### 7.3 `upgrade_bus.py`（实现为 `host/upgrade.py`）流程

```
参数：--port /dev/ttyACM0 --baud 1000000 --protocol dxl|fee --id 200
      --image build/..._slot_a.img --slot a|b [--confirm]
 1. 用 v1/host/bus.py 的 Link 打开端口（不要另写协议层）
 2. ping() 必须成功；打印读回的 model number，判定"应用态"还是"boot 态"
 3. 若在应用态：写 VCMD_BOOT（DXL: WRITE 到 DXL_VENDOR_ADDR+V_CMD；Fee: FEE_VENDOR_ADDR+V_CMD），
    等 300 ms，重新 ping，确认 model number 变成 BOOT_MODEL_NUMBER
 4. 写 UPG_TARGET / UPG_IMGLEN / UPG_IMGCRC / UPG_VER / UPG_MAGIC，然后 UPG_CMD=UPG_BEGIN
    等 ACK（擦除最坏 1.2 s，超时要 ≥2 s）
 5. 循环：按 16 B 分块，写 UPG_SEQ/UPG_OFF/UPG_BLKLEN/UPG_BLKCRC，再写 UPG_DATA
    每块读 UPG_ACK_SEQ + UPG_ACK_CRC + UPG_STATUS 核对；失败重发（3 次）
 6. 写 UPG_CMD=UPG_END，等 ACK，核对节点复算的 CRC32
 7. 可选：UPG_CMD=UPG_REBOOT；等待重新 ping 到"应用态"的 model number
 8. 打印速率、重传次数、最终版本；任何阶段失败都要 UPG_ABORT
```

### 7.4 要新增的测试（延续 `test_protocols.c` 风格）

主机侧（gcc，`IMU_TO_DXL_HOST_TEST=1`，RAM 后备 flash）：

| 用例 | 断言 |
|---|---|
| `test_app_header_layout` | `sizeof(app_header_t) == 64`、`offsetof` 与 §3.1 表一致、`APP_HDR_OFF == 0x200` |
| `test_crc32_vectors` | C 实现与 `zlib.crc32` 的 5 组向量一致（空、`"123456789"`、全 `0xFF`、随机 4 KB、分段拼接） |
| `test_header_check_ok / bad_magic / bad_crc / bad_entry / bad_len / bad_board` | 每个失败分支都返回"无效"，且**不**发生跳转（用回调计数验证） |
| `test_boot_decision_matrix` | §4.3 决策表逐行，含"两个都无效 → boot 模式"、"trial 3 次 → 回滚" |
| `test_upgrade_begin_erase_range` | 擦除页数 = `ceil(imglen/2048)`，且**从不**触及 boot 区与配置页 |
| `test_upgrade_block_seq` | 顺序块成功；乱序/重复/越界块被拒且不改 flash；块 CRC 错误被拒 |
| `test_upgrade_end_crc` | 篡改一个字节后 `UPG_END` 失败且 header 未写 |
| `test_upgrade_abort` | `UPG_ABORT` 后 `UPG_STATUS`/`UPG_ACK_SEQ` 归零，已写内容不被继续使用 |
| `test_boot_ping_model` | boot 模式下 DXL/Fee 的 PING 回包 model/version 与 §6.5 一致，且与 `DXL_MODEL_NUMBER`/`FEE_MODEL_NUMBER` 不同 |

主机侧 Python（`bus_smoke.py` 风格，`--self-test` 不接硬件）：

| 用例 | 断言 |
|---|---|
| `upgrade_codec` | 生成 16 B 块帧的字节序列与 golden 向量一致（两种协议） |
| `upgrade_full_image_dry_run` | 对 96 KB 随机镜像走"内存节点"状态机，最终节点 CRC == host CRC |

### 7.5 用 J-Link 在台面上要验证的事（先于任何现场升级）

1. **布局**：`./build.sh` 后 `arm-none-eabi-objdump -h` 确认应用 `.vectors` 仍在
   `slot A/B base`、`.app_header` 恰好在 `+0x200`。
2. **boot 镜像不越界**：boot ELF 的 `.text` 结束地址 < `0x08008000`；`--print-memory-usage`
   中 FLASH 用量 < 32 KB。
3. **跳转正确性**：J-Link 在 `boot_main.c` 的 `__set_MSP` 前后下断点，确认
   MSP == `*(uint32_t*)slot_base`、PC == `*(uint32_t*)(slot_base+4)`；
   单步到应用 `main()` 的第一条指令，确认 `SCB->VTOR == slot_base`（用 J-Link 读
   `0xE000ED08`）。
4. **只有 A 有效** / **只有 B 有效** / **两个都有效且 ping-pong 切换** 三种构型各刷一次，
   确认每次启动的 slot 与 §4.3 预测一致。
5. **CRC 拒绝**：用 J-Link 往某个 slot 中部写一个字节（`w4` 一小段），复位后必须
   启动另一个 slot；两个都改坏则进 boot 模式并且 PING 回 `0xB007`。
6. **trial 回滚**：把 A 刷成 `--trial` 且故意让它起不来（例如临时让应用在 5 s 内
   `NVIC_SystemReset()`），确认 3 次后启动 B。
7. **掉电演练**：在 `UPG_BEGIN` 擦除中途、数据写到 50%、`UPG_END` 前后各拔一次电，
   每次上电都必须是"能启动的旧镜像"或"boot 模式可重刷"，**不允许变砖**。
8. **静默时长**：J-Link 计时（或 boot 用一个 GPIO 翻转）测 `UPG_BEGIN` 的擦除时长，
   确认与 §5.6 的估算同量级，并把它写进 host 脚本的超时参数。
9. **不打扰舵机**：在总线上挂一个真实舵机，升级前后各做一次 `sync_read`，确认
   舵机仍正常应答，且升级帧没有被舵机误响应（用逻辑分析仪看波形）。
10. **量产镜像**：J-Link 烧 `slot` 镜像（不是 `APP_SLOT=0` 的开发镜像）后，
    `./build.sh smoke` 必须全过。

---

## 8. 为什么不用别的方案

| 方案 | 优点 | 为什么不用 |
|---|---|---|
| 单 slot + 外部编程器（现状 `APP_SLOT=0`） | 最简单、零 bootloader 代码、无 header 解析 | 现场机器人没有 SWD 可接；一次刷写失败（掉电、线松）就变砖。**开发板保留这个构型是对的，量产不能保留。** |
| 半 flash 交换（half-flash swap） | 不需要 ping-pong 计数，永远"另一半是备份" | 96 KB 应用只能拿到 96 KB 中的一半（128 KB 总 flash 里 boot+config 占 34 KB）。本芯片 256 KB 里可用 254 KB：A/B 方案给应用 96 KB ×2，半交换只能给 ~110 KB ×1 对，**换来的好处（少几个计数变量）不值**。而且"正在擦的那一半"在逻辑上就是 A/B，只是没名字。 |
| 只用调试 UART（USART0）升级 | 可以复用现成的 dbg 控制台、不用担心舵机总线时序 | 机器人装好以后 USART0 可能根本没引出；总线上已经有一根可靠的、带 CRC 的（Dynamixel）高速链路；调试串口升级等于把"能升级"绑定到一台必须拆机的 PC。**可以作为补充（`BUS_MIRROR_ENABLE=1` 现在就让 USART0 说协议），不能作为唯一通道。** |
| 靠芯片内置 bootloader（GD32 的 ROM boot） | 零代码 | GD32F30x 的 ROM boot 是 USART0/I2C 的固定协议，需要指定 BOOT 引脚电平与专门的下载器，现场机器人同样不可达；且它**不做**应用校验与回滚。 |
| 单 slot + 应用内自擦写（A/B 都不要） | 省一半 flash | 擦除时必须执行擦除代码，只能把擦除例程搬到 RAM；一旦掉电就没有可运行的镜像，**直接变砖**。不可接受。 |

**结论**：A/B ping-pong over the motor bus 是唯一同时满足下列全部条件的方案：

1. 现场可恢复（掉电、CRC 失败、新镜像起不来都有出口）；
2. 不增加机器人线束（复用已有的电机总线）；
3. 不牺牲可用 flash（96 KB ×2 对 22 KB 的现有应用是 8 倍余量）；
4. 校验到位（帧 CRC + 块 CRC + 两段镜像 CRC32 + header 一致性）；
5. 不打扰同总线上的舵机（只寻址本 ID、请求-应答节奏、擦除窗口可控）；
6. 主机侧可观测（PING 返回 boot 特征值、`UPG_STATUS`/`UPG_ACK_SEQ` 可读）。

代价是：32 KB boot 区、64 字节 header、一点配置页状态、以及**bootloader 自身只能
用 SWD 更新**（这是刻意的取舍：自升级 bootloader 需要一个不依赖应用的最小传输层，
出错就是永久变砖，收益远小于风险）。

---

## 附：无法从仓库验证的陈述

* 12 MHz HXTAL → 120 MHz 的具体 PLL 配置：来自 `v1/CMakeLists.txt` 的
  `SYS_CLK_SOURCE=HXTAL` / `SYS_CLK_HXTAL_HZ=12000000` 与
  `src/system_gd32f30x.c`，未逐行核对 PLL 倍频寄存器。
* 2 KB 页 / 128 页：来自 GD32F30x 用户手册 Rev3.4（"容量不多于 512 KB 的
  GD32F30x_CL/HD，bank0 闪存页大小为 2KB"）与 256 KB 容量的算术推得；
  未在真机上用 `fmc_page_erase` 逐页验证过 128 页。
* "16 系统 + 68 IRQ = 336 B"是 GD32F303 **HD** 的架构满配值；本工程实际编译出的
  `.vectors` 段是 **304 B（76 项）**，见 §2.1 的交叉验证。本文按 336 B 留余量。
* `UPG_*` 命令码、寄存器地址、`BOOT_MODEL_NUMBER`、`APP_HDR_MAGIC` 等全部是本文档
  的设计值，仓库中尚无对应代码。
* 96 KB 镜像的 CRC32 耗时（10~20 ms）是按"120 MHz、flash 等待周期、按位 CRC32
  约 8 条指令/字节"的估算，未实测。
* `UPG_BEGIN` 擦除 48 页 ≈ 1.2 s 是按"每页 25 ms"估算，未查 GD32F303 数据手册的
  页擦除时间上限（数据手册有 "Page erase time" 条目，未核对具体数值）。

---

## 9. 实现结果与偏差（2026-09-19）

### 9.1 与本文档草案的偏差（都是有意的）

| 草案 | 实现 | 原因 |
|---|---|---|
| `UPG_END` 由节点把 `app_header_t` 写进槽 | **节点只校验、不写** | flash 只能把 1 写成 0：镜像头已经由打包工具写好（头不参与两段 CRC），节点重写可能写不进“更新”的位（例如把 `CONFIRMED` 改回 `TRIAL` 需要把 0 置 1）。打包工具写、节点回读校验更安全 |
| trial 状态记在镜像头的 `flags` | trial 状态记在**配置页**，镜像头的 `TRIAL/CONFIRMED` 只作展示 | 头不可改；而且配置页状态能让“节点是否还要确认”这件事在身份窗里被主机看到（§5.2.1 bit0） |
| 块写 = 控制帧 + 数据帧 | **一帧**（213..255，43 字节或按块长截短），帧内把经过的 `ver/imglen/imgcrc` 重新填一遍 | 每帧在 USB 串口上要一个完整往返（台面实测 2.5 ms/事务）；两帧写法仍然支持，C 测试两者都覆盖 |
| FeeTech 用 `PING` 回 model number 区分应用/boot | 用 `READ` 判断：应用型号 `0x4D49`（'I','M'，2026-09-24 起；此前是像舵机的 `0x0C00`）/ boot 型号 `0x00B7`；应用还会在寄存器 0/1 报真实固件版本 | SCS 的 `PING` 应答不带数据；`0x0C00` 会被飞特工具当成某种 STS，而 boot/应用必须一眼可分（见 §9.6） |
| `UPG_ACK_CRC_L/H`（2 字节） | `UPG_NODE_CRC_0..3`（4 字节，227..230） | 节点复算的是 CRC32 |
| `boot_attempts` 只写配置页 | 写配置页 + **写失败时用 RAM 判定强制回滚** | 见 §6.3；实现里 `persist_trial_attempt()` 失败且 `attempts+1 >= MAX` 时立即改用回滚决策 |
| 身份信息散落在 PING/寄存器 6 | 独立**身份窗 180..195** | 一次 `READ` 就能拿到模式/槽/版本/CRC/启动状态，工具与人工都好用；写 180..255 在应用态返回 `DXL_ERR_ACCESS`，提示先进入 boot 模式 |
| boot 状态并入 `cfg_blob_t` v1 | `CFG_VERSION = 2`，v1 blob 直接判为无效 | 布局变了；`dev_cfg_from_flash()` 失败即用默认值，已实测（空页 → `boot_slot=trial_slot=255`） |
| `UPG_BEGIN` 擦 `ceil(imglen/2K)` 页 | 实现相同，且**页数上限硬检查** | 越界/未对齐由 `boot_flash_slot_*` 拒绝 |

### 9.2 新增文件清单

固件（`v1/src/`）：`app_header.{c,h}`、`boot_crc32.{c,h}`、`boot_flash.{c,h}`、
`boot_upgrade.{c,h}`、`boot_regs.{c,h}`、`boot_decide.{c,h}`、`boot_ram.{c,h}`、
`cfg_blob.{c,h}`、`dev_cfg.c`、`boot_main.c`、`boot_node_sim.c`（主机模拟器，见下）。

主机（`v1/host/`）：`package.py`（打包/校验/自测）、`upgrade.py`（Linux 升级工具）、
`tools/test_crc32.c`、`tools/test_boot.c`、`tools/boot_node_sim.c`、`tools/test_upgrade.py`、
`tools/py.sh`、`tools/run_c_tests.sh`。

链接脚本：`gd32f303cc_boot.ld`（32 KB）；`gd32f303cc_{app,slot_a,slot_b}.ld` 增加
`.boot_ram`（`0x2000BF00`，NOINIT、独立 MEMORY 区、带 `_sp` 越界断言）；
两个 slot 脚本增加 `.app_header`（`slot+0x200`，带 `ADDR`/`SIZEOF` 断言）并把
`.vectors` 用 `FILL(0xFF)` 补齐到 `0x200`。

### 9.3 测试覆盖（全部无硬件可跑）

| 测试 | 内容 |
|---|---|
| `host/tools/test_crc32.c` | 按位 CRC32 对 Python `zlib.crc32` 的 9 组向量（含分段续算与“两段拼接”） |
| `host/tools/test_boot.c` | **9500+ 断言**：头布局/每个校验失败分支、决策表全表、flash 写策略（boot 区与配置页不可达、擦除页数、未擦除拒绝）、会话规则、块序号/CRC/长度/越界、`UPG_END` 校验与提交、配置页往返与 trial 回滚、身份窗、**用真实 DXL/FeeTech 帧跑完整升级**、整槽 96 KB 升级 |
| `host/tools/test_upgrade.py` | 包格式与版本决策（离线）+ **pty 端到端**：把真固件编译成 `boot_node_sim`，用 `upgrade.main()` 走 CLI，校验落盘镜像逐字节、配置页 CRC 与 trial 状态、DXL/FeeTech、短尾块、同版本/降级/受保护槽/坏包/回读等路径，**173 断言** |
| `host/package.py selftest` | 包格式与槽/entry 一致性 |
| `./build.sh test` | 以上全部 + `host/bus.py` 编解码自测 + 网页服务器自测 |

### 9.4 台面实测

见 `docs/upgrade.md` §8（含硬件的两个 bug：J-Link 逐镜像 erase 擦掉 bootloader；
空配置页导致 `.bss` 零值被当成“A 已确认 + A 在 trial”）。要点：
25 KB 镜像 1553~1564 块、0 重传、**9.5 s**（2.6 KiB/s，受 CH340 USB 往返限制）、
节点 CRC 与主机一致、trial 30 s 自确认、3 次未确认自动回滚、单次事务 2.5 ms、
升级后协议 smoke 全过（390 Hz，0 超时）。

### 9.5 仍未实测/未做

* 96 KB 满槽真机升级（主机侧 pty 测试已覆盖 96 KB）与擦除中途掉电演练；
* bootloader 区在 flash 里放两份（§6.6 的可选项）——未做，bootloader 仍只由 SWD 更新；
* 真实舵机与升级帧共总线的波形验证；
* `boot_stay` 与 `UPG_CONFIRM` 的更多组合场景（应用态确认、boot 态拒绝均已实现并测试）。

### 9.6 总线时序改造（2026-09-24，P0/P1）

不是升级功能，但改了 `bus.c` / `uart_port.c` / `dev.c` 这些**两个镜像共用**的文件，
所以记录在这里，细节与实测数据见 `docs/bus_timing_borrow_plan.md`：

| 项 | 实现 | 原因 |
|---|---|---|
| `sync_read` 位次仲裁 | `src/bus_arb.c`：自己的下标 k > 0 时先让 k 个应答槽（`BUS_SLOT_US = 300`），槽满或前面出现新的指令帧才继续 | 真舵机为每个前导 ID 等一槽（实测 ≈295 µs，缺席也占一槽）；抢在别人前面发会撞掉**对方**的帧。运行时 IMU 恒在第 0 位，这条只在调试工具改 ID 顺序时生效 |
| 应答路径 | 帧解析在 USART 中断里，发送走 DMA（USART0_TX = DMA0 CH3 / USART1_TX = DMA0 CH6） | 排在后面的设备只留 ≈295 µs，主循环给不出这个上界；在中断里自旋发 21 字节会饿死其它中断 |
| 遥测发布 | `dev_refresh()` 只在主循环调用（有新样本或每 50 ms），在极短临界区里发布；`dev_read()` 不再刷新 | 读发生在中断里，不能做 ADC/浮点/厂商窗重写；临界区保证中断读到的是整块新或整块旧 |
| 间隙判据 | 字节间隙 > `BUS_GAP_US`(500) 且**帧解析进行中**才丢弃残帧并计 `gaps` | 20 ms 整帧超时会把"截断帧 + 紧随的新帧"粘起来；空闲间隔不是故障 |
| 回声门 | 发送期间 + 发送后 2 字节丢弃（`tx_busy` / `echo_skip`），取代原来的 `+2 ms` 窗口（**此行的 2 字节随后被时间窗取代：`UART_ECHO_WINDOW_BYTES = 3` 个字节时间，见 `docs/bus_timing_borrow_plan.md` §4.6.1**） | 原窗口是整数除法的产物，会把紧随其后的主机指令一起吞掉 |
| FeeTech 契约块 | 56..70 = 12 控制 + 计数 + 状态 + 保留 0（`src/telem_pack.c`）；20 字节诊断块在 128 | 舵机在 56 只答 15 字节，而 `sync_read` 对一个 ID 列表只有一个长度；读 20 字节会让一次事务从 4.9 ms 涨到约 5.7 ms |
| 应答 STATUS | 恒 0；内部错误记到厂商窗 `V_LAST_ERR`（179） | 飞特工具把非 0 的 ack 状态当舵机故障位；boot 模式同样处理 |
| FeeTech `0x08` REBOOT | `fee.c` 里不应答地重启，走 `dev_request_reboot()`（应用与 bootloader 共用） | 真机 fw 3.46 实测：0x08 存在、不应答、823 ms 回来、EEPROM 设置保留、RAM 增益回落 EEPROM；本项目的旧手册指令表缺这一条（见 `docs/bus_timing_borrow_plan.md` §1.7） |
