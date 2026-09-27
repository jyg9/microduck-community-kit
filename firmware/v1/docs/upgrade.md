# imu_to_dxl v1 现场升级指南（A/B 槽 + 总线升级工具）

本文是**使用者视角**的升级文档：怎么构建、怎么第一次烧录、怎么在机器人上通过电机总线升级、
出问题怎么恢复。设计动机与取舍见 `flash_layout.md`，寄存器级细节见本文 §5。

状态：**已实现并在真机验证**（2026-09-19，GD32F303CC 开发板 + J-Link + CH340 串口）。
所有实测数字都来自那次台面验证，未标注的地方就是设计值而非实测值。

---

## 1. 为什么需要它

板上只有 SWD 调试口和电机总线两根线。机器人装好以后 SWD 够不到，而**升级失败必须还能用**：

* 两个应用槽（A/B）轮流使用，新镜像先以 **trial（试用）** 身份启动；
* 试用镜像必须在 30 s 内“自我确认”，否则 bootloader 在 3 次启动尝试后**自动回滚**到上一个槽；
* 掉电、CRC 错误、新镜像起不来都有出口：要么跑旧镜像，要么停在 bootloader 里等主机重刷；
* 升级包在**发送前**就被工具校验（容器 CRC32 + 镜像头 + 两段镜像 CRC32），坏包不会上线。

bootloader 自身**只能通过 SWD 更新**。这是刻意的取舍：自升级 bootloader 需要一个不依赖应用的
最小传输层，写错就是永久变砖，收益远小于风险。

---

## 2. 构建

```bash
cd v1
./build.sh                      # 构建全部：应用 + bootloader + A/B 两个槽镜像
EXTRA_CMAKE_ARGS="-DAPP_VERSION_STR=1.1.0" ./build.sh    # 指定版本号
```

产物（`build/`）：

| 文件 | 用途 |
|---|---|
| `gd32f303cc_imu_to_dxl.hex` | `APP_SLOT` 指向的开发镜像（默认 `APP_SLOT=0`，**无 bootloader**，烧在 `0x08000000`，会覆盖 bootloader 区） |
| `gd32f303cc_imu_to_dxl_boot.hex` | bootloader（32 KB @ `0x08000000`） |
| `gd32f303cc_imu_to_dxl_slot_a.hex` / `_slot_b.hex` | 槽镜像，**已回填 64 字节应用头**，可用 J-Link 直接烧 |
| `gd32f303cc_imu_to_dxl_slot_a.ipkg` / `_slot_b.ipkg` | **升级包**，交给升级工具 |
| `gd32f303cc_imu_to_dxl_slot_*.json` | 版本/长度/CRC 元数据（CI 与人工核对用） |

版本号是 `major.minor.patch`：`major`/`minor` 各 4 bit、`patch` 8 bit，打包进镜像头的
`image_version`，同时以 Dynamixel 寄存器 6 的 `major<<4|minor` 形式暴露给老工具。

---

## 3. 第一次烧录（一次性，需要 SWD）

```bash
./build.sh flash-pair a     # 一次擦除，然后依次写入 bootloader + slot A
```

**不要**用 `flash-slot`/`flash-boot` 分两次带擦除地烧：J-Link 脚本里的 `erase` 是整片擦除，
第二次会把第一次写的镜像擦掉（这个坑在台面上踩过，见 §8）。`flash-pair` 只擦一次。
`flash-slot` 现在不带擦除，用于替换某一个槽。

上电后应用从 slot A 启动，用手机器人总线就是一条普通的 `imu_to_dxl` 节点：

```bash
./build.sh status           # 应用态 / slot A / 版本 / 镜像 CRC / boot 状态
```

`status` 打印的 `boot state` 来自配置页（见 §5.3）：`confirmed slot`、`trial`、`attempts`。

---

## 4. 用升级工具升级

### 4.1 离线检查一个包

```bash
./host/upgrade.py verify build/gd32f303cc_imu_to_dxl_slot_b.ipkg    # 0 = 可用，1 = 不可用
./host/upgrade.py info   build/gd32f303cc_imu_to_dxl_slot_b.ipkg    # 打印全部元数据
```

`verify` 检查的东西（任何一条不过就拒绝）：

* 容器魔数 `IUP1`、格式版本、头部长度；
* 整个 payload 的 CRC32（掉包/截断）与“两段拼接”CRC32（节点在 `UPG_END` 会重算同一个值）；
* 镜像头魔数 `AHM1`、头版本、`image_size` 与文件长度一致且 4 字节对齐；
* `crc_low`/`crc_high` 重算一致、`entry == 向量表[1]` 且带 Thumb 位；
* 板号、升级协议版本、包声明的槽与 `entry` 地址所属的槽一致；
* `no-boot` 标志。

### 4.2 看节点需不需要升级

```bash
./build.sh status build/gd32f303cc_imu_to_dxl_slot_b.ipkg
```

输出里的 `verdict` 四选一：

| verdict | 含义 | 退出码 |
|---|---|---|
| `UPGRADE` | 包比运行中的新 → 建议升级 | 0 |
| `SAME` | 版本相同（会区分“同一镜像”和“同版本不同构建”） | 0 |
| `DOWNGRADE` | 包更旧 → 需要 `--allow-downgrade` | 3 |
| `REFUSED`/错误 | 包不可用、目标槽不对、节点不在可升级状态 | 1/2/5 |

运行中的镜像没有有效头（例如用调试器直接烧的 `.bin`）时，任何包都算 `UPGRADE`。

### 4.3 升级

```bash
./build.sh upgrade build/gd32f303cc_imu_to_dxl_slot_b.ipkg --yes
# 等价于
./host/upgrade.py upgrade <pkg> --port /dev/ttyUSB0 --baud 1000000 --protocol auto --yes
```

工具做的事，按顺序：

1. 校验包（§4.1），任何问题在**总线上一个字节都没发**之前就退出；
2. 探测协议（先 Dynamixel 再 FeeTech）、读身份窗、比较版本（§4.2）；
3. 决定目标槽：**包为哪个槽链接就写哪个槽**（槽镜像是按槽基址链接的，写错槽必然验证失败）；
   若目标槽就是当前运行的槽，直接拒绝并给出“为另一个槽构建”的提示；
4. 若节点在应用态：写 vendor 命令 `VCMD_BOOT`（`V_CMD=6`），节点保存 `boot_stay`、
   写 SRAM 握手字并软复位，工具轮询直到看到 bootloader（默认 6 s 超时）；
5. `UPG_BEGIN` 擦除目标槽（按 `ceil(imglen/2 KB)` 页），然后按 16 字节分块写入：
   **每块一帧**（`213..255`）携带序号/偏移/长度/块 CRC16 + 数据，
   再读一次状态窗核对 `UPG_STATUS`/`UPG_ACK_SEQ`；失败重发同序号（默认 3 次，幂等）；
6. `UPG_END`：节点重算整镜像 CRC32 并与主机值比对，再回读校验应用头（魔数/长度/entry/
   两段 CRC/板号），全部通过才把该槽标成 trial；工具核对节点回报的 CRC；
7. 默认 `--reboot`：`UPG_REBOOT` → bootloader 清 `boot_stay` 并复位 → 新镜像以 trial 启动，
   工具等它出现在总线上并打印版本；
8. 任何阶段失败都会发 `UPG_ABORT` 并给非零退出码。

### 4.4 其它子命令

```bash
./host/upgrade.py boot       --port /dev/ttyUSB0   # 让应用复位进 bootloader 并停住
./host/upgrade.py confirm    --port /dev/ttyUSB0   # 立刻确认当前 trial 镜像
./host/upgrade.py abort      --port /dev/ttyUSB0   # 丢弃正在进行的会话
./host/upgrade.py readback   --port /dev/ttyUSB0 --slot b --offset 0x200 --length 64
                                                   # 通过 bootloader 回读 flash
```

常用选项：`--protocol auto|dxl|fee`、`--id N`、`--baud`、`--timeout`、`--retries`、
`--slot a|b`、`--allow-downgrade`、`--force`（同版本也重刷）、`--no-reboot`、
`--no-enter-boot`、`--dry-run`、`--quiet`。

退出码：`0` 成功/无需升级，`1` 包不可用，`2` 目标槽或节点状态不允许，`3` 版本更旧且未允许，
`4` 升级成功但节点没在超时内回到应用态，`5` 总线/协议错误。

---

## 5. 协议与寄存器

### 5.1 复用现有帧

升级没有新帧类型、没有新的校验层：Dynamixel 用 `INST_READ/WRITE`，FeeTech 用
`READ/WRITE`，都是普通寄存器读写。因此 `dxl2.c`/`fee.c`/`bus.c` 在应用和 bootloader 里
是**同一份代码**，主机侧也复用 `host/bus.py` 的编解码。总线上其它舵机看到的只是
“发给 ID 200 的读写”，不需要任何特殊处理。

### 5.2 身份窗（180..195，只读）

| 地址 | 含义 |
|---|---|
| 180..181 | 魔数 `0x5055`（`'U','P'`） |
| 182 | 模式：0 = 应用，1 = bootloader |
| 183 | 槽：0=A、1=B、255=无 |
| 184 | 标志：bit0 = **有效 trial**、bit1 = 已确认、bit7 = 镜像头无效/未打包 |
| 185..186 | 运行镜像的 `image_version`（bootloader 里是“将要运行”的那个槽） |
| 187..190 | 该镜像的“两段拼接”CRC32 |
| 191 | 扩展版本：1 表示 192..195 有效（老固件为 0） |
| 192 | 配置页里的 `boot_slot`（最近确认的槽） |
| 193 | 配置页里的 `trial_slot`（还在试用的槽） |
| 194 | 已尝试的 trial 启动次数 |
| 195 | `boot_stay`（bootloader 是否被要求停住） |

bit0 是**有效 trial**而不是镜像头里的位：镜像头在打包时就固定了，flash 只能把 1 写成 0，
所以“已确认”这件事记在配置页里，由固件把它折算到这一位上。工具在 `mode=boot` 时把
“存在 trial 槽”就视为 on trial（下一个上电就会跑它），在应用态则看“trial 槽 == 当前槽”。

### 5.3 升级会话窗（208..255）

| 地址 | 名称 | 读写 | 说明 |
|---|---|---|---|
| 208..209 | `UPG_MAGIC` | r/w | `0x4247`；写对即打开会话（写错即关闭） |
| 210 | `UPG_CMD` | w | 1=BEGIN 2=DATA 3=END 4=ABORT 5=CONFIRM 6=REBOOT 7=READBACK，写完即清零 |
| 211 | `UPG_STATUS` | r | 0 idle / 1 erased / 2 receiving / 3 block ok / 4 verified / 5 committed；`0x80` 位表示错误 |
| 212 | `UPG_TARGET` | r/w | 0=A、1=B |
| 213..214 | `UPG_SEQ` | r/w | 块序号，从 0 起；节点接受 `ack+1`（新块）或 `ack`（重传） |
| 215..218 | `UPG_OFF` | r/w | 块在槽内的字节偏移（LE32，4 字节对齐） |
| 219 | `UPG_BLKLEN` | r/w | 1..16；写数据窗时必须与实写字节数一致 |
| 220..221 | `UPG_BLKCRC` | w | 块数据的 CRC16（`crc16_dxl`） |
| 222..223 | `UPG_ACK_SEQ` | r | 最近一次成功写入的块序号 |
| 224 | `UPG_ERRCODE` | r | 1 块 CRC、2 越界、3 flash 编程失败、4 状态机、5 目标槽非法、6 缺 magic、7 镜像校验失败、8 长度 |
| 225..226 | `UPG_VER` | r/w | 待写镜像的 `image_version`（节点会与镜像头交叉核对） |
| 227..230 | `UPG_NODE_CRC` | r | 节点在 `UPG_END` 重算出的整镜像 CRC32 |
| 231 | 保留 | r | 0 |
| 232..235 | `UPG_IMGLEN` | r/w | 整镜像长度 |
| 236..239 | `UPG_IMGCRC` | r/w | 主机算的整镜像 CRC32（与节点同序） |
| 240..255 | `UPG_DATA` | r/w | 数据窗（16 字节；READBACK 也回填到这里） |

用户不必手写这些帧——`host/upgrade.py` 已经封装；这一节是给固件维护者和
非 Python 主机的参考。**关键字：块写的长度由“实际写入数据窗的字节数”决定**，
所以最后一块可以短于 16 字节（工具就是这么发的，真机上 25016 字节的最后一块是 8 字节）。

### 5.4 写入策略（为什么不会变砖）

* 升级状态机只能通过 `boot_flash_slot_*` 写 flash，这两个函数**只接受槽内偏移**，
  越界/未对齐一律拒绝 → bootloader 区和配置页在物理上不可达；
* `UPG_BEGIN` 拒绝目标槽 == “bootloader 即将运行的槽且另一个槽不可启动”的情况，
  **绝不擦掉最后一个可启动镜像**；
* 每块写后回读比对；结束前重算整镜像 CRC32 并回读校验应用头；
* `UPG_END` **不重写**镜像头（原因见 `flash_layout.md` §9）：头由打包工具写好，
  节点只校验。flash 只能把 1 写成 0，节点重写反而可能写不进更“新”的位。

---

## 6. trial / 确认 / 回滚

| 事件 | 结果 |
|---|---|
| 提交成功（`UPG_END` 通过） | 配置页：`trial_slot = 目标槽`、`attempts = 0`、`pingpong +1`、`boot_stay = 0` |
| 上电/复位且存在 trial 槽、`attempts < 3` | 运行该槽，`attempts +1`（**跳转前**持久化） |
| 应用连续运行 ≥ 30 s 且期间有过一次合法总线帧 | 自己确认：`boot_slot = 自己`、`trial_slot = 无`，写配置页 |
| 应用运行 ≥ 60 s 但主机从未发过帧 | 同样确认（否则一台没人理的机器人会永远回滚） |
| `attempts` 达到 3 仍没确认 | 下一次启动**放弃 trial**、回滚到上一个已确认槽，并把 `attempts` 清零 |
| 主机显式确认 | 应用态写 `V_CMD=5`（`upgrade.py confirm`） |
| 主机显式切换槽 | 应用态写 `V_CMD=7`：下一次启动跑另一个槽 |
| 主机要求进 bootloader | 应用态写 `V_CMD=6`（`upgrade.py boot`），`boot_stay` 落盘、SRAM 握手字置位、软复位 |
| boot 态 30 s 没有主机来访且存在可启动槽 | 清 `boot_stay`，按决策正常启动（主机失联不会把机器人吊死） |

实测（§8）：升级后 `boot state: trial B (1 attempt)` → 35 s 后
`confirmed slot B, trial -`；连续 3 次复位未确认 → 自动回到另一个槽。

---

## 7. 失败模式与恢复

| 现象 | 原因 | 处理 |
|---|---|---|
| `status` 显示 `mode: bootloader` | 应用两个槽都不可启动，或上次要求过 `boot_stay` | 直接 `upgrade` 刷一个槽即可 |
| 升级后节点起不来 | 新镜像有 bug（头合法但应用崩溃） | 3 次尝试后自动回滚；也可 `upgrade.py boot` + 刷另一个槽 |
| 升级中途掉电 | 目标槽部分写入、头未变（其实头是主机写的，可能已完整） | 目标槽头校验不过 → 启动旧槽；重发升级即可 |
| `status` 报 `image crc` 与包不一致 | 板上是另一份构建 | 正常，`--force` 可重刷 |
| 升级工具报 `did not answer` | 总线/端口/ID 不对，或节点在重启 | 检查 `--port`/`--id`/`--baud`，`status` 重试 |
| `UPG_END failed ... ERR_VERIFY` | 传输中出错或包与槽不匹配 | 包的槽/版本已在上线前校验，实际多为线路问题；重发 |
| 想确认“到底跑的是哪份构建” | — | `status` 的 `image crc` + `.json` 里的 `sha256` |

**恢复的底线**：只要 bootloader 区没被擦，节点一定能重新进入升级状态。所以
`APP_SLOT=0` 的开发镜像（`./build.sh flash`）会覆盖 bootloader —— 只在实验台上用。

---

## 8. 台面实测记录（2026-09-19）

硬件：GD32F303CC 开发板，J-Link SWD，CH340 USB 串口（`/dev/ttyUSB0`），1 Mbps。

| 项目 | 结果 |
|---|---|
| `flash-pair a` → 应用运行 | `mode=application slot=A v1.0.0 image crc=0x2644d68b` |
| 总线升级 slot B（24844 B, Dynamixel） | 1553 块，0 重传，**12.68 s**，节点 CRC == 包 CRC，重启后 `v1.2.3 in slot B (on trial)` |
| 总线升级 slot A（24844 B） | 1553 块，0 重传，9.60 s |
| 单帧块写优化后（25016 B） | 1564 块，0 重传，**9.53 s**，2.6 KiB/s |
| 单次总线事务延迟 | ping 2.51 ms、读 16 字节 2.61 ms（**CH340 USB 往返主导**，不是 MCU） |
| trial 自确认 | 升级后 `trial B (1 attempt)` → 35 s 后 `confirmed slot B, trial -` |
| trial 回滚 | 提交到 B 后 3 次复位（每次 <30 s）→ 第 3 次后自动回到 A（CRC 0x69103c11） |
| 保护规则 | 为正在运行的槽构建的包被拒绝；`--force` 只跳过版本判断，不跳过该保护 |
| 协议 smoke（升级后） | Dynamixel 与 FeeTech 全过，2542 次轮询 0 超时 0 坏帧，~390 Hz |
| 身份窗 | `55 50 00 00 00 00 10 82 4d ee b4 01 ff ff 00 00`（应用态、slot A、非 trial、boot_slot=无、trial=无） |

### 台面发现并修掉的两个 bug

1. **J-Link 逐镜像擦除把 bootloader 擦掉了**：原来每烧一个 hex 就跑一次带 `erase` 的脚本，
   结果刷 slot A 时把刚刷好的 bootloader 擦成 `0xFF`，芯片一上电就从 `0xFFFFFFFF`
   取向量 → HardFault（J-Link 读到 `PC=0xFFFFFFFE`、`IPSR=3`）。
   现在 `flash_files` 生成一个脚本、只擦一次，`flash-slot` 默认不擦。
2. **空配置页时启动状态是 `.bss` 全零**：`dev.c` 只给 `dev_cfg_t` 设了默认值，
   `cfg_boot_t` 留着零 → “slot A 已确认 + slot A 在试用”这种假状态被报给主机。
   现在 `dev_cfg_defaults()` 同时 `cfg_boot_defaults()`，`dev_boot_init()` 和
   bootloader 侧再做一次越界归一化。修完后新烧的板子身份窗是
   `boot_slot=255 trial_slot=255`。

### 还没在真机上验证的

* 96 KB 满槽升级（真机只跑了 25 KB 的镜像；主机侧 pty 测试跑过 96 KB）；
* 擦除中途/写一半掉电演练（设计见 `flash_layout.md` §6.1，未实测）；
* 真实舵机挂在同一条总线上时的互联影响（台面只有一个节点）；
* 量产镜像（`APP_SLOT=1|2`）与 microduck 的真实控制回路联调。
