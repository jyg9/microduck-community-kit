# -*- coding: utf-8 -*-
"""microduck 整机 15 台舵机的初始化（编号 + 位置校准）。

关节表来源
----------
数值抄自 microduck 官方机器人软件的飞特适配版
``software/microduck_feetech``：

* ``duck-control/src/model.rs`` 的 ``JOINT_IDS``（15 个总线 ID）、
  ``DEFAULT_POSITION``（home 姿态关节角，单位弧度）、``FACTORY_ID = 1``、
  ``EXPECTED_BAUD_CODE = 0``、``IMU_DXL_ID = 200``；
* ``duck-ipc-proto/src/lib.rs`` 的 ``JOINT_NAMES``（关节名，顺序与 ID 表一一对应）。

``model.rs`` 里那三条常量还把两件事钉死了，本模块同样依赖它们：

* **ID 1 与 200 都不属于任何关节**，所以"总线上出现 ID 1"就等于
  "有一颗还没编号的新舵机"；
* 新舵机出厂就是 **ID 1 / 1 Mbps**，不需要换波特率去找它。

一台新舵机出厂时：主 ID = 1、波特率编码 = 0（1 Mbps）、
锁标志（55 号） = 1（写 EPROM 掉电不保存）、应答状态级别（8 号） = 1。
所以"初始化一台"就是把 1 号改写成它对应的关节 ID，并按需要做好位置校准。

写入锁的措辞很容易记反，这里统一按厂商内存表的字面意思写：
**写 0 = 关闭写入锁**（EPROM 写入掉电保存，能改 ID/波特率），
**写 1 = 打开写入锁**（EPROM 写入掉电不保存）。所以"解锁"是写 0、"上锁"是写 1。

位置坐标与负值（真机实测结论）
------------------------------
在 HD-1910-C001（固件 3.46）上实测，三件事按重要性排列：

1. **出厂设置下负的目标位置不生效。** 出厂 ``9/11 号角度限制 = 0/4095``，位置坐标被夹在
   一圈 ``[0, 4095]`` 内：写目标 −500 **协议层被接受**（42 号回读 −500），
   但 67 号"目标位置回读"是 0、舵机不动。把 ``9 号 = 0、11 号 = 0``
   （厂商内存表："多圈绝对位置控制时此值为 0"）之后，同一个 −500 真的走到了。
   所以 **microduck 整机必须打开多圈**：关节表里有 4 个 home 姿态角是负的。
   界面上这个开关**默认关闭**（台面单机调试一圈内更安全），两个方向都会真的写
   9/11 号：勾选写 0/0（多圈），取消写 0/4095（单圈行程）。
2. **打开多圈、位置偏移(31) 保持 0 时，坐标是干净的**：实测写目标
   −500 → 当前位置 −500、−1000 → −999、−300 → −301，67 号目标回读同步跟随；
   同时 31 号 = 0 时读数就是编码器值。**这就是推荐配置**，
   机械零位由"把舵盘装在 home 姿态"决定，偏移只用来做小幅修正。
3. **拿大偏移去搬原点不可靠。** 写一个较大的偏移会让固件把编码器坐标**重锚**：
   实测偏移 −3591→−3091（+500）后读数从 −4096 变成 −500（差 3596），
   而 −3591→−4591（−1000）后读数从 −4096 变成 −3096（差 1000）——同一根轴、
   同一台舵机，写偏移前后的差值不是同一个常量；读数还总是被折进约 ±2 圈的窗口。
   手册没写这些（`docs/飞特通讯协议说明.md` 第 12.5 节列为未确认项），
   因此 ``_calibrate_offset`` **不做公式假设**：有界搜索候选偏移 + 逐个回读验证，
   全部不中就把原偏移写回去并如实报失败。
   经验上"读数离目标越近（所需偏移越小）越可靠"，所以校准前把关节摆到 home 姿态时，
   尽量让读数落在 0 附近。

另外 ``CAL(0x0B)`` 的语义是确定的：把**此刻的物理位置**写成 **2048 计数（180°）**，
并相应改写 31 号偏移（实测 CAL 前 偏移 = 0、位置 = 72；CAL 后 偏移 = −1976、位置 = 2048）。

`duck-control` 的 ``DEFAULT_POSITION`` 用的是有符号关节角，而
``feetech.rs::position_rad`` 按"BIT15 为方向位"解读位置字段——打开多圈之后
舵机给的正是这种写法，两者方向一致。
"""

from __future__ import absolute_import

import math
import time

from .memory_table import decode_signed_magnitude, encode_signed_magnitude
from .protocol import (
    ADDR_ACC,
    ADDR_BAUD_RATE,
    ADDR_GOAL_POSITION,
    ADDR_GOAL_SPEED,
    ADDR_ID,
    ADDR_LOCK,
    ADDR_MAX_ANGLE_LIMIT,
    ADDR_MIN_ANGLE_LIMIT,
    ADDR_MODE,
    ADDR_POSITION_KD_EPROM,
    ADDR_POSITION_KD_RAM,
    ADDR_POSITION_KP_EPROM,
    ADDR_POSITION_KP_RAM,
    ADDR_POSITION_OFFSET,
    ADDR_PRESENT_MOVING,
    ADDR_PRESENT_POSITION,
    ADDR_RESPONSE_LEVEL,
    ADDR_SECOND_ID,
    ADDR_TORQUE_ENABLE,
    ADDR_UNLOAD_CONDITION,
    EXPECTED_BAUD_CODE,
    FACTORY_ID,
    HLSBus,
    IMU_BUS_ID,
    ProtocolError,
    UNLOAD_BIT_OVER_CURRENT,
    UNLOAD_BIT_VOLTAGE,
)

# ── 换算常量（与 duck-control/src/feetech.rs 一致）───────────────────────────
COUNTS_PER_REV = 4096.0
RAD_PER_COUNT = 2.0 * math.pi / COUNTS_PER_REV
DEG_PER_COUNT = 360.0 / COUNTS_PER_REV
# 位置偏移（31 号）量程 ±4095 计数，也就是最多挪一圈。
OFFSET_LIMIT = 4095
#: 位置读数的一圈计数。实测本机（HD-1910-C001 / 固件 3.46）：
#: ``位置(56) ≡ 编码器值 − 位置偏移(31) (mod 4096)``；负值有时回绕成 0~4095，
#: 有时带 BIT15 方向位直接报负值（两种写法都实测到过），所以比较时按一圈取模。
POSITION_WRAP = 4096
#: 位置校准允许的机械误差（计数）。1 计数 = 0.087°，8 计数 ≈ 0.70°。
POSITION_TOLERANCE = 8
# EEPROM 写入后的稳定时间：应答先到，flash 单元还没写完。
EEPROM_SETTLE = 0.02


def _wrap_delta(got, want, wrap=POSITION_WRAP):
    """两个位置读数在一圈上的最短距离，用来做与表示方式无关的比较。"""
    delta = (int(got) - int(want)) % wrap
    return min(delta, wrap - delta)


#: 锁标志的两个取值，按厂商内存表的字面措辞命名，避免"打开/关闭"歧义。
LOCK_EPROM_SAVED = 0  # 写 0：关闭写入锁，EPROM 写入掉电保存
LOCK_EPROM_VOLATILE = 1  # 写 1：打开写入锁，EPROM 写入掉电不保存

# ── 位置环增益：批量初始化写入的默认值 ──────────────────────────────────────
# 为什么要写：舵机出厂 EPROM 的位置环 P 是厂商默认的 32，但 **microduck 的
# `robotd` 会在启动时按 `robotd.toml` 的 `gain` 把 RAM 50 覆盖成它自己的值**
# （`duck-control/src/bus.rs` 的 `set_gain`），而 `deploy/robotd.toml` 那份默认
# 是给 XL330 的 0..16383 标度调的 200 —— 在 FeeTech 的 0..254 上就是顶格。
#
# 真机实测（2026-09-30，HD-1910-C001 / 固件 3.46，15 台装在一只悬空的鸭子上）：
#   写 RAM 50 = 200：left_hip_yaw（ID 20）自激振荡，峰值电流 4966–5388 mA，
#     目标不动时自报速度 4.37 rad/s，壳温升到 74 °C 且持续上升，人耳听到剧烈抖动；
#   写 RAM 50 = 32 ：同一关节峰值电流 110 mA、平均 20 mA、速度峰值 0.307 rad/s、
#     通电 6 s 温度不变（48→49 °C），其余 14 台电流同样近 0。
# Kd 取 0 是与 `set_gain` 对齐：厂商出厂 EPROM 的 D 是 40，而 microduck 刻意把
# Kd 写 0（不让舵机自己的阻尼叠在策略输出上）。初始化写 0 可以让"robotd 还没启动"
# 的那段时间里舵机行为与 robotd 接管后一致。
#
# 两个值都写两处：EEPROM 21/22（掉电保存，上电时加载进 RAM）与 RAM 50/51
# （立即生效，不必等下一次上电）。**注意**：robotd 一旦启动仍会用
# `robotd.toml` 的 `gain` 覆盖 RAM 50，所以这两个值只保证"出厂/未接管时是对的"，
# 整机增益仍需与 `robotd.toml` 保持一致（补丁说明未决事项 2/3）。
POSITION_KP_DEFAULT = 32
POSITION_KD_DEFAULT = 0
#: 位置环增益寄存器的合法范围（厂商内存表：0 ~ 254）。
POSITION_GAIN_MAX = 254

# ── 9/11 号角度限制：多圈开关的两个取值 ─────────────────────────────────────
#: 多圈绝对位置控制：厂商内存表在 9/11 号上写明"多圈绝对位置控制时此值为 0"。
LIMITS_MULTITURN = (0, 0)
#: 单圈行程（出厂值）：9 号 = 0、11 号 = 4095，位置被夹在一圈内。
LIMITS_SINGLE_TURN = (0, 4095)

# ── 19 号「卸载条件」：默认关闭电压保护与过流保护 ────────────────────────────
# 厂商内存表 19 号是位域（见 memory_table.PROTECT_BITS）：
#   BIT0 电压保护 / BIT1 磁编码保护 / BIT2 过热保护 / BIT3 过流保护
# 置 1 = 开启，触发时舵机**卸载**（不再输出扭矩）。
#
# microduck 站在一条腿上的时候扭矩被卸掉就是摔倒，所以这两项**默认不打开**：
# 电压保护会在电池瞬间跌落时卸力，过流保护会在踩到东西/被撞时卸力，
# 两者都属于"宁可不保护也不要中途松腿"的取舍，交给装配/调试的人按需打开。
# 工具只动 BIT0 和 BIT3，BIT1/BIT2（磁编码、过热）由厂商固件决定，**读改写保留**。
PROTECT_VOLTAGE_DEFAULT = False
PROTECT_OVER_CURRENT_DEFAULT = False

#: 出厂新舵机的两个特征。
FRESH_ID = FACTORY_ID

# ── 关节表 ──────────────────────────────────────────────────────────────────
# (关节名, 总线 ID, 部位, home 姿态角/弧度)
# 顺序 = duck-control/src/model.rs 的 JOINT_IDS 顺序 = JOINT_NAMES 顺序。
_JOINT_SOURCE = [
    ("left_hip_yaw", 20, "左腿", 0.0),
    ("left_hip_roll", 21, "左腿", -0.0873),
    ("left_hip_pitch", 22, "左腿", -0.4579),
    ("left_knee", 23, "左腿", -0.0049),
    ("left_ankle", 24, "左腿", 0.4530),
    ("neck_pitch", 30, "头颈", 0.3491),
    ("head_pitch", 31, "头颈", 0.3491),
    ("head_yaw", 32, "头颈", 0.0),
    ("head_roll", 33, "头颈", 0.0),
    ("mouth", 34, "头颈", 0.0),
    ("right_hip_yaw", 10, "右腿", 0.0),
    ("right_hip_roll", 11, "右腿", 0.0873),
    ("right_hip_pitch", 12, "右腿", 0.4579),
    ("right_knee", 13, "右腿", 0.0049),
    ("right_ankle", 14, "右腿", -0.4530),
]


def _build_joints():
    joints = []
    for index, (name, joint_id, group, home_rad) in enumerate(_JOINT_SOURCE):
        joints.append({
            "index": index,
            "name": name,
            "id": joint_id,
            "group": group,
            "home_rad": home_rad,
            "home_deg": round(home_rad / math.pi * 180.0, 3),
            "home_counts": int(round(home_rad / RAD_PER_COUNT)),
        })
    return joints


#: 15 个关节，顺序与 microduck 的 JOINT_IDS / JOINT_NAMES 一致。
JOINTS = _build_joints()
JOINT_IDS = [item["id"] for item in JOINTS]
#: 头颈那 5 个关节连在一起，界面上按部位分组显示。
JOINT_GROUPS = ["左腿", "头颈", "右腿"]


def get_joints():
    return [dict(item) for item in JOINTS]


def joint_by_id(joint_id):
    for item in JOINTS:
        if item["id"] == int(joint_id):
            return item
    return None


# ── 校准方式 ────────────────────────────────────────────────────────────────
#: 每台舵机在写 ID 之后要做的"初始位置校准"，四种方式对应四种装配工艺。
CALIBRATE_MODES = [
    {
        "id": "cal",
        "name": "中位校准（CAL 指令）",
        "desc": "执行厂商 CAL(0x0B)：把该关节此刻的物理位置写成位置偏移中点。"
                "实测（HD-1910-C001 / 固件 3.46）校准后位置读数是 2048 计数（180°），"
                "并把 31 号位置偏移改写成相应数值。执行前需要把关节摆到装配基准位。",
        "needs_reference": True,
    },
    {
        "id": "offset",
        "name": "按 home 姿态写位置偏移",
        "desc": "把 31 号位置偏移改成“关节此刻的读数 = 该关节 home 姿态角”。"
                "这条角来自 duck-control 的 DEFAULT_POSITION。"
                "注意：实测大偏移会让固件重锚坐标，所以只在“当前读数离目标较近”时可靠；"
                "工具会有界搜索候选并逐个回读验证，全不中就把偏移写回原值并报失败。"
                "推荐做法是让关节在 home 姿态时读数落在 0 附近（舵盘换个齿位即可）。",
        "needs_reference": True,
    },
    {
        "id": "mid",
        "name": "转到位置 0（写目标位置 0）",
        "desc": "让输出轴转到位置坐标 0（0°）并保持扭矩，方便按舵盘标记装配，不写 EPROM。"
                "注意厂商文档里的“中点”是 2048 计数（180°），和这里的 0 不是同一个位置。",
        "needs_reference": False,
    },
    {
        "id": "none",
        "name": "不校准，只改 ID",
        "desc": "只写 ID/波特率，位置零位由机械装配决定。装好之后再整体校准可以选这项。",
        "needs_reference": False,
    },
]

CALIBRATE_IDS = [item["id"] for item in CALIBRATE_MODES]


def calibrate_modes():
    return [dict(item) for item in CALIBRATE_MODES]


# ── 小工具 ──────────────────────────────────────────────────────────────────
def _word(low, high):
    return int(low) | (int(high) << 8)


def _signed(raw, bit=15):
    return decode_signed_magnitude(int(raw), bit)


def _signed_bytes(value, bit=15):
    return encode_signed_magnitude(int(value), bit).to_bytes(2, "little")


class StepLog(object):
    """初始化过程的步骤清单，直接转成 JSON 给前端逐步显示。"""

    def __init__(self):
        self.items = []

    def add(self, name, ok, detail=""):
        self.items.append({"name": name, "ok": bool(ok), "detail": detail or ""})
        return ok

    def to_list(self):
        return [dict(item) for item in self.items]

    def failed(self):
        return [item for item in self.items if not item["ok"]]


class Provisioner(object):
    """把一颗新舵机（出厂 ID 1）改写成指定关节，并完成初始位置校准。

    整个过程都在调用方的串口连接上同步执行，阻塞式、不可重入：
    初始化期间不应该有别的线程或页面在打总线。
    """

    def __init__(self, bus):
        self.bus = bus

    # -- 基础读写：应答可能丢，所以一律用回读来判定结果 ──────────────────
    def _write(self, servo_id, addr, data):
        """写一个寄存器，返回 (是否收到应答, 说明)。

        应答状态级别（8 号）为 0 的舵机对写指令是静默的，总线上的偶发丢包也会
        表现为超时。这两种情况都不代表写失败，所以这里不抛异常，让调用方回读校验。
        """
        try:
            self.bus.write(servo_id, addr, data)
            return True, "已应答"
        except ProtocolError as exc:
            if exc.code in ("timeout", "checksum", "frame_length", "data_length", "data_length"):
                return False, "未收到应答（%s），改用回读校验" % exc.message
            raise

    def _read_byte(self, servo_id, addr, timeout=None):
        data = self.bus.read(servo_id, addr, 1, timeout=timeout).data
        return data[0]

    def _read_word_signed(self, servo_id, addr):
        data = self.bus.read(servo_id, addr, 2).data
        return _signed(_word(data[0], data[1]))

    def _write_check(self, log, name, servo_id, addr, data, expect_byte):
        """写一个单字节寄存器并回读校验。expect_byte 为 None 时只写不校验。"""
        acked, note = self._write(servo_id, addr, data)
        if expect_byte is None:
            return log.add(name, True, note)
        try:
            got = self._read_byte(servo_id, addr)
        except ProtocolError as exc:
            return log.add(name, False, "写入后回读失败：%s" % exc.message)
        ok = got == expect_byte
        detail = "回读 = %d（期望 %d）" % (got, expect_byte)
        if not acked:
            detail = note + "；" + detail
        return log.add(name, ok, detail)

    def _write_gain(self, servo_id, log, kp=POSITION_KP_DEFAULT,
                    kd=POSITION_KD_DEFAULT):
        """写位置环增益：EEPROM 21/22（掉电保存）+ RAM 50/51（立即生效）。

        只解锁一次、写完两个地址再上锁：两次 `_write_eeprom` 会在中间多做一次
        解锁/上锁，而这里两个地址是同一件事。之后 RAM 与 EEPROM 各回读一次校验。
        """
        kp = max(0, min(POSITION_GAIN_MAX, int(kp)))
        kd = max(0, min(POSITION_GAIN_MAX, int(kd)))
        self._write(servo_id, ADDR_LOCK, bytes([LOCK_EPROM_SAVED]))
        acked, note = self._write(
            servo_id, ADDR_POSITION_KP_EPROM, bytes([kp]))
        acked2, note2 = self._write(
            servo_id, ADDR_POSITION_KD_EPROM, bytes([kd]))
        time.sleep(EEPROM_SETTLE)
        self._write(servo_id, ADDR_LOCK, bytes([LOCK_EPROM_VOLATILE]))

        self._write_check(log, "写位置环 P（21 号 EPROM → 50 号 Kp）",
                          servo_id, ADDR_POSITION_KP_RAM, bytes([kp]), kp)
        self._write_check(log, "写位置环 D（22 号 EPROM → 51 号 Kd）",
                          servo_id, ADDR_POSITION_KD_RAM, bytes([kd]), kd)

        detail = "Kp=%d Kd=%d" % (kp, kd)
        if not (acked and acked2):
            detail += "（EPROM 写入未收到应答：%s / %s）" % (note, note2)
        detail += ("；robotd 启动后会按 robotd.toml 的 gain 覆盖 RAM 50，"
                   "两边保持一致才有意义")
        return log.add("写位置环增益", True, detail)

    def _write_unload_condition(self, servo_id, log, voltage=None, over_current=None):
        """写 19 号「卸载条件」的 BIT0（电压保护）与 BIT3（过流保护）。

        19 号是 EPROM 位域，所以按**读改写**处理：只动 BIT0/BIT3，
        BIT1（磁编码）与 BIT2（过热）保持舵机原值，避免"初始化顺便把厂商默认
        打开的保护关掉"这种事。``voltage`` / ``over_current`` 为 None 表示这一次
        不碰那一位，True/False 表示显式置位/清零。

        参数默认见 ``PROTECT_VOLTAGE_DEFAULT`` / ``PROTECT_OVER_CURRENT_DEFAULT``
        （都不打开）：触发卸载 = 关节突然松掉 = 机器人摔倒，所以默认不开。
        """
        targets = []
        if voltage is not None:
            targets.append((UNLOAD_BIT_VOLTAGE, bool(voltage), "电压保护"))
        if over_current is not None:
            targets.append((UNLOAD_BIT_OVER_CURRENT, bool(over_current), "过流保护"))
        if not targets:
            return log.add("写卸载条件（19 号）", True, "按设置跳过（保持舵机原值）")

        try:
            before = self._read_byte(servo_id, ADDR_UNLOAD_CONDITION)
        except ProtocolError as exc:
            return log.add("写卸载条件（19 号）", False,
                           "写入前回读失败：%s" % exc.message)

        value = before
        for bit, enabled, _label in targets:
            value = (value | (1 << bit)) if enabled else (value & ~(1 << bit))
        value &= 0xFF
        self._write_eeprom(servo_id, ADDR_UNLOAD_CONDITION, bytes([value]))
        try:
            after = self._read_byte(servo_id, ADDR_UNLOAD_CONDITION)
        except ProtocolError as exc:
            return log.add("写卸载条件（19 号）", False,
                           "写入后回读失败：%s" % exc.message)

        ok = after == value
        detail = ("19 号 %d (0x%02X) → %d (0x%02X)：%s"
                  % (before, before, after, after,
                     "、".join("%s（BIT%d）%s" % (label, bit, "打开" if enabled else "关闭")
                               for bit, enabled, label in targets)))
        if ok and after != before:
            detail += "；其余位保持原值"
        if not ok:
            detail += "（期望 %d (0x%02X)）" % (value, value)
        return log.add("写卸载条件（19 号）", ok, detail)

    def _write_eeprom(self, servo_id, addr, data):
        """按厂商 unLockEprom/LockEprom 的方式写一个 EPROM 寄存器。

        注意 55 号写 0 才是"关闭写入锁"（掉电保存），写 1 是"打开写入锁"。
        写完立刻上锁，避免中途断电留下一个不设防的舵机。
        """
        self._write(servo_id, ADDR_LOCK, bytes([LOCK_EPROM_SAVED]))
        acked, note = self._write(servo_id, addr, data)
        time.sleep(EEPROM_SETTLE)
        # 写主 ID 之后，这颗舵机只认新 ID 了：把锁再发给旧 ID 会石沉大海，
        # 既留下一个没上锁的舵机，又白等一个超时。所以按写入的值改地址
        # （microduck 的 duck-control/src/bus.rs::write_eeprom 也是这么做的）。
        relock_to = data[0] if addr == ADDR_ID and len(data) == 1 else servo_id
        self._write(relock_to, ADDR_LOCK, bytes([LOCK_EPROM_VOLATILE]))
        return acked, note

    # -- 读一个舵机的完整身份 ────────────────────────────────────────────
    def snapshot(self, servo_id, timeout=None):
        """读回初始化结果：版本、EPROM 关键项、模式、锁、15 字节反馈。"""
        info = {"id": int(servo_id)}
        version = self.bus.read(servo_id, 0, 5, timeout=timeout).data
        info.update({
            "firmware": "%d.%d" % (version[0], version[1]),
            "servo_version": "%d.%d" % (version[3], version[4]),
            "endian": version[2],
        })
        config = self.bus.read(servo_id, ADDR_ID, 4, timeout=timeout).data
        info.update({
            "id_read": config[0],
            "baud_code": config[1],
            "second_id": config[2],
            "response_level": config[3],
        })
        info["mode"] = self._read_byte(servo_id, ADDR_MODE, timeout=timeout)
        info["lock"] = self._read_byte(servo_id, ADDR_LOCK, timeout=timeout)
        info["min_angle_limit"] = self.bus.read_word(
            servo_id, ADDR_MIN_ANGLE_LIMIT, timeout=timeout
        )
        info["max_angle_limit"] = self.bus.read_word(
            servo_id, ADDR_MAX_ANGLE_LIMIT, timeout=timeout
        )
        info["multiturn"] = (info["min_angle_limit"] == 0
                             and info["max_angle_limit"] == 0)
        info["position_offset"] = self._read_word_signed(servo_id, ADDR_POSITION_OFFSET)
        info["torque"] = self._read_byte(servo_id, ADDR_TORQUE_ENABLE, timeout=timeout)
        info["unload_condition"] = self._read_byte(
            servo_id, ADDR_UNLOAD_CONDITION, timeout=timeout)
        info["protect_voltage"] = bool(info["unload_condition"] & (1 << UNLOAD_BIT_VOLTAGE))
        info["protect_over_current"] = bool(
            info["unload_condition"] & (1 << UNLOAD_BIT_OVER_CURRENT))

        feedback = self.bus.feedback(servo_id, timeout=timeout).data
        position = _signed(_word(feedback[0], feedback[1]))
        info.update({
            "feedback_hex": feedback.hex(" ").upper(),
            "position": position,
            "position_deg": round(position * DEG_PER_COUNT, 3),
            "speed": _signed(_word(feedback[2], feedback[3])),
            "load": _signed(_word(feedback[4], feedback[5]), 10),
            "voltage": round(feedback[6] * 0.1, 2),
            "temperature": feedback[7],
            "status": feedback[9],
            "moving": feedback[10],
            "target_position": _signed(_word(feedback[11], feedback[12])),
            "current_ma": round(_signed(_word(feedback[13], feedback[14])) * 6.5, 1),
        })
        return info

    # -- 预检 ────────────────────────────────────────────────────────────
    def precheck(self, target_id, source_id=FRESH_ID):
        """上电前检查：确认该改的舵机在、不该动的舵机不在。"""
        target = joint_by_id(target_id)
        source_id = int(source_id)
        result = {
            "target_id": int(target_id),
            "target": target,
            "source_id": source_id,
            "factory_id": FRESH_ID,
            "imu_bus_id": IMU_BUS_ID,
            "source_present": False,
            "target_present": False,
            "present_joints": [],
            "imu_present": False,
            "source_snapshot": None,
            "warnings": [],
        }

        def ping(sid):
            try:
                return self.bus.ping(sid) is not None
            except ProtocolError:
                return False

        for item in JOINTS:
            if ping(item["id"]):
                result["present_joints"].append(item["id"])
        result["source_present"] = ping(source_id)
        result["target_present"] = ping(int(target_id))
        result["imu_present"] = ping(IMU_BUS_ID)

        if result["source_present"]:
            try:
                result["source_snapshot"] = self.snapshot(source_id)
            except ProtocolError as exc:
                result["warnings"].append(
                    "ID %d 能应答 PING 但读不到参数：%s" % (source_id, exc.message)
                )

        if not result["source_present"]:
            result["warnings"].append(
                "总线上没有 ID %d 的应答。新舵机出厂就是 ID 1，请确认它已上电、"
                "波特率是 1 Mbps，或者先做一次 ID 扫描。" % source_id
            )
        if source_id == FRESH_ID and result["target_present"]:
            result["warnings"].append(
                "目标 ID %d 已经在应答了，说明这个关节已经编过号。"
                "如果确实要重新初始化/重新校准，请改用“重新初始化”模式。" % target_id
            )
        if source_id != FRESH_ID and not result["target_present"]:
            result["warnings"].append(
                "重新初始化模式要求 ID %d 在线，但它没有应答。" % source_id
            )
        if result["imu_present"]:
            result["warnings"].append(
                "ID %d（IMU 节点）也在这条总线上；初始化只动目标舵机，不受影响。" % IMU_BUS_ID
            )
        others = [i for i in result["present_joints"] if i != int(target_id)]
        if others:
            result["warnings"].append(
                "总线上已经有其它关节在线：%s。它们不会被改写，但请确认目标 ID 没有重复。"
                % ",".join(str(i) for i in others)
            )
        return result

    # -- 点检：15 个关节谁在、状态如何 ───────────────────────────────────
    def census(self):
        """逐个 PING 15 个关节 ID，在线的顺便读一份快照（缺哪个不影响其它）。"""
        items = []
        for item in JOINTS:
            row = {
                "index": item["index"],
                "name": item["name"],
                "id": item["id"],
                "group": item["group"],
                "present": False,
                "error": "",
                "snapshot": None,
            }
            try:
                self.bus.ping(item["id"])
                row["present"] = True
            except ProtocolError as exc:
                row["error"] = exc.message
                items.append(row)
                continue
            try:
                row["snapshot"] = self.snapshot(item["id"])
            except ProtocolError as exc:
                row["error"] = "在线但参数读取失败：%s" % exc.message
            items.append(row)
        present = [row for row in items if row["present"]]
        return {
            "items": items,
            "present_count": len(present),
            "total": len(JOINTS),
        }

    # -- 多圈绝对位置控制 ────────────────────────────────────────────────
    def _set_multiturn(self, servo_id, log, enable=True):
        """按需把 9/11 号角度限制写成「多圈(0/0)」或「单圈(0/4095)」。

        厂商内存表在 9/11 号上写得很清楚："多圈绝对位置控制时此值为 0"，
        而出厂 11 号是 4095。真机实测（HD-1910-C001 / 固件 3.46）：

        * 出厂 ``11 号 = 4095`` 时，写目标位置 −500 **协议层被接受**
          （42 号回读 −500），但**舵机不动**，67 号"目标位置回读"是 0，
          位置读数被夹在一圈内（负结果回绕成 ``4096 + 值``）；
        * 把 ``9 号 = 0、11 号 = 0`` 之后，同一个 −500 真的走到了：
          当前位置 = −500（0x81F4）、67 号回读 = −500。

        microduck 的 ``DEFAULT_POSITION`` 里有 4 个负角关节，所以整机上这一步
        是"能不能站对"的前提；但**台面单机调试默认不开**：多圈会取消固件行程
        限制，一圈内更安全。

        这个开关两个方向都**真的写寄存器**：``enable=False`` 不是"跳过"，
        而是把 9/11 号写回单圈行程（0/4095）并回读校验——否则对一台已经开过多圈
        的舵机，取消勾选毫无作用（这正是旧版"选项不起作用"的原因）。
        """
        low_want, high_want = LIMITS_MULTITURN if enable else LIMITS_SINGLE_TURN
        name = ("打开多圈位置控制（9/11 号写 0）" if enable
                else "关闭多圈，恢复单圈行程（9/11 号写 0/4095）")
        acked = []
        for addr, value in ((ADDR_MIN_ANGLE_LIMIT, low_want),
                            (ADDR_MAX_ANGLE_LIMIT, high_want)):
            self._write(servo_id, ADDR_LOCK, bytes([LOCK_EPROM_SAVED]))
            ack, note = self._write(servo_id, addr, int(value).to_bytes(2, "little"))
            time.sleep(EEPROM_SETTLE)
            self._write(servo_id, ADDR_LOCK, bytes([LOCK_EPROM_VOLATILE]))
            acked.append(ack)
        try:
            low = self.bus.read_word(servo_id, ADDR_MIN_ANGLE_LIMIT)
            high = self.bus.read_word(servo_id, ADDR_MAX_ANGLE_LIMIT)
        except ProtocolError as exc:
            return False, log.add(name, False, exc.message)
        ok = low == low_want and high == high_want
        detail = ("回读 9 号 = %d、11 号 = %d（期望 %d/%d：%s）"
                  % (low, high, low_want, high_want,
                     "取消固件行程限制，负关节角可用" if enable
                     else "行程锁在一圈内，负目标会被夹到 0"))
        if not all(acked):
            detail += "；有一次写入没收到应答，但回读是对的"
        return ok, log.add(name, ok, detail)

    # -- 校准 ────────────────────────────────────────────────────────────
    def _read_position(self, servo_id):
        return self._read_word_signed(servo_id, ADDR_PRESENT_POSITION)

    def _read_offset(self, servo_id):
        return self._read_word_signed(servo_id, ADDR_POSITION_OFFSET)

    def _calibrate_cal(self, servo_id, log):
        """厂商 CAL 指令：把当前位置写成位置偏移中点。

        实测（固件 3.46）：校准后位置读数是 **2048 计数（180°）**，不是 0；
        31 号位置偏移被改写成 ``编码器值 − 2048``。所以这一步同时把偏移和读数变化
        都记进步骤明细，使用者能直接看到 CAL 到底做了什么，而不是"应该变成 0"。
        """
        before = self._read_position(servo_id)
        offset_before = self._read_offset(servo_id)
        try:
            frame = self.bus.recal(servo_id)
        except ProtocolError as exc:
            return False, log.add("中位校准 CAL(0x0B)", False, exc.message), before, None
        time.sleep(0.05)
        after = self._read_position(servo_id)
        offset_after = self._read_offset(servo_id)
        ok = log.add(
            "中位校准 CAL(0x0B)",
            True,
            "位置 %d → %d 计数，位置偏移 %d → %d（STATUS=%d）。"
            "实测校准后读数为 2048 计数（180°）"
            % (before, after, offset_before, offset_after, frame.status),
        )
        return ok, ok, before, after

    @staticmethod
    def _position_error(got, want, multiturn):
        """读数与目标的偏差。

        多圈模式下位置是真正的有符号绝对值，差一整圈就是差一整圈，必须精确比较；
        没打开多圈时位置被固件夹在一圈内，负结果会回绕，所以按一圈取最短距离。
        """
        if multiturn:
            return abs(int(got) - int(want))
        return _wrap_delta(got, want)

    def _calibrate_offset(self, servo_id, target_counts, log, multiturn=True):
        """把 31 号位置偏移调到“此刻读数 = target_counts”。

        真机实测到的事实（HD-1910-C001 / 固件 3.46）：

        * 位置与偏移相差一个常量：``位置 = 编码器值 − 偏移``（读数为负时，
          一圈内模式还会把结果回绕成 ``4096 + 值``）；
        * **写一个较大的偏移会让固件把编码器坐标整体重锚**：
          实测偏移 −3591→−3091（+500）之后读数从 −4096 变成 −500（差 3596），
          而 −3591→−4591（−1000）之后读数从 −4096 变成 −3096（差 1000）——
          同一台舵机、同一根轴，写偏移前后的差值不是同一个常量；
          读数还总被折进约 ±2 圈的窗口。
        * 手册没写这些（`docs/飞特通讯协议说明.md` 第 12.5 节把它列为未确认项），
          所以**经验规律是"所需偏移越小越可靠"**，公式覆盖不了重锚。

        因此这里不做公式假设，改成有界搜索 + 逐个回读验证：

        1. 由 ``位置 = 编码器 − 偏移`` 反推编码器值，算出"正/反两种约定"下
           各自需要的偏移，再把每个候选按 ±一整圈展开（覆盖重锚）；
        2. 按"离当前偏移最近"排序——偏移动得越小，越不容易触发重锚；
        3. 逐个写下去、读回来，**只有落在容差内才算成功**；
        4. 全都不行就**把原偏移写回去**，绝不留一个半校准的舵机。

        最终判据永远只有回读结果，而不是任何一条公式。
        """
        want = int(target_counts)
        ofs0 = self._read_offset(servo_id)
        pos0 = self._read_position(servo_id)
        raw = pos0 + ofs0

        candidates = []
        for base in (raw - want, want - raw):
            for shift in (0, -POSITION_WRAP, POSITION_WRAP, -2 * POSITION_WRAP, 2 * POSITION_WRAP):
                value = base + shift
                if abs(value) <= OFFSET_LIMIT and value not in candidates:
                    candidates.append(value)
        candidates.sort(key=lambda value: abs(value - ofs0))

        tried = []
        best = None
        for value in candidates[:6]:
            self._write_eeprom(servo_id, ADDR_POSITION_OFFSET, _signed_bytes(value))
            time.sleep(0.05)
            pos = self._read_position(servo_id)
            error = self._position_error(pos, want, multiturn)
            tried.append("偏移 %d → 读数 %d（差 %d）" % (value, pos, error))
            if best is None or error < best[2]:
                best = (value, pos, error)
            if error <= POSITION_TOLERANCE:
                return True, log.add(
                    "按 home 姿态写位置偏移",
                    True,
                    "位置偏移 %d → %d；位置读数 %d → %d（目标 %d 计数 = %.3f°，"
                    "误差 %d 计数 ≈ %.2f°）"
                    % (ofs0, value, pos0, pos, want, want * DEG_PER_COUNT,
                       error, error * DEG_PER_COUNT),
                )

        # 没成功：把原偏移写回去，别留一个半校准状态。
        self._write_eeprom(servo_id, ADDR_POSITION_OFFSET, _signed_bytes(ofs0))
        time.sleep(0.05)
        restored = self._read_offset(servo_id)
        detail = ("没能把读数调到 %d 计数（当前读数 %d、当前偏移 %d；试过 %d 个候选都没进 "
                  "%d 计数容差）：%s。已把位置偏移恢复为 %d。"
                  "通常是关节没摆到 home 姿态，或需要把舵盘换个齿位让偏移落在 ±%d 内。"
                  % (want, pos0, ofs0, len(tried), POSITION_TOLERANCE,
                     "；".join(tried), restored, OFFSET_LIMIT))
        return False, log.add("按 home 姿态写位置偏移", False, detail)

    def _calibrate_mid(self, servo_id, log, speed=60, acc=30, timeout=4.0):
        """写目标位置 0，等它到位，然后保持扭矩以便装舵盘。"""
        self._write(servo_id, ADDR_MODE, bytes([0]))
        self._write(servo_id, ADDR_TORQUE_ENABLE, b"\x01")
        try:
            self.bus.write(servo_id, ADDR_ACC, bytes([acc]))
        except ProtocolError:
            pass
        try:
            self.bus.write(servo_id, ADDR_GOAL_SPEED, int(speed).to_bytes(2, "little"))
        except ProtocolError:
            pass
        try:
            self.bus.write(servo_id, ADDR_GOAL_POSITION, _signed_bytes(0))
        except ProtocolError as exc:
            return False, log.add("转到中位（目标位置 0）", False, exc.message)

        deadline = time.monotonic() + max(0.5, float(timeout))
        position = self._read_position(servo_id)
        while time.monotonic() < deadline:
            position = self._read_position(servo_id)
            # 位置读数按一圈回绕，0 与 4095 是相邻的两个点。
            if min(position % POSITION_WRAP, POSITION_WRAP - position % POSITION_WRAP) <= 8:
                break
            time.sleep(0.1)
        error = min(position % POSITION_WRAP, POSITION_WRAP - position % POSITION_WRAP)
        ok = error <= 8
        return ok, log.add(
            "转到中位（目标位置 0）",
            ok,
            "当前读数 = %d 计数（误差 %d 计数 ≈ %.2f°）；扭矩保持开启便于装舵盘"
            % (position, error, error * DEG_PER_COUNT),
        )

    # -- 主流程 ──────────────────────────────────────────────────────────
    def provision(self, target_id, calibrate="cal", write_response_level=True,
                  multiturn=False, mode="fresh", speed=60, acc=30, timeout=None,
                  gain_kp=POSITION_KP_DEFAULT, gain_kd=POSITION_KD_DEFAULT,
                  write_gain=True,
                  protect_voltage=PROTECT_VOLTAGE_DEFAULT,
                  protect_over_current=PROTECT_OVER_CURRENT_DEFAULT):
        """把一颗舵机初始化成 target_id 对应的关节。

        mode：
          * ``"fresh"`` —— 源是出厂 ID 1 的新舵机（默认）；
          * ``"reinit"`` —— 源就是目标 ID 本身，用于重新编号 / 重新校准。

        gain_kp / gain_kd：写进位置环 RAM 50/51 与 EEPROM 21/22 的增益，
        默认是厂商默认的 32 / 0。**不要用 robotd 那份给 XL330 调过的 200**
        （实测 200 会让关节自激振荡、5 A、74 °C，见本模块顶部的常数说明）。

        multiturn：**默认 False**（单圈行程 9/11 = 0/4095）。
        整机装配时 microduck 需要打开多圈——15 个关节里有 4 个的 home 姿态角
        是负的，出厂 ``11 号 = 4095`` 会把负目标夹到 0（真机实测，见
        `_set_multiturn`）；但台面上单机调试时一圈内更安全，所以默认不打开。
        **两个方向都会真的写寄存器**：False 是把 9/11 号写回 0/4095，不是跳过。

        protect_voltage / protect_over_current：19 号「卸载条件」的 BIT0/BIT3，
        默认都关闭（触发卸载 = 关节松掉 = 摔倒）。按读改写处理，BIT1/BIT2 不动。
        """
        target = joint_by_id(target_id)
        if target is None:
            raise ValueError("ID %s 不是 microduck 的关节 ID" % target_id)
        calibrate = str(calibrate or "cal")
        if calibrate not in CALIBRATE_IDS:
            raise ValueError("校准方式必须是 %s 之一" % "/".join(CALIBRATE_IDS))
        mode = str(mode or "fresh")
        if mode not in ("fresh", "reinit"):
            raise ValueError("mode 必须是 fresh 或 reinit")

        target_id = int(target_id)
        source_id = FRESH_ID if mode == "fresh" else target_id
        multiturn = bool(multiturn)
        protect_voltage = bool(protect_voltage)
        protect_over_current = bool(protect_over_current)
        log = StepLog()
        result = {
            "target": target,
            "mode": mode,
            "calibrate": calibrate,
            "source_id": source_id,
            "multiturn": multiturn,
            "protect": {
                "voltage": protect_voltage,
                "over_current": protect_over_current,
            },
            "steps": [],
            "ok": False,
            "error": "",
            "snapshot": None,
            "calibration": {},
        }

        def finish(ok, error=""):
            result["ok"] = ok
            result["error"] = error
            result["steps"] = log.to_list()
            return result

        # 1. 预检
        check = self.precheck(target_id, source_id)
        result["precheck"] = check
        if not check["source_present"]:
            return finish(False, "ID %d 没有应答，无法开始。" % source_id)
        if mode == "fresh" and check["target_present"]:
            return finish(
                False,
                "ID %d 已经在应答了：这个关节已经编过号。请改用“重新初始化”模式，"
                "或换一颗还没编号的舵机。" % target_id,
            )
        log.add("预检", True, "ID %d 在线；目标 ID %d %s"
                % (source_id, target_id,
                   "已在线（重新初始化）" if check["target_present"] else "未占用"))

        # 2. 关闭扭矩。厂商的 unLockEprom 也是先关扭矩再解锁。
        self._write(source_id, ADDR_TORQUE_ENABLE, b"\x00")
        log.add("关闭扭矩", True, "写 40 号 = 0")

        # 3. 写主 ID（EPROM）：写完之后舵机就开始用新 ID 应答。
        if mode == "fresh":
            acked, note = self._write_eeprom(source_id, ADDR_ID, bytes([target_id]))
            try:
                got = self._read_byte(target_id, ADDR_ID)
            except ProtocolError as exc:
                return finish(False, "写入 ID %d 之后没有应答：%s" % (target_id, exc.message))
            if got != target_id:
                return finish(False, "写入 ID 后发现 %d 号应答返回的 ID 是 %d" % (target_id, got))
            detail = "ID %d → %d" % (source_id, target_id)
            if not acked:
                detail += "（%s）" % note
            log.add("写主 ID（5 号，EPROM）", True, detail)
        else:
            got = self._read_byte(source_id, ADDR_ID)
            if got != target_id:
                return finish(False, "重新初始化要求 ID %d，但它报的 ID 是 %d" % (target_id, got))
            log.add("确认主 ID（5 号）", True, "已是 %d，无需改写" % target_id)

        # 4. 波特率：新舵机出厂就是 1 Mbps，这一步通常是原值回写。
        acked, note = self._write_eeprom(target_id, ADDR_BAUD_RATE, bytes([EXPECTED_BAUD_CODE]))
        try:
            baud = self._read_byte(target_id, ADDR_BAUD_RATE)
        except ProtocolError as exc:
            return finish(False, "写入波特率后回读失败：%s" % exc.message)
        ok = baud == EXPECTED_BAUD_CODE
        if not ok:
            return finish(
                False,
                "波特率回读为 %d，期望 %d。若它真的被改成别的速率，这台舵机需要在"
                "新速率下重新连接才能继续。" % (baud, EXPECTED_BAUD_CODE),
            )
        log.add("写波特率（6 号，EPROM）", True,
                "编码 = %d（1 Mbps）%s" % (baud, "" if acked else "；" + note))

        # 5. 应答状态级别：写 1，保证后续所有指令都有应答（microduck 的
        #    bus.rs 靠应答判断写入是否成功）。
        if write_response_level:
            acked, note = self._write_eeprom(
                target_id, ADDR_RESPONSE_LEVEL, b"\x01"
            )
            try:
                level = self._read_byte(target_id, ADDR_RESPONSE_LEVEL)
            except ProtocolError as exc:
                return finish(False, "写入应答状态级别后回读失败：%s" % exc.message)
            log.add("写应答状态级别（8 号，EPROM）", level == 1,
                    "回读 = %d（期望 1）%s" % (level, "" if acked else "；" + note))

        # 6. 9/11 号角度限制：多圈(0/0) 或 单圈(0/4095)，两个方向都真的写。
        ok, _step = self._set_multiturn(target_id, log, enable=multiturn)
        if not ok:
            if multiturn:
                return finish(
                    False,
                    "打开多圈位置控制失败：9/11 号角度限制没能写成 0，"
                    "负目标位置会被夹到 0，负角关节无法到位。",
                )
            return finish(
                False,
                "恢复单圈行程失败：9/11 号角度限制没能写成 0/4095。",
            )

        # 7. 19 号卸载条件：电压保护 / 过流保护（默认都关闭，读改写只动 BIT0/BIT3）
        self._write_unload_condition(
            target_id, log,
            voltage=protect_voltage, over_current=protect_over_current,
        )

        # 8. 位置环增益：EEPROM 21/22 + RAM 50/51
        result["gain"] = {"kp": int(gain_kp), "kd": int(gain_kd)}
        if write_gain:
            self._write_gain(target_id, log, kp=gain_kp, kd=gain_kd)
        else:
            log.add("写位置环增益", True, "按设置跳过（保持舵机原值）")

        # 9. 初始位置校准
        if calibrate == "cal":
            self._write(target_id, ADDR_LOCK, bytes([LOCK_EPROM_SAVED]))
            ok, _step, before, after = self._calibrate_cal(target_id, log)
            self._write(target_id, ADDR_LOCK, bytes([LOCK_EPROM_VOLATILE]))
            result["calibration"] = {"mode": "cal", "before": before, "after": after}
            if not ok:
                return finish(False, "中位校准失败：" + log.items[-1]["detail"])
        elif calibrate == "offset":
            ofs_before = self._read_offset(target_id)
            pos_before = self._read_position(target_id)
            ok, _step = self._calibrate_offset(
                target_id, target["home_counts"], log, multiturn=multiturn
            )
            result["calibration"] = {
                "mode": "offset",
                "target_counts": target["home_counts"],
                "target_wrapped": target["home_counts"] % POSITION_WRAP,
                "target_deg": target["home_deg"],
                "offset_before": ofs_before,
                "position_before": pos_before,
                "position_after": self._read_position(target_id),
                "position_offset": self._read_offset(target_id),
            }
            if not ok:
                return finish(False, "位置偏移校准未达到目标，请检查机械姿态。")
        elif calibrate == "mid":
            ok, _step = self._calibrate_mid(target_id, log, speed=speed, acc=acc)
            result["calibration"] = {"mode": "mid", "position": self._read_position(target_id)}
            if not ok:
                return finish(False, "转向中位没有到位，请检查舵机是否被卡住。")
        else:
            log.add("初始位置校准", True, "按设置跳过（只改 ID）")

        # 10. 打开写入锁（写 1），保护 EPROM 不再被误写。
        self._write(target_id, ADDR_LOCK, bytes([LOCK_EPROM_VOLATILE]))
        try:
            lock = self._read_byte(target_id, ADDR_LOCK)
        except ProtocolError as exc:
            return finish(False, "回读锁标志失败：%s" % exc.message)
        log.add("打开写入锁（55 号写 1）", lock == LOCK_EPROM_VOLATILE,
                "回读 = %d（1 = EPROM 掉电不保存）" % lock)

        # 11. 结束前把扭矩关掉，除非是"转到中位"需要它保持住以便装舵盘。
        if calibrate == "mid":
            log.add("扭矩", True, "保持开启：输出轴停在 0 点，方便按标记装舵盘")
        else:
            self._write(target_id, ADDR_TORQUE_ENABLE, b"\x00")
            try:
                torque = self._read_byte(target_id, ADDR_TORQUE_ENABLE)
            except ProtocolError as exc:
                return finish(False, "回读扭矩失败：%s" % exc.message)
            log.add("关闭扭矩", torque == 0, "回读 40 号 = %d（期望 0）" % torque)

        # 10. 回读校验
        try:
            result["snapshot"] = self.snapshot(target_id, timeout=timeout)
        except ProtocolError as exc:
            return finish(False, "回读校验失败：%s" % exc.message)
        snap = result["snapshot"]
        checks = [
            ("ID", snap["id_read"] == target_id, snap["id_read"]),
            ("波特率编码", snap["baud_code"] == EXPECTED_BAUD_CODE, snap["baud_code"]),
            ("锁标志", snap["lock"] == LOCK_EPROM_VOLATILE, snap["lock"]),
            ("舵机状态", snap["status"] == 0, snap["status"]),
        ]
        if calibrate != "mid":
            checks.append(("扭矩", snap["torque"] == 0, snap["torque"]))
        if multiturn:
            checks.append((
                "多圈位置控制",
                snap["min_angle_limit"] == 0 and snap["max_angle_limit"] == 0,
                "9/11 号 = %d/%d" % (snap["min_angle_limit"], snap["max_angle_limit"]),
            ))
        else:
            checks.append((
                "单圈行程",
                snap["min_angle_limit"] == LIMITS_SINGLE_TURN[0]
                and snap["max_angle_limit"] == LIMITS_SINGLE_TURN[1],
                "9/11 号 = %d/%d" % (snap["min_angle_limit"], snap["max_angle_limit"]),
            ))
        checks.append((
            "卸载条件",
            snap["protect_voltage"] == protect_voltage
            and snap["protect_over_current"] == protect_over_current,
            "19 号 = %d（电压保护 %s、过流保护 %s）"
            % (snap["unload_condition"],
               "开" if snap["protect_voltage"] else "关",
               "开" if snap["protect_over_current"] else "关"),
        ))
        verified = all(item[1] for item in checks)
        log.add(
            "回读校验",
            verified,
            "；".join("%s = %s%s" % (name, value, "" if good else "（不符）")
                     for name, good, value in checks),
        )
        if snap["status"] != 0:
            return finish(False, "舵机状态字节非 0（0x%02X），上电后可能不输出扭矩。" % snap["status"])
        if not verified:
            bad = "、".join(name for name, good, _ in checks if not good)
            return finish(False, "回读校验未通过：%s。请断电重来，或先用“寄存器/原始指令”页核对。" % bad)
        return finish(True)


def main():
    """离线打印关节表，方便与 model.rs 对照。"""
    print("microduck 关节表（%d 台）" % len(JOINTS))
    print("%-4s %-18s %-6s %-10s %-11s %s" % ("序", "关节名", "ID", "部位", "home(°)", "home(计数)"))
    for item in JOINTS:
        print("%-4d %-18s %-6d %-10s %-11.3f %d"
              % (item["index"] + 1, item["name"], item["id"], item["group"],
                 item["home_deg"], item["home_counts"]))
    print("出厂 ID = %d，IMU 总线 ID = %d，期望波特率编码 = %d"
          % (FRESH_ID, IMU_BUS_ID, EXPECTED_BAUD_CODE))


if __name__ == "__main__":
    main()
