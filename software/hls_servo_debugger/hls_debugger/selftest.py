# -*- coding: utf-8 -*-
"""不依赖真实硬件的协议自检。

运行：
    .venv/bin/python -m hls_debugger.selftest
"""

from __future__ import absolute_import

import os
import re
import subprocess
import sys
import time

from . import provision
from .memory_table import (
    decode_range,
    decode_signed_magnitude,
    encode_signed_magnitude,
    encode_value,
    feedback_to_dict,
    find_field,
)
from .protocol import HLSBus, build_frame, INST_PING, INST_READ, INST_WRITE


class FakeSerial(object):
    def __init__(self, data=b""):
        self.data = bytearray(data)
        self.writes = []

    def read(self, size):
        if not self.data:
            return b""
        out = bytes(self.data[:size])
        del self.data[:size]
        return out

    def write(self, data):
        self.writes.append(bytes(data))

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass


def response(sid, data=b"", status=0):
    length = len(data) + 2
    checksum = (~(sid + length + status + sum(data))) & 0xFF
    return b"\xff\xff" + bytes([sid, length, status]) + bytes(data) + bytes([checksum])


def test_build_frames():
    assert build_frame(200, INST_PING).hex().upper() == "FFFFC8020134"
    assert build_frame(200, INST_READ, 56, b"\x0c").hex().upper() == "FFFFC80402380CED"
    assert build_frame(200, INST_WRITE, 5, b"\x2a").hex().upper() == "FFFFC80403052A01"


def test_transaction():
    bus = HLSBus()
    bus._serial = FakeSerial(response(1))
    bus.connected = True
    bus.timeout = 0.2
    frame = bus.ping(1)
    assert frame.id == 1 and frame.status == 0 and frame.data == b""
    assert bus._serial.writes[-1].hex().upper() == "FFFF010201FB"


def test_read_and_sync_read():
    bus = HLSBus()
    bus._serial = FakeSerial(response(1, b"\x34\x12"))
    bus.connected = True
    bus.timeout = 0.2
    assert bus.read(1, 56, 2).data == b"\x34\x12"

    bus._serial = FakeSerial(response(2, b"\x22\x02") + response(1, b"\x11\x01"))
    bus.connected = True
    result = bus.sync_read([1, 2], 56, 2)
    assert result[1].data == b"\x11\x01"
    assert result[2].data == b"\x22\x02"


def test_memory_table():
    field = find_field(42)
    for value in (0, 4095, -4095, 32767, -32767):
        raw = encode_value(field, value)
        assert int.from_bytes(raw, "little") is not None
    data = bytes.fromhex("02 00 00 00")
    fields = decode_range(0, data)
    assert fields[0]["name"] == "固件主版本号"
    assert fields[0]["value"] == 2


def test_feedback_decode():
    # 位置 0x1234, 速度 0x0010, 负载 0x0002, 电压 0x78, 温度 0x1e,
    # async 0, status 0, moving 0, 目标位置 0x1234, 电流 0x0005
    data = bytes.fromhex("34 12 10 00 02 00 78 1e 00 00 00 34 12 05 00")
    fb = feedback_to_dict(data)
    assert fb["position"]["raw"] == 0x1234
    assert fb["current"]["mA"] == 32.5


# ── 整机初始化：从站模拟 + 流程自检 ─────────────────────────────────────────

class FakeServoSerial(object):
    """一个够用的 HLS 从站模拟：只在内存里维护寄存器，用来离线跑初始化流程。

    支持 PING / READ / WRITE / CAL；写入 5 号寄存器时整台舵机改到新 ID 上应答，
    和真机行为一致。

    位置模型默认是**真机实测的约定** ``读数 = 编码器原始值 − 位置偏移(31)``
    （HD-1910-C001 / 固件 3.46；CAL 之后 偏移 −1976、读数 2048）。
    ``invert_convention=True`` 可以换成相反的约定，
    用来验证 provision 在没有把握时会自己回读并改试另一种方向。
    """

    def __init__(self, ids, raw_position=1234, invert_convention=False):
        self.invert_convention = bool(invert_convention)
        self.servos = {}
        for sid in ids:
            sid = int(sid)
            self.servos[sid] = {
                "id": sid,
                "raw_position": int(raw_position),
                "offset": 0,
                # 出厂的角度限制：9 号 = 0、11 号 = 4095（行程锁在一圈内）。
                "min_limit": 0,
                "max_limit": 4095,
                "regs": {0: 3, 1: 46, 2: 0, 3: 10, 4: 31, 5: sid,
                         6: 0, 7: 253, 8: 1, 33: 4, 40: 0, 41: 0,
                         # 相位 18 号。**真机出厂是 0x34**（bit 4 角度反馈 = 全角度、
                         # bit 2 驱动桥方向、bit 5 集成 H 桥，15 个舵机一致，2026-10-05
                         # 实测），此时位置读数不会自己回绕。这里默认 0（单圈反馈）
                         # 是因为下面那些记录的用例（CAL 之后读数 2048 等）是在"读数会被
                         # 折进一圈"的前提下量出来的；要测真机那一档，用例里显式写
                         # `servo["regs"][18] = 0x34`，见
                         # test_provision_rewinds_an_out_of_turn_reading_without_moving_the_joint。
                         18: 0,
                         # 出厂 EPROM 位置环 P/D = 32/40，上电加载进 RAM 50/51
                         # （真机回读确认，见 provision.POSITION_KP_DEFAULT）。
                         21: 32, 22: 40, 50: 32, 51: 40,
                         48: 1000, 55: 1, 65: 0, 66: 0},
            }
        self.data = bytearray()
        self.writes = []

    # -- 串口接口 ------------------------------------------------------
    def read(self, size):
        if not self.data:
            return b""
        out = bytes(self.data[:size])
        del self.data[:size]
        return out

    def write(self, frame):
        frame = bytes(frame)
        self.writes.append(frame)
        if len(frame) < 6 or frame[0] != 0xFF or frame[1] != 0xFF:
            return
        sid, inst = frame[2], frame[4]
        servo = self.servos.get(sid)
        if inst == 0x01:  # PING
            if servo is not None:
                self.data += self._ack(sid, b"")
            return
        if servo is None:
            return
        if inst == 0x02:  # READ
            addr, count = frame[5], frame[6]
            payload = bytes(self._read(servo, addr + i) for i in range(count))
            self.data += self._ack(sid, payload)
        elif inst == 0x03:  # WRITE
            self._write(servo, frame[5], frame[6:-1])
            self.data += self._ack(sid, b"")
        elif inst == 0x0B:  # CAL：把当前位置写成单圈中点 2048
            servo["offset"] = self._offset_for(servo, 2048)
            self.data += self._ack(sid, b"")

    def flush(self):
        pass

    def reset_input_buffer(self):
        pass

    # -- 内部 ----------------------------------------------------------
    @staticmethod
    def _ack(sid, data):
        length = len(data) + 2
        checksum = (~(sid + length + 0 + sum(data))) & 0xFF
        return b"\xff\xff" + bytes([sid, length, 0]) + bytes(data) + bytes([checksum])

    def _multiturn(self, servo):
        """9/11 号都为 0 = 多圈绝对位置控制；否则行程被夹在一圈内。"""
        return servo["min_limit"] == 0 and servo["max_limit"] == 0

    def _full_angle_feedback(self, servo):
        """18 号相位 bit 4：角度反馈模式（0 单圈 / 1 全角度）。

        真机实测（2026-10-05，HD-1910-C001 / 固件 3.46）：**出厂 18 号 = 0x34，
        bit 4 = 1，15 个舵机完全一致**。这一位打开时位置读数**不会自己回绕**，
        即使 9/11 号是单圈的 0/4095：关节被手推过一整圈之后 56 号会读到 4096 以上
        （实测 ``head_yaw`` = **6289**）。robotd 的驱动层只读单圈字段，超出的读数
        会被取模屏蔽 —— 看上去"到位"，其实差着一整圈，下一次写目标就把关节转回去。
        """
        return bool(servo["regs"].get(18, 0) & 0x10)

    def _position(self, servo):
        if self.invert_convention:
            delta = servo["raw_position"] + servo["offset"]
        else:
            delta = servo["raw_position"] - servo["offset"]
        if self._multiturn(servo) or self._full_angle_feedback(servo):
            return delta           # 多圈 / 全角度反馈：有符号绝对值，可超出 0~4095
        return delta % 4096        # 一圈内：固件把负结果回绕成 0~4095

    def _offset_for(self, servo, position):
        """让 _position() 返回 position 所需的 31 号偏移。"""
        if self.invert_convention:
            base = position - servo["raw_position"]
        else:
            base = servo["raw_position"] - position
        return base if self._multiturn(servo) else base % 4096

    def _raw_for(self, servo, position):
        """让 _position() 返回 position 所需的编码器原始值（装成"舵机转到位"）。"""
        if self.invert_convention:
            base = position - servo["offset"]
        else:
            base = position + servo["offset"]
        return base if self._multiturn(servo) else base % 4096

    def _read(self, servo, addr):
        if addr in (56, 57):
            raw = encode_signed_magnitude(self._position(servo), 15)
            return (raw >> (8 * (addr - 56))) & 0xFF
        if addr in (31, 32):
            raw = encode_signed_magnitude(servo["offset"], 15)
            return (raw >> (8 * (addr - 31))) & 0xFF
        if addr in (9, 10):
            return (servo["min_limit"] >> (8 * (addr - 9))) & 0xFF
        if addr in (11, 12):
            return (servo["max_limit"] >> (8 * (addr - 11))) & 0xFF
        if addr in (67, 68):
            raw = encode_signed_magnitude(self._position(servo), 15)
            return (raw >> (8 * (addr - 67))) & 0xFF
        if addr == 62:
            return 74  # 7.4 V
        if addr == 63:
            return 30
        return servo["regs"].get(addr, 0)

    def _write(self, servo, addr, payload):
        if addr == 5 and len(payload) == 1:
            new_id = payload[0]
            self.servos.pop(servo["id"], None)
            servo["id"] = new_id
            servo["regs"][5] = new_id
            self.servos[new_id] = servo
            return
        if addr in (9, 10):
            servo["min_limit"] = payload[0] | (payload[1] << 8)
            return
        if addr in (11, 12):
            servo["max_limit"] = payload[0] | (payload[1] << 8)
            return
        if addr in (31, 32) and len(payload) == 2:
            servo["offset"] = decode_signed_magnitude(
                payload[0] | (payload[1] << 8), 15
            )
            return
        if addr == 56 and len(payload) == 2:
            position = decode_signed_magnitude(payload[0] | (payload[1] << 8), 15)
            servo["raw_position"] = self._raw_for(servo, position)
            return
        if addr in (42, 43) and len(payload) == 2:
            # 写目标位置：直接当作到位，方便测 "转到位置 0"。
            target = decode_signed_magnitude(payload[0] | (payload[1] << 8), 15)
            servo["raw_position"] = self._raw_for(servo, target)
            return
        servo["regs"][addr] = payload[0]


def fake_bus(ids, raw_position=1234, invert_convention=False):
    bus = HLSBus()
    bus._serial = FakeServoSerial(
        ids, raw_position=raw_position, invert_convention=invert_convention
    )
    bus.connected = True
    bus.timeout = 0.02
    return bus


def _rust_source(name):
    """仓库里那份 Rust 源码，找不到就返回 None。

    本工具也会被单独拷到机器鸭核心板上跑（那里没有 Rust 树），所以与源码对拍的那两条
    用例必须在缺文件时**跳过**而不是失败——否则板子上的 selftest 永远过不了。
    """
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.abspath(os.path.join(here, "..", "..", ".."))
    for relative in (
        os.path.join("software", "microduck_feetech", "duck-control", "src", name),
        os.path.join("duck-control", "src", name),
    ):
        path = os.path.join(root, relative)
        if os.path.isfile(path):
            return path
    return None


def test_position_direction_matches_the_driver():
    """`POSITION_DIRECTION` / `POSITION_CENTER` 必须与 duck-control 的驱动层一致。

    这是本工具唯一一个能"静默地把 15 个关节全对到镜像位置"的常量，所以它不能只靠注释。
    读的是源码文本（`feetech.rs` 的 `pub const`），和 `cargo test` 无关。
    """
    path = _rust_source("feetech.rs")
    if path is None:
        print("  (跳过：找不到 feetech.rs，本机只有调试工具)")
        return
    with open(path, "r", encoding="utf-8") as handle:
        text = handle.read()

    def const(name):
        match = re.search(
            r"pub const %s\s*:\s*[A-Za-z0-9_]+\s*=\s*(-?[0-9]+(?:\.[0-9]+)?)" % name, text)
        assert match, "feetech.rs 里找不到 %s" % name
        return float(match.group(1))

    assert const("POSITION_CENTER") == provision.POSITION_CENTER, (
        "调试工具的 POSITION_CENTER 与 feetech.rs 不一致")
    assert const("POSITION_DIRECTION") == provision.POSITION_DIRECTION, (
        "调试工具的 POSITION_DIRECTION 与 feetech.rs 不一致："
        "drift 会让关节表整列镜像（见 provision.py 顶部）")


def test_joint_table_matches_microduck():
    """关节表必须和 microduck 的 duck-control/src/model.rs 完全一致。"""
    expected_ids = [20, 21, 22, 23, 24, 30, 31, 32, 33, 34, 10, 11, 12, 13, 14]
    expected_names = [
        "left_hip_yaw", "left_hip_roll", "left_hip_pitch", "left_knee", "left_ankle",
        "neck_pitch", "head_pitch", "head_yaw", "head_roll", "mouth",
        "right_hip_yaw", "right_hip_roll", "right_hip_pitch", "right_knee", "right_ankle",
    ]
    assert provision.JOINT_IDS == expected_ids
    assert [item["name"] for item in provision.JOINTS] == expected_names
    assert len(provision.JOINTS) == len(set(provision.JOINT_IDS)) == 15
    # ID 1 和 200 都不属于任何关节，这正是"ID 1 = 新舵机"成立的前提。
    assert provision.FRESH_ID == 1
    assert provision.FRESH_ID not in provision.JOINT_IDS
    assert provision.IMU_BUS_ID not in provision.JOINT_IDS
    # home 姿态角与 model.rs 的 DEFAULT_POSITION 一致（弧度 -> 计数，4096/圈）。
    home_counts = {item["name"]: item["home_counts"] for item in provision.JOINTS}
    assert home_counts["left_hip_pitch"] == -299
    assert home_counts["neck_pitch"] == 228
    assert home_counts["right_hip_pitch"] == 299
    assert home_counts["left_hip_yaw"] == 0


def test_provision_fresh_servo_gets_its_joint_id():
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(10, calibrate="none")
    assert result["ok"], result["error"] + str(result["steps"])
    assert bus._serial.servos.get(1) is None, "旧 ID 1 不应答了"
    assert bus._serial.servos[10]["regs"][5] == 10
    assert bus._serial.servos[10]["regs"][6] == 0
    assert bus._serial.servos[10]["regs"][8] == 1
    assert bus._serial.servos[10]["regs"][55] == 1, "结束时必须已经上锁"
    snap = result["snapshot"]
    assert snap["id_read"] == 10
    assert snap["baud_code"] == 0
    assert snap["lock"] == 1
    assert snap["status"] == 0
    assert result["precheck"]["target_present"] is False


def test_provision_enables_multiturn_so_negative_angles_work():
    """出厂 11 号 = 4095 会把负目标夹到 0；整机装配必须把 9/11 号都写成 0。

    真机实测：写完 0/0 之后写目标 −500，当前位置才真的变成 −500。
    """
    bus = fake_bus([1])
    assert bus._serial.servos[1]["max_limit"] == 4095
    result = provision.Provisioner(bus).provision(22, calibrate="none", multiturn=True)
    assert result["ok"], result["error"] + str(result["steps"])
    assert bus._serial.servos[22]["min_limit"] == 0
    assert bus._serial.servos[22]["max_limit"] == 0
    assert result["snapshot"]["multiturn"] is True
    # 关节表里确实有负角关节，这一步不是可有可无的。
    negative = [j["name"] for j in provision.JOINTS if j["home_counts"] < 0]
    assert "left_hip_pitch" in negative and len(negative) == 4


def test_multiturn_defaults_to_off_single_turn_limits():
    """多圈默认关闭，而且"关闭"是**真的写回单圈行程**，不是跳过不写。"""
    assert provision.LIMITS_MULTITURN == (0, 0)
    assert provision.LIMITS_SINGLE_TURN == (0, 4095)
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(22, calibrate="none")
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["multiturn"] is False
    assert bus._serial.servos[22]["min_limit"] == 0
    assert bus._serial.servos[22]["max_limit"] == 4095
    assert result["snapshot"]["multiturn"] is False
    names = [item["name"] for item in result["steps"]]
    assert "关闭多圈，恢复单圈行程（9/11 号写 0/4095）" in names


def test_provision_restores_single_turn_on_an_already_multiturn_servo():
    """旧版"取消勾选 = 跳过"对已经开过多圈的舵机毫无作用，这里钉住修复。"""
    bus = fake_bus([22], raw_position=100)
    provisioner = provision.Provisioner(bus)
    # 先打开多圈（模拟上次初始化留下的状态）
    first = provisioner.provision(22, calibrate="none", mode="reinit", multiturn=True)
    assert first["ok"], first["error"] + str(first["steps"])
    assert bus._serial.servos[22]["max_limit"] == 0
    # 再关掉：必须真的回到 0/4095
    second = provisioner.provision(22, calibrate="none", mode="reinit", multiturn=False)
    assert second["ok"], second["error"] + str(second["steps"])
    assert bus._serial.servos[22]["min_limit"] == 0
    assert bus._serial.servos[22]["max_limit"] == 4095
    assert second["snapshot"]["multiturn"] is False
    assert not [item for item in second["steps"] if not item["ok"]]


def test_provision_can_keep_the_factory_single_turn_limit():
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(22, calibrate="none", multiturn=False)
    assert result["ok"], result["error"] + str(result["steps"])
    assert bus._serial.servos[22]["max_limit"] == 4095
    assert result["snapshot"]["multiturn"] is False
    assert "多圈位置控制" not in [item["name"] for item in result["steps"]
                                  if not item["ok"]]


def test_unload_condition_is_read_modify_write_and_defaults_to_off():
    """19 号只动 BIT0(电压)/BIT3(过流)，BIT1(磁编码)/BIT2(过热) 必须原样保留。"""
    # 出厂/现状：BIT1+BIT2 打开（0b0110 = 6）
    bus = fake_bus([1])
    bus._serial.servos[1]["regs"][19] = 0b0110
    result = provision.Provisioner(bus).provision(20, calibrate="none")
    assert result["ok"], result["error"] + str(result["steps"])
    # 默认两个保护都关闭：BIT0/BIT3 清零，BIT1/BIT2 保留
    assert bus._serial.servos[20]["regs"][19] == 0b0110
    assert result["protect"] == {"voltage": False, "over_current": False}
    assert result["snapshot"]["protect_voltage"] is False
    assert result["snapshot"]["protect_over_current"] is False

    # 勾选两项：BIT0/BIT3 置位，BIT1/BIT2 仍然保留
    bus = fake_bus([1])
    bus._serial.servos[1]["regs"][19] = 0b0110
    result = provision.Provisioner(bus).provision(
        20, calibrate="none", protect_voltage=True, protect_over_current=True)
    assert result["ok"], result["error"] + str(result["steps"])
    assert bus._serial.servos[20]["regs"][19] == 0b1111
    assert result["snapshot"]["protect_voltage"] is True
    assert result["snapshot"]["protect_over_current"] is True

    # 只勾电压：过流位保持 0，磁编码/过热不动
    bus = fake_bus([1])
    bus._serial.servos[1]["regs"][19] = 0b0110
    result = provision.Provisioner(bus).provision(
        20, calibrate="none", protect_voltage=True)
    assert result["ok"], result["error"] + str(result["steps"])
    assert bus._serial.servos[20]["regs"][19] == 0b0111


def test_unload_condition_is_part_of_the_readback_check():
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(
        20, calibrate="none", protect_over_current=True)
    assert result["ok"], result["error"] + str(result["steps"])
    check = [item for item in result["steps"] if item["name"] == "回读校验"][0]
    assert "卸载条件" in check["detail"]
    assert "过流保护 开" in check["detail"]
    assert result["snapshot"]["unload_condition"] == 1 << 3


def test_calibration_ofs_can_save_and_relock():
    """04 页 CAL：save=True 写完补上锁（55 号 = 1），save=False 保持解锁。"""
    for save, expected_lock in ((True, 1), (False, 0)):
        bus = fake_bus([10], raw_position=1234)
        frame, lock_frame = bus.calibration_ofs(10, save=save)
        assert frame.status == 0
        assert bus._serial.servos[10]["regs"][55] == expected_lock, (save, expected_lock)
        assert (lock_frame is not None) is save
        # CAL 的语义：当前位置变成单圈中点 2048。
        position = bus.read_word(10, 56)
        assert decode_signed_magnitude(position, 15) == 2048


def test_provision_cal_calibration_lands_on_the_2048_midpoint():
    """CAL 的真机语义：当前位置变成单圈中点 2048 计数，偏移随之改写。"""
    bus = fake_bus([1], raw_position=1234)
    result = provision.Provisioner(bus).provision(24, calibrate="cal")
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["calibration"]["before"] == 1234
    assert result["calibration"]["after"] == 2048
    assert result["snapshot"]["position"] == 2048
    # 读数 = (编码器 − 偏移) mod 4096 ⇒ 偏移 ≡ 1234 − 2048
    assert (result["snapshot"]["position_offset"] - (1234 - 2048)) % 4096 == 0


def test_joint_table_carries_both_the_angle_and_the_servo_count():
    """关节角计数（可负）与要写到舵机上的计数（0..4095，2048 = 关节角 0）必须都对。

    这两个数混起来就是整机装歪：`home_counts` 是 model.rs 的 DEFAULT_POSITION，
    `home_count` 才是 42/56 号字段里的值。两者之间还夹着一个**方向**（feetech.rs 的
    `POSITION_DIRECTION = −1`，实测），所以 `home_count ≠ 2048 + home_counts`——
    left_hip_pitch 的 −299 计数对应 **2347**，不是 1749。
    """
    assert provision.POSITION_CENTER == 2048
    assert provision.POSITION_DIRECTION == -1.0
    expected = {
        "left_hip_yaw": 0, "left_hip_roll": -57, "left_hip_pitch": -299,
        "left_knee": -3, "left_ankle": 295, "neck_pitch": 228, "head_pitch": 228,
        "head_yaw": 0, "head_roll": 0, "mouth": 0, "right_hip_yaw": 0,
        "right_hip_roll": 57, "right_hip_pitch": 299, "right_knee": 3,
        "right_ankle": -295,
    }
    for joint in provision.JOINTS:
        counts = expected[joint["name"]]
        assert joint["home_counts"] == counts, joint["name"]
        assert joint["direction"] == provision.POSITION_DIRECTION, joint["name"]
        assert joint["home_count"] == int(round(
            provision.POSITION_CENTER + counts / provision.POSITION_DIRECTION)), joint["name"]
        assert 0 <= joint["home_count"] <= 4095, joint["name"]
    # 举两个具体的：髋俯仰的 home 角是 −0.4579 rad（−299 计数），方向 −1 →
    # 舵机字段值 **2347**（旧表写的 1749 是镜像位置）；右踝的 +295 计数 → 1753。
    by_name = {j["name"]: j for j in provision.JOINTS}
    assert by_name["left_hip_pitch"]["home_count"] == 2347
    assert by_name["right_hip_pitch"]["home_count"] == 1749
    assert by_name["left_ankle"]["home_count"] == 1753
    assert by_name["right_ankle"]["home_count"] == 2343
    assert by_name["left_hip_yaw"]["home_count"] == 2048
    # 头颈两个 +20° 的关节：2048 − 228 = 1820。
    assert by_name["neck_pitch"]["home_count"] == 1820
    assert by_name["head_pitch"]["home_count"] == 1820


def test_provision_offset_calibration_hits_the_home_pose():
    bus = fake_bus([1], raw_position=1234)
    result = provision.Provisioner(bus).provision(20, calibrate="offset")
    assert result["ok"], result["error"] + str(result["steps"])
    # right_hip_yaw 的 home 姿态角是 0，也就是舵机的 2048。
    assert result["calibration"]["target_counts"] == provision.POSITION_CENTER
    assert result["calibration"]["target_joint_counts"] == 0
    assert result["calibration"]["position_after"] == provision.POSITION_CENTER
    assert result["snapshot"]["position"] == provision.POSITION_CENTER
    assert result["snapshot"]["joint_counts"] == 0
    assert (result["snapshot"]["position_offset"] - (1234 - 2048)) % 4096 == 0


def test_offset_calibration_wraps_a_negative_home_angle_without_multiturn():
    """单圈字段里位置就是 0..4095：方向 −1 时负 home 角表现为 2048+|角|，判等要按取模。"""
    bus = fake_bus([22], raw_position=100)
    result = provision.Provisioner(bus).provision(
        22, calibrate="offset", mode="reinit", multiturn=False)
    assert result["ok"], result["error"] + str(result["steps"])
    # left_hip_pitch 的 home 角是 −299 计数，方向 −1 → 舵机字段值 2347。
    assert result["calibration"]["target_counts"] == 2347
    assert result["calibration"]["target_joint_counts"] == -299
    assert result["snapshot"]["position"] == 2347
    assert result["snapshot"]["joint_counts"] == -299
    assert result["snapshot"]["multiturn"] is False


def test_offset_calibration_is_exact_when_multiturn_is_on():
    """多圈模式下读数是有符号绝对值：差一整圈就是真差一圈，必须精确到位。"""
    bus = fake_bus([22], raw_position=100)
    result = provision.Provisioner(bus).provision(
        22, calibrate="offset", mode="reinit", multiturn=True)
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["snapshot"]["multiturn"] is True
    assert result["snapshot"]["position"] == 2347, result["steps"][-1]["detail"]


def test_offset_calibration_survives_the_opposite_sign_convention():
    """符号约定万一和实测相反，也必须靠回读自己纠正，而不是写完就报成功。"""
    bus = fake_bus([1], raw_position=1234, invert_convention=True)
    result = provision.Provisioner(bus).provision(20, calibrate="offset")
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["calibration"]["position_after"] == provision.POSITION_CENTER
    # 约定相反时所需偏移也反号：offset ≡ 目标 − 编码器（正约定是 编码器 − 目标）。
    assert (result["snapshot"]["position_offset"] - (2048 - 1234)) % 4096 == 0


def test_provision_refuses_an_occupied_target_id():
    bus = fake_bus([1, 10])
    result = provision.Provisioner(bus).provision(10, calibrate="none")
    assert not result["ok"]
    assert "已经在应答" in result["error"]
    # 拒绝时不应该动任何一颗舵机的 ID。
    assert bus._serial.servos[1]["regs"][5] == 1
    assert bus._serial.servos[10]["regs"][5] == 10


def test_provision_reinit_mode_recalibrates_an_existing_servo():
    bus = fake_bus([10], raw_position=3000)
    provisioner = provision.Provisioner(bus)
    check = provisioner.precheck(10, source_id=10)
    assert check["source_present"] and check["target_present"]
    result = provisioner.provision(10, calibrate="offset", mode="reinit")
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["snapshot"]["id_read"] == 10
    # 重新初始化不能改写主 ID，但要重新做校准（right_hip_yaw 的关节角 0 = 计数 2048）。
    assert result["snapshot"]["position"] == provision.POSITION_CENTER
    assert result["snapshot"]["joint_counts"] == 0
    assert (result["snapshot"]["position_offset"] - (3000 - 2048)) % 4096 == 0


def test_offset_calibration_offset_always_fits_the_register():
    """校准后的偏移始终落在 31 号 ±4095 的量程内，位置精确落在 home 角上。"""
    for raw in (0, 1, 2048, 3000):
        bus = fake_bus([22], raw_position=raw)
        result = provision.Provisioner(bus).provision(
            22, calibrate="offset", mode="reinit", multiturn=True)
        assert result["ok"], (raw, result["error"], result["steps"])
        snap = result["snapshot"]
        assert abs(snap["position_offset"]) <= provision.OFFSET_LIMIT, (raw, snap)
        assert snap["position"] == 2347, (raw, snap)


def test_offset_calibration_restores_the_offset_instead_of_faking_it():
    """调不到目标时必须如实报失败，并把原来的偏移写回去，不留半校准状态。

    单圈模式下 0..4095 里的**任何**目标都到得了（偏移 = 编码器 − 目标，且落在 ±4095 内），
    而且判等是取模的，所以"一圈之外的目标"也一样到得了。要让这条路径真的走不通，
    得用多圈模式的精确判等 + 一个偏移量程也够不着的目标：编码器 0、目标 5000 需要
    −5000 的偏移，超过 31 号的 ±4095。
    """
    bus = fake_bus([22], raw_position=0)
    log = provision.StepLog()
    ok, _ = provision.Provisioner(bus)._calibrate_offset(22, 5000, log, multiturn=True)
    assert not ok
    detail = log.items[-1]["detail"]
    assert "没能把读数调到" in detail and "恢复" in detail, detail
    assert bus._serial.servos[22]["offset"] == 0, "失败时必须把偏移恢复原值"


def test_rewind_offset_moves_whole_turns_and_refuses_the_impossible():
    """整圈挪偏移的算式：能在 31 号量程内就把读数搬回单圈，超了就明确放弃。"""
    # 真机那一例：head_yaw 读数 6289、偏移 −668 → 偏移 +4096 = 3428，读数回到 2193。
    assert provision.rewind_offset(6289, -668) == 3428
    assert (6289 - 2193) % provision.POSITION_WRAP == 0        # 正好一整圈
    assert provision.rewind_offset(6289, -45) == 4051          # 刚好还在 ±4095 内
    assert provision.rewind_offset(6289, 4000) is None         # 要 +4096 就超量程
    assert provision.rewind_offset(2193, -668) is None         # 已经在窗口里
    assert provision.rewind_offset(-500, 0) is None            # 0 → −4096，超量程


def test_provision_rewinds_an_out_of_turn_reading_without_moving_the_joint():
    """56 号必须落在单圈里，而初始化过去不查这一条。

    这批舵机出厂相位 18 号 = 0x34（bit 4 全角度反馈），位置读数**不会自己回绕**，
    被手推过一整圈之后会读到 4096 以上；robotd 只读单圈字段、超出就取模屏蔽，
    于是"目标 == 实测"看着正常，下一次写目标却让关节转回去一整圈
    （2026-10-05 真机 head_yaw 6289，`robotctl robot init` 时脖子转了两圈）。
    """
    bus = fake_bus([22])
    servo = bus._serial.servos[22]
    servo["regs"][18] = 0x34                # 真机出厂相位：全角度反馈
    servo["offset"] = -668
    servo["raw_position"] = 6289 - 668      # 读数 = 编码器 − 偏移 = 6289，在单圈外
    assert bus._serial._position(servo) == 6289, "全角度反馈下固件不回绕"
    p = provision.Provisioner(bus)
    log = provision.StepLog()
    ok, detail = p.rewind_position(22, log)
    assert ok, detail
    assert servo["offset"] == 3428
    assert servo["raw_position"] - servo["offset"] == 2193
    # 关节角按一圈取模算，整圈平移对它不变 —— 物理姿态没动。
    assert (2193 % provision.POSITION_WRAP) == (6289 % provision.POSITION_WRAP)
    assert log.items and log.items[-1]["ok"], log.to_list()


def test_mid_calibration_targets_2048_not_zero():
    """「转到中位」写的是 2048（关节角 0），不是 0——0 是一圈的另一端（≈ −180°）。"""
    bus = fake_bus([1], raw_position=1234)
    result = provision.Provisioner(bus).provision(20, calibrate="mid")
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["calibration"]["position"] == provision.POSITION_CENTER
    assert result["snapshot"]["joint_counts"] == 0
    assert "转到中位（目标位置 2048）" in [s["name"] for s in result["steps"]]


def test_gain_default_is_the_vendors_value_not_robots_200():
    """初始化写的 Kp 必须是厂商默认量级，不能是 robotd 那个给 XL330 调的 200。

    真机实测（2026-09-30，15 台装在悬空鸭子上）：RAM 50 = 200 时 left_hip_yaw
    自激振荡——峰值电流 4966 mA、壳温 74 °C 且持续上升；改成 32 后峰值 110 mA、
    温度不升。这条测试就是防止有人把默认值改回去。
    """
    assert provision.POSITION_KP_DEFAULT == 32
    assert provision.POSITION_KP_DEFAULT < 200
    assert 0 <= provision.POSITION_KP_DEFAULT <= provision.POSITION_GAIN_MAX
    assert provision.POSITION_KD_DEFAULT == 0, "Kd 与 set_gain 的写法对齐（写 0）"


def test_provision_writes_a_safe_position_gain():
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(20, calibrate="none")
    assert result["ok"], result["error"] + str(result["steps"])
    regs = bus._serial.servos[20]["regs"]
    assert regs[21] == provision.POSITION_KP_DEFAULT, "EEPROM 21（位置环 P）"
    assert regs[22] == provision.POSITION_KD_DEFAULT, "EEPROM 22（位置环 D）"
    assert regs[50] == provision.POSITION_KP_DEFAULT, "RAM 50 立即生效"
    assert regs[51] == provision.POSITION_KD_DEFAULT
    assert regs[21] != 200
    assert regs[55] == 1, "写完必须重新上锁"
    assert result["gain"] == {"kp": 32, "kd": 0}
    names = [item["name"] for item in result["steps"]]
    assert "写位置环增益" in names
    assert all(item["ok"] for item in result["steps"] if item["name"].startswith("写位置环"))


def test_provision_gain_is_configurable_and_clamped():
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(20, calibrate="none", gain_kp=64, gain_kd=8)
    assert result["ok"], result["error"] + str(result["steps"])
    regs = bus._serial.servos[20]["regs"]
    assert (regs[21], regs[22], regs[50], regs[51]) == (64, 8, 64, 8)

    # 超出 0..254 的值按寄存器范围夹住，而不是回绕成一个很小的数。
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(20, calibrate="none", gain_kp=9999, gain_kd=-5)
    assert result["ok"], result["error"] + str(result["steps"])
    regs = bus._serial.servos[20]["regs"]
    assert regs[21] == provision.POSITION_GAIN_MAX == 254
    assert regs[22] == 0


def test_provision_can_leave_the_gain_alone():
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(20, calibrate="none", write_gain=False)
    assert result["ok"], result["error"] + str(result["steps"])
    regs = bus._serial.servos[20]["regs"]
    assert (regs[21], regs[22], regs[50], regs[51]) == (32, 40, 32, 40), "跳过时保持原值"
    assert "写位置环增益" in [item["name"] for item in result["steps"]]


def test_port_holders_detects_another_process():
    """串口被别的进程占着时必须查得出来。

    `robotd` 用 `TIOCEXCL` 打开 `/dev/ttyS2`，但那个排他标志只挡没有 `CAP_SYS_ADMIN` 的
    进程——两个 root 进程可以同时打开同一个 tty（2026-10-05 在板子上实测如此）。
    互吃应答帧的现象是"每个 tick 随机丢几个舵机"，所以这条检查不能只靠内核。
    """
    from . import server

    assert server.port_holders("/etc/hostname") == [], "普通文件不是串口"
    assert server.port_holders("/dev/definitely-not-here") == []

    handle = open("/dev/zero", "rb")
    child = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(10)"],
                             stdin=handle)
    try:
        found = []
        for _ in range(60):
            found = [item for item in server.port_holders("/dev/zero")
                     if item["pid"] == child.pid]
            if found:
                break
            time.sleep(0.05)
        assert found, "没能发现持有 /dev/zero 的子进程（pid %d）" % child.pid
        assert found[0]["name"], found
    finally:
        child.kill()
        child.wait()
        handle.close()


def test_census_reports_who_is_on_the_bus():
    bus = fake_bus([10, 24])
    data = provision.Provisioner(bus).census()
    assert data["total"] == 15
    assert data["present_count"] == 2
    present = sorted(row["id"] for row in data["items"] if row["present"])
    assert present == [10, 24]
    row = [item for item in data["items"] if item["id"] == 10][0]
    assert row["snapshot"]["id_read"] == 10


def main():
    test_build_frames()
    test_transaction()
    test_read_and_sync_read()
    test_memory_table()
    test_feedback_decode()
    test_joint_table_matches_microduck()
    test_position_direction_matches_the_driver()
    test_provision_fresh_servo_gets_its_joint_id()
    test_rewind_offset_moves_whole_turns_and_refuses_the_impossible()
    test_provision_rewinds_an_out_of_turn_reading_without_moving_the_joint()
    test_provision_enables_multiturn_so_negative_angles_work()
    test_multiturn_defaults_to_off_single_turn_limits()
    test_provision_restores_single_turn_on_an_already_multiturn_servo()
    test_provision_can_keep_the_factory_single_turn_limit()
    test_unload_condition_is_read_modify_write_and_defaults_to_off()
    test_unload_condition_is_part_of_the_readback_check()
    test_calibration_ofs_can_save_and_relock()
    test_gain_default_is_the_vendors_value_not_robots_200()
    test_provision_writes_a_safe_position_gain()
    test_provision_gain_is_configurable_and_clamped()
    test_provision_can_leave_the_gain_alone()
    test_provision_cal_calibration_lands_on_the_2048_midpoint()
    test_joint_table_carries_both_the_angle_and_the_servo_count()
    test_provision_offset_calibration_hits_the_home_pose()
    test_mid_calibration_targets_2048_not_zero()
    test_offset_calibration_wraps_a_negative_home_angle_without_multiturn()
    test_offset_calibration_is_exact_when_multiturn_is_on()
    test_offset_calibration_survives_the_opposite_sign_convention()
    test_provision_refuses_an_occupied_target_id()
    test_provision_reinit_mode_recalibrates_an_existing_servo()
    test_offset_calibration_offset_always_fits_the_register()
    test_offset_calibration_restores_the_offset_instead_of_faking_it()
    test_port_holders_detects_another_process()
    test_census_reports_who_is_on_the_bus()
    print("HLS debugger self-test: OK")


if __name__ == "__main__":
    main()
