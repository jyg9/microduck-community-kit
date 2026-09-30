# -*- coding: utf-8 -*-
"""不依赖真实硬件的协议自检。

运行：
    .venv/bin/python -m hls_debugger.selftest
"""

from __future__ import absolute_import

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

    def _position(self, servo):
        if self.invert_convention:
            delta = servo["raw_position"] + servo["offset"]
        else:
            delta = servo["raw_position"] - servo["offset"]
        if self._multiturn(servo):
            return delta           # 多圈：真正的有符号绝对值，可超出 0~4095
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
    """出厂 11 号 = 4095 会把负目标夹到 0；初始化必须把 9/11 号都写成 0。

    真机实测：写完 0/0 之后写目标 −500，当前位置才真的变成 −500。
    """
    bus = fake_bus([1])
    assert bus._serial.servos[1]["max_limit"] == 4095
    result = provision.Provisioner(bus).provision(22, calibrate="none")
    assert result["ok"], result["error"] + str(result["steps"])
    assert bus._serial.servos[22]["min_limit"] == 0
    assert bus._serial.servos[22]["max_limit"] == 0
    assert result["snapshot"]["multiturn"] is True
    # 关节表里确实有负角关节，这一步不是可有可无的。
    negative = [j["name"] for j in provision.JOINTS if j["home_counts"] < 0]
    assert "left_hip_pitch" in negative and len(negative) == 4


def test_provision_can_keep_the_factory_single_turn_limit():
    bus = fake_bus([1])
    result = provision.Provisioner(bus).provision(22, calibrate="none", multiturn=False)
    assert result["ok"], result["error"] + str(result["steps"])
    assert bus._serial.servos[22]["max_limit"] == 4095
    assert result["snapshot"]["multiturn"] is False
    assert "多圈位置控制" not in [item["name"] for item in result["steps"]
                                  if not item["ok"]]


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


def test_provision_offset_calibration_hits_the_home_pose():
    bus = fake_bus([1], raw_position=1234)
    result = provision.Provisioner(bus).provision(20, calibrate="offset")
    assert result["ok"], result["error"] + str(result["steps"])
    # right_hip_yaw 的 home 姿态是 0 计数。
    assert result["calibration"]["target_counts"] == 0
    assert result["calibration"]["position_after"] == 0
    assert result["snapshot"]["position"] == 0
    assert (result["snapshot"]["position_offset"] - 1234) % 4096 == 0


def test_offset_calibration_wraps_a_negative_home_angle_without_multiturn():
    """没打开多圈时位置被夹在一圈内：负 home 角表现为 4096+角，校准要按取模判等。

    这正是出厂设置下的真实行为，也是"负角关节不到位"的原因。
    """
    bus = fake_bus([22], raw_position=100)
    result = provision.Provisioner(bus).provision(
        22, calibrate="offset", mode="reinit", multiturn=False)
    assert result["ok"], result["error"] + str(result["steps"])
    # left_hip_pitch 的 home 是 −299 计数，一圈内表现为 3797。
    assert result["calibration"]["target_counts"] == -299
    assert result["snapshot"]["position"] == 3797
    assert result["snapshot"]["multiturn"] is False


def test_offset_calibration_is_exact_when_multiturn_is_on():
    """打开多圈后位置是有符号绝对值：差一整圈就是真差一圈，必须精确到位。"""
    bus = fake_bus([22], raw_position=100)
    result = provision.Provisioner(bus).provision(22, calibrate="offset", mode="reinit")
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["snapshot"]["multiturn"] is True
    assert result["snapshot"]["position"] == -299, result["steps"][-1]["detail"]


def test_offset_calibration_survives_the_opposite_sign_convention():
    """符号约定万一和实测相反，也必须靠回读自己纠正，而不是写完就报成功。"""
    bus = fake_bus([1], raw_position=1234, invert_convention=True)
    result = provision.Provisioner(bus).provision(20, calibrate="offset")
    assert result["ok"], result["error"] + str(result["steps"])
    assert result["calibration"]["position_after"] == 0
    assert (result["snapshot"]["position_offset"] - (-1234)) % 4096 == 0


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
    # 重新初始化不能改写主 ID，但要重新做校准。
    assert result["snapshot"]["position"] == 0
    assert (result["snapshot"]["position_offset"] - 3000) % 4096 == 0


def test_offset_calibration_offset_always_fits_the_register():
    """校准后的偏移始终落在 31 号 ±4095 的量程内，位置精确落在 home 角上。"""
    for raw in (0, 1, 2048, 3000):
        bus = fake_bus([22], raw_position=raw)
        result = provision.Provisioner(bus).provision(22, calibrate="offset", mode="reinit")
        assert result["ok"], (raw, result["error"], result["steps"])
        snap = result["snapshot"]
        assert abs(snap["position_offset"]) <= provision.OFFSET_LIMIT, (raw, snap)
        assert snap["position"] == -299, (raw, snap)


def test_offset_calibration_restores_the_offset_instead_of_faking_it():
    """调不到目标时必须如实报失败，并把原来的偏移写回去，不留半校准状态。"""
    # 编码器在 4095、目标 −299：一圈内没有能同时满足的偏移。
    bus = fake_bus([22], raw_position=4095)
    result = provision.Provisioner(bus).provision(22, calibrate="offset", mode="reinit")
    assert not result["ok"]
    detail = result["steps"][-1]["detail"]
    assert "没能把读数调到" in detail and "恢复" in detail, detail
    assert bus._serial.servos[22]["offset"] == 0, "失败时必须把偏移恢复原值"


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
    test_provision_fresh_servo_gets_its_joint_id()
    test_provision_enables_multiturn_so_negative_angles_work()
    test_provision_can_keep_the_factory_single_turn_limit()
    test_provision_cal_calibration_lands_on_the_2048_midpoint()
    test_provision_offset_calibration_hits_the_home_pose()
    test_offset_calibration_wraps_a_negative_home_angle_without_multiturn()
    test_offset_calibration_is_exact_when_multiturn_is_on()
    test_offset_calibration_survives_the_opposite_sign_convention()
    test_provision_refuses_an_occupied_target_id()
    test_provision_reinit_mode_recalibrates_an_existing_servo()
    test_offset_calibration_offset_always_fits_the_register()
    test_offset_calibration_restores_the_offset_instead_of_faking_it()
    test_census_reports_who_is_on_the_bus()
    print("HLS debugger self-test: OK")


if __name__ == "__main__":
    main()
