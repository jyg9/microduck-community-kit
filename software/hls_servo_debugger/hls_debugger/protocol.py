# -*- coding: utf-8 -*-
"""飞特 SCS/HLS 串口协议主机侧实现。

与 FTServo_Linux-main/src/SCS.cpp、HLSCL.cpp 的行为保持一致：
帧格式 FF FF ID LEN INST ADDR DATA... ~SUM
校验和 = ~(ID + LEN + INST + ADDR + SUM(DATA)) & 0xFF
无地址指令（PING/REG_ACTION/RECOVERY/RESET/CAL）参与校验的 ADDR 按 0 计算。
"""

from __future__ import absolute_import

import threading
import time
from collections import deque

import serial

from .memory_table import encode_signed_magnitude

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_REG_WRITE = 0x04
INST_REG_ACTION = 0x05
INST_RECOVERY = 0x06
INST_RESET = 0x0A
INST_CAL = 0x0B
INST_SYNC_READ = 0x82
INST_SYNC_WRITE = 0x83

BROADCAST_ID = 0xFE
NO_ADDR_INSTRUCTIONS = {INST_PING, INST_REG_ACTION, INST_RECOVERY, INST_RESET, INST_CAL}

# 出厂新舵机的主 ID。microduck 的总线上没有任何关节使用 ID 1（IMU 节点是 200），
# 所以"总线上的 ID 1"就是"一颗还没编号的新舵机"。
FACTORY_ID = 1
# 出厂新舵机的波特率编码，0 = 1 Mbps，与工具默认波特率一致。
EXPECTED_BAUD_CODE = 0
# microduck 的 IMU 节点也挂在这条总线上，占用 ID 200。
IMU_BUS_ID = 200

# HLS 内存表关键地址
ADDR_ID = 5
ADDR_BAUD_RATE = 6
ADDR_SECOND_ID = 7
ADDR_RESPONSE_LEVEL = 8
ADDR_MIN_ANGLE_LIMIT = 9
ADDR_MAX_ANGLE_LIMIT = 11
#: 19 号「卸载条件」位域：置 1 表示开启对应保护，触发时舵机卸载（卸力）。
#: 完整位域见 memory_table.PROTECT_BITS（0 电压 / 1 磁编码 / 2 过热 / 3 过流）。
ADDR_UNLOAD_CONDITION = 19
UNLOAD_BIT_VOLTAGE = 0
UNLOAD_BIT_OVER_CURRENT = 3
#: 位置环 P/D 的 EEPROM 默认值，上电时分别加载进 50/51 两个 RAM 寄存器。
ADDR_POSITION_KP_EPROM = 21
ADDR_POSITION_KD_EPROM = 22
ADDR_POSITION_OFFSET = 31
ADDR_MODE = 33
ADDR_TORQUE_ENABLE = 40
ADDR_ACC = 41
ADDR_GOAL_POSITION = 42
ADDR_GOAL_CURRENT = 44
ADDR_GOAL_SPEED = 46
ADDR_TORQUE_LIMIT = 48
#: 位置环 P/D 的 RAM 值，立即生效（`duck-control` 的 `set_gain` 写的就是这两个）。
ADDR_POSITION_KP_RAM = 50
ADDR_POSITION_KD_RAM = 51
ADDR_LOCK = 55
ADDR_PRESENT_POSITION = 56
ADDR_PRESENT_SPEED = 58
ADDR_PRESENT_LOAD = 60
ADDR_PRESENT_VOLTAGE = 62
ADDR_PRESENT_TEMPERATURE = 63
ADDR_PRESENT_MOVING = 66
ADDR_PRESENT_CURRENT = 69

MODE_POSITION = 0
MODE_WHEEL = 1
MODE_ELECTRIC = 2
MODE_PWM = 3


class ProtocolError(Exception):
    """通信协议/串口错误。"""

    def __init__(self, message, code="protocol", tx=None, rx=None, detail=None):
        super(ProtocolError, self).__init__(message)
        self.message = message
        self.code = code
        self.tx = tx
        self.rx = rx
        self.detail = detail

    def to_dict(self):
        return {
            "message": self.message,
            "code": self.code,
            "tx": _hex_text(self.tx),
            "rx": _hex_text(self.rx),
            "detail": self.detail,
        }


class Frame(object):
    def __init__(self, sid, length, status, data, raw):
        self.id = sid
        self.length = length
        self.status = status
        self.data = data
        self.raw = raw

    def to_dict(self):
        return {
            "id": self.id,
            "length": self.length,
            "status": self.status,
            "data": list(self.data),
            "data_hex": _hex_text(self.data),
            "raw_hex": _hex_text(self.raw),
        }


def _hex_text(data):
    if data is None:
        return None
    try:
        return bytes(data).hex(" ").upper()
    except Exception:
        return str(data)


def build_frame(servo_id, instruction, addr=0, data=None):
    """构造一条主机指令帧。"""
    servo_id = int(servo_id) & 0xFF
    instruction = int(instruction) & 0xFF
    data = bytes(data or b"")

    if instruction in NO_ADDR_INSTRUCTIONS:
        length = 2
        # 协议要求无地址指令的 ADDR 按 0 参与校验和
        checksum = (servo_id + length + instruction + 0) & 0xFF
        body = bytes([servo_id, length, instruction])
    else:
        addr = int(addr) & 0xFF
        length = 3 + len(data)
        if length > 250:
            raise ValueError("指令数据过长")
        checksum = (servo_id + length + instruction + addr + sum(data)) & 0xFF
        body = bytes([servo_id, length, instruction, addr]) + data

    checksum = (~checksum) & 0xFF
    frame = b"\xff\xff" + body + bytes([checksum])
    return frame


def parse_frame(raw):
    """解析一帧完整应答；用于 raw_exchange 等场景。"""
    raw = bytes(raw)
    if len(raw) < 6:
        raise ProtocolError("应答帧长度不足", "frame_length", rx=raw)
    idx = raw.find(b"\xff\xff")
    if idx < 0 or len(raw) < idx + 6:
        raise ProtocolError("未找到 FF FF 帧头", "frame_header", rx=raw)
    raw = raw[idx:]
    sid = raw[2]
    length = raw[3]
    if length < 2 or len(raw) < length + 4:
        raise ProtocolError("应答帧 LEN 非法或长度不足", "frame_length", rx=raw)
    frame = raw[:length + 4]
    checksum = frame[-1]
    calc = (~sum(frame[2:-1])) & 0xFF
    if checksum != calc:
        raise ProtocolError(
            "应答帧校验和错误：收到 0x%02X，计算 0x%02X" % (checksum, calc),
            "checksum", rx=frame,
        )
    status = frame[4]
    data = frame[5:-1]
    return Frame(sid, length, status, data, frame)


class HLSBus(object):
    """HLS/SCS 总线主机。一个实例对应一个串口。"""

    def __init__(self, log_callback=None, log_maxlen=3000):
        self._serial = None
        self._lock = threading.RLock()
        self._logs = deque(maxlen=log_maxlen)
        self._log_callback = log_callback
        self.port = None
        self.baudrate = None
        self.timeout = 0.1  # 单次事务等待毫秒级，API 中可调
        self.connected = False

    # ------------------------------------------------------------------
    # 串口连接
    # ------------------------------------------------------------------
    def connect(self, port, baudrate=1000000, timeout=0.1):
        with self._lock:
            self.disconnect()
            try:
                serial_timeout = max(0.001, min(float(timeout), 0.05))
                self._serial = serial.Serial(
                    port=port,
                    baudrate=int(baudrate),
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=serial_timeout,
                    write_timeout=max(0.05, float(timeout)),
                    inter_byte_timeout=serial_timeout,
                )
            except Exception as exc:
                self._serial = None
                self.connected = False
                raise ProtocolError(
                    "打开串口失败：%s" % exc, "serial_open", detail=str(exc)
                )
            self.port = port
            self.baudrate = int(baudrate)
            self.timeout = float(timeout)
            self.connected = True
            try:
                self._serial.reset_input_buffer()
                self._serial.reset_output_buffer()
            except Exception:
                pass
            self._log("SYS", None, "已连接 %s @ %s" % (port, baudrate))
            return self.status()

    def disconnect(self):
        with self._lock:
            if self._serial is not None:
                try:
                    self._serial.close()
                except Exception:
                    pass
            if self.connected:
                self._log("SYS", None, "已断开串口")
            self._serial = None
            self.connected = False
            self.port = None
            self.baudrate = None

    def status(self):
        return {
            "connected": bool(self._serial is not None and self.connected),
            "port": self.port,
            "baudrate": self.baudrate,
            "timeout": self.timeout,
            "logs": self.logs(),
        }

    # ------------------------------------------------------------------
    # 日志
    # ------------------------------------------------------------------
    def _log(self, direction, raw, note="", error=None, extra=None):
        entry = {
            "ts": time.time(),
            "time": time.strftime("%H:%M:%S"),
            "direction": direction,
            "hex": _hex_text(raw) if raw is not None else "",
            "note": note or "",
            "error": error or "",
            "extra": extra or {},
        }
        self._logs.append(entry)
        if self._log_callback:
            try:
                self._log_callback(entry)
            except Exception:
                pass
        return entry

    def logs(self, limit=300):
        items = list(self._logs)
        if limit:
            items = items[-int(limit):]
        return items

    def clear_logs(self):
        self._logs.clear()

    # ------------------------------------------------------------------
    # 底层收发
    # ------------------------------------------------------------------
    def _require_serial(self):
        if self._serial is None or not self.connected:
            raise ProtocolError("串口尚未连接", "not_connected")

    def _clear_input(self):
        try:
            self._serial.reset_input_buffer()
        except Exception:
            pass

    def _write(self, raw, note=""):
        self._require_serial()
        try:
            self._serial.write(raw)
            self._serial.flush()
        except Exception as exc:
            raise ProtocolError("串口写入失败：%s" % exc, "serial_write", tx=raw)
        self._log("TX", raw, note)

    def _read_frame(self, deadline):
        """从截止时间内读取并解析一帧应答。"""
        self._require_serial()
        state = 0
        frame = bytearray()
        while time.monotonic() < deadline:
            try:
                chunk = self._serial.read(1)
            except Exception as exc:
                raise ProtocolError("串口读取失败：%s" % exc, "serial_read", rx=bytes(frame))
            if not chunk:
                continue
            byte = chunk[0]
            if state == 0:
                state = 1 if byte == 0xFF else 0
            elif state == 1:
                if byte == 0xFF:
                    state = 2
                else:
                    state = 0
            elif state == 2:
                frame = bytearray([byte])
                state = 3
            elif state == 3:
                if byte < 2:
                    state = 0
                    continue
                frame.append(byte)
                state = 4
            else:
                frame.append(byte)
                if len(frame) == frame[1] + 2:
                    if len(frame) < 4:  # ID LEN STATUS SUM
                        state = 0
                        continue
                    checksum = (~sum(frame[:-1])) & 0xFF
                    if checksum != frame[-1]:
                        raise ProtocolError(
                            "应答帧校验和错误：收到 0x%02X，计算 0x%02X"
                            % (frame[-1], checksum),
                            "checksum", rx=bytes(frame),
                        )
                    return Frame(
                        sid=frame[0],
                        length=frame[1],
                        status=frame[2],
                        data=bytes(frame[3:-1]),
                        raw=bytes(frame),
                    )
        raise ProtocolError("等待应答超时", "timeout")

    def _transact(self, frame, expected_id, expected_data_len=0, timeout=None, note=""):
        """发送指令并等待指定 ID、指定数据长度的应答。"""
        self._require_serial()
        timeout = self.timeout if timeout is None else float(timeout)
        with self._lock:
            self._clear_input()
            self._write(frame, note)
            deadline = time.monotonic() + max(0.01, timeout)
            last_rx = None
            while time.monotonic() < deadline:
                try:
                    resp = self._read_frame(deadline)
                except ProtocolError as exc:
                    if exc.code == "timeout":
                        break
                    # 校验和/帧同步错误时继续尝试读取下一帧
                    self._log("RX-ERR", exc.rx, exc.message, error=exc.code)
                    continue
                last_rx = resp.raw
                self._log("RX", resp.raw, note, extra={"id": resp.id, "status": resp.status})
                if resp.id != expected_id:
                    continue
                if expected_data_len is not None and len(resp.data) != expected_data_len:
                    raise ProtocolError(
                        "应答数据长度错误：期望 %d 字节，收到 %d 字节"
                        % (expected_data_len, len(resp.data)),
                        "data_length", tx=frame, rx=resp.raw,
                    )
                return resp
            raise ProtocolError(
                "等待 ID=%s 应答超时（%.0f ms）" % (expected_id, timeout * 1000),
                "timeout", tx=frame, rx=last_rx,
            )

    def _write_only(self, frame, note=""):
        with self._lock:
            self._clear_input()
            self._write(frame, note)

    # ------------------------------------------------------------------
    # 基础指令：PING / READ / WRITE / REG / RESET / CAL
    # ------------------------------------------------------------------
    def ping(self, servo_id, timeout=None):
        frame = build_frame(servo_id, INST_PING)
        return self._transact(frame, servo_id, 0, timeout=timeout, note="PING ID=%s" % servo_id)

    def read(self, servo_id, addr, length, timeout=None):
        length = int(length)
        if length < 1 or length > 250:
            raise ValueError("读取长度应在 1~250 字节")
        frame = build_frame(servo_id, INST_READ, addr, bytes([length]))
        return self._transact(
            frame, servo_id, length, timeout=timeout,
            note="READ ID=%s ADDR=%s LEN=%s" % (servo_id, addr, length),
        )

    def write(self, servo_id, addr, data, timeout=None):
        data = bytes(data)
        if len(data) < 1:
            raise ValueError("写入数据不能为空")
        frame = build_frame(servo_id, INST_WRITE, addr, data)
        return self._transact(
            frame, servo_id, 0, timeout=timeout,
            note="WRITE ID=%s ADDR=%s LEN=%s" % (servo_id, addr, len(data)),
        )

    def reg_write(self, servo_id, addr, data, timeout=None):
        data = bytes(data)
        frame = build_frame(servo_id, INST_REG_WRITE, addr, data)
        return self._transact(
            frame, servo_id, 0, timeout=timeout,
            note="REG_WRITE ID=%s ADDR=%s LEN=%s" % (servo_id, addr, len(data)),
        )

    def reg_action(self, servo_id=0xFE, timeout=None):
        frame = build_frame(servo_id, INST_REG_ACTION)
        if int(servo_id) == 0xFE:
            self._write_only(frame, "REG_ACTION 广播（无应答）")
            return None
        return self._transact(frame, servo_id, 0, timeout=timeout, note="REG_ACTION ID=%s" % servo_id)

    def reset(self, servo_id, timeout=None):
        frame = build_frame(servo_id, INST_RESET)
        return self._transact(frame, servo_id, 0, timeout=timeout, note="RESET ID=%s" % servo_id)

    def recovery(self, servo_id, timeout=None):
        frame = build_frame(servo_id, INST_RECOVERY)
        return self._transact(frame, servo_id, 0, timeout=timeout, note="RECOVERY ID=%s" % servo_id)

    def recal(self, servo_id, timeout=None):
        frame = build_frame(servo_id, INST_CAL)
        return self._transact(frame, servo_id, 0, timeout=timeout, note="CAL ID=%s" % servo_id)

    # ------------------------------------------------------------------
    # 同步读 / 同步写
    # ------------------------------------------------------------------
    def sync_read(self, ids, addr, length, timeout=None):
        ids = [int(x) & 0xFF for x in ids]
        if not ids:
            raise ValueError("ID 列表不能为空")
        if len(set(ids)) != len(ids):
            raise ValueError("ID 列表中存在重复")
        if int(length) < 1 or int(length) > 250:
            raise ValueError("读取长度应为 1~250")
        data = bytes([int(length)]) + bytes(ids)
        frame = build_frame(BROADCAST_ID, INST_SYNC_READ, addr, data)
        timeout = self.timeout if timeout is None else float(timeout)
        # 为多节点预留汇聚时间，同时避免单节点超时被放大
        total_timeout = max(timeout, timeout * len(ids) + 0.03)
        with self._lock:
            self._clear_input()
            self._write(frame, "SYNC_READ IDS=%s ADDR=%s LEN=%s" % (ids, addr, length))
            deadline = time.monotonic() + total_timeout
            results = {}
            last_error = None
            while time.monotonic() < deadline and len(results) < len(ids):
                try:
                    resp = self._read_frame(deadline)
                except ProtocolError as exc:
                    if exc.code == "timeout":
                        break
                    last_error = exc
                    self._log("RX-ERR", exc.rx, exc.message, error=exc.code)
                    continue
                self._log("RX", resp.raw, "SYNC_READ 应答 ID=%s" % resp.id,
                          extra={"id": resp.id, "status": resp.status})
                if resp.id in ids and len(resp.data) == int(length):
                    results[resp.id] = resp
            missing = [i for i in ids if i not in results]
            if missing:
                detail = "未收到同步读应答的 ID：" + ",".join(str(i) for i in missing)
                if last_error:
                    detail += "；最后错误：" + last_error.message
                raise ProtocolError(detail, "sync_read_missing", tx=frame,
                                    rx=results and [results[i].raw for i in results])
            return results

    def sync_write(self, ids, addr, values, timeout=None):
        """同步写。values 为与 ids 等长的 bytes 列表，每项长度必须一致。"""
        ids = [int(x) & 0xFF for x in ids]
        if not ids:
            raise ValueError("ID 列表不能为空")
        values = [bytes(v) for v in values]
        if len(ids) != len(values):
            raise ValueError("ID 数量与数据项数量不一致")
        item_len = len(values[0])
        if item_len < 1:
            raise ValueError("同步写数据不能为空")
        if any(len(v) != item_len for v in values):
            raise ValueError("所有同步写数据项长度必须一致")
        payload = bytes([item_len])
        for sid, val in zip(ids, values):
            payload += bytes([sid]) + val
        frame = build_frame(BROADCAST_ID, INST_SYNC_WRITE, addr, payload)
        self._write_only(
            frame, "SYNC_WRITE IDS=%s ADDR=%s LEN=%s" % (ids, addr, item_len)
        )
        return frame

    # ------------------------------------------------------------------
    # HLS 应用层指令，对应 HLSCL.cpp
    # ------------------------------------------------------------------
    def write_pos_ex(self, servo_id, position, speed, acc=0, torque=0, reg=False):
        position = int(position)
        speed = int(speed) & 0xFFFF
        acc = int(acc) & 0xFF
        torque = int(torque) & 0xFFFF
        pos_raw = encode_signed_magnitude(position, 15)
        data = (
            bytes([acc])
            + pos_raw.to_bytes(2, "little")
            + torque.to_bytes(2, "little")
            + speed.to_bytes(2, "little")
        )
        if reg:
            return self.reg_write(servo_id, ADDR_ACC, data)
        return self.write(servo_id, ADDR_ACC, data)

    def sync_write_pos_ex(self, ids, positions, speeds, accs=None, torques=None):
        if accs is None:
            accs = [0] * len(ids)
        if torques is None:
            torques = [0] * len(ids)
        values = []
        for i, position in enumerate(positions):
            pos_raw = encode_signed_magnitude(int(position), 15)
            data = (
                bytes([int(accs[i]) & 0xFF])
                + pos_raw.to_bytes(2, "little")
                + (int(torques[i]) & 0xFFFF).to_bytes(2, "little")
                + (int(speeds[i]) & 0xFFFF).to_bytes(2, "little")
            )
            values.append(data)
        return self.sync_write(ids, ADDR_ACC, values)

    def write_speed(self, servo_id, speed, acc=0, torque=0, reg=False):
        speed = int(speed)
        acc = int(acc) & 0xFF
        torque = int(torque) & 0xFFFF
        speed_raw = encode_signed_magnitude(speed, 15)
        data = (
            bytes([acc])
            + b"\x00\x00"
            + torque.to_bytes(2, "little")
            + speed_raw.to_bytes(2, "little")
        )
        if reg:
            return self.reg_write(servo_id, ADDR_ACC, data)
        return self.write(servo_id, ADDR_ACC, data)

    def sync_write_speed(self, ids, speeds, accs=None, torques=None):
        if accs is None:
            accs = [0] * len(ids)
        if torques is None:
            torques = [0] * len(ids)
        values = []
        for i, speed in enumerate(speeds):
            speed_raw = encode_signed_magnitude(int(speed), 15)
            data = (
                bytes([int(accs[i]) & 0xFF])
                + b"\x00\x00"
                + (int(torques[i]) & 0xFFFF).to_bytes(2, "little")
                + speed_raw.to_bytes(2, "little")
            )
            values.append(data)
        return self.sync_write(ids, ADDR_ACC, values)

    def write_torque_current(self, servo_id, torque):
        """模式 2 恒流模式：写 44 号目标电流，BIT15 为方向。"""
        raw = encode_signed_magnitude(int(torque), 15)
        return self.write(servo_id, ADDR_GOAL_CURRENT, raw.to_bytes(2, "little"))

    def write_pwm(self, servo_id, value):
        """模式 3 PWM 开环：44 号量程 -1000~1000，BIT10 为方向。"""
        raw = encode_signed_magnitude(int(value), 10)
        return self.write(servo_id, ADDR_GOAL_CURRENT, raw.to_bytes(2, "little"))

    def set_mode(self, servo_id, mode):
        mode = int(mode)
        # 厂商内存表把 33 号写成 0~3，但同一份内存表在 25 号"积分限制值"里注明
        # "位置模式 0 与模式 4 生效"，且实测 HD-1910-C001（固件 3.46）出厂即为 4，
        # 与模式 0 同属位置环，所以 4 也必须允许写入。
        if mode not in (0, 1, 2, 3, 4):
            raise ValueError("运行模式必须为 0~4")
        return self.write(servo_id, ADDR_MODE, bytes([mode]))

    def servo_mode(self, servo_id):
        return self.set_mode(servo_id, MODE_POSITION)

    def wheel_mode(self, servo_id):
        return self.set_mode(servo_id, MODE_WHEEL)

    def ele_mode(self, servo_id):
        return self.set_mode(servo_id, MODE_ELECTRIC)

    def pwm_mode(self, servo_id):
        return self.set_mode(servo_id, MODE_PWM)

    def enable_torque(self, servo_id, enable):
        enable = int(enable)
        if enable not in (0, 1, 2):
            raise ValueError("扭矩开关必须为 0、1 或 2")
        return self.write(servo_id, ADDR_TORQUE_ENABLE, bytes([enable]))

    def unlock_eprom(self, servo_id):
        # 对应 HLSCL::unLockEprom()：先关扭力，再写锁标志 0
        self.enable_torque(servo_id, 0)
        return self.write(servo_id, ADDR_LOCK, b"\x00")

    def lock_eprom(self, servo_id):
        return self.write(servo_id, ADDR_LOCK, b"\x01")

    def calibration_ofs(self, servo_id, save=True, timeout=None):
        """对应 HLSCL::CalibrationOfs()：先关扭力、解锁 EPROM，再执行 CAL。

        CAL 会把位置偏移(31 号)写掉，而 31 号在 EPROM 里：**只有解锁（55 号 = 0）
        的时候写才会掉电保存**，所以解锁是必须的。

        ``save=True``（默认）在 CAL 之后补一次写锁（55 号 = 1），把舵机恢复成
        「EPROM 掉电不保存」的保护状态；CAL 本身已经写完并保存，上锁只是关掉
        后续误写 EPROM 的入口，不会撤销这次校准。``save=False`` 则保持解锁，
        方便连续做多次 CAL / 位置偏移调试。

        返回 ``(CAL 应答帧, 上锁应答帧或 None)``。
        """
        self.enable_torque(servo_id, 0)
        self.unlock_eprom(servo_id)
        frame = self.recal(servo_id, timeout=timeout)
        lock_frame = self.lock_eprom(servo_id) if save else None
        return frame, lock_frame

    # ------------------------------------------------------------------
    # 高频读取辅助
    # ------------------------------------------------------------------
    def feedback(self, servo_id, timeout=None):
        """一次读取 56~70 共 15 字节，与 HLSCL::FeedBack() 一致。"""
        return self.read(servo_id, ADDR_PRESENT_POSITION, 15, timeout=timeout)

    def read_bytes(self, servo_id, addr, length, timeout=None):
        return self.read(servo_id, addr, length, timeout=timeout).data

    def read_byte(self, servo_id, addr, timeout=None):
        return self.read(servo_id, addr, 1, timeout=timeout).data[0]

    def read_word(self, servo_id, addr, timeout=None):
        data = self.read(servo_id, addr, 2, timeout=timeout).data
        return data[0] | (data[1] << 8)

    def write_byte(self, servo_id, addr, value):
        return self.write(servo_id, addr, bytes([int(value) & 0xFF]))

    def write_word(self, servo_id, addr, value):
        return self.write(servo_id, addr, (int(value) & 0xFFFF).to_bytes(2, "little"))

    # ------------------------------------------------------------------
    # 扫描与原始收发
    # ------------------------------------------------------------------
    def scan(self, start=0, end=253, timeout=None):
        """按 ID 范围 Ping 扫描。

        默认扫满整个 0~253 合法 ID 空间：早期默认只扫到 20，导致 ID>20 的舵机
        被当成"扫描不到"（实测 ID 21 就是这么漏掉的）。
        """
        found = []
        start = max(0, min(253, int(start)))
        end = max(0, min(253, int(end)))
        if end < start:
            start, end = end, start
        for sid in range(start, end + 1):
            try:
                resp = self.ping(sid, timeout=timeout)
                found.append({"id": sid, "status": resp.status})
            except ProtocolError as exc:
                if exc.code == "not_connected":
                    raise
                continue
        return found

    def identify(self, servo_id, timeout=None):
        info = {"id": int(servo_id)}
        try:
            data = self.read(servo_id, 0, 5, timeout=timeout).data
            info.update({
                "firmware_major": data[0],
                "firmware_minor": data[1],
                "endian": data[2],
                "servo_major": data[3],
                "servo_minor": data[4],
            })
        except ProtocolError as exc:
            info["error"] = exc.message
        return info

    def raw_exchange(self, tx_bytes, wait_ms=100):
        """直接发送任意字节并收集串口返回，用于高级/协议分析。"""
        tx_bytes = bytes(tx_bytes)
        self._require_serial()
        wait_ms = max(1, int(wait_ms))
        with self._lock:
            self._clear_input()
            self._write(tx_bytes, "RAW TX")
            deadline = time.monotonic() + wait_ms / 1000.0
            rx = bytearray()
            while time.monotonic() < deadline:
                try:
                    chunk = self._serial.read(256)
                except Exception as exc:
                    raise ProtocolError("串口读取失败：%s" % exc, "serial_read", tx=tx_bytes, rx=bytes(rx))
                if chunk:
                    rx.extend(chunk)
                else:
                    time.sleep(0.002)
            if rx:
                self._log("RX", bytes(rx), "RAW RX")
            return bytes(rx)


def main_test():
    """打印协议自检帧，便于开发时核对。"""
    vectors = [
        (build_frame(200, INST_PING), "FF FF C8 02 01 34"),
        (build_frame(200, INST_READ, 56, b"\x0c"), "FF FF C8 04 02 38 0C ED"),
        (build_frame(200, INST_WRITE, 5, b"\x2a"), "FF FF C8 04 03 05 2A 01"),
    ]
    for frame, expected in vectors:
        got = _hex_text(frame)
        print("%-40s %s" % (got, "OK" if got == expected else "EXPECTED " + expected))


if __name__ == "__main__":
    main_test()
