# -*- coding: utf-8 -*-
"""HLS 舵机浏览器调试工具后端。

运行：
    uv run hls-debugger
或：
    .venv/bin/python -m hls_debugger
"""

from __future__ import absolute_import

import argparse
import glob
import os
import threading
import time
import webbrowser

from flask import Flask, jsonify, request, send_from_directory

from . import memory_table
from . import provision
from .memory_table import BAUD_CODES, BAUD_CODE_NAMES, MODE_NAMES
from .protocol import (
    BROADCAST_ID,
    EXPECTED_BAUD_CODE,
    FACTORY_ID,
    HLSBus,
    IMU_BUS_ID,
    ProtocolError,
    parse_frame,
)

try:
    from serial.tools import list_ports
except Exception:  # pragma: no cover
    list_ports = None


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
STATIC_DIR = os.path.join(BASE_DIR, "static")
DEFAULT_PORT = "/dev/ttyACM1"
DEFAULT_BAUD = 1000000

app = Flask(__name__, static_folder=STATIC_DIR, static_url_path="/static")
bus = HLSBus()
provisioner = provision.Provisioner(bus)


# ----------------------------------------------------------------------
# HTTP 辅助
# ----------------------------------------------------------------------
def ok(data=None, **extra):
    body = {"ok": True}
    if data is not None:
        body["data"] = data
    body.update(extra)
    return jsonify(body)


def fail(message, code="error", status_code=400, detail=None, **extra):
    body = {"ok": False, "error": message, "code": code}
    if detail is not None:
        body["detail"] = detail
    body.update(extra)
    return jsonify(body), status_code


def payload():
    data = request.get_json(silent=True)
    if data is None:
        data = {}
    return data


def payload_int(data, key, default=None, minimum=None, maximum=None):
    value = data.get(key, default)
    if value is None or value == "":
        value = default
    value = int(value)
    if minimum is not None and value < minimum:
        raise ValueError("%s 不能小于 %s" % (key, minimum))
    if maximum is not None and value > maximum:
        raise ValueError("%s 不能大于 %s" % (key, maximum))
    return value


def parse_int_list(value, name="ids"):
    if value is None:
        raise ValueError("缺少 %s" % name)
    if isinstance(value, (list, tuple)):
        items = value
    else:
        text = str(value).replace("，", ",").replace(";", ",").replace(" ", ",")
        items = [x for x in text.split(",") if x.strip() != ""]
    result = []
    for item in items:
        number = int(str(item).strip(), 0)
        result.append(number)
    return result


def parse_byte_list(value, name="data"):
    if value is None:
        raise ValueError("缺少 %s" % name)
    if isinstance(value, (list, tuple)):
        return bytes(int(x) & 0xFF for x in value)
    text = str(value).strip()
    if not text:
        return b""
    text = text.replace("0x", "").replace("0X", "")
    text = text.replace(",", " ").replace(";", " ").replace("-", " ").replace(":", " ")
    tokens = [x for x in text.split() if x]
    # 支持 "0102A0" 这种无分隔的连续 HEX
    if len(tokens) == 1 and len(tokens[0]) > 2 and all(
        c in "0123456789abcdefABCDEF" for c in tokens[0]
    ):
        token = tokens[0]
        if len(token) % 2:
            token = "0" + token
        tokens = [token[i:i + 2] for i in range(0, len(token), 2)]
    if not tokens:
        return b""
    return bytes(int(token, 16) & 0xFF for token in tokens)


def handle_protocol(fn):
    """统一处理 ProtocolError/ValueError 的路由包装。"""
    def wrapper(*args, **kwargs):
        try:
            return fn(*args, **kwargs)
        except ProtocolError as exc:
            return fail(exc.message, exc.code, 502, detail=exc.to_dict())
        except ValueError as exc:
            return fail(str(exc), "bad_request", 400)
        except Exception as exc:  # pragma: no cover
            app.logger.exception("unexpected error")
            return fail("内部错误：%s" % exc, "internal", 500)
    wrapper.__name__ = fn.__name__
    return wrapper


def frame_json(frame):
    if frame is None:
        return {"sent": True, "ack": None}
    result = frame.to_dict()
    result["sent"] = True
    result["ack"] = True
    return result


def read_result(servo_id, addr, frame):
    result = frame_json(frame)
    result["addr"] = addr
    try:
        result["fields"] = memory_table.decode_range(addr, frame.data)
    except Exception:
        result["fields"] = []
    return result


# ----------------------------------------------------------------------
# 页面与静态资源
# ----------------------------------------------------------------------
@app.route("/")
def index():
    return send_from_directory(STATIC_DIR, "index.html")


@app.route("/api/health", methods=["GET"])
def api_health():
    return ok({"name": "hls-servo-debugger", "time": time.time()})


@app.route("/api/ports", methods=["GET"])
def api_ports():
    ports = []
    seen = set()
    if list_ports is not None:
        for item in list_ports.comports():
            if item.device not in seen:
                seen.add(item.device)
                ports.append({
                    "device": item.device,
                    "description": item.description or "",
                    "hwid": item.hwid or "",
                })
    for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*", "/dev/ttyTHS*", "/dev/ttyAMA*"):
        for device in sorted(glob.glob(pattern)):
            if device not in seen:
                seen.add(device)
                ports.append({"device": device, "description": "检测到串口设备", "hwid": ""})
    for device in (DEFAULT_PORT, "/dev/ttyUSB0", "/dev/ttyACM0"):
        if device not in seen:
            ports.append({"device": device, "description": "默认候选", "hwid": ""})
    return ok({"ports": ports, "default_port": DEFAULT_PORT})


@app.route("/api/fields", methods=["GET"])
def api_fields():
    return ok({
        "fields": memory_table.get_fields(),
        "groups": memory_table.get_groups(),
        "baud_codes": BAUD_CODES,
        "baud_names": BAUD_CODE_NAMES,
        "mode_names": MODE_NAMES,
    })


# ----------------------------------------------------------------------
# 连接管理
# ----------------------------------------------------------------------
@app.route("/api/status", methods=["GET"])
@handle_protocol
def api_status():
    return ok(bus.status())


@app.route("/api/logs", methods=["GET"])
def api_logs():
    limit = int(request.args.get("limit", 300))
    return ok({"logs": bus.logs(limit)})


@app.route("/api/logs/clear", methods=["POST"])
def api_logs_clear():
    bus.clear_logs()
    return ok()


@app.route("/api/connect", methods=["POST"])
@handle_protocol
def api_connect():
    data = payload()
    port = data.get("port") or DEFAULT_PORT
    baudrate = int(data.get("baudrate") or data.get("baud") or DEFAULT_BAUD)
    timeout_ms = float(data.get("timeout_ms", 100))
    status = bus.connect(port, baudrate, timeout=max(0.01, timeout_ms / 1000.0))
    return ok(status)


@app.route("/api/disconnect", methods=["POST"])
def api_disconnect():
    bus.disconnect()
    return ok(bus.status())


@app.route("/api/auto_detect", methods=["POST"])
@handle_protocol
def api_auto_detect():
    """自动尝试常见波特率，并 Ping 指定 ID（默认 1）。"""
    data = payload()
    port = data.get("port") or DEFAULT_PORT
    bauds = data.get("bauds") or [1000000, 115200, 500000, 250000, 76800, 57600, 38400]
    ids = data.get("ids") or [1]
    timeout_ms = float(data.get("timeout_ms", 80))
    timeout_s = max(0.01, timeout_ms / 1000.0)
    bauds = [int(x) for x in bauds]
    ids = [int(x) for x in parse_int_list(ids, "ids")]
    last_error = None

    for baud in bauds:
        try:
            bus.connect(port, baud, timeout=timeout_s)
        except ProtocolError as exc:
            last_error = exc
            continue
        for sid in ids:
            try:
                frame = bus.ping(sid, timeout=timeout_s)
                return ok({
                    "found": True,
                    "id": sid,
                    "baudrate": baud,
                    "status": frame.status,
                    "port": port,
                })
            except ProtocolError as exc:
                last_error = exc
                continue
        bus.disconnect()

    if last_error is not None:
        return fail("自动探测失败：%s" % last_error.message, "auto_detect_failed", 502)
    return fail("自动探测失败：未找到舵机", "auto_detect_failed", 502)


@app.route("/api/scan", methods=["POST"])
@handle_protocol
def api_scan():
    data = payload()
    start = payload_int(data, "start", 0, 0, 253)
    end = payload_int(data, "end", 253, 0, 253)
    timeout_ms = float(data.get("timeout_ms", 40))
    identify = bool(data.get("identify", False))
    found = bus.scan(start, end, timeout=timeout_ms / 1000.0)
    if identify:
        for item in found:
            info = bus.identify(item["id"])
            item.update(info)
    return ok({"start": start, "end": end, "found": found, "count": len(found)})


# ----------------------------------------------------------------------
# 基础指令
# ----------------------------------------------------------------------
@app.route("/api/ping", methods=["POST"])
@handle_protocol
def api_ping():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    timeout_ms = float(data.get("timeout_ms", 100))
    frame = bus.ping(sid, timeout=timeout_ms / 1000.0)
    return ok({"ping": frame_json(frame), "id": sid, "status": frame.status})


@app.route("/api/read", methods=["POST"])
@handle_protocol
def api_read():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    addr = payload_int(data, "addr", 0, 0, 255)
    length = payload_int(data, "length", 1, 1, 250)
    timeout_ms = float(data.get("timeout_ms", 100))
    frame = bus.read(sid, addr, length, timeout=timeout_ms / 1000.0)
    return ok(read_result(sid, addr, frame))


@app.route("/api/write", methods=["POST"])
@handle_protocol
def api_write():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    addr = payload_int(data, "addr", 0, 0, 255)
    raw = parse_byte_list(data.get("data", data.get("hex")), "data")
    timeout_ms = float(data.get("timeout_ms", 100))
    if not raw:
        raise ValueError("请提供要写入的十六进制字节，例如 00 或 01 02")
    frame = bus.write(sid, addr, raw, timeout=timeout_ms / 1000.0)
    return ok({"write": frame_json(frame), "addr": addr, "data": list(raw)})


@app.route("/api/write_field", methods=["POST"])
@handle_protocol
def api_write_field():
    """按内存表字段类型编码后写入。"""
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    addr = payload_int(data, "addr", 0, 0, 255)
    field = memory_table.find_field(addr)
    if field is None:
        raise ValueError("地址 %s 没有已知字段定义" % addr)
    if field.get("access") != "rw":
        raise ValueError("字段 %s 为只读，不能写入" % field["name"])
    value = data.get("value")
    if value is None or value == "":
        raise ValueError("缺少 value")
    raw = memory_table.encode_value(field, int(value))
    timeout_ms = float(data.get("timeout_ms", 100))
    frame = bus.write(sid, addr, raw, timeout=timeout_ms / 1000.0)
    return ok({
        "write": frame_json(frame),
        "field": field,
        "value": int(value),
        "raw": list(raw),
    })


@app.route("/api/reg_write", methods=["POST"])
@handle_protocol
def api_reg_write():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    addr = payload_int(data, "addr", 0, 0, 255)
    raw = parse_byte_list(data.get("data", data.get("hex")), "data")
    timeout_ms = float(data.get("timeout_ms", 100))
    frame = bus.reg_write(sid, addr, raw, timeout=timeout_ms / 1000.0)
    return ok({"reg_write": frame_json(frame), "addr": addr, "data": list(raw)})


@app.route("/api/reg_action", methods=["POST"])
@handle_protocol
def api_reg_action():
    data = payload()
    sid = payload_int(data, "id", BROADCAST_ID, 0, BROADCAST_ID)
    timeout_ms = float(data.get("timeout_ms", 100))
    if sid == BROADCAST_ID:
        bus.reg_action(BROADCAST_ID)
        return ok({"broadcast": True, "sent": True, "ack": None})
    frame = bus.reg_action(sid, timeout=timeout_ms / 1000.0)
    return ok({"reg_action": frame_json(frame)})


@app.route("/api/reset", methods=["POST"])
@handle_protocol
def api_reset():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    timeout_ms = float(data.get("timeout_ms", 200))
    frame = bus.reset(sid, timeout=timeout_ms / 1000.0)
    return ok({"reset": frame_json(frame)})


@app.route("/api/recovery", methods=["POST"])
@handle_protocol
def api_recovery():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    timeout_ms = float(data.get("timeout_ms", 200))
    frame = bus.recovery(sid, timeout=timeout_ms / 1000.0)
    return ok({"recovery": frame_json(frame)})


@app.route("/api/calibrate", methods=["POST"])
@handle_protocol
def api_calibrate():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    timeout_ms = float(data.get("timeout_ms", 200))
    frame = bus.calibration_ofs(sid)
    return ok({"calibrate": frame_json(frame)})


# ----------------------------------------------------------------------
# HLS 应用层控制
# ----------------------------------------------------------------------
@app.route("/api/feedback", methods=["POST"])
@handle_protocol
def api_feedback():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    timeout_ms = float(data.get("timeout_ms", 100))
    frame = bus.feedback(sid, timeout=timeout_ms / 1000.0)
    result = frame_json(frame)
    result["raw"] = result.get("data")
    result["feedback"] = memory_table.feedback_to_dict(frame.data)
    return ok(result)


@app.route("/api/mode", methods=["POST"])
@handle_protocol
def api_mode():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    mode = payload_int(data, "mode", 0, 0, 4)
    frame = bus.set_mode(sid, mode)
    return ok({"mode": frame_json(frame), "mode_value": mode, "mode_name": MODE_NAMES.get(mode)})


@app.route("/api/torque", methods=["POST"])
@handle_protocol
def api_torque():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    enable = payload_int(data, "enable", 1, 0, 2)
    frame = bus.enable_torque(sid, enable)
    return ok({"torque": frame_json(frame), "enable": enable})


@app.route("/api/move", methods=["POST"])
@handle_protocol
def api_move():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    position = payload_int(data, "position", 0, -32767, 32767)
    speed = payload_int(data, "speed", 0, 0, 65535)
    acc = payload_int(data, "acc", 0, 0, 254)
    torque = payload_int(data, "torque", 0, 0, 65535)
    reg = bool(data.get("reg", False))
    timeout_ms = float(data.get("timeout_ms", 100))
    frame = bus.write_pos_ex(sid, position, speed, acc, torque, reg=reg)
    return ok({
        "move": frame_json(frame),
        "position": position,
        "speed": speed,
        "acc": acc,
        "torque": torque,
        "reg": reg,
    })


@app.route("/api/wheel", methods=["POST"])
@handle_protocol
def api_wheel():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    speed = payload_int(data, "speed", 0, -32767, 32767)
    acc = payload_int(data, "acc", 0, 0, 254)
    torque = payload_int(data, "torque", 0, 0, 65535)
    reg = bool(data.get("reg", False))
    timeout_ms = float(data.get("timeout_ms", 100))
    frame = bus.write_speed(sid, speed, acc, torque, reg=reg)
    return ok({"wheel": frame_json(frame), "speed": speed, "acc": acc, "torque": torque, "reg": reg})


@app.route("/api/electric", methods=["POST"])
@handle_protocol
def api_electric():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    torque = payload_int(data, "torque", 0, -32767, 32767)
    frame = bus.write_torque_current(sid, torque)
    return ok({"electric": frame_json(frame), "torque": torque})


@app.route("/api/pwm", methods=["POST"])
@handle_protocol
def api_pwm():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    value = payload_int(data, "value", 0, -1000, 1000)
    frame = bus.write_pwm(sid, value)
    return ok({"pwm": frame_json(frame), "value": value})


@app.route("/api/lock", methods=["POST"])
@handle_protocol
def api_lock():
    data = payload()
    sid = payload_int(data, "id", 1, 0, 253)
    lock = payload_int(data, "lock", 1, 0, 1)
    if lock == 0:
        frame = bus.unlock_eprom(sid)
    else:
        frame = bus.lock_eprom(sid)
    return ok({"lock": frame_json(frame), "lock_value": lock})


# ----------------------------------------------------------------------
# 同步操作与高级原始指令
# ----------------------------------------------------------------------
@app.route("/api/sync_read", methods=["POST"])
@handle_protocol
def api_sync_read():
    data = payload()
    ids = [int(x) for x in parse_int_list(data.get("ids"), "ids")]
    for sid in ids:
        if not 0 <= sid <= 253:
            raise ValueError("同步读 ID 必须在 0~253")
    addr = payload_int(data, "addr", 56, 0, 255)
    length = payload_int(data, "length", 15, 1, 250)
    timeout_ms = float(data.get("timeout_ms", 150))
    frames = bus.sync_read(ids, addr, length, timeout=timeout_ms / 1000.0)
    result = {}
    for sid in ids:
        frame = frames[sid]
        item = frame_json(frame)
        item["addr"] = addr
        item["fields"] = memory_table.decode_range(addr, frame.data)
        if addr == 56 and len(frame.data) >= 15:
            try:
                item["feedback"] = memory_table.feedback_to_dict(frame.data)
            except Exception:
                pass
        result[str(sid)] = item
    return ok({"addr": addr, "length": length, "results": result})


@app.route("/api/sync_write", methods=["POST"])
@handle_protocol
def api_sync_write():
    data = payload()
    ids = [int(x) for x in parse_int_list(data.get("ids"), "ids")]
    values_raw = data.get("values")
    if not isinstance(values_raw, (list, tuple)):
        raise ValueError("values 必须是数组，每个元素对应一个 ID 的字节数据")
    if len(values_raw) != len(ids):
        raise ValueError("ids 与 values 数量不一致")
    values = [parse_byte_list(item, "values") for item in values_raw]
    addr = payload_int(data, "addr", 42, 0, 255)
    frame = bus.sync_write(ids, addr, values)
    return ok({"sync_write": frame_json(frame), "ids": ids, "addr": addr})


@app.route("/api/sync_move", methods=["POST"])
@handle_protocol
def api_sync_move():
    data = payload()
    ids = [int(x) for x in parse_int_list(data.get("ids"), "ids")]
    positions = [int(x) for x in parse_int_list(data.get("positions"), "positions")]
    speeds = [int(x) for x in parse_int_list(data.get("speeds"), "speeds")]
    accs = data.get("accs")
    torques = data.get("torques")
    if accs is None or str(accs).strip() == "":
        accs = [0] * len(ids)
    else:
        accs = [int(x) for x in parse_int_list(accs, "accs")]
    if torques is None or str(torques).strip() == "":
        torques = [0] * len(ids)
    else:
        torques = [int(x) for x in parse_int_list(torques, "torques")]
    if not (len(ids) == len(positions) == len(speeds) == len(accs) == len(torques)):
        raise ValueError("ids/positions/speeds/accs/torques 长度必须一致")
    frame = bus.sync_write_pos_ex(ids, positions, speeds, accs, torques)
    return ok({"sync_move": frame_json(frame), "ids": ids, "count": len(ids)})


@app.route("/api/sync_wheel", methods=["POST"])
@handle_protocol
def api_sync_wheel():
    data = payload()
    ids = [int(x) for x in parse_int_list(data.get("ids"), "ids")]
    speeds = [int(x) for x in parse_int_list(data.get("speeds"), "speeds")]
    accs = data.get("accs")
    torques = data.get("torques")
    if accs is None or str(accs).strip() == "":
        accs = [0] * len(ids)
    else:
        accs = [int(x) for x in parse_int_list(accs, "accs")]
    if torques is None or str(torques).strip() == "":
        torques = [0] * len(ids)
    else:
        torques = [int(x) for x in parse_int_list(torques, "torques")]
    if not (len(ids) == len(speeds) == len(accs) == len(torques)):
        raise ValueError("ids/speeds/accs/torques 长度必须一致")
    frame = bus.sync_write_speed(ids, speeds, accs, torques)
    return ok({"sync_wheel": frame_json(frame), "ids": ids, "count": len(ids)})


@app.route("/api/raw", methods=["POST"])
@handle_protocol
def api_raw():
    data = payload()
    tx = parse_byte_list(data.get("tx_hex", data.get("tx")), "tx_hex")
    wait_ms = payload_int(data, "wait_ms", 100, 1, 10000)
    rx = bus.raw_exchange(tx, wait_ms=wait_ms)
    parsed = None
    try:
        parsed = parse_frame(rx).to_dict() if rx else None
    except Exception:
        parsed = None
    return ok({
        "tx_hex": tx.hex(" ").upper(),
        "rx_hex": rx.hex(" ").upper(),
        "rx_bytes": list(rx),
        "first_frame": parsed,
    })


# ----------------------------------------------------------------------
# 整机 15 台舵机的初始化（编号 + 位置校准）
# ----------------------------------------------------------------------
@app.route("/api/joints", methods=["GET"])
def api_joints():
    """microduck 的 15 个关节表，以及初始化相关的常量。"""
    return ok({
        "joints": provision.get_joints(),
        "groups": provision.JOINT_GROUPS,
        "calibrate_modes": provision.calibrate_modes(),
        "factory_id": FACTORY_ID,
        "imu_bus_id": IMU_BUS_ID,
        "expected_baud_code": EXPECTED_BAUD_CODE,
        "baud_names": BAUD_CODE_NAMES,
        "gain_kp_default": provision.POSITION_KP_DEFAULT,
        "gain_kd_default": provision.POSITION_KD_DEFAULT,
        "gain_max": provision.POSITION_GAIN_MAX,
        "gain_note": "位置环增益：初始化写 EEPROM 21/22 + RAM 50/51，"
                     "Kp 默认 %d（厂商默认）。实测 Kp=200 会让关节自激振荡"
                     "（5 A、74 °C），那是 robotd 给 XL330 标度调的 gain；"
                     "robotd 启动后会按 robotd.toml 的 gain 覆盖 RAM 50，两边要一致。"
                     % provision.POSITION_KP_DEFAULT,
        "position_wrap": provision.POSITION_WRAP,
        "position_tolerance": provision.POSITION_TOLERANCE,
        "position_note": "实测本机：位置(56) ≡ 编码器值 − 位置偏移(31) (mod 4096)；"
                         "负值有时回绕成 0~4095、有时带 BIT15 报负值，"
                         "所以校准按一圈取模比较并留 %d 计数容差。"
                         % provision.POSITION_TOLERANCE,
        "multiturn_note": "实测本机：出厂 11 号最大角度限制 = 4095 会把行程锁在一圈内，"
                          "写负目标位置舵机不动；把 9/11 号都写成 0（多圈绝对位置控制）"
                          "之后负目标才会执行。microduck 有 4 个负角关节，因此初始化默认打开。",
        "total": len(provision.JOINTS),
    })


@app.route("/api/provision/precheck", methods=["POST"])
@handle_protocol
def api_provision_precheck():
    """初始化前的上电检查：该在的在不在、不该在的别在。"""
    data = payload()
    if data.get("id") in (None, ""):
        raise ValueError("缺少 id（要初始化的关节 ID）")
    target_id = payload_int(data, "id", None, 0, 253)
    mode = str(data.get("mode") or "fresh")
    source_id = FACTORY_ID if mode != "reinit" else target_id
    if data.get("source_id") not in (None, ""):
        source_id = payload_int(data, "source_id", source_id, 0, 253)
    return ok(provisioner.precheck(target_id, source_id))


@app.route("/api/provision", methods=["POST"])
@handle_protocol
def api_provision():
    """把一颗舵机初始化成指定关节 ID，并按选择完成初始位置校准。

    这会写 EPROM（ID/波特率/应答级别/位置偏移）并可能让舵机转动，
    调用前必须由用户确认机械安全。
    """
    data = payload()
    if data.get("id") in (None, ""):
        raise ValueError("缺少 id（要初始化的关节 ID）")
    target_id = payload_int(data, "id", None, 0, 253)
    calibrate = str(data.get("calibrate") or "cal")
    mode = str(data.get("mode") or "fresh")
    write_response_level = bool(data.get("write_response_level", True))
    write_gain = bool(data.get("write_gain", True))
    gain_kp = payload_int(data, "gain_kp", provision.POSITION_KP_DEFAULT,
                          0, provision.POSITION_GAIN_MAX)
    gain_kd = payload_int(data, "gain_kd", provision.POSITION_KD_DEFAULT,
                          0, provision.POSITION_GAIN_MAX)
    multiturn = bool(data.get("multiturn", True))
    speed = payload_int(data, "speed", 60, -32767, 32767)
    acc = payload_int(data, "acc", 30, 0, 254)
    timeout_ms = float(data.get("timeout_ms", 100))
    result = provisioner.provision(
        target_id,
        calibrate=calibrate,
        write_response_level=write_response_level,
        write_gain=write_gain,
        gain_kp=gain_kp,
        gain_kd=gain_kd,
        multiturn=multiturn,
        mode=mode,
        speed=speed,
        acc=acc,
        timeout=max(0.01, timeout_ms / 1000.0),
    )
    return ok(result)


@app.route("/api/provision/census", methods=["POST"])
@handle_protocol
def api_provision_census():
    """点检：逐个 PING 15 个关节，返回在线的和它们的状态。"""
    return ok(provisioner.census())


@app.route("/api/memory_map", methods=["GET"])
def api_memory_map():
    return ok({
        "fields": memory_table.get_fields(),
        "groups": memory_table.get_groups(),
        "baud_codes": BAUD_CODES,
        "baud_names": BAUD_CODE_NAMES,
        "mode_names": MODE_NAMES,
    })


def main(argv=None):
    parser = argparse.ArgumentParser(description="HLS 舵机浏览器调试工具")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true", help="不自动打开浏览器")
    parser.add_argument("--debug", action="store_true", help="Flask 调试模式")
    args = parser.parse_args(argv)

    url = "http://%s:%s/" % (args.host, args.port)
    if not args.no_browser:
        threading.Timer(0.8, lambda: webbrowser.open(url)).start()
    print("HLS 舵机调试工具已启动：%s" % url)
    app.run(host=args.host, port=args.port, debug=args.debug, threaded=True)


if __name__ == "__main__":
    main()
