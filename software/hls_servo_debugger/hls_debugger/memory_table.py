# -*- coding: utf-8 -*-
"""HLS 系列舵机内存表定义、编解码与常用换算。

数据来源：飞特官方《磁编码 HLS 舵机-内存表解析》以及
FTServo_Linux-main/src/HLSCL.h。这里只做主机侧调试工具需要的定义。
"""

from __future__ import absolute_import

BAUD_CODES = {
    0: 1000000,
    1: 500000,
    2: 250000,
    3: 128000,
    4: 115200,
    5: 76800,
    6: 57600,
    7: 38400,
}

BAUD_CODE_NAMES = {
    0: "1 Mbps",
    1: "500 kbps",
    2: "250 kbps",
    3: "128 kbps",
    4: "115200",
    5: "76800",
    6: "57600",
    7: "38400",
}

MODE_NAMES = {
    0: "位置伺服模式（位置+限流）",
    1: "恒速模式（恒速+限流）",
    2: "恒流模式（限流，速度不可控）",
    3: "PWM 开环调速模式",
    # 厂商《HLS 系列舵机内存表》25 号"积分限制值"备注写明"位置模式 0 与模式 4 生效"，
    # 说明新固件存在模式 4（位置类）。实测舵机（固件 3.46 / 舵机 10.31）33 号即为 4。
    4: "位置模式（新固件，与模式 0 同属位置环）",
}

TORQUE_ENABLE_NAMES = {
    0: "关闭扭力输出",
    1: "打开扭力输出",
    2: "阻尼输出",
}

RESPONSE_LEVEL_NAMES = {
    0: "除读/PING外不应答",
    1: "所有指令均应答",
}

LOCK_NAMES = {
    0: "关闭写入锁（EPROM 掉电保存）",
    1: "打开写入锁（EPROM 掉电不保存）",
}

PHASE_BITS = [
    {"bit": 0, "name": "伺服相位 / 磁编码类型（0 AS5600 / 1 MT6701）"},
    {"bit": 1, "name": "电流反馈方向相位（0 正向 / 1 反向）"},
    {"bit": 2, "name": "驱动桥方向相位（0 正向 / 1 反向）"},
    {"bit": 3, "name": "速度方向相位（固件 <=3.41）"},
    {"bit": 4, "name": "角度反馈模式（0 单圈 / 1 全角度）"},
    {"bit": 5, "name": "驱动桥配置（0 独立 H 桥 / 1 集成 H 桥）"},
    {"bit": 6, "name": "PWM 频率（0 24kHz / 1 16kHz）"},
    {"bit": 7, "name": "位置反馈方向相位（0 正向 / 1 反向）"},
]

PROTECT_BITS = [
    {"bit": 0, "name": "电压保护"},
    {"bit": 1, "name": "磁编码保护"},
    {"bit": 2, "name": "过热保护"},
    {"bit": 3, "name": "过流保护"},
]

STATUS_BITS = [
    {"bit": 0, "name": "电压状态异常"},
    {"bit": 1, "name": "磁编码状态异常"},
    {"bit": 2, "name": "温度状态异常"},
    {"bit": 3, "name": "电流状态异常"},
]

MOVING_BITS = [
    {"bit": 0, "name": "运动中"},
    {"bit": 1, "name": "运动中（到达目标并停止后清零）"},
]


def _field(name, addr, length, group, access="rw", dtype="u8", **kwargs):
    data = {
        "name": name,
        "addr": addr,
        "hex": "0x%02X" % addr,
        "length": length,
        "group": group,
        "access": access,
        "dtype": dtype,
    }
    data.update(kwargs)
    return data


FIELDS = [
    # 2.1 版本信息
    _field("固件主版本号", 0, 1, "version", "r", "u8", desc="出厂初始值 3"),
    _field("固件次版本号", 1, 1, "version", "r", "u8", desc="40~59"),
    _field("END", 2, 1, "version", "r", "u8", desc="0 表示小端存储结构"),
    _field("舵机主版本号", 3, 1, "version", "r", "u8", desc="出厂初始值 10"),
    _field("舵机次版本号", 4, 1, "version", "r", "u8", desc="--"),

    # 2.2 EPROM 配置
    _field("主 ID", 5, 1, "eprom", "rw", "u8", min=0, max=253, default=1,
           unit="号", desc="总线上唯一的主 ID（推荐 0~252，避免 0xFD）"),
    _field("波特率", 6, 1, "eprom", "rw", "enum", options=BAUD_CODE_NAMES, default=0,
           unit="编码", desc="0=1M, 1=500k, 2=250k, 3=128k, 4=115200, 5=76800, 6=57600, 7=38400"),
    _field("副 ID", 7, 1, "eprom", "rw", "u8", min=0, max=253, default=0,
           unit="号", desc="副 ID 适用于写、异常写、异常执行与同步写指令；实测出厂常见 253(0xFD) 表示未设置"),
    _field("应答状态级别", 8, 1, "eprom", "rw", "enum", options=RESPONSE_LEVEL_NAMES,
           min=0, max=1, default=1, desc="0=除读/PING外静默；1=所有指令均应答"),
    _field("最小角度限制", 9, 2, "eprom", "rw", "u16", min=0, max=4094,
           scale=0.087, unit="°", default=0,
           desc="多圈绝对位置控制时该值为 0（与 11 号同时写 0）"),
    _field("最大角度限制", 11, 2, "eprom", "rw", "u16", min=0, max=4095,
           scale=0.087, unit="°", default=4095,
           desc="多圈绝对位置控制时该值为 0（与 9 号同时写 0）；出厂 4095 会把行程锁在一圈内，"
                "负目标位置会被夹到 0——真机实测确认"),
    _field("最高温度上限", 13, 1, "eprom", "rw", "u8", min=0, max=100,
           unit="°C", default=70, desc="超过该温度可能触发保护"),
    _field("最高输入电压", 14, 1, "eprom", "rw", "u8", min=0, max=254,
           scale=0.1, unit="V", desc="0.1 V/计数"),
    _field("最低输入电压", 15, 1, "eprom", "rw", "u8", min=0, max=254,
           scale=0.1, unit="V", default=40, desc="0.1 V/计数，初始值约 4.0 V"),
    _field("最大扭矩", 16, 2, "eprom", "rw", "u16", min=0, max=1000,
           scale=0.1, unit="%", default=980, desc="上电赋值给 48 号转矩限制"),
    _field("相位", 18, 1, "eprom", "rw", "bitfield", bits=PHASE_BITS,
           min=0, max=254, desc="特殊功能字节，无特别需求不可修改"),
    _field("卸载条件", 19, 1, "eprom", "rw", "bitfield", bits=PROTECT_BITS,
           min=0, max=254, desc="对应位置 1 表示开启相应保护"),
    _field("LED 报警条件", 20, 1, "eprom", "rw", "bitfield", bits=PROTECT_BITS,
           min=0, max=254, desc="对应位置 1 表示开启闪灯报警"),
    _field("位置环 P 比例系数", 21, 1, "eprom", "rw", "u8", min=0, max=254,
           desc="上电赋值给 50 号 Kp"),
    _field("位置环 D 微分系数", 22, 1, "eprom", "rw", "u8", min=0, max=254,
           desc="上电赋值给 51 号 Kd"),
    _field("位置环 I 积分系数", 23, 1, "eprom", "rw", "u8", min=0, max=254,
           default=0, desc="上电赋值给 52 号 Ki（位置模式无效）"),
    _field("最小启动力", 24, 1, "eprom", "rw", "u8", min=0, max=254,
           scale=0.1, unit="%", desc="设置最小输出启动扭矩"),
    _field("积分限制值", 25, 1, "eprom", "rw", "u8", min=0, max=254,
           default=0, desc="最大积分值 = 该值 * 4；0=关闭"),
    _field("正向不灵敏区", 26, 1, "eprom", "rw", "u8", min=0, max=16,
           scale=0.087, unit="°", default=1),
    _field("负向不灵敏区", 27, 1, "eprom", "rw", "u8", min=0, max=16,
           scale=0.087, unit="°", default=1),
    _field("保护电流", 28, 2, "eprom", "rw", "u16", min=0, max=2047,
           scale=6.5, unit="mA", desc="上电赋值给 44 号目标电流"),
    _field("角度分辨率", 30, 1, "eprom", "rw", "u8", min=1, max=128,
           default=1, desc="传感器最小分辨角度的放大系数"),
    _field("位置偏移", 31, 2, "eprom", "rw", "i16", sign_bit=15,
           min=-4095, max=4095, scale=0.087, unit="°", default=0,
           desc="BIT15 为方向位；CAL 指令会写入该值"),
    _field("运行模式", 33, 1, "eprom", "rw", "enum", options=MODE_NAMES,
           min=0, max=4, default=0,
           desc="0 位置 / 1 恒速 / 2 恒流 / 3 PWM 开环 / 4 位置（新固件）"),
    _field("电流环 P 比例系数", 34, 1, "eprom", "rw", "u8", min=0, max=254),
    _field("电流环 I 积分系数", 35, 1, "eprom", "rw", "u8", min=0, max=254),
    _field("无定义", 36, 1, "eprom", "rw", "u8", min=0, max=255),
    _field("速度闭环 P 比例系数", 37, 1, "eprom", "rw", "u8", min=0, max=254,
           desc="恒速模式(1)下的速度环比例系数"),
    _field("过流保护时间", 38, 1, "eprom", "rw", "u8", min=0, max=254,
           scale=10, unit="ms", default=200),
    _field("速度闭环 I 积分系数", 39, 1, "eprom", "rw", "u8", min=0, max=254,
           desc="恒速模式(1)下的速度环积分系数"),

    # 2.3 SRAM 控制
    _field("扭矩开关", 40, 1, "sram", "rw", "enum", options=TORQUE_ENABLE_NAMES,
           min=0, max=2, default=0, desc="0 关 / 1 开 / 2 阻尼输出"),
    _field("加速度", 41, 1, "sram", "rw", "u8", min=0, max=254,
           scale=8.7, unit="°/s²", default=0, desc="0 表示最大加速度"),
    _field("目标位置", 42, 2, "sram", "rw", "i16", sign_bit=15,
           min=-32767, max=32767, scale=0.087, unit="°", default=0,
           desc="BIT15 为方向位；4096 计数/圈，可多圈"),
    _field("目标电流/PWM", 44, 2, "sram", "rw", "i16", sign_bit=15,
           min=-2047, max=2047, scale=6.5, unit="mA", default=None,
           desc="模式 0~2：目标电流，6.5 mA/计数，BIT15 方向；模式 3：PWM 量程 -1000~1000，BIT10 方向"),
    _field("运行速度", 46, 2, "sram", "rw", "i16", sign_bit=15,
           min=-32767, max=32767, scale=0.732, unit="RPM", default=None,
           desc="控制最高运行速度；0=停止；恒速模式 BIT15 为方向位"),
    _field("转矩限制", 48, 2, "sram", "rw", "u16", min=0, max=1000,
           scale=0.1, unit="%", default=None, desc="上电由 16 号最大扭矩赋值"),
    _field("Kp", 50, 1, "sram", "rw", "u8", min=0, max=254,
           desc="位置模式位置环比例系数(1/8)；恒速模式速度环比例系数"),
    _field("Kd", 51, 1, "sram", "rw", "u8", min=0, max=254,
           desc="位置模式位置环微分系数(1/4)；恒速模式无效"),
    _field("Ki", 52, 1, "sram", "rw", "u8", min=0, max=254,
           desc="位置模式无效；恒速模式速度环积分系数"),
    _field("无定义", 53, 1, "sram", "rw", "u8", min=0, max=255),
    _field("无定义", 54, 1, "sram", "rw", "u8", min=0, max=255),
    _field("锁标志", 55, 1, "sram", "rw", "enum", options=LOCK_NAMES,
           min=0, max=1, default=1, desc="0=EPROM 掉电保存；1=EPROM 掉电不保存"),

    # 2.4 SRAM 反馈
    _field("当前位置", 56, 2, "feedback", "r", "i16", sign_bit=15,
           scale=0.087, unit="°", desc="BIT15 为方向位，4096 计数/圈"),
    _field("当前速度", 58, 2, "feedback", "r", "i16", sign_bit=15,
           scale=0.732, unit="RPM", desc="BIT15 为方向位"),
    _field("当前负载", 60, 2, "feedback", "r", "i16", sign_bit=10,
           scale=0.1, unit="%", desc="BIT10 为方向位（注意不是 BIT15）"),
    _field("当前电压", 62, 1, "feedback", "r", "u8", scale=0.1, unit="V"),
    _field("当前温度", 63, 1, "feedback", "r", "u8", unit="°C"),
    _field("异步写标志", 64, 1, "feedback", "r", "u8", default=0),
    _field("舵机状态", 65, 1, "feedback", "r", "bitfield", bits=STATUS_BITS),
    _field("移动标志", 66, 1, "feedback", "r", "bitfield", bits=MOVING_BITS,
           desc="BIT0 运动中；BIT1 运动中，到达目标停止后清零"),
    _field("目标位置（回读）", 67, 2, "feedback", "r", "i16", sign_bit=15,
           scale=0.087, unit="°"),
    _field("当前电流", 69, 2, "feedback", "r", "i16", sign_bit=15,
           scale=6.5, unit="mA", desc="BIT15 为电流方向位"),
    _field("无定义", 71, 2, "feedback", "r", "u16"),
    _field("电流偏置", 73, 2, "feedback", "r", "u16"),

    # 2.5 出厂参数
    _field("vFk(*10)", 77, 1, "factory", "r", "u8"),
    _field("vKgI", 78, 1, "factory", "r", "u8"),
    _field("pFk(*10)", 79, 1, "factory", "r", "u8"),
    _field("移动速度阀值", 80, 1, "factory", "r", "u8"),
    _field("DTs(ms)", 81, 1, "factory", "r", "u8"),
    _field("eFk(*10)", 82, 1, "factory", "r", "u8"),
    _field("Vk(ms)", 83, 1, "factory", "r", "u8"),
    _field("最大速度限制", 84, 1, "factory", "r", "u8"),
    _field("加速度限制", 85, 1, "factory", "r", "u8"),
    _field("加速度倍数", 86, 1, "factory", "r", "u8"),
]

GROUPS = [
    {"id": "version", "name": "版本信息", "start": 0, "end": 4,
     "desc": "只读版本号与字节序标志"},
    {"id": "eprom", "name": "EPROM 配置", "start": 5, "end": 39,
     "desc": "需要关注写入锁；建议先关闭扭力再改关键参数"},
    {"id": "sram", "name": "SRAM 控制", "start": 40, "end": 55,
     "desc": "运行时可写的控制量"},
    {"id": "feedback", "name": "SRAM 反馈", "start": 56, "end": 73,
     "desc": "只读状态量，推荐一次读取 56~70（15 字节）"},
    {"id": "factory", "name": "出厂参数", "start": 77, "end": 86,
     "desc": "只读，一般无需修改"},
]


def get_fields():
    """返回完整字段定义的浅拷贝，方便 JSON 序列化。"""
    return [dict(item) for item in FIELDS]


def get_groups():
    return [dict(item) for item in GROUPS]


def find_field(addr):
    for item in FIELDS:
        if item["addr"] == addr:
            return item
    return None


def fields_in_range(start, length):
    end = start + length
    result = []
    for item in FIELDS:
        if item["addr"] >= start and item["addr"] + item["length"] <= end:
            result.append(item)
    return result


def _le(raw_bytes):
    value = 0
    for i, byte in enumerate(raw_bytes):
        value |= int(byte) << (8 * i)
    return value


def _be(raw_bytes):
    value = 0
    for byte in raw_bytes:
        value = (value << 8) | int(byte)
    return value


def decode_signed_magnitude(raw, sign_bit=15):
    """飞特 16 位字段方向位约定：BIT15 为 1 表示负，其余位为绝对值。"""
    raw = int(raw)
    mask = 1 << sign_bit
    if raw & mask:
        return -(raw & ~mask)
    return raw


def encode_signed_magnitude(value, sign_bit=15):
    value = int(value)
    mask = 1 << sign_bit
    if value < 0:
        magnitude = -value
        if magnitude > (mask - 1):
            raise ValueError("负值超出可表示范围")
        return magnitude | mask
    if value > (mask - 1):
        raise ValueError("正值超出可表示范围")
    return value


def decode_value(field, raw):
    dtype = field.get("dtype", "u8")
    raw = int(raw)
    if dtype == "i16":
        return decode_signed_magnitude(raw, field.get("sign_bit", 15))
    return raw


def encode_value(field, value):
    """把用户输入转换成小端字节。"""
    dtype = field.get("dtype", "u8")
    length = int(field.get("length", 1))
    if dtype == "i16":
        raw = encode_signed_magnitude(int(value), field.get("sign_bit", 15))
    else:
        raw = int(value)
        if field.get("min") is not None and raw < int(field["min"]):
            raise ValueError("数值小于允许范围")
        if field.get("max") is not None and raw > int(field["max"]):
            raise ValueError("数值大于允许范围")
    if raw < 0:
        raise ValueError("无符号字段不能为负")
    if raw >= (1 << (8 * length)):
        raise ValueError("数值超出字段长度")
    return raw.to_bytes(length, "little")


def scaled_value(field, value):
    """把原始计数换算成工程量（如 电压 44 -> 4.4 V）。无 scale 时返回 None。

    注意：只用于显示。写回总线时仍然使用原始计数值，避免换算误差。
    """
    scale = field.get("scale")
    if not scale or isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    if field.get("dtype") == "enum":
        return None
    return round(value * scale, 4)


def format_value(field, value, scaled):
    """生成"原始值 + 工程量"的显示文本，例如 '44 计数 ≈ 4.4 V'。"""
    unit = field.get("unit") or ""
    name = field.get("options", {}).get(value) if field.get("dtype") == "enum" else None
    if name:
        return "%s (%s)" % (value, name)
    if scaled is None:
        return ("%s %s" % (value, unit)).strip()
    return "%s %s ≈ %s %s" % (value, "计数", scaled, unit)


def decode_range(start, data):
    """解码一段连续内存数据，返回与字段表重叠的字段。"""
    data = bytes(data)
    result = []
    for field in fields_in_range(start, len(data)):
        offset = field["addr"] - start
        raw_bytes = data[offset:offset + field["length"]]
        raw = _le(raw_bytes)
        value = decode_value(field, raw)
        scaled = scaled_value(field, value)
        item = {
            "name": field["name"],
            "addr": field["addr"],
            "hex": field["hex"],
            "length": field["length"],
            "access": field["access"],
            "dtype": field["dtype"],
            "raw": raw,
            "value": value,
            "scaled": scaled,
            "text": format_value(field, value, scaled),
            "unit": field.get("unit", ""),
            "scale": field.get("scale"),
            "desc": field.get("desc", ""),
            "min": field.get("min"),
            "max": field.get("max"),
            "options": field.get("options"),
            "bits": field.get("bits"),
            "raw_hex": raw_bytes.hex(" ").upper(),
        }
        result.append(item)
    return result


def feedback_to_dict(data):
    """把 HLSCL::FeedBack() 读取的 56~70 共 15 字节转换为易用字典。"""
    data = bytes(data)
    if len(data) < 15:
        raise ValueError("反馈数据长度不足 15 字节")

    def word(offset):
        return data[offset] | (data[offset + 1] << 8)

    position_raw = decode_signed_magnitude(word(0), 15)
    speed_raw = decode_signed_magnitude(word(2), 15)
    load_raw = decode_signed_magnitude(word(4), 10)
    voltage_raw = data[6]
    temperature_raw = data[7]
    async_flag = data[8]
    status = data[9]
    moving = data[10]
    target_pos_raw = decode_signed_magnitude(word(11), 15)
    current_raw = decode_signed_magnitude(word(13), 15)

    return {
        "raw_hex": data.hex(" ").upper(),
        "position": {
            "raw": position_raw,
            "deg": round(position_raw * 0.087, 3),
            "turns": round(position_raw / 4096.0, 4),
        },
        "speed": {
            "raw": speed_raw,
            "rpm": round(speed_raw * 0.732, 3),
        },
        "load": {
            "raw": load_raw,
            "percent": round(load_raw * 0.1, 2),
        },
        "voltage": {
            "raw": voltage_raw,
            "volt": round(voltage_raw * 0.1, 2),
        },
        "temperature": {
            "raw": temperature_raw,
            "celsius": temperature_raw,
        },
        "async_write_flag": async_flag,
        "status": {
            "raw": status,
            "bits": [{"bit": b["bit"], "name": b["name"],
                      "active": bool(status & (1 << b["bit"]))}
                     for b in STATUS_BITS],
            "ok": status == 0,
        },
        "moving": {
            "raw": moving,
            "moving": bool(moving & 1),
            "in_position": not bool(moving & 1) if (moving & 2) == 0 else False,
            "bits": [{"bit": b["bit"], "name": b["name"],
                      "active": bool(moving & (1 << b["bit"]))}
                     for b in MOVING_BITS],
        },
        "target_position": {
            "raw": target_pos_raw,
            "deg": round(target_pos_raw * 0.087, 3),
        },
        "current": {
            "raw": current_raw,
            "mA": round(current_raw * 6.5, 1),
        },
    }
