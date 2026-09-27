# -*- coding: utf-8 -*-
"""不依赖真实硬件的协议自检。

运行：
    .venv/bin/python -m hls_debugger.selftest
"""

from __future__ import absolute_import

import time

from .memory_table import decode_range, encode_value, feedback_to_dict, find_field
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


def main():
    test_build_frames()
    test_transaction()
    test_read_and_sync_read()
    test_memory_table()
    test_feedback_decode()
    print("HLS debugger self-test: OK")


if __name__ == "__main__":
    main()
