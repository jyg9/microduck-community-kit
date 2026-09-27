# -*- coding: utf-8 -*-
"""真实硬件连通性测试（只读，不会让舵机运动）。

用法：
    .venv/bin/python hw_test.py [--port /dev/ttyACM1] [--baud 1000000]
                                [--ids 0-253] [--no-scan-baud]

测试内容：
    1. 串口设备信息（sysfs/driver）
    2. 打开串口
    3. 指定波特率下 Ping 扫描 ID
    4. 若一个都没扫到，自动遍历常用波特率再扫一遍
    5. 对每个在线 ID：读版本、ID、波特率、写锁、模式、扭矩、15 字节实时反馈

本脚本只发送 PING 和 READ，不发送任何 WRITE/MOVE/RESET/CAL，安全无动作。
"""

from __future__ import absolute_import

import argparse
import glob
import os
import sys

from hls_debugger.memory_table import (
    MODE_NAMES as MODE_NAMES_FULL,
    decode_range,
    feedback_to_dict,
)
from hls_debugger.protocol import (
    ADDR_LOCK,
    ADDR_MODE,
    ADDR_PRESENT_POSITION,
    ADDR_TORQUE_ENABLE,
    HLSBus,
    ProtocolError,
)

BAUDS = [1000000, 115200, 500000, 250000, 128000, 76800, 57600, 38400]


def mode_name(value):
    """模式名；未知取值也照原样显示，不吞掉硬件真实值。"""
    name = MODE_NAMES_FULL.get(value)
    if name:
        return name
    return "未知模式（保留原值 %d）" % value


def hr(title):
    print("\n" + "=" * 68)
    print(title)
    print("=" * 68)


def device_info(port):
    hr("1. 串口设备信息")
    if not os.path.exists(port):
        print("  [!] %s 不存在" % port)
        return False
    st = os.stat(port)
    print("  设备节点 : %s" % port)
    print("  类型     : char, major=%d minor=%d" % (os.major(st.st_rdev), os.minor(st.st_rdev)))
    print("  权限     : %o  可读=%s 可写=%s"
          % (st.st_mode & 0o777, os.access(port, os.R_OK), os.access(port, os.W_OK)))

    name = os.path.basename(port)
    sysdir = "/sys/class/tty/%s/device" % name
    if os.path.exists(sysdir):
        real = os.path.realpath(sysdir)
        print("  sysfs    : %s" % real)
        for probe in ("driver", "interface"):
            p = os.path.join(sysdir, probe)
            if os.path.islink(p):
                print("  %-8s : %s" % (probe, os.path.basename(os.readlink(p))))
        for pat in ("idVendor", "idProduct", "manufacturer", "product", "serial"):
            try:
                with open(os.path.join(real, pat)) as fh:
                    print("  %-8s : %s" % (pat, fh.read().strip()))
            except Exception:
                pass
    try:
        links = glob.glob("/dev/serial/by-id/*")
        if links:
            print("  by-id    : %s" % ", ".join(sorted(os.path.basename(x) for x in links)))
    except Exception:
        pass
    return True


def scan_baud(port, baud, start, end, timeout):
    bus = HLSBus()
    try:
        bus.connect(port, baudrate=baud, timeout=timeout)
    except ProtocolError as exc:
        print("  [!] %s @ %d 打开失败：%s" % (port, baud, exc.message))
        return None, []
    try:
        found = bus.scan(start, end, timeout=timeout)
    finally:
        bus.disconnect()
    return bus, found


def describe(bus, sid):
    print("\n  --- ID %d ---" % sid)

    # 版本信息：地址 0..4（固件主/次、端序、舵机主/次）
    try:
        raw = bus.read_bytes(sid, 0, 5)
        print("  版本     : 固件 %d.%d  端序 %d  舵机 %d.%d  [raw %s]"
              % (raw[0], raw[1], raw[2], raw[3], raw[4], raw.hex(" ").upper()))
    except ProtocolError as exc:
        print("  版本     : 读取失败 - %s" % exc.message)

    # EPROM/状态字段：5(ID) 6(波特率) 33(模式) 40(扭矩) 55(写锁)
    for addr, label in ((5, "ID"), (6, "波特率索引"), (33, "工作模式"),
                        (40, "扭矩使能"), (55, "写锁")):
        try:
            val = bus.read_byte(sid, addr)
            extra = ""
            if addr == 33:
                extra = " (%s)" % mode_name(val)
            print("  %-9s: %d%s  [地址 %d]" % (label, val, extra, addr))
        except ProtocolError as exc:
            print("  %-9s: 读取失败 - %s" % (label, exc.message))

    # 实时反馈 56~70 共 15 字节
    try:
        resp = bus.feedback(sid)
        data = feedback_to_dict(resp.data)
        print("  实时反馈 : %s" % data["raw_hex"])
        print("    位置   : %d 计数 = %.3f°  (%.4f 圈)"
              % (data["position"]["raw"], data["position"]["deg"], data["position"]["turns"]))
        print("    速度   : %d = %.3f RPM" % (data["speed"]["raw"], data["speed"]["rpm"]))
        print("    负载   : %d = %.2f%%" % (data["load"]["raw"], data["load"]["percent"]))
        print("    电压   : %.2f V" % data["voltage"]["volt"])
        print("    温度   : %d ℃" % data["temperature"]["celsius"])
        flags = [b["name"] for b in data["status"]["bits"] if b["active"]]
        print("    移动中 : %s   状态: %d %s   异步写标志: %d"
              % (data["moving"]["moving"], data["status"]["raw"],
                 ("[%s]" % ",".join(flags)) if flags else "(正常)",
                 data["async_write_flag"]))
        print("    目标位 : %d 计数" % data["target_position"]["raw"])
        print("    电流   : %d = %.1f mA" % (data["current"]["raw"], data["current"]["mA"]))
        print("  字段解码 :")
        for item in decode_range(ADDR_PRESENT_POSITION, resp.data):
            print("    +%-3d %-14s = %s" % (item["addr"], item["name"], item["text"]))
        return data
    except ProtocolError as exc:
        print("  实时反馈 : 读取失败 - %s" % exc.message)
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM1")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--ids", default="0-253",
                    help="ID 扫描范围，默认全总线 0-253；不要用 0-20，会漏掉高 ID 舵机")
    ap.add_argument("--timeout", type=float, default=0.06)
    ap.add_argument("--no-scan-baud", action="store_true", help="不遍历其它波特率")
    args = ap.parse_args()

    lo, _, hi = args.ids.partition("-")
    start, end = int(lo), int(hi or lo)

    if not device_info(args.port):
        return 2

    hr("2. Ping 扫描  (ID %d~%d @ %d bps)" % (start, end, args.baud))
    bus, found = scan_baud(args.port, args.baud, start, end, args.timeout)
    if bus is None:
        return 3

    used_baud = args.baud
    if not found and not args.no_scan_baud:
        for baud in BAUDS:
            if baud == args.baud:
                continue
            print("  [%d bps] 无应答，改试 %d bps ..." % (used_baud, baud))
            _, found = scan_baud(args.port, baud, start, end, args.timeout)
            if found:
                used_baud = baud
                break

    if not found:
        print("\n  [x] %d~%d 全范围无应答。" % (start, end))
        print("      可能原因：舵机未供电 / 总线接线 / 波特率不匹配 / 总线被占用。")
        return 1

    print("\n  [√] 发现 %d 个在线 ID（波特率 %d）：%s"
          % (len(found), used_baud, ", ".join(str(f["id"]) for f in found)))

    hr("3. 在线舵机信息读取（只读，不产生动作）")
    bus = HLSBus()
    bus.connect(args.port, baudrate=used_baud, timeout=args.timeout)
    try:
        for item in found:
            describe(bus, item["id"])
        hr("4. 原始收发日志（最后 12 条）")
        for entry in bus.logs()[-12:]:
            print("  %-3s %-22s %s" % (entry.get("direction"), entry.get("note", ""),
                                       entry.get("hex", "")))
    finally:
        bus.disconnect()

    print("\n结论：与真实 HLS 舵机通讯成功（只读验证完成）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
