#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RTK 模块测试脚本 - 直接读取并解析数据
"""
import serial
import time
import sys

def test_rtk_module(port='/dev/ttyACM3', baudrate=115200):
    print(f"正在测试 RTK 模块: {port} @ {baudrate}")
    print("=" * 60)

    try:
        # 打开串口
        ser = serial.Serial(port, baudrate, timeout=1)
        print(f"✓ 串口已打开")

        # 读取数据
        print("\n等待数据... (按 Ctrl+C 停止)\n")

        line_count = 0
        nmea_count = 0
        binary_count = 0

        buffer = b''

        while line_count < 50:  # 读取50行数据
            data = ser.read(100)
            if len(data) > 0:
                buffer += data

                # 尝试解析 NMEA 格式 ($开头)
                if b'$' in buffer:
                    lines = buffer.split(b'\n')
                    for line in lines[:-1]:
                        line = line.strip()
                        if line.startswith(b'$'):
                            try:
                                text = line.decode('ascii', errors='ignore')
                                print(f"[NMEA] {text}")
                                nmea_count += 1
                                line_count += 1
                            except:
                                pass
                    buffer = lines[-1]

                # 如果不是 NMEA，显示二进制数据
                elif len(buffer) > 20:
                    hex_str = buffer[:20].hex()
                    print(f"[二进制] {hex_str}... ({len(buffer)} 字节)")
                    binary_count += 1
                    line_count += 1
                    buffer = buffer[20:]

            time.sleep(0.1)

        ser.close()

        print("\n" + "=" * 60)
        print(f"测试完成:")
        print(f"  NMEA 消息: {nmea_count}")
        print(f"  二进制数据块: {binary_count}")

        if nmea_count > 0:
            print("\n✓ 检测到 NMEA 格式数据 - 可以使用 NMEA 驱动")
        elif binary_count > 0:
            print("\n✓ 检测到二进制数据 - 需要使用 Unicore/UM982 二进制驱动")
        else:
            print("\n✗ 未检测到有效数据")

    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        return 1

    return 0

if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM3'
    sys.exit(test_rtk_module(port))
