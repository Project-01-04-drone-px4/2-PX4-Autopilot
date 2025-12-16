#!/usr/bin/env python3
"""
MAVLink to PlotJuggler JSON Converter
将 jMAVSim 的 MAVLink 数据转换为 PlotJuggler 支持的 JSON 格式

用法:
    python3 mavlink_to_plotjuggler.py [选项]

示例:
    # 默认配置（从 18502 接收，发送到本地 18570）
    python3 mavlink_to_plotjuggler.py

    # 自定义端口
    python3 mavlink_to_plotjuggler.py --mavlink-port 18502 --plotjuggler-port 18570

    # 指定服务器 IP（如果 jMAVSim 在远程服务器）
    python3 mavlink_to_plotjuggler.py --mavlink-host 10.10.20.173 --mavlink-port 18502
"""

import socket
import json
import time
import argparse
import sys
import math
from datetime import datetime
from collections import defaultdict

try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink2
except ImportError:
    print("错误: 需要安装 pymavlink")
    print("安装命令: pip3 install --user pymavlink")
    sys.exit(1)


class MAVLinkToPlotJuggler:
    def __init__(self, mavlink_host='127.0.0.1', mavlink_port=14550,
                 plotjuggler_host='127.0.0.1', plotjuggler_port=18571,
                 verbose=False):
        """
        初始化 MAVLink 到 PlotJuggler 转换器

        参数:
            mavlink_host: MAVLink 源地址（PX4/jMAVSim 服务器）
            mavlink_port: MAVLink UDP 端口（PX4 GCS 端口，默认 18570）
            plotjuggler_host: PlotJuggler 目标地址
            plotjuggler_port: PlotJuggler UDP 端口（默认 18571，避免与 PX4 冲突）
            verbose: 是否显示详细信息
        """
        self.mavlink_host = mavlink_host
        self.mavlink_port = mavlink_port
        self.plotjuggler_host = plotjuggler_host
        self.plotjuggler_port = plotjuggler_port
        self.verbose = verbose

        # 创建 UDP socket 用于发送到 PlotJuggler
        self.plotjuggler_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # 使用 mavutil 连接到 PX4
        # PX4 在 18570 上监听，但我们需要连接到它发送数据的端口
        # 由于启用了广播模式，PX4 会发送到所有连接的客户端
        try:
            # 方法：使用 udp 连接到 PX4 的远程端口（14550 是标准 GCS 端口）
            # 但实际 PX4 可能配置为发送到其他端口
            # 先尝试连接到标准端口 14550
            try:
                self.mav = mavutil.mavlink_connection(
                    f'udp:{mavlink_host}:14550',  # 标准 GCS 端口
                    dialect='common',
                    input=True
                )
                if self.verbose:
                    print("连接到标准 GCS 端口 14550")
            except:
                # 如果 14550 不行，尝试连接到指定的端口
                self.mav = mavutil.mavlink_connection(
                    f'udp:{mavlink_host}:{mavlink_port}',
                    dialect='common',
                    input=True
                )
                if self.verbose:
                    print(f"连接到端口 {mavlink_port}")

            # 等待一下让连接建立
            time.sleep(0.5)

            # 发送一个心跳来建立连接
            try:
                self.mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                if self.verbose:
                    print("已发送心跳建立连接")
            except Exception as e:
                if self.verbose:
                    print(f"发送心跳失败（可能不影响）: {e}")

        except Exception as e:
            print(f"错误: 无法创建 MAVLink 连接: {e}")
            print("\n提示:")
            print(f"  1. 确保 PX4/jMAVSim 正在运行")
            print(f"  2. 尝试使用标准端口: python3 mavlink_to_plotjuggler.py --mavlink-port 14550")
            print(f"  3. 检查 QGC 连接的端口（通常是 14550）")
            raise

        # 创建 MAVLink 解析器（用于备用）
        from pymavlink.dialects.v20 import common as mavlink2
        self.mavlink_parser = mavlink2.MAVLink(None, srcSystem=255, srcComponent=0)

        # 消息计数器
        self.message_count = defaultdict(int)
        self.start_time = time.time()
        self.last_stats_time = time.time()

        print(f"MAVLink 接收: {mavlink_host}:{mavlink_port}")
        print(f"PlotJuggler 发送: {plotjuggler_host}:{plotjuggler_port}")
        print()
        print("在 Windows PlotJuggler 中配置:")
        print(f"  - Address: 0.0.0.0 或留空（监听所有接口）")
        print(f"  - Port: {plotjuggler_port}")
        print(f"  - Protocol: json")
        print()
        print("按 Ctrl+C 停止")
        print("-" * 60)

    def sanitize_value(self, value):
        """清理值，确保可以序列化为 JSON"""
        if value is None:
            return None
        elif isinstance(value, (int, bool)):
            return value
        elif isinstance(value, float):
            # 处理 NaN 和 Infinity
            if math.isnan(value):
                return None  # 或返回 0
            elif math.isinf(value):
                return None  # 或返回 0
            else:
                return value
        elif isinstance(value, (list, tuple)):
            return [self.sanitize_value(v) for v in value]
        elif isinstance(value, str):
            return value
        elif hasattr(value, '__dict__'):
            return str(value)
        else:
            try:
                # 尝试转换为基本类型
                return float(value) if not math.isnan(float(value)) else None
            except (ValueError, TypeError):
                return str(value)

    def convert_message_to_json(self, msg):
        """
        将 MAVLink 消息转换为 PlotJuggler JSON 格式

        PlotJuggler JSON 格式:
        {
            "timestamp": 1234567890.123,
            "message_type": "ATTITUDE",
            "field1": value1,
            "field2": value2,
            ...
        }
        """
        if msg is None:
            return None

        # 获取当前时间戳（秒，带小数）
        timestamp = time.time()

        # 创建 JSON 对象
        json_data = {
            "timestamp": timestamp,
            "message_type": msg.get_type()
        }

        # 添加消息的所有字段
        for field in msg.fieldnames:
            try:
                value = getattr(msg, field, None)
                # 清理值以确保 JSON 兼容
                json_data[field] = self.sanitize_value(value)
            except Exception as e:
                if self.verbose:
                    print(f"警告: 无法处理字段 {field}: {e}")
                continue

        return json_data

    def send_to_plotjuggler(self, json_data):
        """发送 JSON 数据到 PlotJuggler"""
        if json_data is None:
            return

        try:
            # 转换为 JSON 字符串（每行一个 JSON 对象）
            # 使用 ensure_ascii=False 支持非 ASCII 字符
            # 使用 allow_nan=False 确保不包含 NaN/Infinity
            json_str = json.dumps(json_data, ensure_ascii=False, allow_nan=False) + '\n'

            self.plotjuggler_socket.sendto(
                json_str.encode('utf-8'),
                (self.plotjuggler_host, self.plotjuggler_port)
            )
        except (ValueError, TypeError) as e:
            # JSON 序列化错误（可能包含 NaN/Infinity）
            if self.verbose:
                print(f"JSON 序列化错误: {e}")
                print(f"数据: {json.dumps(json_data, default=str, allow_nan=True)[:200]}")
        except Exception as e:
            if self.verbose:
                print(f"发送错误: {e}")

    def print_stats(self):
        """打印统计信息"""
        elapsed = time.time() - self.last_stats_time
        if elapsed < 5.0:  # 每5秒打印一次
            return

        total = sum(self.message_count.values())
        print(f"\n统计信息 (过去 {elapsed:.1f} 秒):")
        print(f"  总消息数: {total}")
        print(f"  消息类型分布:")
        for msg_type, count in sorted(self.message_count.items(),
                                      key=lambda x: x[1], reverse=True)[:10]:
            print(f"    {msg_type}: {count}")
        print("-" * 60)

        self.message_count.clear()
        self.last_stats_time = time.time()

    def run(self):
        """主循环：接收 MAVLink 消息并转换为 JSON"""
        print("开始接收 MAVLink 消息...")
        print("等待数据...")

        try:
            while True:
                try:
                    # 使用 mavutil 接收 MAVLink 消息
                    msg = self.mav.recv_match(blocking=False, timeout=0.1)

                    if msg is not None:
                        # 转换为 JSON
                        json_data = self.convert_message_to_json(msg)

                        if json_data:
                            # 发送到 PlotJuggler
                            self.send_to_plotjuggler(json_data)

                            # 更新统计
                            msg_type = json_data.get('message_type', 'UNKNOWN')
                            self.message_count[msg_type] += 1

                            # 显示详细信息（如果启用）
                            if self.verbose:
                                print(f"[{msg_type}] {json.dumps(json_data, default=str)[:100]}...")

                except socket.timeout:
                    # 超时是正常的，继续循环
                    pass
                except Exception as e:
                    if self.verbose:
                        print(f"处理消息时出错: {e}")

                # 定期打印统计信息
                self.print_stats()

        except KeyboardInterrupt:
            print("\n\n停止转换器...")
            elapsed = time.time() - self.start_time
            print(f"运行时间: {elapsed:.1f} 秒")
            print(f"总消息数: {sum(self.message_count.values())}")
        finally:
            if hasattr(self, 'mav') and self.mav:
                try:
                    self.mav.close()
                except:
                    pass
            self.plotjuggler_socket.close()
            print("已关闭连接")


def main():
    parser = argparse.ArgumentParser(
        description='将 MAVLink 数据转换为 PlotJuggler JSON 格式',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 默认配置（发送到 Windows 10.10.20.47）
  python3 mavlink_to_plotjuggler.py

  # 自定义 Windows IP
  python3 mavlink_to_plotjuggler.py --plotjuggler-host 10.10.20.47

  # 详细输出
  python3 mavlink_to_plotjuggler.py --verbose
        """
    )

    parser.add_argument(
        '--mavlink-host',
        default='127.0.0.1',
        help='MAVLink 源地址（PX4/jMAVSim 服务器 IP，默认: 127.0.0.1）'
    )

    parser.add_argument(
        '--mavlink-port',
        type=int,
        default=14550,
        help='MAVLink UDP 端口（标准 GCS 端口，默认: 14550。如果不行，尝试 18570）'
    )

    parser.add_argument(
        '--plotjuggler-host',
        default='10.10.20.47',
        help='PlotJuggler 目标地址（Windows 客户端 IP，默认: 10.10.20.47）'
    )

    parser.add_argument(
        '--plotjuggler-port',
        type=int,
        default=18571,
        help='PlotJuggler UDP 端口（默认: 18571，避免与 PX4 端口冲突）'
    )

    parser.add_argument(
        '--verbose', '-v',
        action='store_true',
        help='显示详细信息'
    )

    args = parser.parse_args()

    # 创建转换器并运行
    converter = MAVLinkToPlotJuggler(
        mavlink_host=args.mavlink_host,
        mavlink_port=args.mavlink_port,
        plotjuggler_host=args.plotjuggler_host,
        plotjuggler_port=args.plotjuggler_port,
        verbose=args.verbose
    )

    converter.run()


if __name__ == '__main__':
    main()

