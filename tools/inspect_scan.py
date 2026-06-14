#!/usr/bin/env python3
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class ScanInspector(Node):
    def __init__(self):
        super().__init__('scan_inspector')
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(LaserScan, '/scan', self.on_scan, qos)
        self.timer = self.create_timer(8.0, self.on_timeout)

    def on_scan(self, msg):
        finite = []
        front = []
        left = []
        right = []
        for i, value in enumerate(msg.ranges):
            if not math.isfinite(value):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            finite.append(value)
            if abs(angle) <= 0.35:
                front.append(value)
            elif angle > 0.35:
                left.append(value)
            else:
                right.append(value)

        print(f'frame_id={msg.header.frame_id}')
        print(
            f'angles=[{msg.angle_min:.3f}, {msg.angle_max:.3f}] '
            f'increment={msg.angle_increment:.4f} beams={len(msg.ranges)}'
        )
        print(f'range=[{msg.range_min:.2f}, {msg.range_max:.2f}] finite={len(finite)}')
        if finite:
            print(f'finite range min={min(finite):.2f} max={max(finite):.2f}')
        for name, values in [('front', front), ('left', left), ('right', right)]:
            if values:
                print(f'{name}: count={len(values)} min={min(values):.2f} max={max(values):.2f}')
            else:
                print(f'{name}: count=0')
        sys.stdout.flush()
        os._exit(0)

    def on_timeout(self):
        print('timed out waiting for /scan')
        sys.stdout.flush()
        os._exit(2)


def main():
    rclpy.init()
    node = ScanInspector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()


if __name__ == '__main__':
    main()
