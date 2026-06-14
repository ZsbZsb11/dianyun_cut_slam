#!/usr/bin/env python3
import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


MANUAL_T_BASE_CLOUD = (0.24, 0.0, 0.34)
MANUAL_Q_BASE_CLOUD = (-0.5, 0.5, -0.5, 0.5)
SLICE_MIN_Z = 0.12
SLICE_MAX_Z = 0.45


def rotate(q, p):
    x, y, z, w = q
    px, py, pz = p

    # q * p * q^-1, expanded to avoid extra dependencies.
    tx = 2.0 * (y * pz - z * py)
    ty = 2.0 * (z * px - x * pz)
    tz = 2.0 * (x * py - y * px)
    return (
        px + w * tx + (y * tz - z * ty),
        py + w * ty + (z * tx - x * tz),
        pz + w * tz + (x * ty - y * tx),
    )


class CloudInspector(Node):
    def __init__(self):
        super().__init__('cloud_inspector')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self.sub = self.create_subscription(
            PointCloud2,
            '/camera/points',
            self.on_cloud,
            qos,
        )
        self.timer = self.create_timer(8.0, self.on_timeout)
        self.last_cloud = None

    def on_cloud(self, msg):
        self.last_cloud = msg
        try:
            tf = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(f'waiting for TF base_link <- {msg.header.frame_id}: {exc}')
            return

        self.report(msg, (
            tf.transform.rotation.x,
            tf.transform.rotation.y,
            tf.transform.rotation.z,
            tf.transform.rotation.w,
        ), (
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ), source='tf')

    def report(self, msg, q, t, source):
        raw = []
        base = []
        for x, y, z in point_cloud2.read_points(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        ):
            p = (float(x), float(y), float(z))
            if not all(math.isfinite(v) for v in p):
                continue
            raw.append(p)
            rp = rotate(q, p)
            base.append((rp[0] + t[0], rp[1] + t[1], rp[2] + t[2]))

        print(f'frame_id={msg.header.frame_id}')
        print(f'transform_source={source}')
        print(f'size={msg.width}x{msg.height} finite={len(base)}')
        print(f'tf_base_from_cloud=translation({t[0]:.3f},{t[1]:.3f},{t[2]:.3f}) '
              f'quat({q[0]:.3f},{q[1]:.3f},{q[2]:.3f},{q[3]:.3f})')
        if base:
            rxs, rys, rzs = zip(*raw)
            bxs, bys, bzs = zip(*base)
            print(f'raw x=[{min(rxs):.3f}, {max(rxs):.3f}] y=[{min(rys):.3f}, {max(rys):.3f}] z=[{min(rzs):.3f}, {max(rzs):.3f}]')
            print(f'base x=[{min(bxs):.3f}, {max(bxs):.3f}] y=[{min(bys):.3f}, {max(bys):.3f}] z=[{min(bzs):.3f}, {max(bzs):.3f}]')
            print(f'points in front x>0.35: {sum(1 for x, _, _ in base if x > 0.35)}')
            print(f'points near floor |z|<0.08: {sum(1 for _, _, z in base if abs(z) < 0.08)}')
            print(
                f'points in scan slice {SLICE_MIN_Z:.2f}<=z<={SLICE_MAX_Z:.2f}: '
                f'{sum(1 for _, _, z in base if SLICE_MIN_Z <= z <= SLICE_MAX_Z)}'
            )
            print('base sample=' + ', '.join(
                f'({x:.2f},{y:.2f},{z:.2f})'
                for x, y, z in base[::max(1, len(base) // 8)][:8]
            ))
        sys.stdout.flush()
        os._exit(0)

    def on_timeout(self):
        if self.last_cloud is None:
            print('timed out waiting for /camera/points')
            sys.stdout.flush()
            os._exit(2)

        print('timed out waiting for TF; using manual static transform from launch')
        self.report(
            self.last_cloud,
            MANUAL_Q_BASE_CLOUD,
            MANUAL_T_BASE_CLOUD,
            source='manual_fallback',
        )


def main():
    rclpy.init()
    node = CloudInspector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()


if __name__ == '__main__':
    main()
