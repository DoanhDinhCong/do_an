#!/usr/bin/env python3
import math
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped

import tf2_ros
from tf2_ros import TransformException


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def point_in_polygon(px: float, py: float, poly_xy: List[float]) -> bool:
    if not poly_xy or len(poly_xy) < 6:
        return False
    n = len(poly_xy) // 2
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly_xy[2 * i], poly_xy[2 * i + 1]
        xj, yj = poly_xy[2 * j], poly_xy[2 * j + 1]
        intersect = ((yi > py) != (yj > py)) and \
                    (px < (xj - xi) * (py - yi) / (yj - yi + 1e-12) + xi)
        if intersect:
            inside = not inside
        j = i
    return inside


def parse_box(box_param) -> Optional[Tuple[float, float, float, float]]:
    if box_param is None:
        return None
    if isinstance(box_param, (list, tuple)) and len(box_param) == 4:
        return float(box_param[0]), float(box_param[1]), \
               float(box_param[2]), float(box_param[3])
    if isinstance(box_param, str):
        try:
            parts = [float(p.strip()) for p in
                     box_param.replace('[', '').replace(']', '').split(',')]
            if len(parts) == 4:
                return parts[0], parts[1], parts[2], parts[3]
        except Exception:
            return None
    return None


def parse_polygon(poly_param) -> List[float]:
    if poly_param is None:
        return []

    if isinstance(poly_param, (list, tuple)):
        try:
            vals = [float(v) for v in poly_param]
            return vals if len(vals) >= 6 and len(vals) % 2 == 0 else []
        except Exception:
            return []

    if isinstance(poly_param, str):
        s = poly_param.strip().replace('[', '').replace(']', '')
        try:
            vals = [float(p.strip()) for p in s.split(',') if p.strip() != '']
            if len(vals) >= 6 and len(vals) % 2 == 0:
                return vals
        except Exception:
            pass
    return []


class LaserScanMergerSync(Node):
    def __init__(self):
        super().__init__('laser_scan_merger_sync')
        desc = ParameterDescriptor(dynamic_typing=True)

        # ===== Topics (GIỮ Y NGUYÊN NODE 1) =====
        self.declare_parameter('scan1_topic', '/scan1', descriptor=desc)
        self.declare_parameter('scan2_topic', '/scan2', descriptor=desc)
        self.declare_parameter('merged_topic', '/scan_merged', descriptor=desc)
        self.declare_parameter('target_frame', 'base_link', descriptor=desc)

        # ===== Scan config =====
        self.declare_parameter('angle_min', -math.pi, descriptor=desc)
        self.declare_parameter('angle_max',  math.pi, descriptor=desc)
        self.declare_parameter('angle_increment', math.radians(1.0), descriptor=desc)
        self.declare_parameter('range_min', 0.15, descriptor=desc)
        self.declare_parameter('range_max', 12.0, descriptor=desc)

        # ===== Merge behavior =====
        self.declare_parameter('overlap_method', 'closest', descriptor=desc)
        self.declare_parameter('sync_tolerance', 0.1, descriptor=desc)  # seconds

        # ===== Masking =====
        self.declare_parameter('scan1_mask_box', None, descriptor=desc)
        self.declare_parameter('scan2_mask_box', None, descriptor=desc)
        self.declare_parameter('body_polygon', [], descriptor=desc)

        # ===== Get params =====
        self.scan1_topic = self.get_parameter('scan1_topic').value
        self.scan2_topic = self.get_parameter('scan2_topic').value
        self.merged_topic = self.get_parameter('merged_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.angle_inc = float(self.get_parameter('angle_increment').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)

        self.overlap_method = self.get_parameter('overlap_method').value
        self.sync_tol = float(self.get_parameter('sync_tolerance').value)

        self.scan1_mask = parse_box(self.get_parameter('scan1_mask_box').value)
        self.scan2_mask = parse_box(self.get_parameter('scan2_mask_box').value)
        self.body_polygon = parse_polygon(self.get_parameter('body_polygon').value)

        if self.angle_inc <= 0.0 or self.angle_max <= self.angle_min:
            self.get_logger().error("Invalid angle config. Reset to defaults.")
            self.angle_min = -math.pi
            self.angle_max =  math.pi
            self.angle_inc = math.radians(1.0)

        self.num_points = int(round((self.angle_max - self.angle_min) / self.angle_inc)) + 1

        # ===== TF =====
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ===== State =====
        self.scan1: Optional[LaserScan] = None
        self.scan2: Optional[LaserScan] = None
        self.scan1_stamp: Optional[Time] = None
        self.scan2_stamp: Optional[Time] = None

        # ===== Pub/Sub =====
        self.sub1 = self.create_subscription(
            LaserScan, self.scan1_topic, self.cb_scan1, 10)
        self.sub2 = self.create_subscription(
            LaserScan, self.scan2_topic, self.cb_scan2, 10)
        self.pub = self.create_publisher(
            LaserScan, self.merged_topic, 10)

        self.get_logger().info(
            f"laser_scan_merger_sync started. in: [{self.scan1_topic}, {self.scan2_topic}] → out: {self.merged_topic}"
        )

    # =====================================================
    # Callbacks
    # =====================================================
    def cb_scan1(self, msg: LaserScan):
        self.scan1 = msg
        self.scan1_stamp = Time.from_msg(msg.header.stamp)
        self.try_merge()

    def cb_scan2(self, msg: LaserScan):
        self.scan2 = msg
        self.scan2_stamp = Time.from_msg(msg.header.stamp)
        self.try_merge()

    # =====================================================
    # Merge logic (timestamp-sync)
    # =====================================================
    def try_merge(self):
        if self.scan1 is None or self.scan2 is None:
            return

        dt = abs((self.scan1_stamp - self.scan2_stamp).nanoseconds) / 1e9
        if dt > self.sync_tol:
            return

        try:
            tf1 = self.get_tf(self.scan1)
            tf2 = self.get_tf(self.scan2)
        except Exception:
            return

        ranges = [math.inf] * self.num_points

        self.process_scan(self.scan1, tf1, ranges, self.scan1_mask)
        self.process_scan(self.scan2, tf2, ranges, self.scan2_mask)

        # mask polygon thân robot
        if self.body_polygon:
            for i in range(self.num_points):
                r = ranges[i]
                if not math.isfinite(r):
                    continue
                ang = self.angle_min + i * self.angle_inc
                x = r * math.cos(ang)
                y = r * math.sin(ang)
                if point_in_polygon(x, y, self.body_polygon):
                    ranges[i] = math.inf

        self.publish_scan(ranges)

        # reset tránh merge scan cũ
        self.scan1 = None
        self.scan2 = None

    # =====================================================
    # TF handling (1 lần / scan)
    # =====================================================
    def get_tf(self, scan: LaserScan):
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan.header.frame_id,
                Time(),
                timeout=Duration(seconds=0.05)
            )
        except TransformException as e:
            self.get_logger().warning(f"TF {scan.header.frame_id}->{self.target_frame} fail: {e}")
            raise

        q = t.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        return {
            'x': t.transform.translation.x,
            'y': t.transform.translation.y,
            'cos': math.cos(yaw),
            'sin': math.sin(yaw)
        }

    # =====================================================
    # Process one scan
    # =====================================================
    def process_scan(self, scan: LaserScan, tf, ranges_out: List[float],
                     mask_box: Optional[Tuple[float, float, float, float]]):

        a = scan.angle_min
        inc = scan.angle_increment

        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue

            ang = a + i * inc
            xs = r * math.cos(ang)
            ys = r * math.sin(ang)

            # mask box (sensor frame)
            if mask_box is not None:
                xmin, xmax, ymin, ymax = mask_box
                if xmin <= xs <= xmax and ymin <= ys <= ymax:
                    continue

            # transform to base_link
            xb = tf['cos'] * xs - tf['sin'] * ys + tf['x']
            yb = tf['sin'] * xs + tf['cos'] * ys + tf['y']

            dist = math.hypot(xb, yb)
            if dist < self.range_min or dist > self.range_max:
                continue

            theta = math.atan2(yb, xb)
            idx = int(round((theta - self.angle_min) / self.angle_inc))

            if 0 <= idx < self.num_points:
                self.update_point(ranges_out, idx, dist)

    # =====================================================
    # Overlap handling
    # =====================================================
    def update_point(self, ranges: List[float], idx: int, r: float):
        if self.overlap_method == 'closest':
            if r < ranges[idx]:
                ranges[idx] = r

        elif self.overlap_method == 'average':
            if math.isinf(ranges[idx]):
                ranges[idx] = r
            else:
                ranges[idx] = 0.5 * (ranges[idx] + r)

        elif self.overlap_method == 'newest':
            ranges[idx] = r

    # =====================================================
    # Publish
    # =====================================================
    def publish_scan(self, ranges: List[float]):
        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.target_frame

        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_min + (self.num_points - 1) * self.angle_inc
        msg.angle_increment = self.angle_inc
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        msg.ranges = [r if math.isfinite(r) else math.inf for r in ranges]
        msg.intensities = []

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = LaserScanMergerSync()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
