#!/usr/bin/env python3
import json
import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import NavSatFix, PointCloud2
from std_msgs.msg import String


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def finite_or_none(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    return value if math.isfinite(value) else None


class OdinRtkCaptureNode(Node):
    def __init__(self):
        super().__init__("odin_rtk_capture_node")
        self.declare_parameter("cloud_topic", "/odin_b/odin1/cloud_render")
        self.declare_parameter("fix_topic", "/gnss/fix")
        self.declare_parameter("heading_topic", "/gnss/heading")
        self.declare_parameter("status_topic", "/gnss/status")
        self.declare_parameter("meta_topic", "/odin_rtk/bound_meta")
        self.declare_parameter("bound_cloud_topic", "/odin_rtk/bound_cloud")
        self.declare_parameter("republish_cloud", True)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.fix_topic = self.get_parameter("fix_topic").value
        self.heading_topic = self.get_parameter("heading_topic").value
        self.status_topic = self.get_parameter("status_topic").value
        self.meta_topic = self.get_parameter("meta_topic").value
        self.bound_cloud_topic = self.get_parameter("bound_cloud_topic").value
        self.republish_cloud = bool(self.get_parameter("republish_cloud").value)

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        reliable_qos = QoSProfile(depth=20)

        self.latest_fix = None
        self.latest_heading = None
        self.latest_status = {}
        self.cloud_count = 0
        self.started_at = time.time()

        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, sensor_qos)
        self.create_subscription(NavSatFix, self.fix_topic, self.fix_cb, sensor_qos)
        self.create_subscription(Vector3Stamped, self.heading_topic, self.heading_cb, sensor_qos)
        self.create_subscription(String, self.status_topic, self.status_cb, reliable_qos)
        self.meta_pub = self.create_publisher(String, self.meta_topic, reliable_qos)
        self.cloud_pub = (
            self.create_publisher(PointCloud2, self.bound_cloud_topic, sensor_qos)
            if self.republish_cloud
            else None
        )
        self.get_logger().info(
            "Odin RTK capture node ready: "
            f"cloud={self.cloud_topic} meta={self.meta_topic} bound_cloud={self.bound_cloud_topic}"
        )

    def fix_cb(self, msg):
        self.latest_fix = {
            "stamp": stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "latitude": finite_or_none(msg.latitude),
            "longitude": finite_or_none(msg.longitude),
            "altitude_m": finite_or_none(msg.altitude),
            "status": int(msg.status.status),
            "service": int(msg.status.service),
        }

    def heading_cb(self, msg):
        self.latest_heading = {
            "stamp": stamp_to_float(msg.header.stamp),
            "frame_id": msg.header.frame_id,
            "heading_deg": finite_or_none(msg.vector.x),
            "pitch_deg": finite_or_none(msg.vector.y),
            "baseline_m": finite_or_none(msg.vector.z),
        }

    def status_cb(self, msg):
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_status = {"raw": msg.data}

    def cloud_cb(self, msg):
        self.cloud_count += 1
        cloud_stamp = stamp_to_float(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        fix_age = None
        heading_age = None
        if self.latest_fix and cloud_stamp > 0 and self.latest_fix.get("stamp", 0) > 0:
            fix_age = cloud_stamp - self.latest_fix["stamp"]
        if self.latest_heading and cloud_stamp > 0 and self.latest_heading.get("stamp", 0) > 0:
            heading_age = cloud_stamp - self.latest_heading["stamp"]

        payload = {
            "version": 1,
            "cloud_topic": self.cloud_topic,
            "cloud_seq": self.cloud_count,
            "cloud_stamp": cloud_stamp,
            "cloud_frame_id": msg.header.frame_id,
            "cloud_points_hint": int(msg.width) * int(msg.height),
            "node_time": now,
            "wall_time": time.time(),
            "rtk_fix": self.latest_fix,
            "rtk_heading": self.latest_heading,
            "rtk_status": self.latest_status,
            "time_delta": {
                "cloud_minus_fix_sec": fix_age,
                "cloud_minus_heading_sec": heading_age,
            },
            "binding_state": {
                "has_fix": self.latest_fix is not None,
                "has_heading": self.latest_heading is not None,
                "has_status": bool(self.latest_status),
            },
        }
        self.meta_pub.publish(String(data=json.dumps(payload, ensure_ascii=False, allow_nan=False)))
        if self.cloud_pub:
            self.cloud_pub.publish(msg)


def main():
    rclpy.init()
    node = OdinRtkCaptureNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
