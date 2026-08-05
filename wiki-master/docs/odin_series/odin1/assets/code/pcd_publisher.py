
#!/usr/bin/env python3
"""
PCD (XYZRGB) -> ROS2 PointCloud2
使用 open3d，直接 python3 运行
"""

###################### 可修改区域 ######################
PCD_PATH  = "/home/hugo/git/pcd_publisher/office_0203_0.03.pcd"   # <<< 改这里
FRAME_ID = "map"
TOPIC    = "/pcd_points"
PUB_HZ   = 1.0
#######################################################

import rclpy
from rclpy.node import Node

import numpy as np
import open3d as o3d
import struct

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


def rgb_to_float(r, g, b):
    """RGB(uint8) -> packed float32"""
    rgb_uint32 = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack("f", struct.pack("I", rgb_uint32))[0]


class PCDPublisher(Node):
    def __init__(self):
        super().__init__("pcd_rgb_publisher")

        self.pub = self.create_publisher(PointCloud2, TOPIC, 10)
        self.timer = self.create_timer(1.0 / PUB_HZ, self.timer_cb)

        self.cloud_msg = self.load_pcd(PCD_PATH)
        self.get_logger().info(f"Loaded PCD with RGB: {PCD_PATH}")

    def load_pcd(self, path: str) -> PointCloud2:
        pcd = o3d.io.read_point_cloud(path)

        points = np.asarray(pcd.points, dtype=np.float32)
        colors = np.asarray(pcd.colors, dtype=np.float32)  # [0,1]

        cloud_data = []
        for i in range(points.shape[0]):
            r = int(colors[i, 0] * 255)
            g = int(colors[i, 1] * 255)
            b = int(colors[i, 2] * 255)
            rgb_f = rgb_to_float(r, g, b)

            cloud_data.append((
                points[i, 0],
                points[i, 1],
                points[i, 2],
                rgb_f
            ))

        header = Header()
        header.frame_id = FRAME_ID

        fields = [
           PointField(name="x",   offset=0,  datatype=PointField.FLOAT32, count=1),
           PointField(name="y",   offset=4,  datatype=PointField.FLOAT32, count=1),
           PointField(name="z",   offset=8,  datatype=PointField.FLOAT32, count=1),
           PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]


        return point_cloud2.create_cloud(header, fields, cloud_data)

    def timer_cb(self):
        self.cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.cloud_msg)


def main():
    rclpy.init()
    node = PCDPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

