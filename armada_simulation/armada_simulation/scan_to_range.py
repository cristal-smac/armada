"""Gazebo n'a pas de capteur « range » : le HC-SR04 y est simulé par un lidar à 1 rayon (LaserScan).
Ce noeud le convertit en sensor_msgs/Range, le type que publie le firmware sur `floor/range`.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Range


class ScanToRange(Node):
    def __init__(self):
        super().__init__("scan_to_range")
        self.declare_parameter("field_of_view", 0.26)  # rad, ~15° comme un HC-SR04
        self._fov = self.get_parameter("field_of_view").value
        self._pub = self.create_publisher(Range, "floor/range", qos_profile_sensor_data)
        self.create_subscription(LaserScan, "gz/floor_scan", self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, scan: LaserScan):
        if not scan.ranges:
            return
        msg = Range()
        msg.header = scan.header
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self._fov
        msg.min_range = scan.range_min
        msg.max_range = scan.range_max
        msg.range = float(scan.ranges[0])  # inf si rien dans la portée, comme le firmware
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = ScanToRange()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
