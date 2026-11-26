"""Complète l'interface d'un robot ARMADA à partir de son seul topic `odom`.

Le firmware (micro-ROS) et la simulation publient tous deux `odom` (nav_msgs/Odometry). Ce noeud, lancé à
l'identique pour le robot réel et simulé, en déduit :

  - la TF  <ns>/odom -> <ns>/base_footprint   (le firmware ne publie pas /tf : trop lourd pour l'ESP32)
  - `joint_states` des deux roues, pour que robot_state_publisher les fasse tourner dans RViz

Les angles de roues sont intégrés depuis la vitesse de l'odométrie (cinématique différentielle).
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster


class OdomAdapter(Node):
    def __init__(self):
        super().__init__("odom_adapter")
        # Valeurs de armada.urdf : wheel_left_joint à y=+0.04775, wheel_right_joint à y=-0.04775, rayon 38.6 mm.
        self.declare_parameter("wheel_radius", 0.0385963)
        self.declare_parameter("wheel_separation", 0.0955)
        self.declare_parameter("left_joint", "wheel_left_joint")
        self.declare_parameter("right_joint", "wheel_right_joint")
        self.declare_parameter("publish_tf", True)

        self._radius = self.get_parameter("wheel_radius").value
        self._sep = self.get_parameter("wheel_separation").value
        self._joints = [self.get_parameter("left_joint").value, self.get_parameter("right_joint").value]
        self._publish_tf = self.get_parameter("publish_tf").value

        self._angles = [0.0, 0.0]
        self._last_stamp = None
        self._tf = TransformBroadcaster(self)
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        # best effort : compatible avec un publisher fiable (simulation) comme best effort (micro-ROS sur Wi-Fi)
        self.create_subscription(Odometry, "odom", self._on_odom, qos_profile_sensor_data)

    def _on_odom(self, msg: Odometry):
        stamp = msg.header.stamp

        if self._publish_tf:
            t = TransformStamped()
            t.header = msg.header
            t.child_frame_id = msg.child_frame_id
            t.transform.translation.x = msg.pose.pose.position.x
            t.transform.translation.y = msg.pose.pose.position.y
            t.transform.translation.z = msg.pose.pose.position.z
            t.transform.rotation = msg.pose.pose.orientation
            self._tf.sendTransform(t)

        now = stamp.sec + stamp.nanosec * 1e-9
        if self._last_stamp is not None:
            dt = now - self._last_stamp
            if 0.0 < dt < 1.0:  # ignore les sauts (reconnexion, reset de la simulation)
                v = msg.twist.twist.linear.x
                w = msg.twist.twist.angular.z
                v_left = v - w * self._sep / 2.0
                v_right = v + w * self._sep / 2.0
                self._angles[0] = math.remainder(self._angles[0] + v_left / self._radius * dt, 2 * math.pi)
                self._angles[1] = math.remainder(self._angles[1] + v_right / self._radius * dt, 2 * math.pi)
        self._last_stamp = now

        js = JointState()
        js.header.stamp = stamp
        js.name = self._joints
        js.position = list(self._angles)
        self._js_pub.publish(js)


def main():
    rclpy.init()
    node = OdomAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
