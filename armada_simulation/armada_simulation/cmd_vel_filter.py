"""Reproduit côté simulation ce que fait le firmware avant de faire tourner les roues.

    cmd_vel (public, comme le robot réel)  ->  gz/cmd_vel (vers Gazebo)

Comme Robot::set_velocity() / Robot::update() du firmware :
  - la vitesse de chaque roue est limitée séparément à `max_rpm` (comme Robot::set_velocity : une commande trop
    rapide courbe donc légèrement la trajectoire, aussi sur le vrai robot) ;
  - sans commande pendant `command_timeout` secondes, le robot s'arrête.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class CmdVelFilter(Node):
    def __init__(self):
        super().__init__("cmd_vel_filter")
        # Mêmes valeurs que armada_esp.ino
        self.declare_parameter("wheel_diameter", 0.0772)
        self.declare_parameter("robot_width", 0.0925)
        self.declare_parameter("max_rpm", 60.0)
        self.declare_parameter("command_timeout", 0.4)

        self._circ = math.pi * self.get_parameter("wheel_diameter").value
        self._width = self.get_parameter("robot_width").value
        self._max_v = self.get_parameter("max_rpm").value / 60.0 * self._circ  # m/s par roue
        self._timeout = self.get_parameter("command_timeout").value

        self._pub = self.create_publisher(Twist, "gz/cmd_vel", 10)
        # best effort comme l'abonnement du firmware : accepte les mêmes publieurs (fiables ou non) que le vrai robot
        self.create_subscription(Twist, "cmd_vel", self._on_cmd, qos_profile_sensor_data)
        self._last_cmd = None
        self._moving = False
        self.create_timer(0.05, self._watchdog)

    def _on_cmd(self, msg: Twist):
        vx, vw = msg.linear.x, msg.angular.z
        v_left = max(-self._max_v, min(self._max_v, vx - vw * self._width / 2.0))
        v_right = max(-self._max_v, min(self._max_v, vx + vw * self._width / 2.0))

        out = Twist()
        out.linear.x = (v_left + v_right) / 2.0
        out.angular.z = (v_right - v_left) / self._width
        self._pub.publish(out)

        self._last_cmd = self.get_clock().now()
        self._moving = out.linear.x != 0.0 or out.angular.z != 0.0

    def _watchdog(self):
        if not self._moving or self._last_cmd is None:
            return
        if (self.get_clock().now() - self._last_cmd).nanoseconds * 1e-9 > self._timeout:
            self._pub.publish(Twist())
            self._moving = False
            self.get_logger().info("aucune commande reçue : arrêt du robot (comme le firmware)")


def main():
    rclpy.init()
    node = CmdVelFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
