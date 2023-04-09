"""
取矿控制模块
"""
from .tools import RclpyRuntime

from rclpy.node import Node
from std_msgs import msg as std_msgs

import time

QOS = 3


class StretchArmNode(Node):
    def __init__(self):
        super().__init__("gary_liftup_stretch_arm")
        self.declare_parameter("stretch_arm_target", 0.0)

        self.publisher_left_cmd = self.create_publisher(std_msgs.Float64, "/stretch_left_pid/cmd", QOS)
        self.publisher_right_cmd = self.create_publisher(std_msgs.Float64, "/stretch_right_pid/cmd", QOS)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """

        """
        # --- setup message ---
        left_target = std_msgs.Float64()
        right_target = std_msgs.Float64()

        value = self.get_parameter("stretch_arm_target").value

        try:
            left_target.data = - value
            right_target.data = value
        except AssertionError as e:
            logger = self.get_logger()
            logger.warn("AssertionError, " + str(e))
        else:
            print("\r gary_liftup_stretch_arm: [%.2f] get parameter `stretch_arm_target`: %.1f" %
                  (time.time() % 1e5, value),
                  end="", flush=True)

            # --- publish ---
            # self.publisher_left_cmd.publish(left_target)
            self.publisher_right_cmd.publish(right_target)


def start_stretch_arm_node():
    with RclpyRuntime() as rclpy:
        node = StretchArmNode()
        rclpy.spin(node)
