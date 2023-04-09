"""
上下抬升模块
"""
from .tools import RclpyRuntime


from rclpy.node import Node
from std_msgs import msg as std_msgs


import time


QOS = 3


class UpDownNode(Node):
    def __init__(self):
        super().__init__("gary_liftup_updown")
        self.declare_parameter("up_down_target", 0.0)

        self.publisher_left_cmd = self.create_publisher(std_msgs.Float64, "/liftup_left_pid/cmd", QOS)
        self.publisher_right_cmd = self.create_publisher(std_msgs.Float64, "/liftup_right_pid/cmd", QOS)

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """

        """
        # --- setup message ---
        left_target = std_msgs.Float64()
        right_target = std_msgs.Float64()

        value = self.get_parameter("up_down_target").value

        try:
            left_target.data = value
            right_target.data = - value
        except AssertionError as e:
            logger = self.get_logger()
            logger.warn("AssertionError, " + str(e))
        else:
            # NOTE: self.get_parameter("up_down_target").value: 0.0 (type: <class 'float'>)
            print("\r gary_liftup_updown: [%.2f] get parameter `up_down_target`: %.1f" % (time.time() % 1e5, value),
                  end="", flush=True)

            # --- publish ---
            self.publisher_left_cmd.publish(left_target)
            self.publisher_right_cmd.publish(right_target)


def start_up_down_node():
    with RclpyRuntime() as rclpy:
        node = UpDownNode()
        rclpy.spin(node)
