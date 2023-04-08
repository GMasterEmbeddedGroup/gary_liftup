"""
上下抬升模块
"""
from rclpy.node import Node
from std_msgs import msg as std_msgs


from .tools import RclpyRuntime


QOS = 3


class UpDownNode(Node):
    def __init__(self):
        super().__init__("gary_liftup_updown")
        self.publisher_left_cmd = self.create_publisher(std_msgs.Float64, "/liftup_left_pid/cmd", QOS)
        self.publisher_right_cmd = self.create_publisher(std_msgs.Float64, "/liftup_right_pid/cmd", QOS)

        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """

        """
        # --- setup message ---
        left_target = std_msgs.Float64()
        right_target = std_msgs.Float64()

        left_target.data = self.get_parameter("up_down_target").value
        right_target.data = - self.get_parameter("up_down_target").value

        print("------", self.get_parameter("up_down_target").value, type(self.get_parameter("up_down_target").value))
        # --- publish ---
        # self.publisher_left_cmd.publish(left_target)
        # self.publisher_right_cmd.publish(right_target)


def start_up_down_node():
    with RclpyRuntime() as rclpy:
        node = UpDownNode()
        rclpy.spin(node)
