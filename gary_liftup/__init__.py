"""

"""

import rclpy
from rclpy.node import Node
from std_msgs import msg as std_msgs

from datetime import datetime
from functools import partial

QOS = 3


class TestRunningNode(Node):
    def __init__(self, test_1_value: float = 10.0, test_2_value: float = 10.0):
        super().__init__(type(self).__name__)

        self.test_1_value = test_1_value
        self.test_2_value = test_2_value
        self.timer_callback_count: int = 0

        self.test_topic_publisher_1 = self.create_publisher(std_msgs.Float64, "1", QOS)
        self.test_topic_publisher_2 = self.create_publisher(std_msgs.Float64, "2", QOS)
        self.timer = self.create_timer(0.01, self.timer_callback)

    def timer_callback(self):
        """

        """
        # --- logger ---
        times = self.timer_callback_count = (self.timer_callback_count + 1) % 200
        if times == 1:
            logger = self.get_logger()
            logger.info("timer called %d, %s" % (times, format(datetime.now())))

        # --- setup message ---
        test_1_msg = std_msgs.Float64()
        test_2_msg = std_msgs.Float64()

        test_1_msg.data = self.test_1_value
        test_2_msg.data = self.test_2_value

        # --- publish ---
        self.test_topic_publisher_1.publish(test_1_msg)
        self.test_topic_publisher_2.publish(test_2_msg)


class TestCalibrateNode(Node):
    """
    测试校准节点
    """

    def __init__(self):
        super().__init__(type(self).__name__)

        # --- setup values ---
        self.subscriber_1_value = None
        self.subscriber_2_value = None

        # --- setup subscribers ---
        self.subscriber_1 = self.create_subscription(std_msgs.Float64, "TODO: 1",
                                                     partial(self.message_delivered, "subscriber_1_value"),
                                                     QOS)
        self.subscriber_2 = self.create_subscription(std_msgs.Float64, "TODO: 2",
                                                     partial(self.message_delivered, "subscriber_2_value"),
                                                     QOS)

    def message_delivered(self, attr_name, value):
        """

        """
        setattr(self, attr_name, value)


class RclpyRuntime:
    def __enter__(self):
        rclpy.init()

    def __exit__(self, exc_type, exc_val, exc_tb):
        # XXX: 显示错误
        rclpy.shutdown()


def test_running():
    with RclpyRuntime():
        node = TestRunningNode()
        rclpy.spin(node)


def test_calibrate():
    with RclpyRuntime():
        node = TestCalibrateNode()
        rclpy.spin(node)
