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
            logger.info("timer called %s" % format(datetime.now()))

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
        self.timer_callback_count: int = 0
        self.message_delivered_names = set()
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
        将收到的 Topic 的名称作为属性名, 将 Topic 值设置为该属性的值
        """
        setattr(self, attr_name, value)
        self.message_delivered_names.add(attr_name)

        # --- logger ---
        times = self.timer_callback_count = (self.timer_callback_count + 1) % 100
        if times == 1:
            logger = self.get_logger()
            logger.info(
                type(self).__name__ +
                "|".join(str(getattr(self, i)) for i in self.message_delivered_names))


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
