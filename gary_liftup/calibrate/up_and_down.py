"""
此文件负责升降
:author: Juntong Zhu
:date: 2023/4/5
"""

import rclpy
from rclpy.node import Node
from std_msgs import msg as std_msgs


class UpDownNode(Node):
    """
    升降控制节点

    - liftup_left
    - liftup_right
    """
    def __init__(self):
        super().__init__(type(self).__name__)

