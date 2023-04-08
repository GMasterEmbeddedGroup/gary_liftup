"""
小工具
"""
import rclpy


class RclpyRuntime:
    def __enter__(self):
        rclpy.init()
        return rclpy

    def __exit__(self, exc_type, exc_val, exc_tb):
        # XXX: 显示错误
        rclpy.shutdown()
