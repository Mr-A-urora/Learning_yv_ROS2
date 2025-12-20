import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("Python_node")
    node.get_logger().info("你好 Python 节点！")
    node.get_logger().warn("你好 Python 节点！")
    node.get_logger().error("你好 Python 节点！")
    rclpy.spin(node)
    rclpy.shutdown()

