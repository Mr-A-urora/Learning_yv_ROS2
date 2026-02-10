from demo_python_pkg.person_node import PersonNode
import rclpy

class WriterNode(PersonNode):
    def __init__(self, node_name: str, name: str, age: int, book: str) -> None:
        print('WriterNode __init__ 方法被调用了')
        super().__init__(node_name, name, age)
        self.book = book

    def write(self):
        self.get_logger().info(f"{self.age}岁的{self.name}，正在写一本书，书名叫做《{self.book}》")

def main():
    rclpy.init()
    node = WriterNode('zhangsan', '法外狂徒张三', 18, '论快速入狱')
    node.eat("鱼香ROS")
    node.write()
    rclpy.spin(node)
    rclpy.shutdown()
