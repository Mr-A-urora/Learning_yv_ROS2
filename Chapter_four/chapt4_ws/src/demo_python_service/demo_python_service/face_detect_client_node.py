import rclpy
import face_recognition
import cv2
import os
import time
from chapt4_interfaces.srv import FaceDetector
from rclpy.node import Node
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.bridge = CvBridge()
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/test1.jpg')
        self.get_logger().info("人脸检测客户端已经启动！")
        self.client = self.create_client(FaceDetector, 'face_detect')
        self.image = cv2.imread(self.default_image_path)

    def send_request(self):
        # 判断服务端是否在线
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info('等待服务端上线！')

        # 构造Request
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)

        # 发送请求并等待处理完成
        future = self.client.call_async(request)   # 现在的future并没有包含响应结果，需要等待服务端处理完成才会把结果放到future中
        # while not future.done():
            # time.sleep(1.0)   # 休眠当前线程，等待服务处理完成===造成当前线程无法再接受来自服务端的返回，导致永远没有办法完成=》 future.done 无法返回True
        
        # rclpy.spin_until_future_complete(self, future)   # 等待服务端返回响应
        
        def result_callback(result_future):
            response = result_future.result()   # 获取响应
            self.get_logger().info(f'接收到响应， 共检测到有{response.number}张人脸， 耗时{response.use_time}s')
            self.show_response(response)

        future.add_done_callback(result_callback)

    def show_response(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 4)
        cv2.imshow('Face Detect Result', self.image)
        cv2.waitKey(0)   # 也是阻塞的，会导致spin无法正常运行

def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()
