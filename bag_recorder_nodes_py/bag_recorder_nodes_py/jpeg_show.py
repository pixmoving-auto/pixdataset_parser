#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import sys

class CompressedImageSubscriber(Node):
    def __init__(self, topic_name:str):
        super().__init__('compressed_image_subscriber')

        # 订阅 /camera/compressed 话题（修改成你的话题名称）
        self.subscription = self.create_subscription(
            CompressedImage,
            topic_name,
            self.image_callback,
            10
        )
        self.subscription  # 避免 GC 回收

    def image_callback(self, msg):
        try:
            # 将 ROS 2 的 CompressedImage 消息转换成 OpenCV 格式
            np_arr = np.frombuffer(msg.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if image is not None:
                # 显示图像
                cv2.imshow("Compressed Image", image)
                cv2.waitKey(1)
            else:
                self.get_logger().warn("解码失败，收到空图像")

        except Exception as e:
            self.get_logger().error(f"处理图像时出错: {str(e)}")

def main(args=None):
    topic_name = '/gmsl/camera_image_jpeg'
    if(len(sys.argv)==2):
        topic_name = sys.argv[1]   # 其他命令行参数
    rclpy.init(args=args)
    node = CompressedImageSubscriber(topic_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
