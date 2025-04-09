import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.serialization import serialize_message
from std_msgs.msg import String
from rclpy.clock import Clock
from message_filters import Subscriber, TimeSynchronizer, ApproximateTimeSynchronizer
from sensor_msgs.msg import Temperature, FluidPressure
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
# import tf2_ros
# import sensor_msgs.point_cloud2 as pc2
import numpy as np
import cv2
import sys
from pcl import PointCloud
import pcl
import ros2_numpy
# import rospy
import os
import pdb
import json
from datetime import datetime

import sys
sys.path.append("src/bag_recorder_nodes_py/bag_recorder_nodes_py/")
from jpeg_show import CompressedImageSubscriber
from msg_convert import msg2np

from ros2_threadpool import ROS2ThreadPool


pix_time = ""
location = 'university_city'
weather = 'cloudy'
vehicle = '59th'


class TimeSyncNode(Node):
    def __init__(self, save_dir:str):
        super().__init__('sync_node')
        # qos = QoSProfile(depth=10)
        # self.temp_pub = self.create_publisher(Temperature, 'temp', qos)
        # self.fluid_pub = self.create_publisher(FluidPressure, 'fluid', qos)
        self.sequence_num = 0
        self.frames_one_sequence = 200
        self.save_hz = 2
        self.real_time_hz = 10
        self.current_sequence = 0
        self.currnet_frame = 0
        self.save_dir = save_dir

        camera_front1 = "/orin1/electronic_rearview_mirror/camera1/camera_image_jpeg"
        camera_back2 = "/orin1/electronic_rearview_mirror/camera2/camera_image_jpeg"
        camera_front_left3 = "/electronic_rearview_mirror/camera2/camera_image_jpeg"
        camera_front_right4 = "/electronic_rearview_mirror/camera3/camera_image_jpeg"           
        camera_back_left5 = "/electronic_rearview_mirror/camera4/camera_image_jpeg"
        camera_back_right6 = "/electronic_rearview_mirror/camera5/camera_image_jpeg"
        pointcloud = "/sensing/lidar/concatenated/pointcloud"

        qos_profile = QoSProfile(
			reliability = QoSReliabilityPolicy.BEST_EFFORT,
			history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
		)
        # elf.subscribe = self.create_subscription(Odometry,"odom",self.callback,qos_profile)

        self.camera1_sub = Subscriber(self, CompressedImage, camera_front1) #front
        self.camera2_sub = Subscriber(self, CompressedImage, camera_back2) # back
        self.camera3_sub = Subscriber(self, CompressedImage, camera_front_left3) #front_left
        self.camera4_sub = Subscriber(self, CompressedImage, camera_front_right4) #front_right
        self.camera5_sub = Subscriber(self, CompressedImage, camera_back_left5) #back_left
        self.camera6_sub = Subscriber(self, CompressedImage, camera_back_right6) #back_right
        self.pointcloud_sub = Subscriber(self, PointCloud2, pointcloud, qos_profile=qos_profile)

        self.time_sync = ApproximateTimeSynchronizer([self.pointcloud_sub,
                                                      self.camera1_sub, self.camera2_sub,
                                                      self.camera3_sub, self.camera4_sub,
                                                      self.camera5_sub, self.camera6_sub],
                                                     10, 0.1)
        self.time_sync.registerCallback(self.SyncCallback)

        self.info = {}
        self.info['time'] = ''
        self.info['location'] = location
        self.info['weather'] = weather
        self.info['vehicle'] = vehicle
        self.info['tf_data'] = []

        # self.tf_data = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # create thread pool
        # pdb.set_trace()
        # self._executor = MultiThreadedExecutor(num_threads=5)
        # self._executor.add_node(self)
        # self._executor.create_task(self.task)
        # self.future = Node


        # ros2 threadpool
        self.pool = ROS2ThreadPool(num_threads=10)
        self.pool.start()



    def save_to_json(self, dir, data):
        with open(os.path.join(dir, 'data.json'), 'w') as json_file:
            json.dump(data, json_file, indent=4)

    def transform_to_dict(self, transform):
        """将TransformStamped消息转换为字典"""
        return {
            'header': {
                'stamp': {
                    'sec': transform.header.stamp.sec,
                    'nanosec': transform.header.stamp.nanosec
                },
                'frame_id': transform.header.frame_id
            },
            'child_frame_id': transform.child_frame_id,
            'transform': {
                'translation': {
                    'x': transform.transform.translation.x,
                    'y': transform.transform.translation.y,
                    'z': transform.transform.translation.z
                },
                'rotation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                }
            }
            
        }


    def SyncCallback(self, pointcloud, camera1, camera2, camera3, camera4, camera5, camera6):

        if (self.current_sequence % (self.real_time_hz/self.save_hz) != 0):
            pass

        trans = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
        tf_dict = self.transform_to_dict(trans)
        self.info['tf_data'].append(tf_dict)
        print(tf_dict)

        sequence_dir = os.path.join(self.save_dir, "sequence{:0>5d}".format(self.current_sequence))
        self.create_dir(sequence_dir)

        # sumbit task to threadpool
        success = self.pool.submit_task(self.save_pointcloud, pointcloud, os.path.join(sequence_dir, "LIDAR"))
        success = self.pool.submit_task(self.save_image, camera1, os.path.join(sequence_dir, "CAM_FRONT"))
        success = self.pool.submit_task(self.save_image, camera2, os.path.join(sequence_dir, "CAM_BACK"))
        success = self.pool.submit_task(self.save_image, camera3, os.path.join(sequence_dir, "CAM_FRONT_LEFT"))
        success = self.pool.submit_task(self.save_image, camera4, os.path.join(sequence_dir, "CAM_FRONT_RIGHT"))
        success = self.pool.submit_task(self.save_image, camera5, os.path.join(sequence_dir, "CAM_BACK_LEFT"))
        success = self.pool.submit_task(self.save_image, camera6, os.path.join(sequence_dir, "CAM_BACK_RIGHT"))
        # if success:
        #     self.pool.get_logger().info(f"已提交任务: {task_data}")
        # time.sleep(0.5)  # 控制任务提交频率
        # return

        self.currnet_frame += 1

        # pdb.set_trace()
        if self.currnet_frame >= self.frames_one_sequence :
            self.currnet_frame = self.currnet_frame % self.frames_one_sequence
            self.current_sequence += 1

            dt = datetime.fromtimestamp(pointcloud.header.stamp.sec)
            time_s = "{:0>4d}-{:0>2d}-{:0>2d} {:0>2d}:{:0>2d}:{:0>2d}".format(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)
            self.info['time'] = time_s
            self.save_to_json(sequence_dir, self.info)
            self.info['tf_data'] = []




    def create_dir(self, sequence_dir):
        lidar_dir = os.path.join(sequence_dir, "LIDAR")
        cam_front_dir = os.path.join(sequence_dir, "CAM_FRONT")
        cam_front_right_dir = os.path.join(sequence_dir, "CAM_FRONT_RIGHT")
        cam_front__left_dir = os.path.join(sequence_dir, "CAM_FRONT_LEFT")
        cam_back_dir = os.path.join(sequence_dir, "CAM_BACK")
        cam_back_left_dir = os.path.join(sequence_dir, "CAM_BACK_LEFT")
        cam_back_right_dir = os.path.join(sequence_dir, "CAM_BACK_RIGHT")
        if (os.path.exists(sequence_dir) == False):
            os.makedirs(sequence_dir)
            os.makedirs(lidar_dir)
            os.makedirs(cam_front_dir)
            os.makedirs(cam_front_right_dir)
            os.makedirs(cam_front__left_dir)
            os.makedirs(cam_back_dir)
            os.makedirs(cam_back_left_dir)
            os.makedirs(cam_back_right_dir)


    def save_image(self, msg, dir):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        filename = "{:0>10d}-{:0>9d}".format(msg.header.stamp.sec, msg.header.stamp.nanosec) + '.jpeg'
        cv2.imwrite(os.path.join(dir, filename), image)

    def save_pointcloud(self, msg, dir):
        np_cloud = msg2np(msg)
        cloud = pcl.PointCloud_PointXYZI()
        cloud.from_array(np_cloud.view(np.float32))
        filename = "{:0>10d}-{:0>9d}".format(msg.header.stamp.sec, msg.header.stamp.nanosec) + '.pcd'
        pcl.save(cloud, os.path.join(dir, filename))
        
        # pc_data = ros2_numpy.numpify(msg)
        # cloud = pcl.PointCloud()
        # cloud.from_array(np.concatenate((pc_data['xyz'], pc_data['intensity']), axis=1).view(np.float32))


def main(args=None):
    topic_name = '/orin1/electronic_rearview_mirror/camera2/camera_image_jpeg'
    if(len(sys.argv)!=2):
        print("Please enter saved path")
        exit(1)
    if (os.path.exists(sys.argv[1])==False):
        print("Directory is illegal, Input again!")
        exit(1)
    save_dir = sys.argv[1]

    rclpy.init(args=args)
    # node = CompressedImageSubscriber(topic_name)
    node = TimeSyncNode(save_dir)
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
