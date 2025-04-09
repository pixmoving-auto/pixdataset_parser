import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rosbag2_py
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import numpy as np
import cv2
from pcl import PointCloud
import pcl
import os
import pdb
import json


import sys
sys.path.append("src/bag_recorder_nodes_py/bag_recorder_nodes_py/")
from jpeg_show import CompressedImageSubscriber
from msg_convert import msg2np

from ros2_threadpool import ROS2ThreadPool



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
        # self.current_sequence = 0
        self.currnet_frame = 0
        self.save_dir = save_dir

        topic_camera_front1 = "/orin1/electronic_rearview_mirror/camera1/camera_image_jpeg"
        topic_camera_back2 = "/orin1/electronic_rearview_mirror/camera2/camera_image_jpeg"
        topic_camera_front_left3 = "/electronic_rearview_mirror/camera2/camera_image_jpeg"
        topic_camera_front_right4 = "/electronic_rearview_mirror/camera3/camera_image_jpeg"           
        topic_camera_back_left5 = "/electronic_rearview_mirror/camera4/camera_image_jpeg"
        topic_camera_back_right6 = "/electronic_rearview_mirror/camera5/camera_image_jpeg"
        topic_pointcloud = "/sensing/lidar/concatenated/pointcloud"

        # pdb.set_trace()
        self.raw_dir = os.path.join(self.save_dir, 'raw')
        self.create_dir(self.raw_dir)
        self.f_lidar_list = open(os.path.join(self.raw_dir, 'lidar_list.txt'), 'w', encoding='utf-8')
        self.f_cam1_list = open(os.path.join(self.raw_dir, 'cam_front_list.txt'), 'w', encoding='utf-8')
        self.f_cam2_list = open(os.path.join(self.raw_dir, 'cam_back_list.txt'), 'w', encoding='utf-8')
        self.f_cam3_list = open(os.path.join(self.raw_dir, 'cam_front_left_list.txt'), 'w', encoding='utf-8')
        self.f_cam4_list = open(os.path.join(self.raw_dir, 'cam_front_right_list.txt'), 'w', encoding='utf-8')
        self.f_cam5_list = open(os.path.join(self.raw_dir, 'cam_back_left_list.txt'), 'w', encoding='utf-8')
        self.f_cam6_list = open(os.path.join(self.raw_dir, 'cam_back_right_list.txt'), 'w', encoding='utf-8')
        self.f_tf_list = open(os.path.join(self.raw_dir, 'tf_list.txt'), 'w', encoding='utf-8')


        qos_profile = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )
        self.pointcloud_sub = self.create_subscription(PointCloud2, topic_pointcloud, self.pointcloud_cb, qos_profile=qos_profile)
        self.camera1_sub = self.create_subscription(CompressedImage, topic_camera_front1, self.camera1_cb, 10)
        self.camera2_sub = self.create_subscription(CompressedImage, topic_camera_back2, self.camera2_cb, 10)
        self.camera3_sub = self.create_subscription(CompressedImage, topic_camera_front_left3, self.camera3_cb, 10)
        self.camera4_sub = self.create_subscription(CompressedImage, topic_camera_front_right4, self.camera4_cb, 10)
        self.camera5_sub = self.create_subscription(CompressedImage, topic_camera_back_left5, self.camera5_cb, 10)
        self.camera6_sub = self.create_subscription(CompressedImage, topic_camera_back_right6, self.camera6_cb, 10)
        

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


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

    def tf_cb(self):
        trans = self.tf_buffer.lookup_transform('map','base_link',rclpy.time.Time())
        tf_dict = self.transform_to_dict(trans)
        self.f_tf_list.write(json.dumps(tf_dict, ensure_ascii=False) + '\n')
        self.f_tf_list.flush()
        # print(tf_dict)

    def pointcloud_cb(self, pointcloud):
        success = self.pool.submit_task(self.save_pointcloud, pointcloud, os.path.join(self.raw_dir, "LIDAR"))
        filename = "{:0>10d}.{:0>9d}".format(pointcloud.header.stamp.sec, pointcloud.header.stamp.nanosec) + '.pcd'
        self.f_lidar_list.write(filename+'\n')
        self.f_lidar_list.flush()
        self.tf_cb()

    def camera1_cb(self, camera):
        success = self.pool.submit_task(self.save_image, camera, os.path.join(self.raw_dir, "CAM_FRONT"))
        filename = "{:0>10d}.{:0>9d}".format(camera.header.stamp.sec, camera.header.stamp.nanosec) + '.jpeg'
        self.f_cam1_list.write(filename+'\n')
        self.f_cam1_list.flush()

    def camera2_cb(self, camera):
        success = self.pool.submit_task(self.save_image, camera, os.path.join(self.raw_dir, "CAM_BACK"))
        filename = "{:0>10d}.{:0>9d}".format(camera.header.stamp.sec, camera.header.stamp.nanosec) + '.jpeg'
        self.f_cam2_list.write(filename+'\n')
        self.f_cam2_list.flush()
    def camera3_cb(self, camera):
        success = self.pool.submit_task(self.save_image, camera, os.path.join(self.raw_dir, "CAM_FRONT_LEFT"))
        filename = "{:0>10d}.{:0>9d}".format(camera.header.stamp.sec, camera.header.stamp.nanosec) + '.jpeg'
        self.f_cam3_list.write(filename+'\n')
        self.f_cam3_list.flush()
    def camera4_cb(self, camera):
        success = self.pool.submit_task(self.save_image, camera, os.path.join(self.raw_dir, "CAM_FRONT_RIGHT"))
        filename = "{:0>10d}.{:0>9d}".format(camera.header.stamp.sec, camera.header.stamp.nanosec) + '.jpeg'
        self.f_cam4_list.write(filename+'\n')
        self.f_cam4_list.flush()
    def camera5_cb(self, camera):
        success = self.pool.submit_task(self.save_image, camera, os.path.join(self.raw_dir, "CAM_BACK_LEFT"))
        filename = "{:0>10d}.{:0>9d}".format(camera.header.stamp.sec, camera.header.stamp.nanosec) + '.jpeg'
        self.f_cam5_list.write(filename+'\n')
        self.f_cam5_list.flush()
    def camera6_cb(self, camera):
        success = self.pool.submit_task(self.save_image, camera, os.path.join(self.raw_dir, "CAM_BACK_RIGHT"))
        filename = "{:0>10d}.{:0>9d}".format(camera.header.stamp.sec, camera.header.stamp.nanosec) + '.jpeg'
        self.f_cam6_list.write(filename+'\n')
        self.f_cam6_list.flush()


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
        filename = "{:0>10d}.{:0>9d}".format(msg.header.stamp.sec, msg.header.stamp.nanosec) + '.jpeg'
        cv2.imwrite(os.path.join(dir, filename), image)

    def save_pointcloud(self, msg, dir):
        np_cloud = msg2np(msg)
        cloud = pcl.PointCloud_PointXYZI()
        cloud.from_array(np_cloud.view(np.float32))
        filename = "{:0>10d}.{:0>9d}".format(msg.header.stamp.sec, msg.header.stamp.nanosec) + '.pcd'
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
