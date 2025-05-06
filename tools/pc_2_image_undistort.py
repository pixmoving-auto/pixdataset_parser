import copy
from typing import List, Optional, Tuple
import cv2
import numpy as np
from scipy.spatial.transform import Rotation as R
from PIL import Image
import pcl
import json
import yaml
import pdb
import os
import math



data_root = '/home/pix/ws/dataset/pix/university_city'
sequence_index = 'sequence00000'
camera_index = 'camera1'
idx = 0


camera_index_map = {
    'camera1': 'CAM_FRONT',
    'camera2': 'CAM_BACK',
    'camera3': 'CAM_FRONT_LEFT',
    'camera4': 'CAM_FRONT_RIGHT',
    'camera5': 'CAM_BACK_LEFT',
    'camera6': 'CAM_BACK_RIGHT',
}

def read_pcd(pcd_file):
    return pcl.load(pcd_file).to_array().astype(np.float32)
    with open(pcd_file, 'rb') as f:
        data = f.read()
        data0 = data[data.find(b"DATA binary") + 12:]
        pdb.set_trace()
        points = np.frombuffer(data0, dtype=np.float32).reshape(-1, 4)
        o = np.zeros((len(points), 1))
        points = np.hstack((points, o))
        points = points.astype(np.float32)
    return points


def project(points, image, M1, M2):
    """
    points: Nx3
    image: opencv img, 表示要投影的图像
    M1: 内参矩阵 K, 4*4
    M2: 外参矩阵， 4*4

    return: points 在像素坐标系下的坐标 N*4, 实际只用 N*2

    """
    resolution = image.shape

    coords = points[:, 0:3]
    ones = np.ones(len(coords)).reshape(-1, 1)
    coords = np.concatenate([coords, ones], axis=1)

    range_p = (points[:,0] * points[:,0] + points[:,1] * points[:,1] + points[:,2] * points[:,2])
    range_p = np.sqrt(range_p).reshape(-1, 1)
    coords = np.concatenate([coords, range_p], axis=1)

    transform = copy.deepcopy(M1 @ M2).reshape(4, 4)
    # coords @ transform.T == (transform @ coords.T).T
    coords[:,:4] = coords[:,:4] @ transform.T
    coords = coords[np.where(coords[:, 2] > 0)]

    coords[:, 2] = np.clip(coords[:, 2], a_min=1e-5, a_max=1e5)
    coords[:, 0] /= coords[:, 2]
    coords[:, 1] /= coords[:, 2]

    coords = coords[np.where(coords[:, 0] > 0)]
    coords = coords[np.where(coords[:, 0] < resolution[1])]
    coords = coords[np.where(coords[:, 1] > 0)]
    coords = coords[np.where(coords[:, 1] < resolution[0])]


    return coords


def show_with_opencv(image, coords=None, save=None, dir=None):
    """
    image: opencv image
    coords: 像素坐标系下的点, N*4
    """
    canvas = image.copy()
    cv2.putText(canvas,
                text= camera_index + ',' + camera_index_map[camera_index],
                org=(0, 90),
                fontFace=cv2.FONT_HERSHEY_PLAIN,
                fontScale=5.0,
                thickness=5,
                color=(0, 0, 255))
    canvas = cv2.cvtColor(canvas, cv2.COLOR_RGB2BGR)
    # 画点
    range_p = np.clip(coords[:,-1] * 5, 0, 255).astype(np.int32)
    if coords is not None:
        for index in range(coords.shape[0]):
            p = (int(coords[index, 0]), int(coords[index, 1]))          
            color = (255-range_p[index].astype(int).item(), 0, range_p[index].astype(int).item())
            cv2.circle(canvas, p, 2, color=color, thickness=1)
    canvas = canvas.astype(np.uint8)
    canvas = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)

    print(canvas.shape)
    canvas = cv2.resize(canvas, (1920, 1080))

    if save:
        if (os.path.exists(dir) == False):
            os.makedirs(dir)

        img_canvas = Image.fromarray(canvas)
        # pdb.set_trace()
        img_canvas.save(dir+camera_index + '_' + str('{:0>3d}'.format(idx)) +'map.jpeg')

    # cv2.namedWindow("image")  # 创建一个image的窗口
    # cv2.imshow("image", canvas)  # 显示图像
    # cv2.waitKey(0)  # 默认为0，无限等待


def get_calibration(sensor_kit_file, sensor_base_file, camera_intrinsic_file):
    with open(sensor_kit_file, 'r') as file:
        sensor_kit_data = yaml.load(file, Loader=yaml.FullLoader)

    with open(sensor_base_file, 'r') as file:
        sensor_base_data = yaml.load(file, Loader=yaml.FullLoader)
    
    with open(camera_intrinsic_file, 'r') as file:
        camera_intrinsic_data = yaml.load(file, Loader=yaml.FullLoader)

    return sensor_kit_data, sensor_base_data, camera_intrinsic_data


def undistort(k_matrix: np.array, d_matrix: np.array, frame):
    h, w = frame.shape[:2]
    # pdb.set_trace()
    mapx, mapy = cv2.initUndistortRectifyMap(k_matrix, d_matrix, None, k_matrix, (w, h), 5)
    return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

def distortionCorrection(k_matrix: np.array, d_matrix: np.array, frame):
    undistort_frame = undistort(k_matrix, d_matrix, frame)
    return undistort_frame

def distort(camera_intrinsic_data, save=None):
    K = np.array(camera_intrinsic_data['camera_matrix']['data']).reshape(
        camera_intrinsic_data['camera_matrix']['rows'], camera_intrinsic_data['camera_matrix']['cols'])

    D = np.array(camera_intrinsic_data['distortion_coefficients']['data']).reshape(14, 1).astype(np.float32)

    img_dir = os.path.join(data_root, sequence_index, camera_index_map[camera_index])
    list_files = os.listdir(img_dir)
    list_files.sort()
    img_before = os.path.join(img_dir, list_files[idx])

    resolution=(1920, 1080)
    img = Image.open(img_before)
    img = img.resize(resolution)
    img_bgr = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)

    img_distort_np = distortionCorrection(K, D, img_bgr)

    img_distort_np = cv2.cvtColor(img_distort_np, cv2.COLOR_BGR2RGB)

    # pdb.set_trace()
    if save:
        img_distort = Image.fromarray(img_distort_np)
        img_distort = img_distort.resize((1920, 1080))
        img_distort.save(camera_index +  'distort.jpeg')

    return img_distort_np


def get_M1andM2(sensor_kit_data, sensor_base_data, camera_intrinsic_data):
    M1 = np.eye(4)
    M1[:3][...,:3] = np.array(camera_intrinsic_data['camera_matrix']['data']).reshape(
        camera_intrinsic_data['camera_matrix']['rows'], camera_intrinsic_data['camera_matrix']['cols'])

    

    euler = [sensor_kit_data['sensor_kit_base_link'][camera_index+'/camera_link']['roll'],
             sensor_kit_data['sensor_kit_base_link'][camera_index+'/camera_link']['pitch'],
             sensor_kit_data['sensor_kit_base_link'][camera_index+'/camera_link']['yaw']]
    sensor_kit2camera_R = R.from_euler('xyz', euler)
    sensor_kit2camera_matrix = sensor_kit2camera_R.as_matrix()
    sensor_kit2camera = np.eye(4)
    sensor_kit2camera[:3][...,:3] = sensor_kit2camera_matrix
    sensor_kit2camera[...,-1][:3] = np.array([sensor_kit_data['sensor_kit_base_link'][camera_index+'/camera_link']['x'],
                                           sensor_kit_data['sensor_kit_base_link'][camera_index+'/camera_link']['y'],
                                           sensor_kit_data['sensor_kit_base_link'][camera_index+'/camera_link']['z']])
    # pdb.set_trace()

    euler = [sensor_base_data['base_link']['sensor_kit_base_link']['roll'],
             sensor_base_data['base_link']['sensor_kit_base_link']['pitch'],
             sensor_base_data['base_link']['sensor_kit_base_link']['yaw']]
    base_link2sensor_kit_R = R.from_euler('xyz', euler)
    base_link2sensor_kit_matrix = base_link2sensor_kit_R.as_matrix()
    base_link2sensor_kit = np.eye(4)
    base_link2sensor_kit[:3][...,:3] = base_link2sensor_kit_matrix
    base_link2sensor_kit[...,-1][:3] = np.array([sensor_base_data['base_link']['sensor_kit_base_link']['x'],
                                                 sensor_base_data['base_link']['sensor_kit_base_link']['y'],
                                                 sensor_base_data['base_link']['sensor_kit_base_link']['z']])

    M2 = np.linalg.inv(sensor_kit2camera) @ np.linalg.inv(base_link2sensor_kit)
    # M2 = base_link2sensor_kit @ sensor_kit2camera
    # pdb.set_trace()

    return M1, M2                                            

def main():

    lidar_dir = os.path.join(data_root, sequence_index, 'LIDAR')
    list_files = os.listdir(lidar_dir)
    list_files.sort()
    lidar_file = os.path.join(lidar_dir, list_files[idx])
    # pdb.set_trace()

    points = read_pcd(lidar_file)
    # img = cv2.imread('/home/pix/ws/dataset/pix/ws/src/bag_recorder_nodes_py/bag_recorder_nodes_py/img_after.jpeg')


    sensor_kit_file = '/home/pix/ws/parameter/sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensor_kit_calibration.yaml'
    sensor_base_file = '/home/pix/ws/parameter/sensor_kit/robobus_sensor_kit_description/extrinsic_parameters/sensors_calibration.yaml'
    camera_intrinsic_file = '/home/pix/ws/parameter/sensor_kit/robobus_sensor_kit_description/intrinsic_parameters/'+camera_index+'_params.yaml'
    
    sensor_kit_data, sensor_base_data, camera_intrinsic_data = get_calibration(sensor_kit_file, sensor_base_file, camera_intrinsic_file)
    img = distort(camera_intrinsic_data, save=False)
    # pdb.set_trace()

    M1, M2 = get_M1andM2(sensor_kit_data, sensor_base_data, camera_intrinsic_data)
    

    coords = project(points, img, M1, M2)
    # pdb.set_trace()
    show_with_opencv(img, coords=coords, save=True, dir=str(os.path.join(data_root, sequence_index, camera_index_map[camera_index]))+ '_map/')


if __name__ == '__main__':
    # main()
    for i in range(0, 200):
        main()
        idx +=1
