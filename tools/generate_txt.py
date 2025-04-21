import os
import sys
import pdb


camera_index_map = {
    'camera1': 'CAM_FRONT',
    'camera2': 'CAM_BACK',
    'camera3': 'CAM_FRONT_LEFT',
    'camera4': 'CAM_FRONT_RIGHT',
    'camera5': 'CAM_BACK_LEFT',
    'camera6': 'CAM_BACK_RIGHT',
}


def main(load_dir):

    lidar_dir = os.path.join(load_dir, 'LIDAR')
    list_files = os.listdir(lidar_dir)
    list_files.sort()

    camera1_dir = os.path.join(load_dir, camera_index_map['camera1'])
    camera1_files = os.listdir(camera1_dir)
    camera1_files.sort()

    camera2_dir = os.path.join(load_dir, camera_index_map['camera2'])
    camera2_files = os.listdir(camera2_dir)
    camera2_files.sort()

    camera3_dir = os.path.join(load_dir, camera_index_map['camera3'])
    camera3_files = os.listdir(camera3_dir)
    camera3_files.sort()

    camera4_dir = os.path.join(load_dir, camera_index_map['camera4'])
    camera4_files = os.listdir(camera4_dir)
    camera4_files.sort()

    camera5_dir = os.path.join(load_dir, camera_index_map['camera5'])
    camera5_files = os.listdir(camera5_dir)
    camera5_files.sort()

    camera6_dir = os.path.join(load_dir, camera_index_map['camera6'])
    camera6_files = os.listdir(camera6_dir)
    camera6_files.sort()

    f_lidar_list = open(os.path.join(load_dir, 'lidar_list.txt'), 'w', encoding='utf-8')
    f_cam_list = {}
    f_cam_list['camera1'] = open(os.path.join(load_dir, 'cam_front_list.txt'), 'w', encoding='utf-8')
    f_cam_list['camera2'] = open(os.path.join(load_dir, 'cam_back_list.txt'), 'w', encoding='utf-8')
    f_cam_list['camera3'] = open(os.path.join(load_dir, 'cam_front_left_list.txt'), 'w', encoding='utf-8')
    f_cam_list['camera4'] = open(os.path.join(load_dir, 'cam_front_right_list.txt'), 'w', encoding='utf-8')
    f_cam_list['camera5'] = open(os.path.join(load_dir, 'cam_back_left_list.txt'), 'w', encoding='utf-8')
    f_cam_list['camera6'] = open(os.path.join(load_dir, 'cam_back_right_list.txt'), 'w', encoding='utf-8')

    for ele in list_files:
        f_lidar_list.write(ele + '\n')
    f_lidar_list.flush()
    f_lidar_list.close()


    for ele in camera1_files:
        f_cam_list['camera1'].write(ele + '\n')
    f_cam_list['camera1'].flush()
    f_cam_list['camera1'].close()

    for ele in camera2_files:
        f_cam_list['camera2'].write(ele + '\n')
    f_cam_list['camera2'].flush()
    f_cam_list['camera2'].close()

    for ele in camera3_files:
        f_cam_list['camera3'].write(ele + '\n')
    f_cam_list['camera3'].flush()
    f_cam_list['camera3'].close()

    for ele in camera4_files:
        f_cam_list['camera4'].write(ele + '\n')
    f_cam_list['camera4'].flush()
    f_cam_list['camera4'].close()

    for ele in camera5_files:
        f_cam_list['camera5'].write(ele + '\n')
    f_cam_list['camera5'].flush()
    f_cam_list['camera5'].close()

    for ele in camera6_files:
        f_cam_list['camera6'].write(ele + '\n')
    f_cam_list['camera6'].flush()
    f_cam_list['camera6'].close()


if __name__ == '__main__':
    if(len(sys.argv)!=2):
        print("Please enter load path")
        exit(1)
    if (os.path.exists(sys.argv[1])==False):
        print("Directory is illegal, Input again!")
        exit(1)
    load_dir = sys.argv[1]


    main(load_dir)