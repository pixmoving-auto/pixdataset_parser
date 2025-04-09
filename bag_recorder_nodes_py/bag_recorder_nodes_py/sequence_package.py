import os
import sys
import numpy as np
import math
import shutil
import pdb
from datetime import datetime
import json
# def


pix_time = ""
location = 'university_city'
weather = 'cloudy'
vehicle = '59th'



sequence_num = 0
frames_one_sequence = 200
save_hz = 2
real_hz = 10
current_sequence = 0
currnet_frame = 0

last_index = [0,0,0,0,0,0]
camera_index_map = {
    'camera1': 'CAM_FRONT',
    'camera2': 'CAM_BACK',
    'camera3': 'CAM_FRONT_LEFT',
    'camera4': 'CAM_FRONT_RIGHT',
    'camera5': 'CAM_BACK_LEFT',
    'camera6': 'CAM_BACK_RIGHT',
}


def find_nearst(value:float, value_list, last_index, last_diff):
    if (last_index >= len(value_list)-1):
        return last_index, last_diff
    diff = math.fabs(value - value_list[last_index])
    if diff > last_diff:
        return last_index-1, last_diff

    return find_nearst(value, value_list, last_index + 1, diff)


def create_dir(sequence_dir):
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



def save_to_json(dir, data):
    with open(os.path.join(dir, 'data.json'), 'w') as json_file:
        json.dump(data, json_file, indent=4)
        
def main(load_dir:str, save_dir:str):

    raw_dir = os.path.join(load_dir, 'raw')

    f_lidar_list = open(os.path.join(raw_dir, 'lidar_list.txt'), 'r', encoding='utf-8')
    f_cam1_list = open(os.path.join(raw_dir, 'cam_front_list.txt'), 'r', encoding='utf-8')
    f_cam2_list = open(os.path.join(raw_dir, 'cam_back_list.txt'), 'r', encoding='utf-8')
    f_cam3_list = open(os.path.join(raw_dir, 'cam_front_left_list.txt'), 'r', encoding='utf-8')
    f_cam4_list = open(os.path.join(raw_dir, 'cam_front_right_list.txt'), 'r', encoding='utf-8')
    f_cam5_list = open(os.path.join(raw_dir, 'cam_back_left_list.txt'), 'r', encoding='utf-8')
    f_cam6_list = open(os.path.join(raw_dir, 'cam_back_right_list.txt'), 'r', encoding='utf-8')
    f_tf_list = open(os.path.join(raw_dir, 'tf_list.txt'), 'r', encoding='utf-8')
    tf_list = [json.loads(line) for line in f_tf_list]


    lidar_list = f_lidar_list.readlines()
    cam1_list = f_cam1_list.readlines()
    cam2_list = f_cam2_list.readlines()
    cam3_list = f_cam3_list.readlines()
    cam4_list = f_cam4_list.readlines()
    cam5_list = f_cam5_list.readlines()
    cam6_list = f_cam6_list.readlines()

    cam1_array = [float(file.split('.jpeg')[0]) for file in cam1_list]
    cam2_array = [float(file.split('.jpeg')[0]) for file in cam2_list]
    cam3_array = [float(file.split('.jpeg')[0]) for file in cam3_list]
    cam4_array = [float(file.split('.jpeg')[0]) for file in cam4_list]
    cam5_array = [float(file.split('.jpeg')[0]) for file in cam5_list]
    cam6_array = [float(file.split('.jpeg')[0]) for file in cam6_list]

    info = {}
    info['time'] = ''
    info['location'] = location
    info['weather'] = weather
    info['vehicle'] = vehicle
    info['tf_data'] = []

    for i, line in enumerate(lidar_list):
        global current_sequence
        global sequence_num
        global frames_one_sequence
        global save_hz
        global real_hz
        global current_sequence
        global currnet_frame
        global last_index
        global camera_index_map

        if (i % (real_hz/save_hz) != 0):
            continue

        os.path.splitext(line)
        time_str = line.split('.pcd')[0]
        time_float = float(time_str)

        # pdb.set_trace()
        idx_cam1, diff_cam1 = find_nearst(time_float, cam1_array, last_index[1-1], 999999)
        print(diff_cam1)
        last_index[1-1] = idx_cam1
        idx_cam2, diff_cam1 = find_nearst(time_float, cam2_array, last_index[2-1], 999999)
        last_index[2-1] = idx_cam2
        idx_cam3, diff_cam1 = find_nearst(time_float, cam3_array, last_index[3-1], 999999)
        last_index[3-1] = idx_cam3
        idx_cam4, diff_cam1 = find_nearst(time_float, cam4_array, last_index[4-1], 999999)
        last_index[4-1] = idx_cam4
        idx_cam5, diff_cam1 = find_nearst(time_float, cam5_array, last_index[5-1], 999999)
        last_index[5-1] = idx_cam5
        idx_cam6, diff_cam1 = find_nearst(time_float, cam6_array, last_index[6-1], 999999)
        last_index[6-1] = idx_cam6
        


        # pdb.set_trace()
        sequence_dir = os.path.join(save_dir, "sequence{:0>5d}".format(current_sequence))
        create_dir(sequence_dir)
        

        info['tf_data'].append(tf_list[i])
        load_file = os.path.join(raw_dir, 'LIDAR', lidar_list[i].split('\n')[0])
        save_file = os.path.join(sequence_dir, 'LIDAR', lidar_list[i].split('\n')[0])
        shutil.copy(load_file, save_file)
        load_file = os.path.join(raw_dir, camera_index_map['camera1'], cam1_list[idx_cam1].split('\n')[0])
        save_file = os.path.join(sequence_dir, camera_index_map['camera1'], cam1_list[idx_cam1].split('\n')[0])
        shutil.copy(load_file, save_file)
        load_file = os.path.join(raw_dir, camera_index_map['camera2'], cam2_list[idx_cam2].split('\n')[0])
        save_file = os.path.join(sequence_dir, camera_index_map['camera2'], cam2_list[idx_cam2].split('\n')[0])
        shutil.copy(load_file, save_file)
        load_file = os.path.join(raw_dir, camera_index_map['camera3'], cam3_list[idx_cam3].split('\n')[0])
        save_file = os.path.join(sequence_dir, camera_index_map['camera3'], cam3_list[idx_cam3].split('\n')[0])
        shutil.copy(load_file, save_file)
        load_file = os.path.join(raw_dir, camera_index_map['camera4'], cam4_list[idx_cam4].split('\n')[0])
        save_file = os.path.join(sequence_dir, camera_index_map['camera4'], cam4_list[idx_cam4].split('\n')[0])
        shutil.copy(load_file, save_file)
        load_file = os.path.join(raw_dir, camera_index_map['camera5'], cam5_list[idx_cam5].split('\n')[0])
        save_file = os.path.join(sequence_dir, camera_index_map['camera5'], cam5_list[idx_cam5].split('\n')[0])
        shutil.copy(load_file, save_file)
        load_file = os.path.join(raw_dir, camera_index_map['camera6'], cam6_list[idx_cam6].split('\n')[0])
        save_file = os.path.join(sequence_dir, camera_index_map['camera6'], cam6_list[idx_cam6].split('\n')[0])
        shutil.copy(load_file, save_file)

        currnet_frame += 1
        if currnet_frame >= frames_one_sequence :
            currnet_frame = currnet_frame % frames_one_sequence
            
            current_sequence+=1

            # pdb.set_trace()
            dt = datetime.fromtimestamp(tf_list[i]['header']['stamp']['sec'])
            time_s = "{:0>4d}-{:0>2d}-{:0>2d} {:0>2d}:{:0>2d}:{:0>2d}".format(dt.year, dt.month, dt.day, dt.hour, dt.minute, dt.second)
            info['time'] = time_s
            save_to_json(sequence_dir, info)
            info['tf_data'] = []

    
    f_lidar_list.close()
    f_cam1_list.close()
    f_cam2_list.close()
    f_cam3_list.close()
    f_cam4_list.close()
    f_cam5_list.close()
    f_cam6_list.close()




if __name__ == '__main__':
    if(len(sys.argv)!=3):
        print("Please enter loading path and save path")
        exit(1)
    if (os.path.exists(sys.argv[1])==False):
        print("Directory is illegal, Input again!")
        exit(1)
    
    main(sys.argv[1], sys.argv[2])

