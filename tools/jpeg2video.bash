#!/bin/bash

root="/home/pix/ws/dataset/pix/university_city/"
sequence_idx="sequence00000/"
camera_idx='CAM_FRONT_map/'
dir=${root}${sequence_idx}${camera_idx}/*.jpeg
video_name='cam_front.mp4'


# # # 生成按数字顺序排序的文件列表
# ls -v ${root}${sequence_idx}${camera_idx}/*.jpeg > ${root}${sequence_idx}cam_front.txt
# # # 在每行前添加 "file '"

# echo ${root}${sequence_idx}cam_front.txt
# sed -i "s/^/file '/; s/$/'/" ${root}${sequence_idx}cam_front.txt

# ffmpeg -f concat -safe 0 -r 2 -i ${root}${sequence_idx}cam_front.txt \
#        -c:v libx264 -crf 23 -pix_fmt yuv420p ${root}${sequence_idx}cam_front.mp4





ffmpeg -framerate 2 -pattern_type glob -i ${root}${sequence_idx}${camera_idx}'*.jpeg' \
       -c:v libx264 -crf 23 -preset fast -pix_fmt yuv420p \
       ${root}${sequence_idx}${video_name}

