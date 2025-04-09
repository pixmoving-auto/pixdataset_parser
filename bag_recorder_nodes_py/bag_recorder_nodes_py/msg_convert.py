# ref: https://zhuanlan.zhihu.com/p/632221478
# import rospy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
import numpy as np

from sensor_msgs.msg import PointField


# def np2msg(points, frame_id):
#     """
#     numpy 转 msg
#     Args:
#         points:(numpy)
#         frame_id:(str)

#     Returns:(PointCloud2)
#     # return pcl2.create_cloud_xyz32(header, data_process[::3])
#     # return pcl2.create_cloud(header, point_fields, data_process[::3])  # 每三个点取一个
#     """
#     point_fields = [PointField(name='x', offset=0,
#                                datatype=PointField.FLOAT32, count=1),
#                     PointField(name='y', offset=4,
#                                datatype=PointField.FLOAT32, count=1),
#                     PointField(name='z', offset=8,
#                                datatype=PointField.FLOAT32, count=1),
#                     PointField(name='intensity', offset=12,
#                                datatype=PointField.FLOAT32, count=1)]
#     header = Header(frame_id=frame_id, stamp=rospy.Time.now())
#     points_byte = points[:, 0:4].tobytes()
#     # points_byte = data_process.astype(np.float32).tobytes()
#     return PointCloud2(header=header,
#                        height=1,
#                        width=len(points),
#                        is_dense=False,
#                        is_bigendian=False,
#                        fields=point_fields,
#                        point_step=int(len(points_byte) / len(points)),
#                        row_step=len(points_byte),
#                        data=points_byte)


def msg2np(msg: PointCloud2, fileds=('x', 'y', 'z', 'intensity')):
    """
    激光雷达不同, msg 字节编码不同
    Args:
        msg:
        fileds_names:
    Returns: np.array, Nx3 或者 Nx4

    """

    def find_filed(filed):
        # 顺序查找
        for f in msg.fields:
            if f.name == filed:
                return f

    data_types_size = [None,
                       {'name': 'int8', 'size': 1},
                       {'name': 'uint8', 'size': 1},
                       {'name': 'int16', 'size': 2},
                       {'name': 'uint16', 'size': 2},
                       {'name': 'int32', 'size': 4},
                       {'name': 'uint32', 'size': 4},
                       {'name': 'float32', 'size': 4},
                       {'name': 'float64', 'size': 8}]

    dtypes_list = [None, np.int8, np.uint8, np.int16, np.uint16,
                   np.int32, np.uint32, np.float32, np.float64]  # PointCloud2 中有说明

    np_list = []
    for filed in fileds:
        f = find_filed(filed)

        dtype_size = data_types_size[f.datatype]['size']
        msg_total_type = msg.point_step

        item = np.frombuffer(msg.data, dtype=dtypes_list[f.datatype]).reshape(-1, int(
            msg_total_type / dtype_size))[:, int(f.offset / dtype_size)].astype(np.float32)
        np_list.append(item)

    points = np.array(np_list).T
    return points