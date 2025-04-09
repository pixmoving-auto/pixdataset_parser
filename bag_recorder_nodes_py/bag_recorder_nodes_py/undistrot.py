import cv2
from PIL import Image
import numpy as np
import pdb



cam_intrinsics = np.array([[1006.8330277047, 0.0, 970.8729607505],
                           [0.0, 1006.7672677655, 530.4746486724],
                           [0.0, 0.0, 1.0]], dtype=np.float32)

distortion = np.array([[-0.01897931],
                       [0.1906307],
                       [-0.61328069],
                       [0.55905323]], dtype=np.float32)

def undistort(k_matrix: np.array, d_matrix: np.array, frame):
    h, w = frame.shape[:2]
    # pdb.set_trace()
    mapx, mapy = cv2.initUndistortRectifyMap(k_matrix, d_matrix, None, k_matrix, (w, h), 5)
    return cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

def distortionCorrection(k_matrix: np.array, d_matrix: np.array, frame):
    undistort_frame = undistort(k_matrix, d_matrix, frame)
    return undistort_frame


def distort(
    img_before='/home/pix/ws/dataset/pix/ws/1742631352-325712872.jpeg',
    img_after='./img_after.jpg',
    K=cam_intrinsics,
    D=distortion,
    resolution=(1920, 1080)
):
    """使用 OpenCV 图像去畸变

    :param img_before: 要处理的图像
    :param img_after: 处理后的图像完整路径
    :param K: 相机内参，np.array(3x3)
    :param D: 相机镜头畸变系数，np.array(4x1)
    :param resolution: 图像分辨率

    """

    img = Image.open(img_before)
    img = img.resize(resolution)
    img_bgr = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)


    # img_distort = cv2.fisheye.undistortImage(
    #     img_bgr,
    #     K,
    #     D,
    #     None,
    #     K,
    #     resolution
    # )

    D = np.array([[0.8225141288],
                  [0.1053048901],
                  [-3.08832e-05],
                  [3.78157e-05],
                  [0.0002460804],
                  [1.2188400611],
                  [0.3300199278],
                  [0.0111750204],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0],
                  [0]], dtype=np.float32)
    img_distort = distortionCorrection(K, D, img_bgr)


    # undistorted = cv2.undistort(img_bgr, K, D)
    # cv2.namedWindow("Undistorted", cv2.WINDOW_NORMAL)
    # cv2.imshow("Undistorted", undistorted)
    # cv2.resizeWindow("Undistorted", 1920, 1080)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


    img_distort = cv2.cvtColor(img_distort, cv2.COLOR_BGR2RGB)
    img_distort = Image.fromarray(img_distort)
    img_distort = img_distort.resize((1920, 1080))
    img_distort.save(img_after)


def main():
    distort()


main()