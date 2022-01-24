import numpy as np
import os
import glob
import sys
import cv2

# You should replace these 3 lines with the output in calibration step
DIM = (1600, 1200)


def undistort(m_K, m_D, img_path):
    img = cv2.imread(img_path)
    h, w = img.shape[:2]
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(m_K, m_D, np.eye(3), m_K, (h,w), cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    cv2.imwrite(img_path.replace("jpeg", "undistored.jpeg"), undistorted_img)


if __name__ == '__main__':
    K = np.loadtxt(sys.argv[1])
    D = np.loadtxt(sys.argv[2])
    for p in sys.argv[3:]:
        undistort(K, D, p)
