#!/bin/env python3
# source: https://medium.com/@kennethjiang/calibrate-fisheye-lens-using-opencv-part-2-13990f1b157f
# source: https://answers.opencv.org/question/220574/how-to-undistort-fisheye-image-into-max-bounding-rectangle/

import numpy as np
import os
import glob
import sys
import cv2

dim = (1600, 1200)


def undistort(m_k, m_d, img_path, balance, dim2=None, dim3=None):
    
    img = cv2.imread(img_path)
    dim1 = img.shape[:2][::-1]
    if not dim2:
        dim2 = dim1
    if not dim3:
        dim3 = dim1
    
    scaled_k = m_k * dim1[0] / dim[0]  # the values of k is to scale with image dimension.
    print(m_k)
    print(scaled_k)

    new_k = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(scaled_k, m_d, dim2, np.eye(3), balance=balance)
    map1, map2 = cv2.fisheye.initUndistortRectifyMap(scaled_k, m_d, np.eye(3), new_k, dim3, cv2.CV_16SC2)
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
    
    cv2.imwrite(img_path.replace("jpeg", "undistort.jpeg"), undistorted_img)


if __name__ == '__main__':
    K = np.loadtxt(sys.argv[1])
    D = np.loadtxt(sys.argv[2])

    m = 6
    for p in sys.argv[3:]:
        undistort(K, D, p, 1, (1600, 1200), (1600*m, 1200*m))
        # undistort(K, D, p, 0, (1600, 1200))
        
