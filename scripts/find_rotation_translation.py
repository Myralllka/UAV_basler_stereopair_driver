#!?bin/env python3
# sources: 
#  - https://docs.opencv.org/4.x/dc/dc3/tutorial_py_matcher.html

import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import random
import sys


def plot_epipolar_lines(c_u1, c_u2, c_F, img1, img2, header='The epipolar lines using F'):
    """
    Draw epipolar lines for pair of images. The number of lines = size of 'colors' array of this function
    @param c_u1, c_u2: 2d points in homogenous coordinate system, 3xn matrices
    @param c_F: Fundamental matrix
    @param img1, img2: images for c_u1, c_u2 respectively
    @param header: the header for the plot
    """
    colors = ["dimgray", "rosybrown", "maroon", "peru",
              "moccasin", "yellow", "olivedrab", "lightgreen",
              "navy", "royalblue", "indigo", "hotpink"]

    idxs = random.sample(range(c_u1.shape[1]), len(colors))
    c_u1 = c_u1[:, idxs]
    c_u2 = c_u2[:, idxs]
    fig = plt.figure()
    fig.clf()
    fig.suptitle(header)
    plt.subplot(121)
    i = 0
    for x_p1, y_p1, x_p2, y_p2 in zip(c_u1[0], c_u1[1], c_u2[0], c_u2[1]):
        plt.plot([int(x_p1)], [int(y_p1)], color=colors[i], marker="X",
                 markersize=10)
        point2_step2 = np.c_[x_p2, y_p2, 1].reshape(3, 1)

        x = np.linspace(1, img1.shape[1], img1.shape[1])
        ep1_step2 = c_F.T @ point2_step2
        y = -((ep1_step2[2] / ep1_step2[1]) + x * ep1_step2[0] / ep1_step2[1])
        plt.plot(x, y, color=colors[i])

        i += 1
    plt.imshow(img1)

    plt.subplot(122)
    i = 0
    for x_p1, y_p1, x_p2, y_p2 in zip(c_u1[0], c_u1[1], c_u2[0], c_u2[1]):
        plt.plot([int(x_p2)], [int(y_p2)],
                 color=colors[i],
                 marker="X",
                 markersize=10)
        point1_step2 = np.c_[x_p1, y_p1, 1].reshape(3, 1)

        x = np.linspace(1, img1.shape[1], img1.shape[1])
        point1_step2 = point1_step2.reshape(3, 1)
        ep2_step2 = c_F @ point1_step2
        y = -((ep2_step2[2] / ep2_step2[1]) + x * ep2_step2[0] / ep2_step2[1])
        plt.plot(x, y, color=colors[i])
        i += 1

    plt.imshow(img2)
    plt.show()


def drawlines(img1, img2, lines, pts1, pts2):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r, c = img1.shape
    img1 = cv.cvtColor(img1, cv.COLOR_GRAY2BGR)
    img2 = cv.cvtColor(img2, cv.COLOR_GRAY2BGR)
    for r, pt1, pt2 in zip(lines, pts1, pts2):
        color = tuple(np.random.randint(0, 255, 3).tolist())
        x0, y0 = map(int, [0, -r[2] / r[1]])
        x1, y1 = map(int, [c, -(r[2] + r[0] * c) / r[1]])
        img1 = cv.line(img1, (x0, y0), (x1, y1), color, 1)
        img1 = cv.circle(img1, tuple(pt1), 5, color, -1)
        img2 = cv.circle(img2, tuple(pt2), 5, color, -1)
    return img1, img2


def sqc(x):
    """
    Skew-symmetric matrix for cross-product
    Synopsis: S = sqc(x)
    :param x: vector 3×1
    :return: skew symmetric matrix (3×3) for cross product with x
    """
    return np.array([[0, x[2], -x[1]],
                     [-x[2], 0, x[0]],
                     [x[1], -x[0], 0]])


if __name__ == "__main__":
    FLANN_INDEX_KDTREE = 1

    img1_name = sys.argv[1]
    img2_name = sys.argv[2]

    img1 = cv.imread(img1_name, cv.IMREAD_GRAYSCALE)
    img2 = cv.imread(img2_name, cv.IMREAD_GRAYSCALE)

    # print(cv.__version__)

    sift = cv.SIFT_create()

    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)  # or pass empty dictionary

    flann = cv.FlannBasedMatcher(index_params, search_params)

    matches = flann.knnMatch(des1, des2, k=2)
    # Need to draw only good matches, so create a mask
    matchesMask = [[0, 0] for i in range(len(matches))]
    # ratio test as per Lowe's paper

    pts1 = []
    pts2 = []

    # ratio test as per Lowe's paper
    for i, (m, n) in enumerate(matches):
        if m.distance < 0.7 * n.distance:
            pts2.append(kp2[m.trainIdx].pt)
            pts1.append(kp1[m.queryIdx].pt)
    pts1 = np.int32(pts1)
    pts2 = np.int32(pts2)
    F, mask = cv.findFundamentalMat(pts1, pts2, cv.FM_LMEDS)
    # We select only inlier points
    pts1 = pts1[mask.ravel() == 1]
    pts2 = pts2[mask.ravel() == 1]
    R = np.array([[0, 0, 1],
                  [0, 1, 0],
                  [1, 0, 0]])
    t = np.array([83, 0, 83])

    print(F)
    #F = R @ sqc(-t)
    # Find epilines corresponding to points in right image (second image) and
    # drawing its lines on left image
    #lines1 = cv.computeCorrespondEpilines(pts2.reshape(-1, 1, 2), 2, F)
    #lines1 = lines1.reshape(-1, 3)
    #img5, img6 = drawlines(img1, img2, lines1, pts1, pts2)
    ## Find epilines corresponding to points in left image (first image) and
    ## drawing its lines on right image
    #lines2 = cv.computeCorrespondEpilines(pts1.reshape(-1, 1, 2), 1, F)
    #lines2 = lines2.reshape(-1, 3)
    #img3, img4 = drawlines(img2, img1, lines2, pts2, pts1)
    #plt.subplot(121), plt.imshow(img5)
    #plt.subplot(122), plt.imshow(img3)
    #plt.show()
