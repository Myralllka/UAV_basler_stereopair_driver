//
// Created by mrs on 13/05/2022.
//

#ifndef BASLER_STEREOPAIR_DRIVER_UTILS_H
#define BASLER_STEREOPAIR_DRIVER_UTILS_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <apriltag_ros/PointLabeledArray.h>
#include <apriltag_ros/PointLabeled.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <vector>
#include <iostream>

namespace basler_stereo_driver {

    void cam2Rt(const std::vector<cv::Point3d> &td_pts,
                const std::vector<cv::Point2d> &im_pts,
                const cv::Matx<double, 3, 3> &K,
                const cv::Mat &d,
                Eigen::Matrix3d &R,
                Eigen::Matrix<double, 3, 1> &t);

    cv::Mat f2K33(const boost::array<double, 12> &P_in);

    std::vector<cv::Point3d> make_3d_apriltag_points(const std::vector<apriltag_ros::PointLabeled> &in_pts);

    bool u2RT(std::vector<apriltag_ros::PointLabeled> detections,
              const cv::Matx<double, 3, 3> &K,
              Eigen::Matrix3d &R,
              Eigen::Matrix<double, 3, 1> &t);
}

#endif //BASLER_STEREOPAIR_DRIVER_UTILS_H
