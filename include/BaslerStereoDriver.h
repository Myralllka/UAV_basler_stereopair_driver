#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/TransformStamped.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>
#include <mutex>
#include <cmath>
#include <optional>
#include <cstring>
#include <fstream>
#include <vector>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>

/* other important includes */
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/builtin_bool.h>

/* opencv */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/calib3d.hpp"

/* user includes */
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Geometry>

//}

namespace basler_stereo_driver {

/* class BaslerStereoDriver //{ */
    class BaslerStereoDriver : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        void onInit() override;

    private:
        /* flags */
        bool m_is_initialized = false;

        /* ros parameters */
        std::string m_uav_name;
        float m_time_transformation{0.001};
        float m_time_tagcoor{0.001};

        mrs_lib::TransformBroadcaster m_tbroadcaster;

        /* node names parameters */
        std::string m_name_fleft_tag_det;
        std::string m_name_fright_tag_det;
        std::string m_name_base;
        std::string m_name_CL;
        std::string m_name_CR;

        /* tag detection callback data */

        std::vector<geometry_msgs::Point> m_left_tag_poses;
        ros::Time m_timestamp_fleft{0};
        std::mutex m_mut_pose_fleft;

        std::vector<geometry_msgs::Point> m_right_tag_poses;
        ros::Time m_timestamp_fright{0};
        std::mutex m_mut_pose_fright;

        mrs_lib::TransformStamped m_RL_error;
        std::mutex m_mut_RL_correction;

        /* pose filter data */
        size_t m_weight{0};
        std::mutex m_mut_filtered_pose;
        Eigen::Affine3d m_filtered_pose;

        /* other parameters */
        std::string m_config_filename;

        /* fleft camera pose */
        Eigen::Affine3d m_fleft_pose = Eigen::Affine3d::Identity();
        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        void m_cbk_tag_detection_fright(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        void m_cbk_tag_detection_fleft(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        void m_cbk_complete_save_calibration(std_msgs::Bool flag);

        // | --------------------- timer callbacks -------------------- |
        ros::Timer m_tim_find_BL;
        ros::Timer m_tim_tags_coordinates;
        ros::Timer m_tim_fleft_pose;

        void m_tim_cbk_find_BL(const ros::TimerEvent &ev);

        void m_tim_cbk_tagcoor(const ros::TimerEvent &ev);

        void m_tim_cbk_fleft_pose(const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |
        // | ----------------------- subscribers ---------------------- |
        ros::Subscriber m_sub_camera_fleft;
        ros::Subscriber m_sub_camera_fright;

        ros::Subscriber m_sub_complete_calibration;

        // | --------------------- other functions -------------------- |
        std::optional<std::vector<geometry_msgs::Point>>
        m_tag_detection_cbk_body(const std::string &camera_name,
                                 const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        Eigen::Affine3d m_interpolate_pose(const Eigen::Affine3d &input_avg,
                                           const Eigen::Affine3d &other);
    };
//}

}  // namespace basler_stereo_driver
