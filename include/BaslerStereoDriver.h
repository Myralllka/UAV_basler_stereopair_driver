#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

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
#include <mrs_lib/subscribe_handler.h>
#include <mrs_msgs/ImageLabeled.h>
#include <mrs_msgs/ImageLabeledArray.h>

/* other important includes */
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/builtin_bool.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>
#include <image_geometry/pinhole_camera_model.h>

/* opencv */
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/core/eigen.hpp>

/* user includes */

//}

/// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
// source: https://github.com/ethz-asl/ethzasl_msf/blob/5d916120c3e4df5b1ea136c2516c6ad1e3f9bf78/msf_core/include/msf_core/eigen_utils.h#L28
namespace basler_stereo_driver {

/* class BaslerStereoDriver //{ */
    class BaslerStereoDriver : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        void onInit() override;

    private:
        // used for ros log msgs
        const std::string NODENAME{"BaslerStereoDriver"};
        /* flags */
        bool m_is_initialized = false;
        bool m_is_calibrated = false;

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

        /* images for image pair */
        sensor_msgs::Image::ConstPtr imleft;
        sensor_msgs::Image::ConstPtr imright;

        mrs_msgs::ImageLabeledArray::Ptr m_impair;

        /* tag detection callback data */
        std::vector<geometry_msgs::Point> m_left_tag_poses;
        ros::Time m_timestamp_fleft{0};
        std::mutex m_mut_pose_fleft;

        std::vector<geometry_msgs::Point> m_right_tag_poses;
        ros::Time m_timestamp_fright{0};
        std::mutex m_mut_pose_fright;

        Eigen::Affine3d m_RL_error = Eigen::Affine3d::Identity();
        std::mutex m_mut_RL_correction;

        /* pose filter data */
        size_t m_weight{0};
        std::mutex m_mut_filtered_CL_pose;
        Eigen::Affine3d m_filtered_CL_pose;

        /* other parameters */
        std::string m_config_filename;

        /* fleft camera pose */
        Eigen::Affine3d m_fleft_pose = Eigen::Affine3d::Identity();
        Eigen::Affine3d m_fright_pose;
        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        void m_cbk_tag_detection_fright(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        void m_cbk_tag_detection_fleft(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        void m_cbk_complete_save_calibration(std_msgs::Bool flag);

        // | --------------------- timer callbacks -------------------- |
        // collect images and publish them together

        ros::Timer m_tim_collect_images;
        ros::Timer m_tim_find_BL;
        ros::Timer m_tim_tags_coordinates;
        ros::Timer m_tim_fleft_pose;
        ros::Timer m_tim_mse;

        void m_tim_cbk_collect_images(const ros::TimerEvent &ev);

        void m_tim_cbk_find_BL(const ros::TimerEvent &ev);

        void m_tim_cbk_tagcoor(const ros::TimerEvent &ev);

        void m_tim_cbk_fleft_pose(const ros::TimerEvent &ev);

        void m_tim_cbk_tags_errors(const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |
        ros::Publisher m_pub_multiview;

        // | ----------------------- subscribers ---------------------- |
        ros::Subscriber m_sub_camera_fleft;
        ros::Subscriber m_sub_camera_fright;
        ros::Subscriber m_sub_complete_calibration;

        // | ------------------ subscriber handlers ------------------- |
        mrs_lib::SubscribeHandler<sensor_msgs::Image> m_handler_imleft;
        mrs_lib::SubscribeHandler<sensor_msgs::Image> m_handler_imright;

        mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> m_handler_camleftinfo;
        mrs_lib::SubscribeHandler<sensor_msgs::CameraInfo> m_handler_camrightinfo;

        // | --------------------- other functions -------------------- |

        std::optional<std::vector<geometry_msgs::Point>>
        m_tag_detection_cbk_body(const std::string &camera_name,
                                 const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        Eigen::Affine3d m_interpolate_pose(const Eigen::Affine3d &input_avg,
                                           const Eigen::Affine3d &other);
    };
//}

}  // namespace basler_stereo_driver
