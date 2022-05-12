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
#include <apriltag_ros/PointLabeledArray.h>
#include <apriltag_ros/PointLabeled.h>
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
#include <opencv2/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
/* user includes */

//}

/// Returns the 3D cross product Skew symmetric matrix of a given 3D vector.
// source: https://github.com/ethz-asl/ethzasl_msf/blob/5d916120c3e4df5b1ea136c2516c6ad1e3f9bf78/msf_core/include/msf_core/eigen_utils.h#L28
namespace basler_stereo_driver {
    enum {
        // from PointLabeled message
        LEFTUP = 1,
        RIGHTUP = 2,
        RIGHTBOTTOM = 3,
        LEFTBOTTOM = 4
    };

    constexpr float APTAG_SIZE = 0.055;
    constexpr float PADD_SIZE = 0.0135;
    constexpr float APTAG_PADD_SIZE = APTAG_SIZE + PADD_SIZE;


    cv::Mat f2K33(const boost::array<double, 12> &P_in) {
        // transform a K matrix from CameraInfo (boost array) to CvMat
        Eigen::Matrix3d K_out = Eigen::Matrix3d::Identity();
        cv::Mat K_out_cv;
        K_out(0, 0) = P_in[0];
        K_out(0, 2) = P_in[2];
        K_out(1, 1) = P_in[5];
        K_out(1, 2) = P_in[6];
        cv::eigen2cv(K_out, K_out_cv);
        return K_out_cv;
    }

    std::vector<cv::Point3f> make_3d_apriltag_points(const std::vector<apriltag_ros::PointLabeled> &in_pts) {
        std::vector<cv::Point3f> res;
        res.reserve(in_pts.size());
        float x, y;
        for (const auto &in_pt : in_pts) {
            size_t j = in_pt.id;
            switch (in_pt.type) {
                case (LEFTUP):
                    x = APTAG_PADD_SIZE * static_cast<float>((j / 4) % 3);
                    y = APTAG_PADD_SIZE * static_cast<float>((j / 4) % 4);
                    break;
                case (RIGHTUP):
                    x = PADD_SIZE * static_cast<float>(j / 4) +
                        APTAG_SIZE * static_cast<float>((j / 4) % 3);
                    y = APTAG_PADD_SIZE * static_cast<float>((j / 4) % 4);
                    break;
                case (RIGHTBOTTOM):
                    x = PADD_SIZE * static_cast<float>(j / 4) +
                        APTAG_SIZE * static_cast<float>((j / 4 + 1) % 3);
                    y = PADD_SIZE * static_cast<float>(j / 4) +
                        APTAG_SIZE * static_cast<float>((j / 4 + 1) % 4);
                    break;
                case (LEFTBOTTOM):
                    x = APTAG_PADD_SIZE * static_cast<float>((j / 4) % 3);
                    y = PADD_SIZE * static_cast<float>(j / 4) +
                        APTAG_SIZE * static_cast<float>((j / 4) % 4);
                    break;
                default:
                    ROS_ERROR("corner point convertor: wrong point type");
                    break;
            }
            res.emplace_back(x, y, 0);
        }
        return res;
    }

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

        // if camera is not calibrated - what is the calibration algorithm
        std::string m_calib_algo;

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

        Eigen::Affine3d m_LR_error = Eigen::Affine3d::Identity();
        std::mutex m_mut_LR_correction;

        /* pose filter data */
        size_t m_weight{0};
        std::mutex m_mut_calibrated_CR_pose;
        Eigen::Affine3d m_calibrated_CR_pose;

        /* other parameters */
        std::string m_camera_poses_filename;

        /* fleft camera pose */
        Eigen::Affine3d m_fleft_pose;
        Eigen::Affine3d m_fright_pose = Eigen::Affine3d::Identity();
        cv::Mat m_K_CL, m_K_CR;
        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        void m_cbk_tag_detection_fright(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        void m_cbk_tag_detection_fleft(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        void m_cbk_complete_save_calibration(std_msgs::Bool flag);

        // | --------------------- timer callbacks -------------------- |
        // collect images and publish them together

        ros::Timer m_tim_collect_images;
        ros::Timer m_tim_tag_corners;
        ros::Timer m_tim_find_BR;
        ros::Timer m_tim_tags_coordinates;
        ros::Timer m_tim_fright_pose;
        ros::Timer m_tim_mse;

        void m_tim_cbk_corners(const ros::TimerEvent &ev);

        void m_tim_cbk_collect_images(const ros::TimerEvent &ev);

        void m_tim_cbk_find_BR(const ros::TimerEvent &ev);

        void m_tim_cbk_tagcoor(const ros::TimerEvent &ev);

        void m_tim_cbk_fright_pose(const ros::TimerEvent &ev);

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

        mrs_lib::SubscribeHandler<apriltag_ros::PointLabeledArray> m_handler_cornersfleft;
        mrs_lib::SubscribeHandler<apriltag_ros::PointLabeledArray> m_handler_cornersfright;

        // | --------------------- other functions -------------------- |

        std::optional<std::vector<geometry_msgs::Point>>
        m_tag_detection_cbk_body(const std::string &camera_name,
                                 const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        Eigen::Affine3d m_interpolate_pose(const Eigen::Affine3d &input_avg,
                                           const Eigen::Affine3d &other);
    };
//}

}  // namespace basler_stereo_driver
