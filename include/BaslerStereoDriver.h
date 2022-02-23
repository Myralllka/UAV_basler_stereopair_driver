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

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>
#include <mrs_lib/transform_broadcaster.h>

/* other important includes */
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <sensor_msgs/image_encodings.h>

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
        float m_time_transformation{1};
        float m_time_tagcoor{1};

        mrs_lib::TransformBroadcaster m_tbroadcaster;

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
//        geometry_msgs::

        /* other parameters */

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        void m_cbk_tag_detection_fright(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        void m_cbk_tag_detection_fleft(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);

        // | --------------------- timer callbacks -------------------- |
        ros::Timer m_tim_find_BL;
        ros::Timer m_tim_tags_coordinates;

        void m_tim_cbk_find_BL(const ros::TimerEvent &ev);
        void m_tim_cbk_tagcoor(const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |

        // | ----------------------- subscribers ---------------------- |
        ros::Subscriber m_sub_camera_fleft;
        ros::Subscriber m_sub_camera_fright;

        // | --------------------- other functions -------------------- |
        std::optional<std::vector<geometry_msgs::Point>>
        m_tag_detection_cbk_body(const std::string &camera_name,
                                 const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg);
    };
//}

}  // namespace basler_stereo_driver
