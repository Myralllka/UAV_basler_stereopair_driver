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

//}

namespace basler_stereo_driver {

/* class BaslerStereoDriver //{ */
    class BaslerStereoDriver : public nodelet::Nodelet {

    public:
        /* onInit() is called when nodelet is launched (similar to main() in regular node) */
        virtual void onInit();

    private:
        /* flags */
        bool m_is_initialized = false;
        bool m_is_fixed = false;

        /* ros parameters */
        std::string m_uav_name;
        std::string m_fleft_topic_name;
        std::string m_fright_topic_name;
        float m_time_transformation{1};

        mrs_lib::TransformBroadcaster m_tbroadcaster;
        /* other parameters */

        /* estimated camera2 pose */
        float m_rotx = -0.653;
        float m_roty = 0.271;
        float m_rotz = -0.271;
        float m_rotw = 0.653;
        float m_tranx = 0.07;
        float m_trany = 0.07;
        float m_tranz = 0;

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |

        // | --------------------- timer callbacks -------------------- |
        ros::Timer m_tim_transformation;

        [[maybe_unused]] void m_tim_callb_transformation([[maybe_unused]] const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |
        // | ----------------------- subscribers ---------------------- |
        // | --------------------- other functions -------------------- |
        void m_cbk_tag_detection(const apriltag_ros::AprilTagDetectionArray msg);
    };
//}

}  // namespace basler_stereo_driver
