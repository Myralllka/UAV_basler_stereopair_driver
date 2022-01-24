#include <BaslerStereoDriver.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
#include "example.h"

namespace basler_stereo_driver {

/* onInit() method //{ */
    void BaslerStereoDriver::onInit() {

        // | ---------------- set my booleans to false ---------------- |
        // but remember, always set them to their default value in the header file
        // because, when you add new one later, you might forget to come back here

        /* obtain node handle */
        ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

        /* waits for the ROS to publish clock */
        ros::Time::waitForValid();

        // | ------------------- load ros parameters ------------------ |
        /* (mrs_lib implementation checks whether the parameter was loaded or not) */
        mrs_lib::ParamLoader pl(nh, "BaslerStereoDriver");

        pl.loadParam("UAV_NAME", m_uav_name);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[BaslerStereoDriver]: failed to load non-optional parameters!");
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[BaslerStereoDriver]: loaded parameters");
        }
        // | ---------------- some data post-processing --------------- |

        // | ----------------- publishers initialize ------------------ |
        m_pub_example = nh.advertise<nav_msgs::Odometry>("odometry_echo", 1); // last param for queue size

        // | ---------------- subscribers initialize ------------------ |
        m_sub_example = nh.subscribe("/odometry/odom_main", 1, &BaslerStereoDriver::m_callb_example, this); // second parameter for queue size

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("BaslerStereoDriver", m_uav_name);

        // | -------------------- initialize timers ------------------- |
        m_tim_marker = nh.createTimer(ros::Duration(0.1), &BaslerStereoDriver::m_tim_callb_example, this);
        ROS_INFO_ONCE("[BaslerStereoDriver]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |
    [[maybe_unused]] void BaslerStereoDriver::m_callb_example([[maybe_unused]] const nav_msgs::Odometry::ConstPtr &msg) {
        if (not m_is_initialized) return;
    }

// | --------------------- timer callbacks -------------------- |
    [[maybe_unused]] void BaslerStereoDriver::m_tim_callb_example([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;
    }
// | -------------------- other functions ------------------- |

}  // namespace basler_stereo_driver  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(basler_stereo_driver::BaslerStereoDriver, nodelet::Nodelet)
