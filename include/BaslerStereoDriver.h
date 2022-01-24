#pragma once
/* includes //{ */

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some STL includes */
#include <cstdlib>
#include <cstdio>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

/* other important includes */
#include <nav_msgs/Odometry.h>

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

        /* ros parameters */
        std::string m_uav_name;

        /* other parameters */

        // | --------------------- MRS transformer -------------------- |

        mrs_lib::Transformer m_transformer;

        // | ---------------------- msg callbacks --------------------- |
        [[maybe_unused]] void m_callb_example([[maybe_unused]] const nav_msgs::Odometry::ConstPtr &msg);

        // | --------------------- timer callbacks -------------------- |
        ros::Timer m_tim_example;

        [[maybe_unused]] void m_tim_callb_example([[maybe_unused]] const ros::TimerEvent &ev);

        // | ----------------------- publishers ----------------------- |
        ros::Publisher m_pub_example;

        // | ----------------------- subscribers ---------------------- |
        ros::Subscriber m_sub_example;

        // | --------------------- other functions -------------------- |

    };
//}

}  // namespace basler_stereo_driver
