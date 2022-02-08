#include <BaslerStereoDriver.h>

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

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

        // | ---------------- subscribers initialize ------------------ |

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("basler_stereo_driver", m_uav_name);

        // | -------------------- initialize timers ------------------- |

        ROS_ASSERT("here");
        m_tim_transformation = nh.createTimer(ros::Duration(m_time_transformation),
                                              &BaslerStereoDriver::m_tim_callb_transformation, this);
        ROS_INFO_ONCE("[BaslerStereoDriver]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |

// | --------------------- timer callbacks -------------------- |


    [[maybe_unused]] void BaslerStereoDriver::m_tim_callb_transformation([[maybe_unused]] const ros::TimerEvent &ev) {
        ROS_INFO("\n\nhere\n\n");
//        auto transformation = m_transformer.getTransform(
//                "basler_right_optical/tag_1",
//                "basler_left_optical/tag_1");
//        if (transformation.has_value()) {
//            std::cout << transformation->getTransform().transform << std::endl;
//        } else {
//            ROS_ASSERT("no transformation");
//        }

        geometry_msgs::TransformStamped to_left = geometry_msgs::TransformStamped();
        to_left.transform.rotation.x = -0.653 ;
        to_left.transform.rotation.y = 0.271 ;
        to_left.transform.rotation.z = -0.271 ;
        to_left.transform.rotation.w = 0.653;
        to_left.transform.translation.x = 0.05864;
        to_left.transform.translation.y = 0.05864;
        to_left.transform.translation.z = 0;
        to_left.header.stamp = ros::Time::now();
//        to_left.header.frame_id = m_uav_name + "/basler_stereopair/base";
        to_left.header.frame_id = "uav1/basler_stereopair/base";
//        to_left.child_frame_id = m_uav_name + "/basler_stereopair/camera1_fleft_pose";
        to_left.child_frame_id = "uav1/basler_left_optical";
        b.sendTransform(to_left);
        ROS_INFO("[basler_driver] transformstamped sent from %s to %s time %u", to_left.header.frame_id.c_str(), to_left.child_frame_id.c_str(), to_left.header.stamp.nsec);
    }
// | -------------------- other functions ------------------- |

}  // namespace basler_stereo_driver  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(basler_stereo_driver::BaslerStereoDriver, nodelet::Nodelet)
