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
//        m_tim_transformation = nh.createTimer(ros::Duration(m_time_transformation),
//                                              &BaslerStereoDriver::m_tim_callb_transformation, this);
        ROS_INFO_ONCE("[BaslerStereoDriver]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |

// | --------------------- timer callbacks -------------------- |


    [[maybe_unused]] void BaslerStereoDriver::m_tim_callb_transformation([[maybe_unused]] const ros::TimerEvent &ev) {
        double eps_x = 0, eps_y = 0, eps_z = 0;
        if (!m_is_fixed) {
            auto transformation = m_transformer.getTransform(
                    "basler_right_optical/tag_1",
                    "basler_left_optical/tag_1");
            if (transformation.has_value()) {
                std::cout << transformation->getTransform().transform << std::endl;
                eps_x = transformation->getTransform().transform.translation.x;
                eps_y = transformation->getTransform().transform.translation.y;
                eps_z = transformation->getTransform().transform.translation.z;
            } else {
                ROS_WARN("no transformation");
//            return;
            }
            m_is_fixed = true;
        }

        geometry_msgs::TransformStamped to_left = geometry_msgs::TransformStamped();
        to_left.transform.rotation.x = m_rotx;
        to_left.transform.rotation.y = m_roty;
        to_left.transform.rotation.z = m_rotz;
        to_left.transform.rotation.w = m_rotw;
        to_left.transform.translation.x = m_tranx - eps_x;
        to_left.transform.translation.y = m_trany - eps_y;
        to_left.transform.translation.z = m_tranz - eps_z;
        to_left.header.stamp = ros::Time::now();
//        to_left.header.frame_id = m_uav_name + "/basler_stereopair/base";
        to_left.header.frame_id = "uav1/basler_stereopair/base";
//        to_left.child_frame_id = m_uav_name + "/basler_stereopair/camera1_fleft_pose";
        to_left.child_frame_id = "uav1/basler_left_optical";
        auto to_left_mrs = mrs_lib::TransformStamped(to_left.header.frame_id,
                                                     to_left.child_frame_id,
                                                     to_left.header.stamp,
                                                     to_left);
//        if (transformation.has_value()) {
//            auto fixed_transform = m_transformer.transform(to_left_mrs, transformation->getTransform());
//            if (fixed_transform.has_value()) {
//                m_tbroadcaster.sendTransform(fixed_transform.value());
//                ROS_INFO("[basler_driver] no error, publishing transformation");
//            } else {
//                ROS_INFO("[basler_driver] error during transformation");
//                return;
//            }
//        } else {
//        m_tbroadcaster.sendTransform(to_left_mrs.getTransform());
        m_tbroadcaster.sendTransform(to_left);
        ROS_WARN("[basler_driver] error, publishing to_left");
//        }
        ROS_INFO("[basler_driver] transform stamped sent from %s to %s time %u", to_left.header.frame_id.c_str(),
                 to_left.child_frame_id.c_str(), to_left.header.stamp.nsec);
    }
// | -------------------- other functions ------------------- |

}  // namespace basler_stereo_driver  

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(basler_stereo_driver::BaslerStereoDriver, nodelet::Nodelet)
