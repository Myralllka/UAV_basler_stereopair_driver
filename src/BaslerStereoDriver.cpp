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
        m_sub_cfleft = nh.subscribe("/uav1/fleft/tag_detections",
                                    1,
                                    &BaslerStereoDriver::m_cbk_fleft_tag_detection,
                                    this);

        m_sub_cfright = nh.subscribe("/uav1/fright/tag_detections",
                                     1,
                                     &BaslerStereoDriver::m_cbk_fright_tag_detection,
                                     this);

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
    void BaslerStereoDriver::m_cbk_fright_tag_detection(const apriltag_ros::AprilTagDetectionArray msg) {
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: right camera tags detection cbk");
        if (msg.detections.empty()) {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: no tags visible by right camera");
            return;
        }
        auto right_detections = msg.detections;
        std::sort(right_detections.begin(), right_detections.end() - 1, [](auto a, auto b) { return a.id < b.id; });
        right_detections.pop_back();

        std::lock_guard<std::mutex> lt{m_mut_rtpose};
        std::for_each(right_detections.begin(),
                      right_detections.end(),
                      [&](const auto &el) { m_right_tag_poses.push_back(el.pose.pose.pose.position); });
        m_timestamp_frt = msg.header.stamp;
        std::for_each(m_right_tag_poses.begin(),
                      m_right_tag_poses.end(),
                      [](auto &el) { std::cout << el << std::endl; });
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: right camera tags detection cbk complete");
    }


    void BaslerStereoDriver::m_cbk_fleft_tag_detection(const apriltag_ros::AprilTagDetectionArray msg) {
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: left camera tags detection cbk");
        if (msg.detections.empty()) {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: no tags visible by left camera");
            return;
        }

        auto left_detections = msg.detections;
        std::sort(left_detections.begin(), left_detections.end() - 1, [](auto a, auto b) { return a.id < b.id; });
        left_detections.pop_back();
        std::lock_guard<std::mutex> lt{m_mut_ltpose};
        std::for_each(left_detections.begin(),
                      left_detections.end(),
                      [&](const auto &el) { m_left_tag_poses.push_back(el.pose.pose.pose.position); });
        m_timestamp_flt = msg.header.stamp;
        std::for_each(m_left_tag_poses.begin(),
                      m_left_tag_poses.end(),
                      [](auto &el) { std::cout << el << std::endl; });
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: left camera tags detection cbk complete");
    }


// | --------------------- timer callbacks -------------------- |


    [[maybe_unused]] void BaslerStereoDriver::m_tim_callb_transformation([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;

        if (!m_is_fixed) {
            auto transformation = m_transformer.getTransform(
                    "basler_right_optical/tag_1",
                    "basler_left_optical/tag_1");
            if (transformation.has_value()) {
                std::cout << transformation->getTransform().transform << std::endl;
            } else {
                ROS_WARN("no transformation");
            }
            m_is_fixed = true;
        }


        geometry_msgs::TransformStamped to_left = geometry_msgs::TransformStamped();
        to_left.transform.rotation.x = m_rotx;
        to_left.transform.rotation.y = m_roty;
        to_left.transform.rotation.z = m_rotz;
        to_left.transform.rotation.w = m_rotw;
        to_left.transform.translation.x = m_tranx;
        to_left.transform.translation.y = m_trany;
        to_left.transform.translation.z = m_tranz;
        to_left.header.stamp = ros::Time::now();
        to_left.header.frame_id = "uav1/basler_stereopair/base";
        to_left.child_frame_id = "uav1/basler_left_optical";
        auto to_left_mrs = mrs_lib::TransformStamped(to_left.header.frame_id,
                                                     to_left.child_frame_id,
                                                     to_left.header.stamp,
                                                     to_left);
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
