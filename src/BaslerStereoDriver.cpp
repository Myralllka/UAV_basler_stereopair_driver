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


//        m_T_BL_static = geometry_msgs::TransformStamped();
//        m_T_BL_static.transform.rotation.x = m_rotx;
//        m_T_BL_static.transform.rotation.y = m_roty;
//        m_T_BL_static.transform.rotation.z = m_rotz;
//        m_T_BL_static.transform.rotation.w = m_rotw;
//        m_T_BL_static.transform.translation.x = m_tranx;
//        m_T_BL_static.transform.translation.y = m_trany;
//        m_T_BL_static.transform.translation.z = m_tranz;
//        m_T_BL_static.header.stamp = ros::Time::now();
//        m_T_BL_static.header.frame_id = "uav1/basler_stereopair/base";
//        m_T_BL_static.child_frame_id = "uav1/basler_left_optical";

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
        m_transformer = mrs_lib::Transformer("basler_stereo_driver",
                                             m_uav_name);

        // | -------------------- initialize timers ------------------- |

        ROS_ASSERT("[basler stereo driver] timers initialisation");

        m_tim_tags_coordinates = nh.createTimer(ros::Duration(m_time_tagcoor),
                                                &BaslerStereoDriver::m_tim_cbk_tagcoor,
                                                this);

        m_tim_transformation = nh.createTimer(ros::Duration(m_time_transformation),
                                              &BaslerStereoDriver::m_tim_cbk_transformation,
                                              this);

        // initialize the corrected transformation
        Eigen::Affine3d initial_aff_transform = Eigen::Affine3d::Identity();

        geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(initial_aff_transform);

        m_RL_error = mrs_lib::TransformStamped("basler_right_optical/tag_1",
                                               "basler_left_optical/tag_1",
                                               ros::Time::now(),
                                               transform_msg);

        ROS_INFO_ONCE("[BaslerStereoDriver]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |
    void BaslerStereoDriver::m_cbk_fright_tag_detection(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        std::lock_guard<std::mutex> lt{m_mut_rtpose};
        auto res = m_tag_detection_cbk_body("right", msg);
        if (res.has_value()) {
            m_right_tag_poses = res.value();
            m_timestamp_frt = msg->header.stamp;
            ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: right camera tags detection cbk complete");
        } else {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: right camera cnk not completed");
        }
    }


    void BaslerStereoDriver::m_cbk_fleft_tag_detection(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        std::lock_guard<std::mutex> lt{m_mut_ltpose};
        auto res = m_tag_detection_cbk_body("left", msg);
        if (res.has_value()) {
            m_left_tag_poses = res.value();
            m_timestamp_flt = msg->header.stamp;
            ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: left camera tags detection cbk complete");
        } else {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: left camera cnk not completed");
        }
    }


// | --------------------- timer callbacks -------------------- |
    void BaslerStereoDriver::m_tim_cbk_tagcoor([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback will find a 3d position of tags in space from two cameras and
        // save them as a mrs_lib::TransformStamped
        if (not m_is_initialized) return;
        {
            std::scoped_lock lck{m_mut_rtpose, m_mut_ltpose};
            if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
//            if (std::abs(m_timestamp_frt.toSec() - m_timestamp_flt.toSec()) > ros::Duration(0.01).toSec()) {
//                ROS_WARN_THROTTLE(1.0,
//                                  "[basler stereo driver]: tags coordinates timestamps are too far away from each other: %f",
//                                  std::abs(m_timestamp_frt.toSec() - m_timestamp_flt.toSec()));
//                return;
//            }
        }
        const auto d = 3;
        const auto m = 12;
        using mat_t = Eigen::Matrix<double, d, m>;
        auto src = mat_t{};
        auto dst = mat_t{};
        Eigen::Vector3d row_eigen;
        {
            std::lock_guard lock{m_mut_rtpose};
            for (int i = 0; i < m; ++i) {
                tf2::fromMsg(m_right_tag_poses[i], row_eigen);
                src.col(i) = row_eigen;
            }
        }
        {
            std::lock_guard lock{m_mut_ltpose};
            for (int i = 0; i < m; ++i) {
                tf2::fromMsg(m_left_tag_poses[i], row_eigen);
                dst.col(i) = row_eigen;
            }
        }
        auto transform = Eigen::Affine3d{umeyama(src, dst, false)};

        std::lock_guard lock{m_mut_RL_correction};
//        std::cout << "\n**********************************************\n" << "angle:"
//                  << Eigen::AngleAxisd(transform_aff.rotation()).angle()
//                  << "\ntranslation:" << transform_aff.translation().transpose()
//                  << "\n**************************************\n";
        m_RL_error = mrs_lib::TransformStamped("basler_right_optical/tag_1",
                                               "basler_left_optical/tag_1",
                                               ros::Time::now(),
                                               tf2::eigenToTransform(transform));
    }

    void BaslerStereoDriver::m_tim_cbk_transformation([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback to publish camera position firstly as uncorrected estimation of a position (from CAD)
        // and then - T_RL_corrected using least-squares solution for two sets of points
        // T - transformation
        // B - base
        // L - left camera optical pose
        // R - right camera optical pose

        if (not m_is_initialized) return;

        ROS_INFO_THROTTLE(2.0, "[basler_driver] transform RL publisher");

        auto T_BR = m_transformer.getTransform("uav1/basler_stereopair/base",
                                               "uav1/basler_right_optical");
        auto T_BL_original = m_transformer.getTransform("uav1/basler_stereopair/base",
                                                        "uav1/basler_left_optical");
        geometry_msgs::TransformStamped T_BL_computed{};

        if (T_BR.has_value() and T_BL_original.has_value()) {
            std::lock_guard lock{m_mut_RL_correction};
            const auto T_RL = T_BL_original->getTransformEigen() * T_BR->inverse().getTransformEigen();
            const auto T_RL_corrected = m_RL_error.getTransformEigen().inverse() * T_RL;
            const auto T_BL_corrected = T_RL_corrected * T_BR->getTransformEigen();
            // make a new frame - pose estimation
            T_BL_computed = tf2::eigenToTransform(T_BL_corrected);
            T_BL_computed.header.frame_id = "uav1/basler_stereopair/base";
            T_BL_computed.child_frame_id = "uav1/basler_left_optical_corrected";
        } else {
            ROS_ERROR_THROTTLE(2.0, "[basler stereo driver] wrong transformation from L/R to base");
        }
        T_BL_computed.header.stamp = ev.current_real;
        m_tbroadcaster.sendTransform(T_BL_computed);
        ROS_INFO_THROTTLE(2.0, "[basler_driver] transform stamped sent from %s to %s time %u",
                          T_BL_computed.header.frame_id.c_str(),
                          T_BL_computed.child_frame_id.c_str(),
                          T_BL_computed.header.stamp.nsec);
    }
// | -------------------- other functions ------------------- |

    std::optional<std::vector<geometry_msgs::Point>>
    BaslerStereoDriver::m_tag_detection_cbk_body(const std::string &camera_name,
                                                 const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

        if (not m_is_initialized) return std::nullopt;
        if (msg->detections.empty()) {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: no tags visible by %s camera", camera_name.c_str());
            return std::nullopt;
        }
        auto msg_sorted = msg.get()->detections;
        std::sort(msg_sorted.begin(), msg_sorted.end() - 1, [](auto a, auto b) { return a.id < b.id; });
        msg_sorted.pop_back();
        if (msg_sorted.size() != 12) {
            ROS_WARN_THROTTLE(1.0, "[basler stereo pair]: %s camera: %zu out of 12 tags detected",
                              camera_name.c_str(),
                              msg_sorted.size());
            return std::nullopt;
        }
        std::vector<geometry_msgs::PoseWithCovarianceStamped> poses_base_frame;
        for (const apriltag_ros::AprilTagDetection &el: msg_sorted) {
            auto pose_cam_frame = m_transformer.transformSingle("uav1/basler_stereopair/base", el.pose);
            if (pose_cam_frame.has_value()) {
                poses_base_frame.push_back(pose_cam_frame.value());
            } else {
                ROS_WARN_THROTTLE(2.0, "error transforming tag from %s camera to base frame", camera_name.c_str());
            }
        }
        std::vector<geometry_msgs::Point> result;
        for (const auto &el: poses_base_frame) {
            result.push_back(el.pose.pose.position);
        }
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: %s camera tags detection cbk complete", camera_name.c_str());

        return result;
    }

}  // namespace basler_stereo_driver

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(basler_stereo_driver::BaslerStereoDriver, nodelet::Nodelet)
