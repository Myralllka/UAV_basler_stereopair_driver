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
        m_sub_camera_fleft = nh.subscribe("/uav1/fleft/tag_detections",
                                          1,
                                          &BaslerStereoDriver::m_cbk_tag_detection_fleft,
                                          this);

        m_sub_camera_fright = nh.subscribe("/uav1/fright/tag_detections",
                                           1,
                                           &BaslerStereoDriver::m_cbk_tag_detection_fright,
                                           this);

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("basler_stereo_driver",
                                             m_uav_name);

        // | -------------------- initialize timers ------------------- |

        ROS_ASSERT("[basler stereo driver] timers initialisation");

        m_tim_tags_coordinates = nh.createTimer(ros::Duration(m_time_tagcoor),
                                                &BaslerStereoDriver::m_tim_cbk_tagcoor,
                                                this);

        m_tim_find_BL = nh.createTimer(ros::Duration(m_time_transformation),
                                       &BaslerStereoDriver::m_tim_cbk_find_BL,
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
    void BaslerStereoDriver::m_cbk_tag_detection_fright(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        std::lock_guard<std::mutex> lt{m_mut_pose_fright};
        auto res = m_tag_detection_cbk_body("right", msg);
        if (res.has_value()) {
            m_right_tag_poses = res.value();
            m_timestamp_fright = msg->header.stamp;
            ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: right camera tags detection cbk complete");
        } else {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: right camera cnk not completed");
        }
    }


    void BaslerStereoDriver::m_cbk_tag_detection_fleft(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        std::lock_guard<std::mutex> lt{m_mut_pose_fleft};
        auto res = m_tag_detection_cbk_body("left", msg);
        if (res.has_value()) {
            m_left_tag_poses = res.value();
            m_timestamp_fleft = msg->header.stamp;
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
            std::scoped_lock lck{m_mut_pose_fright, m_mut_pose_fleft};
            if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
            if (std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()) > ros::Duration(0.06).toSec()) {
                ROS_WARN_THROTTLE(1.0,
                                  "[basler stereo driver]: tags coordinates timestamps are too far away from each other: %f",
                                  std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()));
                return;
            }
        }
        const auto d = 3;
        const auto m = 12;
        using mat_t = Eigen::Matrix<double, d, m>;
        auto src = mat_t{};
        auto dst = mat_t{};
        Eigen::Vector3d row_eigen;
        {
            std::lock_guard lock{m_mut_pose_fright};
            for (int i = 0; i < m; ++i) {
                tf2::fromMsg(m_right_tag_poses[i], row_eigen);
                src.col(i) = row_eigen;
            }
        }
        {
            std::lock_guard lock{m_mut_pose_fleft};
            for (int i = 0; i < m; ++i) {
                tf2::fromMsg(m_left_tag_poses[i], row_eigen);
                dst.col(i) = row_eigen;
            }
        }
        auto transform = Eigen::Affine3d{umeyama(src, dst, false)};

        std::lock_guard lock{m_mut_RL_correction};
        m_RL_error = mrs_lib::TransformStamped("basler_right_optical/tag_1",
                                               "basler_left_optical/tag_1",
                                               ros::Time::now(),
                                               tf2::eigenToTransform(transform));
    }

    void BaslerStereoDriver::m_tim_cbk_find_BL([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback to publish camera position firstly as uncorrected estimation of a position (from CAD)
        // and then - T_RL_corrected using least-squares solution for two sets of points
        // T - transformation
        // B - base
        // L - left camera optical pose
        // R - right camera optical pose

        if (not m_is_initialized) {
            return;
        }

        {
            std::scoped_lock lck{m_mut_pose_fright, m_mut_pose_fleft};
            if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
            if (std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()) > ros::Duration(0.06).toSec()) {
                ROS_WARN_THROTTLE(1.0,
                                  "[basler stereo driver]: tags coordinates timestamps are too far away from each other: %f",
                                  std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()));
                return;
            }
        }
        ROS_INFO_THROTTLE(2.0, "[basler_driver] transform RL publisher");
        // TODO: check why transformer behave in this way (opposite to expected)
        auto T_BR = m_transformer.getTransform("uav1/basler_right_optical",
                                               "uav1/basler_stereopair/base");
        auto T_BL_original = m_transformer.getTransform("uav1/basler_left_optical",
                                                        "uav1/basler_stereopair/base");
        geometry_msgs::TransformStamped T_BL_result{};

        if (T_BR.has_value() and T_BL_original.has_value() and m_weight) {
            std::lock_guard lock{m_mut_RL_correction};
            const auto T_RL = T_BL_original->getTransformEigen() * T_BR->inverse().getTransformEigen();
            const auto T_RL_corrected = m_RL_error.getTransformEigen().inverse() * T_RL;
            [[maybe_unused]] auto T_BL_corrected = T_RL_corrected * T_BR->getTransformEigen();
            // make a new frame - pose estimation
            T_BL_result = tf2::eigenToTransform(m_interpolate_pose(m_filtered_pose, T_BL_corrected));
        } else {
            if (T_BL_original.has_value()) {
                m_filtered_pose = T_BL_original->getTransformEigen();
                ++m_weight;
            }
            ROS_ERROR_THROTTLE(2.0, "[basler stereo driver] wrong transformation from L/R to base");
        }
        T_BL_result.header.stamp = ev.current_real;
        T_BL_result.header.frame_id = "uav1/basler_stereopair/base";
        T_BL_result.child_frame_id = "uav1/basler_left_optical_corrected";

        auto tag1 = m_transformer.transformSingle("uav1/basler_left_optical_corrected",
                                                  m_transformer.getTransform("basler_left_optical/tag_0",
                                                                             "uav1/basler_left_optical").value().getTransform());
        if (tag1.has_value()) {
            auto transform = tag1.value();
            transform.header.stamp = ros::Time::now();
            transform.header.frame_id = "uav1/basler_left_optical_corrected";
            transform.child_frame_id = "uav1/tag0_modified";
            m_tbroadcaster.sendTransform(transform);
            ROS_INFO_THROTTLE(2.0, "[basler_driver] tag  sent from %s to %s time %u",
                              transform.header.frame_id.c_str(),
                              transform.child_frame_id.c_str(),
                              transform.header.stamp.nsec);
        }
        m_tbroadcaster.sendTransform(T_BL_result);
        ROS_INFO_THROTTLE(2.0, "[basler_driver] transform stamped sent from %s to %s time %u",
                          T_BL_result.header.frame_id.c_str(),
                          T_BL_result.child_frame_id.c_str(),
                          T_BL_result.header.stamp.nsec);
    }
// | -------------------- other functions ------------------- |

    Eigen::Affine3d BaslerStereoDriver::m_interpolate_pose(const Eigen::Affine3d &input_avg,
                                                           const Eigen::Affine3d &other) {
        // apply simple filter
        using quat_t = Eigen::Quaterniond;

        const int wght = 1;
        auto result_translation =
                (input_avg.translation() * (m_weight / wght) + other.translation()) / ((m_weight / wght) + 1);

        auto result_rotation = quat_t{input_avg.rotation()}.slerp(1.0 / (static_cast<double>(m_weight) / wght + 1),
                                                                  quat_t{other.rotation()});
        ++m_weight;
        std::cout << "here\n";
        auto res = Eigen::Affine3d::Identity();
        res.translate(result_translation).rotate(result_rotation);
//        std::cout << "\n**********************************************\n" << "angle:"
//                  << Eigen::AngleAxisd(res.rotation()).angle()
//                  << "\ntranslation:" << res.translation().transpose()
//                  << "\n**********************************************\n";
        return res;
    }

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
