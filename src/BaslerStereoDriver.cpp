#include "BaslerStereoDriver.h"

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
        pl.loadParam("camera_poses_filename", m_config_filename);
        pl.loadParam("base_frame_pose", m_name_base);
        pl.loadParam("m_name_CL", m_name_CL);
        pl.loadParam("is_calibrated", m_is_calibrated);
        pl.loadParam("m_name_CR", m_name_CR);
        pl.loadParam("fleft_tag_det", m_name_fleft_tag_det);
        pl.loadParam("fright_tag_det", m_name_fright_tag_det);

        auto fleft_rotation = pl.loadMatrixStatic2<3, 3>("fleft_camera/rotation");
        auto fleft_translation = pl.loadMatrixStatic2<3, 1>("fleft_camera/translation");

        m_fleft_pose.translate(fleft_translation).rotate(Eigen::Quaterniond{fleft_rotation});

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
        m_transformer = mrs_lib::Transformer("basler_stereo_driver",
                                             m_uav_name);

        // | -------------------- initialize timers ------------------- |

        ROS_ASSERT("[BaslerStereoDriver] timers initialisation");

        // If pair is calibrated - publish the pose as already calibrated parameter
        if (m_is_calibrated) {
            m_tim_fleft_pose = nh.createTimer(ros::Duration(0.0001),
                                              &BaslerStereoDriver::m_tim_cbk_fleft_pose,
                                              this);
        } else {
            m_tim_tags_coordinates = nh.createTimer(ros::Duration(m_time_tagcoor),
                                                    &BaslerStereoDriver::m_tim_cbk_tagcoor,
                                                    this);
            m_tim_find_BL = nh.createTimer(ros::Duration(m_time_transformation),
                                           &BaslerStereoDriver::m_tim_cbk_find_BL,
                                           this);
            // | ---------------- subscribers initialize ------------------ |

            m_sub_camera_fleft = nh.subscribe(m_name_fleft_tag_det,
                                              1,
                                              &BaslerStereoDriver::m_cbk_tag_detection_fleft,
                                              this);

            m_sub_camera_fright = nh.subscribe(m_name_fright_tag_det,
                                               1,
                                               &BaslerStereoDriver::m_cbk_tag_detection_fright,
                                               this);

            m_sub_complete_calibration = nh.subscribe(m_uav_name + "/complete",
                                                      1,
                                                      &BaslerStereoDriver::m_cbk_complete_save_calibration,
                                                      this);
        }
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
            ROS_INFO_THROTTLE(2.0, "[BaslerStereoDriver]: right camera tags detection cbk complete");
        } else {
            ROS_WARN_THROTTLE(2.0, "[BaslerStereoDriver]: right camera cnk not completed");
        }
    }

    void BaslerStereoDriver::m_cbk_tag_detection_fleft(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        std::lock_guard<std::mutex> lt{m_mut_pose_fleft};
        auto res = m_tag_detection_cbk_body("left", msg);
        if (res.has_value()) {
            m_left_tag_poses = res.value();
            m_timestamp_fleft = msg->header.stamp;
            ROS_INFO_THROTTLE(2.0, "[BaslerStereoDriver]: left camera tags detection cbk complete");
        } else {
            ROS_WARN_THROTTLE(2.0, "[BasleStereoDriver]: left camera cnk not completed");
        }
    }

    void BaslerStereoDriver::m_cbk_complete_save_calibration(std_msgs::Bool flag) {
        // if calibration is completed and received flag is True - save all data and close the nodelet
        if (flag.data) {
            std::lock_guard l{m_mut_filtered_pose};
            std::ofstream fout(m_config_filename);
            Eigen::IOFormat fmt{3, 0, ", ", ",\n", "", "", "[", "]"};
            fout << "fleft_camera:\n";
            fout << "\trotation: "
                 << m_filtered_pose.rotation().format(fmt)
                 << std::endl;
            fout << "\ttranslation: " << m_filtered_pose.translation().format(fmt) << std::endl;
            ros::shutdown();
        }
    }

// | --------------------- timer callbacks -------------------- |

    void BaslerStereoDriver::m_tim_cbk_fleft_pose([[maybe_unused]] const ros::TimerEvent &ev) {
        // publish left camera pose (when camera pair is already calibrated)
        if (not m_is_initialized) return;
        geometry_msgs::TransformStamped fleft_pose_stamped = tf2::eigenToTransform(m_fleft_pose);
        fleft_pose_stamped.header.frame_id = m_name_base;
        fleft_pose_stamped.child_frame_id = m_name_CL;
        fleft_pose_stamped.header.stamp = ros::Time::now();
        m_tbroadcaster.sendTransform(fleft_pose_stamped);
    }

    void BaslerStereoDriver::m_tim_cbk_tagcoor([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback will find a 3d position of tags in space from two cameras and
        // save them as a mrs_lib::TransformStamped
        if (not m_is_initialized) return;
        {
            std::scoped_lock lck{m_mut_pose_fright, m_mut_pose_fleft};
            if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
            if (std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()) > ros::Duration(0.2).toSec()) {
                ROS_WARN_THROTTLE(1.0,
                                  "[BaslerStereoDriver]: tags coordinates timestamps are too far away from each other: %f",
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
        m_RL_error = transform;
    }

    void BaslerStereoDriver::m_tim_cbk_find_BL([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback to publish camera position firstly as uncorrected estimation of a position (from CAD)
        // and then - T_RL_corrected using least-squares solution for two sets of points
        // T - transformation
        // B - base
        // L - left camera optical pose
        // R - right camera optical pose

        if (not m_is_initialized) return;

        {
            std::scoped_lock lck{m_mut_pose_fright, m_mut_pose_fleft};
            if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
            if (std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()) > ros::Duration(0.2).toSec()) {
                ROS_WARN_THROTTLE(1.0,
                                  "[BaslerStereoDriver]: tags coordinates timestamps are too far away from each other: %f",
                                  std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()));
                return;
            }
        }
        ROS_INFO_THROTTLE(2.0, "[BaslerStereoDriver] transform RL publisher");

        auto T_BR = m_transformer.getTransform(m_name_CR,
                                               m_name_base);
        auto T_BL_original = m_transformer.getTransform(m_name_CL,
                                                        m_name_base);
        geometry_msgs::TransformStamped T_BL_result;

        if (T_BR.has_value() and T_BL_original.has_value()) {
            std::lock_guard lock{m_mut_RL_correction};
            const auto T_RL = T_BL_original->getTransformEigen() * T_BR->inverse().getTransformEigen();
            const auto T_RL_corrected = m_RL_error.inverse() * T_RL;
            auto T_BL_corrected = T_RL_corrected * T_BR->getTransformEigen();
            // make a new frame - pose estimation
            std::lock_guard l{m_mut_filtered_pose};
            if (not m_weight) {
                m_filtered_pose = T_BL_corrected;
            }
            ++m_weight;
            T_BL_result = tf2::eigenToTransform(m_interpolate_pose(m_filtered_pose, T_BL_corrected));

        } else {
            ROS_ERROR_THROTTLE(2.0, "[BaslerStereoDriver] wrong transformation from L/R to base");
        }
        T_BL_result.header.stamp = ev.current_real;
        T_BL_result.header.frame_id = m_name_base;
        T_BL_result.child_frame_id = m_name_CL + "corrected";

        for (int i = 0; i < 12; ++i) {
            // find a translation for all tags to compare the result
            auto L2tag = m_transformer.getTransform(m_name_CL,
                                                    "basler_left_optical/tag_" + std::to_string(i));

            if (L2tag.has_value()) {
                auto transform = L2tag.value().inverse().getTransform();
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = m_name_CL + "corrected";
                transform.child_frame_id = m_uav_name + "/tag_modified" + std::to_string(i);
                m_tbroadcaster.sendTransform(transform);
            }
        }

        m_tbroadcaster.sendTransform(T_BL_result);
        ROS_INFO_THROTTLE(2.0, "[BaslerStereoDriver] transform stamped sent from %s to %s time %u",
                          T_BL_result.header.frame_id.c_str(),
                          T_BL_result.child_frame_id.c_str(),
                          T_BL_result.header.stamp.nsec);
    }
// | -------------------- other functions ------------------- |

    Eigen::Affine3d BaslerStereoDriver::m_interpolate_pose(const Eigen::Affine3d &input_avg,
                                                           const Eigen::Affine3d &other) {
        // apply simple filter
        using quat_t = Eigen::Quaterniond;

        auto result_translation = (input_avg.translation() * m_weight + other.translation()) / (m_weight + 1);
        auto result_rotation = quat_t{input_avg.rotation()}.slerp(1.0 / (static_cast<double>(m_weight) + 1),
                                                                  quat_t{other.rotation()});
        auto res = Eigen::Affine3d::Identity();
        res.translate(result_translation).rotate(result_rotation);
        return res;
    }

    std::optional<std::vector<geometry_msgs::Point>>
    BaslerStereoDriver::m_tag_detection_cbk_body(const std::string &camera_name,
                                                 const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

        if (not m_is_initialized) return std::nullopt;
        if (msg->detections.empty()) {
            ROS_WARN_THROTTLE(2.0, "[BaslerStereoDriver]: no tags visible by %s camera", camera_name.c_str());
            return std::nullopt;
        }
        auto msg_sorted = msg.get()->detections;
        std::sort(msg_sorted.begin(), msg_sorted.end() - 1, [](auto a, auto b) { return a.id < b.id; });
        msg_sorted.pop_back();
        if (msg_sorted.size() != 12) {
            ROS_WARN_THROTTLE(1.0, "[BaslerStereoDriver]: %s camera: %zu out of 12 tags detected",
                              camera_name.c_str(),
                              msg_sorted.size());
            return std::nullopt;
        }
        std::vector<geometry_msgs::PoseWithCovarianceStamped> poses_base_frame;
        for (const apriltag_ros::AprilTagDetection &el: msg_sorted) {
            auto pose_cam_frame = m_transformer.transformSingle(m_name_base, el.pose);
            if (pose_cam_frame.has_value()) {
                poses_base_frame.push_back(pose_cam_frame.value());
            } else {
                ROS_WARN_THROTTLE(2.0, "[BaslerStereoDriver] error transforming tag from %s camera to base frame",
                                  camera_name.c_str());
            }
        }
        std::vector<geometry_msgs::Point> result;
        for (const auto &el: poses_base_frame) {
            result.push_back(el.pose.pose.position);
        }
        ROS_INFO_THROTTLE(2.0, "[BaslerStereoDriver]: %s camera tags detection cbk complete", camera_name.c_str());

        return result;
    }

}  // namespace basler_stereo_driver

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(basler_stereo_driver::BaslerStereoDriver, nodelet::Nodelet)
