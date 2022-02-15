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

        m_transform_msg = mrs_lib::TransformStamped("basler_right_optical/tag_1",
                                                    "basler_left_optical/tag_1",
                                                    ros::Time::now(),
                                                    transform_msg);

        ROS_INFO_ONCE("[BaslerStereoDriver]: initialized");

        m_is_initialized = true;
    }
//}


// | ---------------------- msg callbacks --------------------- |
    void BaslerStereoDriver::m_cbk_fright_tag_detection(const apriltag_ros::AprilTagDetectionArray msg) {
        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: right camera tags detection cbk");
        if (msg.detections.empty()) {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: no tags visible by right camera");
            return;
        }
        auto right_detections = msg.detections;
        std::sort(right_detections.begin(), right_detections.end() - 1, [](auto a, auto b) { return a.id < b.id; });
        right_detections.pop_back();
        if (right_detections.size() != 12) {
            ROS_WARN_THROTTLE(1.0, "[basler stereo pair]: right camera: %zu out of 12 tags detected",
                              right_detections.size());
            return;
        }
        std::lock_guard<std::mutex> lt{m_mut_rtpose};
        std::for_each(right_detections.begin(),
                      right_detections.end(),
                      [&](const auto &el) { m_right_tag_poses.push_back(el.pose.pose.pose.position); });
        m_timestamp_frt = msg.header.stamp;
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: right camera tags detection cbk complete");
    }


    void BaslerStereoDriver::m_cbk_fleft_tag_detection(const apriltag_ros::AprilTagDetectionArray msg) {
        if (not m_is_initialized) return;
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: left camera tags detection cbk");
        if (msg.detections.empty()) {
            ROS_WARN_THROTTLE(2.0, "[basler_stereo_driver]: no tags visible by left camera");
            return;
        }

        auto left_detections = msg.detections;
        std::sort(left_detections.begin(), left_detections.end() - 1, [](auto a, auto b) { return a.id < b.id; });
        left_detections.pop_back();
        if (left_detections.size() != 12) {
            ROS_WARN_THROTTLE(1.0, "[basler stereo pair]: left camera: %zu out of 12 tags detected",
                              left_detections.size());
            return;
        }
        std::lock_guard<std::mutex> lt{m_mut_ltpose};
        std::for_each(left_detections.begin(),
                      left_detections.end(),
                      [&](const auto &el) { m_left_tag_poses.push_back(el.pose.pose.pose.position); });
        m_timestamp_flt = msg.header.stamp;
        ROS_INFO_THROTTLE(2.0, "[basler_stereo_driver]: left camera tags detection cbk complete");
    }


// | --------------------- timer callbacks -------------------- |
    void BaslerStereoDriver::m_tim_cbk_tagcoor([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback will find a 3d position of tags in space from two cameras and
        // save them as a mrs_lib::TransformStamped

        if (not m_is_initialized) return;
        if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
        {
            std::lock_guard<std::mutex> lg{m_mut_rtpose};
            std::lock_guard<std::mutex> lg2{m_mut_ltpose};

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
            std::lock_guard<std::mutex> lg_r{m_mut_rtpose};
            for (int i = 0; i < m; ++i) {
                tf2::fromMsg(m_right_tag_poses[i], row_eigen);
                src.block(0, i, d, 1) = row_eigen;
            }
        }
        {
            std::lock_guard<std::mutex> lg_l{m_mut_ltpose};
            for (int i = 0; i < m; ++i) {
                tf2::fromMsg(m_left_tag_poses[i], row_eigen);
                dst.block(0, i, d, 1) = row_eigen;
            }
        }
        // No way to directly cast umeyama output to Eigen::Affine3d
        Eigen::Matrix4d translation = umeyama(src, dst);
        Eigen::Affine3d translation_aff;
        translation_aff = translation;
        // And here affine transformation is needed
        std::lock_guard<std::mutex> ltr{m_mut_transform_tags};
        geometry_msgs::TransformStamped transform_msg = tf2::eigenToTransform(translation_aff);
        m_transform_msg = mrs_lib::TransformStamped("basler_right_optical/tag_1",
                                                    "basler_left_optical/tag_1",
                                                    ros::Time::now(),
                                                    transform_msg);
    }

    void BaslerStereoDriver::m_tim_cbk_transformation([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback to publish camera position firstly as uncorrected estimation of a position (from CAD)
        // and then - corrected using least-squares solution for two sets of points

        if (not m_is_initialized) return;

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

        auto to_left_mrs = mrs_lib::TransformStamped{to_left.header.frame_id,
                                                     to_left.child_frame_id,
                                                     ros::Time::now(),
                                                     to_left};
        //
        auto T2p_cam2tag = m_transformer.getTransform("uav1/basler_left_optical", "basler_left_optical/tag_1");
        auto T2T1_base2tag = m_transformer.getTransform("uav1/basler_stereopair/base", "basler_right_optical/tag_1");

        Eigen::Affine3d corrected;

        if (T2p_cam2tag.has_value() and T2T1_base2tag.has_value()) {
            m_mut_transform_tags.lock();
            corrected = T2p_cam2tag.value().inverse().getTransformEigen() *
                        m_transform_msg.getTransformEigen() *
                        T2T1_base2tag.value().getRotationEigen();
            m_mut_transform_tags.unlock();

            to_left_mrs = mrs_lib::TransformStamped{to_left.header.frame_id,
                                                    to_left.child_frame_id,
                                                    to_left.header.stamp,
                                                    tf2::eigenToTransform(corrected)};

            to_left = to_left_mrs.getTransform();
            to_left.header.frame_id = "uav1/basler_stereopair/base";
            to_left.child_frame_id = "uav1/basler_left_optical";
            to_left.header.stamp = ros::Time::now();

        } else {
            ROS_ERROR_THROTTLE(1.0, "[basler stereo pair tag correction] wrong transformation");
        }
        m_tbroadcaster.sendTransform(to_left);
        ROS_INFO("[basler_driver] transform stamped sent from %s to %s time %u",
                 to_left.header.frame_id.c_str(),
                 to_left.child_frame_id.c_str(),
                 to_left.header.stamp.nsec);
    }
// | -------------------- other functions ------------------- |


}  // namespace basler_stereo_driver

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(basler_stereo_driver::BaslerStereoDriver, nodelet::Nodelet)
