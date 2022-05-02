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
        mrs_lib::ParamLoader pl(nh, NODENAME);

        pl.loadParam("UAV_NAME", m_uav_name);
        pl.loadParam("camera_poses_filename", m_config_filename);
        pl.loadParam("base_frame_pose", m_name_base);
        pl.loadParam("m_name_CL", m_name_CL);
        pl.loadParam("m_name_CR", m_name_CR);
        pl.loadParam("is_calibrated", m_is_calibrated);
        pl.loadParam("fleft_tag_det", m_name_fleft_tag_det);
        pl.loadParam("fright_tag_det", m_name_fright_tag_det);
        // stereopair pose parameters
        auto fleft_rotation = pl.loadMatrixStatic2<3, 3>("fleft_camera/rotation");
        auto fleft_translation = pl.loadMatrixStatic2<3, 1>("fleft_camera/translation");

        m_fleft_pose.translate(fleft_translation).rotate(Eigen::Quaterniond{fleft_rotation});

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", NODENAME.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", NODENAME.c_str());
        }
        // | ---------------- some data post-processing --------------- |

        // | ----------------- publishers initialize ------------------ |
        m_pub_multiview = nh.advertise<mrs_msgs::ImageLabeledArray>("multiview_labeled", 1);
        // | ---------------- subscribers initialize ------------------ |

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("basler_stereo_driver");
        m_transformer.setDefaultPrefix(m_uav_name);

        // | -------------------- initialize timers ------------------- |
        // also there is a difference between calibrated and non-calibrated mode
        // so different subscribers are also initialized here

        ROS_INFO_ONCE("[%s] timers initialisation", NODENAME.c_str());

        // If pair is calibrated - publish the pose as already calibrated parameter
        if (not m_is_calibrated) {
            m_tim_tags_coordinates = nh.createTimer(ros::Duration(m_time_tagcoor),
                                                    &BaslerStereoDriver::m_tim_cbk_tagcoor,
                                                    this);
            m_tim_find_BL = nh.createTimer(ros::Duration(m_time_transformation),
                                           &BaslerStereoDriver::m_tim_cbk_find_BL,
                                           this);
            // | ---------------- subscribers initialize ------------------ |
            m_sub_complete_calibration = nh.subscribe(m_uav_name + "/complete",
                                                      1,
                                                      &BaslerStereoDriver::m_cbk_complete_save_calibration,
                                                      this);
        } else {
            m_tim_mse = nh.createTimer(ros::Duration(0.01),
                                       &BaslerStereoDriver::m_tim_cbk_tags_errors,
                                       this);
            // for epipolar lines drawing I'll use subscriber handler
            mrs_lib::SubscribeHandlerOptions shopt{nh};
            shopt.node_name = NODENAME;
            shopt.threadsafe = true;
            shopt.no_message_timeout = ros::Duration(1.0);
            mrs_lib::construct_object(m_handler_imleft,
                                      shopt,
                                      "/" + m_uav_name + "/fleft/tag_detections_image");
//                                      "/" + m_uav_name + "/basler_left/image_rect");
            mrs_lib::construct_object(m_handler_imright,
                                      shopt,
                                      "/" + m_uav_name + "/fright/tag_detections_image");
//                                      "/" + m_uav_name + "/basler_right/image_rect");
            mrs_lib::construct_object(m_handler_camleftinfo,
                                      shopt,
                                      "/" + m_uav_name + "/basler_left/camera_info");
            mrs_lib::construct_object(m_handler_camrightinfo,
                                      shopt,
                                      "/" + m_uav_name + "/basler_right/camera_info");
            // initialize cameras with pinhole modeller
            while (not(m_handler_camleftinfo.newMsg() and m_handler_camrightinfo.newMsg())) {
                ROS_WARN_THROTTLE(1.0, "[%s]: waiting for camera info messages", NODENAME.c_str());
            }
        }
        m_tim_collect_images = nh.createTimer(ros::Duration(0.001),
                                              &BaslerStereoDriver::m_tim_cbk_collect_images,
                                              this);
        m_tim_fleft_pose = nh.createTimer(ros::Duration(0.0001),
                                          &BaslerStereoDriver::m_tim_cbk_fleft_pose,
                                          this);
        // needed
//        m_sub_camera_fleft = nh.subscribe(m_name_fleft_tag_det,
//                                          1,
//                                          &BaslerStereoDriver::m_cbk_tag_detection_fleft,
//                                          this);
//        m_sub_camera_fright = nh.subscribe(m_name_fright_tag_det,
//                                           1,
//                                           &BaslerStereoDriver::m_cbk_tag_detection_fright,
//                                           this);
        ROS_INFO_ONCE("[%s]: initialized", NODENAME.c_str());
        m_is_initialized = true;
    }
//}

// | ---------------------- msg callbacks --------------------- |
    void BaslerStereoDriver::m_cbk_tag_detection_fright(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        if (not m_is_initialized) return;
        std::lock_guard<std::mutex> lt{m_mut_pose_fright};
        auto res = m_tag_detection_cbk_body("right", msg);
        if (res.has_value()) {
            m_right_tag_poses = res.value();
            m_timestamp_fright = msg->header.stamp;
            ROS_INFO_THROTTLE(2.0, "[%s]: right camera tags detection cbk complete", NODENAME.c_str());
        } else {
            ROS_WARN_THROTTLE(2.0, "[%s]: right camera cnk not completed", NODENAME.c_str());
        }
    }

    void BaslerStereoDriver::m_cbk_tag_detection_fleft(const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {
        if (not m_is_initialized) return;
        std::lock_guard<std::mutex> lt{m_mut_pose_fleft};
        auto res = m_tag_detection_cbk_body("left", msg);
        if (res.has_value()) {
            m_left_tag_poses = res.value();
            m_timestamp_fleft = msg->header.stamp;
            ROS_INFO_THROTTLE(2.0, "[%s]: left camera tags detection cbk complete", NODENAME.c_str());
        } else {
            ROS_WARN_THROTTLE(2.0, "[%s]: left camera cnk not completed", NODENAME.c_str());
        }
    }

    void BaslerStereoDriver::m_cbk_complete_save_calibration(std_msgs::Bool flag) {
        if (not m_is_initialized) return;
        // if calibration is completed and received flag is True - save all data and close the nodelet
        if (flag.data) {
            std::lock_guard l{m_mut_filtered_CL_pose};
            std::ofstream fout(m_config_filename);
            Eigen::IOFormat fmt{3, 0, ", ", ",\n", "", "", "[", "]"};
            fout << "fleft_camera:\n";
            fout << "\trotation: "
                 << m_filtered_CL_pose.rotation().format(fmt)
                 << std::endl;
            fout << "\ttranslation: " << m_filtered_CL_pose.translation().format(fmt) << std::endl;
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
                                  "[%s]: tags coordinates timestamps are too far away from each other: %f",
                                  NODENAME.c_str(),
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
        // C - camera

        if (not m_is_initialized) return;

        {
            std::scoped_lock lck{m_mut_pose_fright, m_mut_pose_fleft};
            if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
            if (std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()) > ros::Duration(0.2).toSec()) {
                ROS_WARN_THROTTLE(1.0,
                                  "[%s]: tags coordinates timestamps are too far away from each other: %f",
                                  NODENAME.c_str(),
                                  std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()));
                return;
            }
        }
        ROS_INFO_THROTTLE(2.0, "[%s] transform RL publisher", NODENAME.c_str());

        auto T_BR = m_transformer.getTransform(m_name_CR,
                                               m_name_base);
        auto T_BL_original = m_transformer.getTransform(m_name_CL,
                                                        m_name_base);
        geometry_msgs::TransformStamped T_BL_result;

        if (T_BR.has_value() and T_BL_original.has_value()) {
            std::lock_guard lock{m_mut_RL_correction};
            auto t_BR_eig = Eigen::Affine3d{tf2::transformToEigen(T_BR->transform)};
            const auto T_RL = Eigen::Affine3d{tf2::transformToEigen(T_BL_original->transform)} *
                              t_BR_eig.inverse();
            const auto T_RL_corrected = m_RL_error.inverse() * T_RL;
            auto T_BL_corrected = T_RL_corrected * t_BR_eig;
            // make a new frame - pose estimation
            std::lock_guard l{m_mut_filtered_CL_pose};
            if (not m_weight) {
                m_filtered_CL_pose = T_BL_corrected;
            }
            ++m_weight;
            T_BL_result = tf2::eigenToTransform(m_interpolate_pose(m_filtered_CL_pose, T_BL_corrected));

        } else {
            ROS_ERROR_THROTTLE(2.0, "[%s] wrong transformation from L/R to base", NODENAME.c_str());
        }
        T_BL_result.header.stamp = ev.current_real;
        T_BL_result.header.frame_id = m_name_base;
        T_BL_result.child_frame_id = m_name_CL + "corrected";

        for (int i = 0; i < 12; ++i) {
            // find a translation for all tags to compare the result
            auto L2tag = m_transformer.getTransform(m_name_CL,
                                                    "basler_left_optical/tag_" + std::to_string(i));

            if (L2tag.has_value()) {
//                auto transform = L2tag.value().inverse().getTransform();
                auto transform = geometry_msgs::TransformStamped{};
                transform.transform = tf2::eigenToTransform(
                        Eigen::Affine3d{tf2::transformToEigen(L2tag->transform)}.inverse()).transform;
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = m_name_CL + "corrected";
                transform.child_frame_id = m_uav_name + "/tag_modified" + std::to_string(i);
                m_tbroadcaster.sendTransform(transform);
            }
        }

        m_tbroadcaster.sendTransform(T_BL_result);
        ROS_INFO_THROTTLE(2.0, "[%s] transform stamped sent from %s to %s time %u",
                          NODENAME.c_str(),
                          T_BL_result.header.frame_id.c_str(),
                          T_BL_result.child_frame_id.c_str(),
                          T_BL_result.header.stamp.nsec);
    }

    void BaslerStereoDriver::m_tim_cbk_tags_errors([[maybe_unused]] const ros::TimerEvent &ev) {
        // timer callback to calculate error in euclidean 3d space between tags

        if (not m_is_initialized) return;
        {
            std::scoped_lock lck{m_mut_pose_fright, m_mut_pose_fleft};
            if (m_right_tag_poses.empty() or m_left_tag_poses.empty()) return;
            if (std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()) > ros::Duration(0.1).toSec()) {
                ROS_WARN_THROTTLE(1.0,
                                  "[%s]: tags coordinates timestamps are too far away from each other: %f",
                                  NODENAME.c_str(),
                                  std::abs(m_timestamp_fright.toSec() - m_timestamp_fleft.toSec()));
                return;
            }
            ROS_INFO_THROTTLE(1.0, "[%s]: tags timestamps are okay", NODENAME.c_str());
        }
        double MSE_res = 0;
        double MAE_res = 0;
        for (int i = 0; i < 12; ++i) {
            // find a translation for all tags to compare the result
            auto Ltag2Rtag = m_transformer.getTransform("basler_right_optical/tag_" + std::to_string(i),
                                                        "basler_left_optical/tag_" + std::to_string(i));
            if (Ltag2Rtag.has_value()) {
                auto l2r_norm = Eigen::Affine3d{tf2::transformToEigen(Ltag2Rtag->transform)}.translation().norm();
                MSE_res += std::pow(l2r_norm, 2);
                MAE_res += l2r_norm;
            }
        }
        ROS_INFO_THROTTLE(2.0, "[%s]: \n\t\t\tMSE == %lf\n\t\t\tMAE == %lf",
                          NODENAME.c_str(),
                          MSE_res / 12.0,
                          MAE_res / 12.0);
    }

    //############################################################

// | -------------------- other functions ------------------- |

    std::optional<std::vector<geometry_msgs::Point>>
    BaslerStereoDriver::m_tag_detection_cbk_body(const std::string &camera_name,
                                                 const apriltag_ros::AprilTagDetectionArray::ConstPtr &msg) {

        if (not m_is_initialized) return std::nullopt;
        if (msg->detections.empty()) {
            ROS_WARN_THROTTLE(2.0, "[%s]: no tags visible by %s camera",
                              NODENAME.c_str(),
                              camera_name.c_str());
            return std::nullopt;
        }
        auto msg_sorted = msg.get()->detections;
        std::sort(msg_sorted.begin(), msg_sorted.end() - 1, [](auto a, auto b) { return a.id < b.id; });
        msg_sorted.pop_back();
        if (msg_sorted.size() != 12) {
            ROS_WARN_THROTTLE(1.0, "[%s]: %s camera: %zu out of 12 tags detected",
                              NODENAME.c_str(),
                              camera_name.c_str(),
                              msg_sorted.size());
            return std::nullopt;
        }
        std::vector<geometry_msgs::PoseWithCovarianceStamped> poses_base_frame;
        for (const apriltag_ros::AprilTagDetection &el: msg_sorted) {
            auto pose_cam_frame = m_transformer.transformSingle(el.pose, m_name_base);
            if (pose_cam_frame.has_value()) {
                poses_base_frame.push_back(pose_cam_frame.value());
            } else {
                ROS_WARN_THROTTLE(2.0, "[%s] error transforming tag from %s camera to base frame",
                                  NODENAME.c_str(),
                                  camera_name.c_str());
            }
        }
        std::vector<geometry_msgs::Point> result;
        for (const auto &el: poses_base_frame) {
            result.push_back(el.pose.pose.position);
        }
        ROS_INFO_THROTTLE(2.0, "[%s]: %s camera tags detection cbk complete",
                          NODENAME.c_str(),
                          camera_name.c_str());

        return result;
    }

    void BaslerStereoDriver::m_tim_cbk_collect_images([[maybe_unused]] const ros::TimerEvent &ev) {

        // collect images from almost the same timestamp and publish them as one image
        // I made it because it was so complicated to keep tracking of all image pairs
//        used in python script
// TODO: check for number of subscribers and if there is no subscribers - dont publish images.
        if (not m_is_initialized) return;
        if (m_handler_imleft.newMsg() and m_handler_imright.newMsg()) {
            imleft = m_handler_imleft.getMsg();
            imright = m_handler_imright.getMsg();
            auto time_diff = std::abs(imleft->header.stamp.toSec() - imright->header.stamp.toSec());
            if (time_diff > ros::Duration(0.2).toSec()) {
                ROS_WARN_THROTTLE(1.0,
                                  "[%s]: impair: images coordinates timestamps are too far away from each other: %f",
                                  NODENAME.c_str(),
                                  time_diff);
                return;
            }
            ROS_INFO_THROTTLE(1.0, "[%s]: impair: images timestamps are okay", NODENAME.c_str());
        } else {
            ROS_WARN_THROTTLE(1.0, "[%s]: impair: no new images", NODENAME.c_str());
            return;
        }

        m_impair = boost::make_shared<mrs_msgs::ImageLabeledArray>();
        mrs_msgs::ImageLabeled im_labeled_fright{};
        mrs_msgs::ImageLabeled im_labeled_fleft{};

        im_labeled_fleft.label = "fleft";
        im_labeled_fright.label = "fright";

        im_labeled_fleft.img.header.stamp = imleft->header.stamp;
        im_labeled_fleft.img.header.frame_id = imleft->header.frame_id;
        im_labeled_fleft.img.data = imleft->data;
        im_labeled_fleft.img.step = imleft->step;
        im_labeled_fleft.img.width = imleft->width;
        im_labeled_fleft.img.height = imleft->height;
        im_labeled_fleft.img.encoding = imleft->encoding;
        im_labeled_fleft.img.is_bigendian = imleft->is_bigendian;

        im_labeled_fright.img.header.stamp = imright->header.stamp;
        im_labeled_fright.img.header.frame_id = imright->header.frame_id;
        im_labeled_fright.img.data = imright->data;
        im_labeled_fright.img.step = imright->step;
        im_labeled_fright.img.width = imright->width;
        im_labeled_fright.img.height = imright->height;
        im_labeled_fright.img.encoding = imright->encoding;
        im_labeled_fright.img.is_bigendian = imright->is_bigendian;

        m_impair->header.stamp = imright->header.stamp;
        m_impair->header.frame_id = m_name_base;

        m_impair->imgs_labeled.push_back(im_labeled_fright);
        m_impair->imgs_labeled.push_back(im_labeled_fleft);

        m_pub_multiview.publish(m_impair);
        ROS_INFO_THROTTLE(1.0, "[%s]: impair: impair updated", NODENAME.c_str());
    }

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
}  // namespace basler_stereo_driver

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(basler_stereo_driver::BaslerStereoDriver, nodelet::Nodelet)
