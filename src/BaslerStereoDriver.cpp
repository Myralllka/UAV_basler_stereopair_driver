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
        pl.loadParam("camera_poses_filename", m_camera_poses_filename);
        pl.loadParam("base_frame_pose", m_name_base);
        pl.loadParam("m_name_CL", m_name_CL);
        pl.loadParam("m_name_CR", m_name_CR);
        pl.loadParam("is_calibrated", m_is_calibrated);
        pl.loadParam("calib_algo", m_calib_algo);
        pl.loadParam("fleft_tag_det", m_name_fleft_tag_det);
        pl.loadParam("fright_tag_det", m_name_fright_tag_det);

        if (!pl.loadedSuccessfully()) {
            ROS_ERROR("[%s]: failed to load non-optional parameters!", NODENAME.c_str());
            ros::shutdown();
        } else {
            ROS_INFO_ONCE("[%s]: loaded parameters", NODENAME.c_str());
        }
        // | ---------------- some data post-processing --------------- |

        // | ----------------- publishers initialize ------------------ |
        m_pub_multiview = nh.advertise<mrs_msgs::ImageLabeledArray>("multiview_labeled", 1);
        m_pub_imdebug = nh.advertise<sensor_msgs::Image>("basler_driver_imdebug", 1);
        // | ---------------- subscribers initialize ------------------ |

        // | --------------------- tf transformer --------------------- |
        m_transformer = mrs_lib::Transformer("basler_stereo_driver");
        m_transformer.setDefaultPrefix(m_uav_name);

        // | -------------------- initialize timers ------------------- |
        // also there is a difference between calibrated and non-calibrated mode
        // so different subscribers are also initialized here

        ROS_INFO_ONCE("[%s] timers initialisation", NODENAME.c_str());
        // If pair is calibrated - publish the pose as already calibrated parameter
        mrs_lib::SubscribeHandlerOptions shopt{nh};
        shopt.node_name = NODENAME;
        shopt.threadsafe = true;
        shopt.no_message_timeout = ros::Duration(1.0);

        if (not m_is_calibrated) {
            ros::Duration(1).sleep();
            auto fright_pose_opt = m_transformer.getTransform(m_name_base, m_name_CR);
            if (fright_pose_opt.has_value()) {
                m_fright_pose = Eigen::Affine3d{tf2::transformToEigen(fright_pose_opt->transform)};
            } else {
                ROS_ERROR("fuck");
                ros::shutdown();
            }
            auto fleft_pose_opt = m_transformer.getTransform(m_name_base, m_name_CL);
            if (fleft_pose_opt.has_value()) {
                m_fleft_pose = Eigen::Affine3d{tf2::transformToEigen(fleft_pose_opt->transform)};
            } else {
                ROS_ERROR("fuck");
                ros::shutdown();
            }
            if (m_calib_algo == "PnP") {
                ROS_INFO("[%s]: PnP method calibration!", NODENAME.c_str());
                mrs_lib::construct_object(m_handler_cornersfleft,
                                          shopt,
                                          "/" + m_uav_name + "/fleft/corners");
                mrs_lib::construct_object(m_handler_cornersfright,
                                          shopt,
                                          "/" + m_uav_name + "/fright/corners");

                m_tim_tag_corners = nh.createTimer(ros::Duration(m_time_tagcoor),
                                                   &BaslerStereoDriver::m_tim_cbk_corners,
                                                   this);
            } else if (m_calib_algo == "3d") {
                ROS_INFO("[%s]: 3d apriltags method calibration!", NODENAME.c_str());
                m_tim_tags_coordinates = nh.createTimer(ros::Duration(m_time_tagcoor),
                                                        &BaslerStereoDriver::m_tim_cbk_tagcoor,
                                                        this);
                m_tim_find_BR = nh.createTimer(ros::Duration(m_time_transformation),
                                               &BaslerStereoDriver::m_tim_cbk_find_BR,
                                               this);
            } else {
                ROS_ERROR("[%s]: unknown or unimplemented stereopair calibration algorithm. shutting down",
                          NODENAME.c_str());
                ros::shutdown();
            }
            m_sub_complete_calibration = nh.subscribe(m_uav_name + "/complete",
                                                      1,
                                                      &BaslerStereoDriver::m_cbk_complete_save_calibration,
                                                      this);
        } else {
            // stereopair pose parameters
            auto fright_rotation = pl.loadMatrixStatic2<3, 3>("fright_camera/rotation");
            auto fright_translation = pl.loadMatrixStatic2<3, 1>("fright_camera/translation");

            m_fright_pose.translate(fright_translation).rotate(Eigen::Quaterniond{fright_rotation});
            m_tim_fright_pose = nh.createTimer(ros::Duration(0.001),
                                               &BaslerStereoDriver::m_tim_cbk_fright_pose,
                                               this);
            // for epipolar lines drawing I'll use subscriber handler
            mrs_lib::construct_object(m_handler_imleft,
                                      shopt,
                                      "/" + m_uav_name + "/fleft/tag_detections_image");
//                                      "/" + m_uav_name + "/basler_left/image_rect");
            mrs_lib::construct_object(m_handler_imright,
                                      shopt,
                                      "/" + m_uav_name + "/fright/tag_detections_image");
//                                      "/" + m_uav_name + "/basler_right/image_rect");
        }
        mrs_lib::construct_object(m_handler_camleftinfo,
                                  shopt,
                                  "/" + m_uav_name + "/basler_left/camera_info");
        mrs_lib::construct_object(m_handler_camrightinfo,
                                  shopt,
                                  "/" + m_uav_name + "/basler_right/camera_info");
        // initialize cameras with pinhole model
        while (not(m_handler_camleftinfo.newMsg() and m_handler_camrightinfo.newMsg())) {
            ROS_WARN_THROTTLE(1.0, "[%s]: waiting for camera info messages", NODENAME.c_str());
        }
        m_camera_right.fromCameraInfo(m_handler_camrightinfo.getMsg());
        m_camera_left.fromCameraInfo(m_handler_camleftinfo.getMsg());
        ros::Duration(0.1).sleep();
        m_K_CL = f2K33(m_handler_camleftinfo.getMsg().get()->P);
        m_K_CR = f2K33(m_handler_camrightinfo.getMsg().get()->P);
        // TODO: repair
//        m_tim_collect_images = nh.createTimer(ros::Duration(0.001),
//                                              &BaslerStereoDriver::m_tim_cbk_collect_images,
//                                              this);
        // needed
        m_sub_camera_fleft = nh.subscribe(m_name_fleft_tag_det,
                                          1,
                                          &BaslerStereoDriver::m_cbk_tag_detection_fleft,
                                          this);
        m_sub_camera_fright = nh.subscribe(m_name_fright_tag_det,
                                           1,
                                           &BaslerStereoDriver::m_cbk_tag_detection_fright,
                                           this);
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
//            ROS_INFO_THROTTLE(2.0, "[%s]: right camera tags detection cbk complete", NODENAME.c_str());
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
//            ROS_INFO_THROTTLE(2.0, "[%s]: left camera tags detection cbk complete", NODENAME.c_str());
        } else {
            ROS_WARN_THROTTLE(2.0, "[%s]: left camera cnk not completed", NODENAME.c_str());
        }
    }

    void BaslerStereoDriver::m_cbk_complete_save_calibration(std_msgs::Bool flag) {
        if (not m_is_initialized) return;
        if (m_is_calibrated) return;
        // if pair was not calibrated
        // calibration is completed and received flag is True - save all data and close the nodelet
        if (flag.data) {
            std::lock_guard l{m_mut_calibrated_CR_pose};
            std::ofstream fout(m_camera_poses_filename);
            Eigen::IOFormat fmt{3, 0, ", ", ",\n", "", "", "[", "]"};
            fout << "fright_camera:\n";
            fout << "  rotation: "
                 << m_calibrated_CR_pose.rotation().format(fmt)
                 << std::endl;
            fout << "  translation: " << m_calibrated_CR_pose.translation().format(fmt) << std::endl;
            ROS_INFO("[%s]: calibration completed; shutting down", NODENAME.c_str());
            ros::shutdown();
        }
    }

// | --------------------- timer callbacks -------------------- |

    bool check_PnP(const std::vector<apriltag_ros::PointLabeled>& detections, const Eigen::Matrix3d& K, const Eigen::Matrix3d& R, const Eigen::Vector3d t)
    {
      using vec2d_t = Eigen::Vector2d;
      using vec3d_t = Eigen::Vector3d;
      using vec4d_t = Eigen::Matrix<double, 4, 1>;
      using mat3d_t = Eigen::Matrix<double, 4, 4>;
      using mat3x4d_t = Eigen::Matrix<double, 3, 4>;

      bool all_ok = true;

      // https://docs.opencv.org/4.2.0/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
      mat3d_t tf = Eigen::Matrix<double, 4, 4>::Identity();
      tf.topLeftCorner<3, 3>() = R;
      tf.col(3).head<3>() = t;
      const mat3x4d_t mat4d_to_3d = Eigen::Matrix<double, 3, 4>::Identity();

      const std::vector<cv::Point3d> td_pts_cv = make_3d_apriltag_points(detections);
      std::vector<Eigen::Vector3d> td_pts;
      td_pts.reserve(td_pts_cv.size());
      for (const auto& cv_pt : td_pts_cv)
        td_pts.emplace_back(cv_pt.x, cv_pt.y, cv_pt.z);

      double total_error = 0.0;
      for (size_t it = 0; it < detections.size(); it++)
      {
        const vec3d_t& pt3d = td_pts.at(it);
        const vec4d_t pt3d_h (pt3d.x(), pt3d.y(), pt3d.z(), 1.0);

        // transform the 3d point to the camera frame and normalize it to 2d homogeneous coordinates
        const vec3d_t pt2d = mat4d_to_3d*tf*pt3d_h;
        const vec3d_t pt2d_h = pt2d / pt2d.z();

        if (std::abs(pt2d_h.z() - 1.0) > 1e-9)
        {
          ROS_ERROR_STREAM("[check_PnP]: Point in homogeneous 2D coordinates is not properly normalized (last element is " << pt2d_h.z() << ")!");
          all_ok = false;
          continue;
        }

        // project the point to the image coordinates
        vec3d_t pt_proj = K*pt2d_h;
        pt_proj = pt_proj / pt_proj.z();

          ROS_ERROR_STREAM("[check_PnP]: Reprojected point in homogeneous coordinates is not properly normalized (last element is " << pt2d_h.z() << ")!");
          if (std::abs(pt_proj.z() - 1.0) > 1e-9)
        {
          all_ok = false;
          continue;
        }

        // compare it with the ground-truth
        const vec2d_t ptIm = pt_proj.head<2>();
        const vec2d_t ptIm_gt (detections.at(it).x, detections.at(it).y);
        const double cur_error = (ptIm - ptIm_gt).norm();
        total_error += cur_error;

        if (cur_error > 1e-3)
        {
          ROS_ERROR_STREAM("[check_PnP]: Reprojected point [" << pt3d.transpose() << "] is [" << ptIm.transpose() <<
          "] which is different from the ground-truth [" << ptIm_gt.transpose() << "]!");
          all_ok = false;
        }
      }

      ROS_INFO_STREAM("[check_PnP]: The total reprojection error is " << total_error << "px.");

      return all_ok;
    }

    void BaslerStereoDriver::m_tim_cbk_corners([[maybe_unused]]const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;
        Eigen::Matrix3d R1, R2, R21;
        Eigen::Matrix<double, 3, 1> t1, t2, t21;
        if (m_handler_cornersfleft.newMsg() and m_handler_cornersfright.newMsg()) {
            auto left = m_handler_cornersfleft.getMsg();
            auto right = m_handler_cornersfright.getMsg();
            for (size_t i = 0; i < left->detections.size(); ++i) {
                ROS_INFO_STREAM("[check_detection_points] left pt: " << left->detections[i] << std::endl);
            }
            for (size_t i = 0; i < right->detections.size(); ++i) {
                ROS_INFO_STREAM("[check_detection_points] right pt: " << right->detections[i] << std::endl);
            }
            auto resl = u2RT(left->detections,
                             m_K_CL,
                             R1,
                             t1);
            auto resr = u2RT(right->detections,
                             m_K_CR,
                             R2,
                             t2);
            if (not resl) {
                ROS_WARN("left u2msg error");
                return;
            }
            if (not resr) {
                ROS_WARN("right u2msg error");
                return;
            }
            Eigen::Matrix3d K1, K2;
            cv::cv2eigen(m_K_CL, K1);
            cv::cv2eigen(m_K_CR, K2);

            if (not check_PnP(left->detections, K1, R1, t1))
            {
                ROS_WARN("left reprojection check failed");
                return;
            }

            if (not check_PnP(right->detections, K2, R2, t2))
            {
                ROS_WARN("right reprojection check failed");
                return;
            }

            R21 = R2 * R1.transpose();
//            std::cout << R1 << std::endl;
//            std::cout << R2 << std::endl;
//            std::cout << R21 << std::endl;
            t21 = t2 - R21 * t1;

            Eigen::Affine3d mat21 = Eigen::Affine3d::Identity();

//            Eigen::Affine3d mat1 = Eigen::Affine3d::Identity();
//            Eigen::Affine3d mat2 = Eigen::Affine3d::Identity();
//            mat1.translate(-R1.transpose() * t1).rotate(R1);
//            mat2.translate(-R2.transpose() * t2).rotate(R2);
            mat21.translate(t21).rotate(R21);

            if (Eigen::isfinite(mat21.matrix().array()).all()) {
                auto c2c1 = tf2::eigenToTransform(mat21);
                c2c1.header.stamp = ros::Time::now();
                c2c1.header.frame_id = "uav1/basler_right_optical";
                c2c1.child_frame_id = "c2c1debug";
                m_tbroadcaster.sendTransform(c2c1);
            } else {
                ROS_WARN_THROTTLE(1.0, "NaNs in transformation");
            }


//            auto res_msg = tf2::eigenToTransform(mat1);
//            res_msg.header.stamp = ros::Time::now();
//            res_msg.header.frame_id = "tag_base";
////            res_msg.header.frame_id = "uav1/basler_right_optical";
//            res_msg.child_frame_id = "debug";
//            m_tbroadcaster.sendTransform(res_msg);
//
//            auto res_msg2 = tf2::eigenToTransform(mat2);
//            res_msg2.header.stamp = ros::Time::now();
//            res_msg2.header.frame_id = "tag_base";
//            res_msg2.child_frame_id = "debug2";
//            m_tbroadcaster.sendTransform(res_msg2);
//
//            auto res_msg3 = tf2::eigenToTransform(mat1.inverse());
//            res_msg3.header.stamp = ros::Time::now();
//            res_msg3.header.frame_id = "tag_base";
////            res_msg3.header.frame_id = "uav1/basler_left_optical";
//            res_msg3.child_frame_id = "debug3";
//            m_tbroadcaster.sendTransform(res_msg3);
//
//            auto res_msg4 = tf2::eigenToTransform(mat2.inverse());
//            res_msg4.header.stamp = ros::Time::now();
//            res_msg4.header.frame_id = "tag_base";
//            res_msg4.child_frame_id = "debug4";
//            m_tbroadcaster.sendTransform(res_msg4);
//
        }
    }


    void BaslerStereoDriver::m_tim_cbk_fright_pose([[maybe_unused]] const ros::TimerEvent &ev) {
        // publish right camera pose (when camera pair is already calibrated)
        if (not m_is_initialized) return;
        geometry_msgs::TransformStamped fleft_pose_stamped = tf2::eigenToTransform(m_fright_pose);
        fleft_pose_stamped.header.frame_id = m_name_base;
        fleft_pose_stamped.child_frame_id = m_name_CR;
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
                dst.col(i) = row_eigen;
            }
        }
        {
            std::lock_guard lock{m_mut_pose_fleft};
            for (int i = 0; i < m; ++i) {
                tf2::fromMsg(m_left_tag_poses[i], row_eigen);
                src.col(i) = row_eigen;
            }
        }
        auto transform = Eigen::Affine3d{umeyama(src, dst, false)};

        std::lock_guard lock{m_mut_LR_correction};
        m_LR_error = transform;
    }

    void BaslerStereoDriver::m_tim_cbk_find_BR([[maybe_unused]] const ros::TimerEvent &ev) {
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

        auto T_BR_original = m_transformer.getTransform(m_name_CR,
                                                        m_name_base);
        auto T_BL_original = m_transformer.getTransform(m_name_CL,
                                                        m_name_base);
        geometry_msgs::TransformStamped T_BR_result;

        if (T_BR_original.has_value() and T_BL_original.has_value()) {
            std::lock_guard lock{m_mut_LR_correction};
            const auto T_BR_eig = Eigen::Affine3d{tf2::transformToEigen(T_BR_original->transform)};
            const auto T_BL_eig = Eigen::Affine3d{tf2::transformToEigen(T_BL_original->transform)};
            const auto T_LR = T_BR_eig * T_BL_eig.inverse();
            const auto T_LR_corrected = m_LR_error.inverse() * T_LR;
//            const auto T_LR_corrected = T_LR;
            const auto T_BR_corrected = T_LR_corrected * T_BL_eig;
            // make a new frame - pose estimation
            std::lock_guard l{m_mut_calibrated_CR_pose};
            if (not m_weight) {
                m_calibrated_CR_pose = T_BR_corrected;
            }
            ++m_weight;
            T_BR_result = tf2::eigenToTransform(m_interpolate_pose(m_calibrated_CR_pose, T_BR_corrected));
        } else {
            ROS_ERROR_THROTTLE(2.0, "[%s] wrong transformation from L/R to base", NODENAME.c_str());
        }
        T_BR_result.header.stamp = ev.current_real;
        T_BR_result.header.frame_id = m_name_base;
        T_BR_result.child_frame_id = m_name_CR + "_corrected";

        for (int i = 0; i < 12; ++i) {
            // find a translation for all tags to compare the result
            auto R2tag = m_transformer.getTransform(m_name_CR,
                                                    "basler_right_optical/tag_" + std::to_string(i));

            if (R2tag.has_value()) {
//                auto transform = R2tag.value().inverse().getTransform();
                auto transform = geometry_msgs::TransformStamped{};
                transform.transform = tf2::eigenToTransform(
                        Eigen::Affine3d{tf2::transformToEigen(R2tag->transform)}.inverse()).transform;
                transform.header.stamp = ros::Time::now();
                transform.header.frame_id = m_name_CR + "_corrected";
                transform.child_frame_id = m_uav_name + "/tag_modified" + std::to_string(i);
                m_tbroadcaster.sendTransform(transform);
            }
        }

        m_tbroadcaster.sendTransform(T_BR_result);
        ROS_INFO_THROTTLE(2.0, "[%s] transform stamped sent from %s to %s time %u",
                          NODENAME.c_str(),
                          T_BR_result.header.frame_id.c_str(),
                          T_BR_result.child_frame_id.c_str(),
                          T_BR_result.header.stamp.nsec);
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
