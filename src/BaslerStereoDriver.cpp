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
        pl.loadParam("is_calibrated", m_is_calibrated);
        pl.loadParam("m_name_CR", m_name_CR);
        pl.loadParam("fleft_tag_det", m_name_fleft_tag_det);
        pl.loadParam("fright_tag_det", m_name_fright_tag_det);
        pl.loadParam("cam_fleft_roi/start_x", m_fleft_roi.x);
        pl.loadParam("cam_fleft_roi/start_y", m_fleft_roi.y);
        pl.loadParam("cam_fleft_roi/h", m_fleft_roi.height);
        pl.loadParam("cam_fleft_roi/w", m_fleft_roi.width);
        pl.loadParam("cam_fright_roi/start_x", m_fright_roi.x);
        pl.loadParam("cam_fright_roi/start_y", m_fright_roi.y);
        pl.loadParam("cam_fright_roi/h", m_fright_roi.height);
        pl.loadParam("cam_fright_roi/w", m_fright_roi.width);
        pl.loadParam("corresp/debug_epipolar", m_debug_epipolar);
        pl.loadParam("corresp/debug_matches", m_debug_matches);
        int tmp_thr;
        pl.loadParam("corresp/distance_threshold_px", tmp_thr);
        if (tmp_thr < 0) {
            ROS_INFO_ONCE("[%s]: wrong distance_threshold_px parameter: should be x > 0", NODENAME.c_str());
        }
        m_distance_threshold = static_cast<size_t>(tmp_thr);
//        5375414116326520
        pl.loadParam("corresp/distances_ratio", m_distance_ratio);
        if ((m_distance_ratio > 1) or (m_distance_ratio < 0)) {
            ROS_INFO_ONCE("[%s]: wrong distance_ration parameter: should be 0 < x < 1", NODENAME.c_str());
        }
        int n_features;
        pl.loadParam("corresp/n_features", n_features);
        detector = cv::ORB::create(n_features);
        // intrinsic camera parameters (calibration matrices)
        cv::eigen2cv(pl.loadMatrixStatic2<3, 3>("basler_left/camera_matrix/data"), m_K_CL);
        cv::eigen2cv(pl.loadMatrixStatic2<3, 3>("basler_right/camera_matrix/data"), m_K_CR);

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
        m_pub_im_corresp = nh.advertise<sensor_msgs::Image>("im_corresp", 1);
        m_pub_multiview = nh.advertise<mrs_msgs::ImageLabeledArray>("multiview_labeled", 1);
        m_pub_im_left_epipolar = nh.advertise<sensor_msgs::Image>("epimleft", 1);
        m_pub_im_right_epipolar = nh.advertise<sensor_msgs::Image>("epimright", 1);
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
            m_tim_mse = nh.createTimer(ros::Duration(0.0001),
                                       &BaslerStereoDriver::m_tim_cbk_tags_errors,
                                       this);
            m_tim_corresp = nh.createTimer(ros::Duration(0.0001),
                                           &BaslerStereoDriver::m_tim_cbk_corresp,
                                           this);
            // for epipolar lines drawing I'll use subscriber handler
            mrs_lib::SubscribeHandlerOptions shopt{nh};
            shopt.node_name = NODENAME;
            shopt.threadsafe = true;
            shopt.no_message_timeout = ros::Duration(1.0);
            mrs_lib::construct_object(m_handler_imleft,
                                      shopt,
//                                      "/" + m_uav_name + "/fleft/tag_detections_image");
                                      "/" + m_uav_name + "/basler_left/image_rect");
            mrs_lib::construct_object(m_handler_imright,
                                      shopt,
//                                      "/" + m_uav_name + "/fright/tag_detections_image");
                                      "/" + m_uav_name + "/basler_right/image_rect");
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
            m_camera_right.fromCameraInfo(m_handler_camrightinfo.getMsg());
            m_camera_left.fromCameraInfo(m_handler_camleftinfo.getMsg());
        }
        m_tim_collect_images = nh.createTimer(ros::Duration(0.001),
                                              &BaslerStereoDriver::m_tim_cbk_collect_images,
                                              this);
        m_tim_fleft_pose = nh.createTimer(ros::Duration(0.0001),
                                          &BaslerStereoDriver::m_tim_cbk_fleft_pose,
                                          this);
        // needed
        m_sub_camera_fleft = nh.subscribe(m_name_fleft_tag_det,
                                          1,
                                          &BaslerStereoDriver::m_cbk_tag_detection_fleft,
                                          this);
        m_sub_camera_fright = nh.subscribe(m_name_fright_tag_det,
                                           1,
                                           &BaslerStereoDriver::m_cbk_tag_detection_fright,
                                           this);
        // initiate masks for an image matching part
        mask_left(cv::Rect{m_fleft_roi.x,
                           m_fleft_roi.y,
                           m_fleft_roi.width,
                           m_fleft_roi.height}) = cv::Scalar{255};
        mask_right(cv::Rect{m_fright_roi.x,
                            m_fright_roi.y,
                            m_fright_roi.width,
                            m_fright_roi.height}) = cv::Scalar{255};
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

    void BaslerStereoDriver::m_tim_cbk_collect_images([[maybe_unused]] const ros::TimerEvent &ev) {
        // collect images from almost the same timestamp and publish them as one image
        // I made it because it was so complicated to keep tracking of all image pairs
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

        // calculate reprojection error
//        if (m_handler_imleft.newMsg() or m_handler_imright.newMsg()) {
//            ROS_INFO_THROTTLE(1.0, "[BaslerStereoDriver]: error searching");
//            auto cv_image_left = cv_bridge::toCvCopy(m_handler_imleft.getMsg(), "bgr8").get()->image;
//            auto cv_image_right = cv_bridge::toCvCopy(m_handler_imright.getMsg(), "bgr8").get()->image;
//            {
//                std::scoped_lock lc{m_mut_pose_fright, m_mut_pose_fleft};
//                for (auto &p: m_left_tag_poses) {
//                    cv::Point3d pt_cv{p.x, p.y, p.z};
//                    auto uv = m_camera_left.project3dToPixel(pt_cv);
//                    cv::circle(cv_image_left, uv, 3, generate_random_color(), 3);
//                }
//            }
//            m_pub_im_epileft.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_left).toImageMsg());
//            m_pub_im_epiright.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_right).toImageMsg());
//        } else {
//            ROS_WARN_THROTTLE(1.0, "[BaslerStereoDriver]: no images for error searching");
//        }
    }

    void BaslerStereoDriver::m_tim_cbk_corresp([[maybe_unused]] const ros::TimerEvent &ev) {
        if (not m_is_initialized) return;

        if (m_handler_imleft.newMsg() and m_handler_imright.newMsg()) {
            auto RL = m_transformer.getTransform(m_name_CR,
                                                 m_name_CL);

            auto LR = m_transformer.getTransform(m_name_CL,
                                                 m_name_CR);

            if (not(LR.has_value() and RL.has_value())) {
                ROS_WARN_THROTTLE(2.0, "NO RL OR LR transformation");
                return;
            } else {
                ROS_INFO_THROTTLE(2.0, "[%s]: looking for correspondences", NODENAME.c_str());
            }

            cv::Point3d origin1{LR.value().transform.translation.x,
                                LR.value().transform.translation.y,
                                LR.value().transform.translation.z};

            cv::Point3d origin2{RL.value().transform.translation.x,
                                RL.value().transform.translation.y,
                                RL.value().transform.translation.z};

            cv::Point2d o1 = m_camera_right.project3dToPixel(origin1);
            cv::Point2d o2 = m_camera_left.project3dToPixel(origin2);
            auto cv_image_left = cv_bridge::toCvCopy(m_handler_imleft.getMsg(),
                                                     "bgr8").get()->image;
            auto cv_image_right = cv_bridge::toCvCopy(m_handler_imright.getMsg(),
                                                      "bgr8").get()->image;
            cv::Mat im_gray_left, im_gray_right;
            cv::cvtColor(cv_image_left, im_gray_left, cv::COLOR_BGR2GRAY);
            cv::cvtColor(cv_image_right, im_gray_right, cv::COLOR_BGR2GRAY);

            cv::Mat descriptor1, descriptor2;
            std::vector<cv::KeyPoint> keypoints1, keypoints2;
            // detect features and compute correspondances
            detector->detectAndCompute(im_gray_left,
                                       mask_left,
                                       keypoints1,
                                       descriptor1);
            detector->detectAndCompute(im_gray_right,
                                       mask_right,
                                       keypoints2,
                                       descriptor2);
            [[maybe_unused]] auto w = im_gray_left.size[1];
            if (keypoints1.size() < m_distance_threshold or
                keypoints2.size() < m_distance_threshold) {
                ROS_WARN_THROTTLE(1.0, "[%s]: no keypoints visible", NODENAME.c_str());
                return;
            }

            std::vector<cv::DMatch> matches;
            matcher->match(descriptor1,
                           descriptor2,
                           matches,
                           cv::Mat());

            std::sort(matches.begin(), matches.end());
            const int num_good_matches = matches.size() * m_distance_ratio;
            matches.erase(matches.begin() + num_good_matches, matches.end());

            std::vector<cv::DMatch> matches_filtered;

            for (auto &matche: matches) {
                cv::Point2f pt1_cv = keypoints1[matche.queryIdx].pt;
                cv::Point2f pt2_cv = keypoints2[matche.trainIdx].pt;
                cv::Point3d ray1_cv = m_camera_left.projectPixelTo3dRay(pt1_cv);
                cv::Point3d ray2_cv = m_camera_right.projectPixelTo3dRay(pt2_cv);
                const auto ray1_opt = m_transformer.transform(ray1_cv, LR.value());
                const auto ray2_opt = m_transformer.transform(ray2_cv, RL.value());

                if (not(ray1_opt.has_value() and ray2_opt.has_value())) {
                    ROS_WARN_THROTTLE(2.0, "[%s]: It was not possible to transform a ray", m_uav_name.c_str());
                    return;
                }
                ray1_cv = ray1_opt.value();
                ray2_cv = ray2_opt.value();

                auto epiline2 = cross(m_camera_right.project3dToPixel(ray1_cv), o1);
                auto epiline1 = cross(m_camera_left.project3dToPixel(ray2_cv), o2);

                normalize_line(epiline1);
                normalize_line(epiline2);

                auto dist1 = std::abs(epiline1.dot(cv::Point3d{pt1_cv.x, pt1_cv.y, 1}));
                auto dist2 = std::abs(epiline2.dot(cv::Point3d{pt2_cv.x, pt2_cv.y, 1}));

                if (dist1 > 25 or dist2 > 25) {
                    ROS_WARN_THROTTLE(1.0, "filtered corresp");
                    continue;
                }
                matches_filtered.push_back(matche);
                if (m_debug_epipolar) {
                    // Draw epipolar lines
                    auto color = generate_random_color();
                    cv::circle(cv_image_left, pt1_cv, 3, color, 3);
                    cv::circle(cv_image_right, pt2_cv, 3, color, 3);
                    auto image_pts = line2image(epiline2, w);
                    auto p1 = image_pts.first;
                    auto p2 = image_pts.second;
                    cv::line(cv_image_right, p1, p2, color, 2);

                    image_pts = line2image(epiline1, w);
                    p1 = image_pts.first;
                    p2 = image_pts.second;
                    cv::line(cv_image_left, p1, p2, color, 2);

                    m_pub_im_left_epipolar.publish(
                            cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_left).toImageMsg());
                    m_pub_im_right_epipolar.publish(
                            cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_image_right).toImageMsg());
                    ROS_INFO_THROTTLE(2.0, "published epipolars");
                }
            }
            if (m_debug_matches) {
                cv::Mat im_matches;
                cv::drawMatches(cv_image_left,
                                keypoints1,
                                cv_image_right,
                                keypoints2,
                                matches_filtered,
                                im_matches,
                                cv::Scalar::all(-1),
                                cv::Scalar::all(-1),
                                std::vector<char>(),
                                cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                m_pub_im_corresp.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", im_matches).toImageMsg());
                ROS_INFO_THROTTLE(2.0, "[%s]: Correspondences published", NODENAME.c_str());
            }
        } else {
            ROS_WARN_THROTTLE(2.0, "[%s]: No new images to search for correspondences", NODENAME.c_str());
        }
    }
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

    // ===================== UTILS =====================

    [[maybe_unused]] cv::Scalar BaslerStereoDriver::generate_random_color() {
        std::random_device rd;
        std::mt19937 generator(rd());
        std::uniform_int_distribution<uint8_t> distribution{0, 255};

        uint8_t r = distribution(generator);
        uint8_t g = distribution(generator);
        uint8_t b = distribution(generator);
        return cv::Scalar(b, g, r);
    }

    [[maybe_unused]] void BaslerStereoDriver::draw_epipolar_line(cv::Mat &img,
                                                                 std::vector<cv::Point3f> &line,
                                                                 const std::vector<cv::Point2f> &pts) {
        // source https://docs.opencv.org/3.4/da/de9/tutorial_py_epipolar_geometry.html
        auto w = img.size[1]; // c
        for (size_t i = 0; i < line.size(); ++i) {
//        for (size_t i = 0; i < 100; i += 20) {
            // randomly generate line color
            auto color = generate_random_color();
            auto x0 = .0f;
            auto x1 = static_cast<double>(w);

            double l0 = line[i].x;
            double l1 = line[i].y;
            double l2 = line[i].z;

            double y0 = -l2 / l1;
            double y1 = -(l2 + l0 * w) / l1;

            auto p1 = cv::Point{static_cast<int>(std::round(x0)), static_cast<int>(std::ceil(y0))};
            auto p2 = cv::Point{static_cast<int>(std::round(x1)), static_cast<int>(std::ceil(y1))};

            cv::line(img, p1, p2, color, 3);
            cv::circle(img, pts[i], 2, color, 5);
        }
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
