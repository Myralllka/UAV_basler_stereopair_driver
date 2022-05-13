#include "Utils.h"

#include <utility>

namespace basler_stereo_driver {

    constexpr double APTAG_SIZE = 0.055;
    constexpr double PADD_SIZE = 0.0135;
    constexpr double APTAG_PADD_SIZE = APTAG_SIZE + PADD_SIZE;

    enum {
        // from PointLabeled message
        LEFTUP = 1,
        RIGHTUP = 2,
        RIGHTBOTTOM = 3,
        LEFTBOTTOM = 4
    };

    bool u2RT(std::vector<apriltag_ros::PointLabeled> detections,
              const cv::Matx<double, 3, 3> &K,
              Eigen::Matrix3d &R,
              Eigen::Matrix<double, 3, 1> &t) {
        if (detections.empty()) {
            return false;
        }
        auto f = [](const auto &x, const auto &y) -> bool { return x.id == y.id ? x.type < y.type : x.id < y.id; };
        std::vector<cv::Point2d> pts2d_cv;
        pts2d_cv.reserve(detections.size());
        std::sort(detections.begin(), detections.end(), f);
        for (const auto &pt: detections) {
            pts2d_cv.emplace_back(pt.x, pt.y);
        }
        std::vector<cv::Point3d> td_pts = make_3d_apriltag_points(detections);
        cam2Rt(td_pts, pts2d_cv, K, cv::Mat{}, R, t);

        return true;
    }

    void cam2Rt(const std::vector<cv::Point3d> &td_pts,
                const std::vector<cv::Point2d> &im_pts,
                const cv::Matx<double, 3, 3> &K,
                const cv::Mat &d,
                Eigen::Matrix3d &R,
                Eigen::Matrix<double, 3, 1> &t) {
        cv::Mat R_cv;
        cv::Mat t_cv, r_cv;
        try {
//            check the result
            cv::solvePnP(td_pts,
                         im_pts,
                         K,
                         d,
                         r_cv,
                         t_cv,
                         false);
            cv::Rodrigues(r_cv, R_cv);
            cv::cv2eigen(R_cv, R);
            cv::cv2eigen(t_cv, t);
        } catch (cv::Exception &e) {
            std::cout << e.what() << std::endl;
        }
    }

    cv::Mat f2K33(const boost::array<double, 12> &P_in) {
        // transform a K matrix from CameraInfo (boost array) to CvMat
        Eigen::Matrix3d K_out = Eigen::Matrix3d::Identity();
        cv::Mat K_out_cv;
        K_out(0, 0) = P_in[0];
        K_out(0, 2) = P_in[2];
        K_out(1, 1) = P_in[5];
        K_out(1, 2) = P_in[6];
        cv::eigen2cv(K_out, K_out_cv);
        return K_out_cv;
    }

    std::vector<cv::Point3d> make_3d_apriltag_points(const std::vector<apriltag_ros::PointLabeled> &in_pts) {
        std::vector<cv::Point3d> res;
        res.reserve(in_pts.size());
        double x, y;
        for (const auto &in_pt: in_pts) {
            size_t j = in_pt.id;
            switch (in_pt.type) {
                case (LEFTUP):
                    x = APTAG_PADD_SIZE * static_cast<double>((j % 3));
                    y = APTAG_PADD_SIZE * static_cast<double>((j / 3));
                    break;
                case (RIGHTUP):
                    x = APTAG_SIZE + APTAG_PADD_SIZE * static_cast<double>((j % 3));
                    y = APTAG_PADD_SIZE * static_cast<double>((j / 3));
                    break;
                case (RIGHTBOTTOM):
                    x = APTAG_SIZE + APTAG_PADD_SIZE * static_cast<double>((j % 3));
                    y = APTAG_SIZE + APTAG_PADD_SIZE * static_cast<double>((j / 3));
                    break;
                case (LEFTBOTTOM):
                    x = APTAG_PADD_SIZE * static_cast<double>((j % 3));
                    y = APTAG_SIZE + APTAG_PADD_SIZE * static_cast<double>((j / 3));
                    break;
                default:
                    ROS_ERROR("corner point convertor: wrong point type");
                    break;
            }
            res.emplace_back(x, y, 0);
        }
        return res;
    }
}