#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <sys/stat.h>
#include "common_code.hpp"

std::vector<cv::Point3f>
generate_3d_calibration_points(const cv::Size &board_size,
                               float square_size) {
    std::vector<cv::Point3f> ret_v;
    for (int y = 1; y <= board_size.height; y++) {
        for (int x = 1; x <= board_size.width; x++) {
            ret_v.push_back(cv::Point3f(x * square_size, y * square_size, 0.0));
        }
    }
    CV_Assert(ret_v.size() == (long unsigned int) board_size.width * board_size.height);
    return ret_v;
}


bool
find_chessboard_corners(const cv::Mat &img, const cv::Size &board_size,
                        std::vector<cv::Point2f> &corner_points,
                        const char *wname) {
    CV_Assert(img.type() == CV_8UC3);
    bool was_found = false;

    was_found = cv::findChessboardCorners(img, board_size, corner_points);
    if (was_found) {
        cv::Mat aux;
        cv::cvtColor(img, aux, cv::COLOR_BGR2GRAY);
        cv::cornerSubPix(aux, corner_points, cv::Size(5, 5), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30,
                                          0.01));
    }
    return was_found;
}

void
save_calibration_parameters(cv::FileStorage &fs,
                            const cv::Size &camera_size,
                            float error,
                            const cv::Mat &camera_matrix_left,
                            const cv::Mat &camera_matrix_right,
                            const cv::Mat &dist_coeffs_left,
                            const cv::Mat &dist_coeffs_right,
                            const cv::Mat &rvec,
                            const cv::Mat &tvec,
                            const cv::Mat &E,
                            const cv::Mat &F
) {
    CV_Assert(fs.isOpened());
    CV_Assert(camera_matrix_left.type() == CV_64FC1 && camera_matrix_left.rows == 3 && camera_matrix_left.cols == 3);
    CV_Assert(camera_matrix_right.type() == CV_64FC1 && camera_matrix_right.rows == 3 && camera_matrix_right.cols == 3);
    CV_Assert(dist_coeffs_left.type() == CV_64FC1 && dist_coeffs_left.rows == 1 && dist_coeffs_left.cols == 5);
    CV_Assert(dist_coeffs_right.type() == CV_64FC1 && dist_coeffs_right.rows == 1 && dist_coeffs_right.cols == 5);
    // CV_Assert(rvec.type()==CV_64FC1 && rvec.rows==3 && rvec.cols==1);
    // CV_Assert(tvec.type()==CV_64FC1 && tvec.rows==3 && tvec.cols==1);

    fs << "image-width" << camera_size.width;
    fs << "image-height" << camera_size.height;
    fs << "error" << error;
    fs << "left-camera-matrix" << camera_matrix_left;
    fs << "left-distorsion-coefficients" << dist_coeffs_left;
    fs << "right-camera-matrix" << camera_matrix_right;
    fs << "right-distorsion-coefficients" << dist_coeffs_right;
    fs << "rvec" << rvec;
    fs << "tvec" << tvec;
    fs << "E" << E;
    fs << "F" << F;
    CV_Assert(fs.isOpened());
}


bool IsPathExist(const std::string &s) {
    struct stat buffer;
    return (stat(s.c_str(), &buffer) == 0);
}