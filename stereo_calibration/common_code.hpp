#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

std::vector<cv::Point3f> generate_3d_calibration_points(const cv::Size &board_size,
                                                        float square_size);

bool find_chessboard_corners(const cv::Mat &img, const cv::Size &board_size,
                             std::vector<cv::Point2f> &corner_points,
                             const char *wname = nullptr);

void save_calibration_parameters(cv::FileStorage &fs,
                                 const cv::Size &camera_size,
                                 float error,
                                 const cv::Mat &camera_matrix_left,
                                 const cv::Mat &camera_matrix_right,
                                 const cv::Mat &dist_coeffs_left,
                                 const cv::Mat &dist_coeffs_right,
                                 const cv::Mat &rvec,
                                 const cv::Mat &tvec,
                                 const cv::Mat &E,
                                 const cv::Mat &F);

bool IsPathExist(const std::string &s);