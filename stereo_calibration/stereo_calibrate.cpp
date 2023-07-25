#include <iostream>
#include <exception>
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d.hpp>

#include "common_code.hpp"
#include "dirreader.h"

const cv::String keys =
        "{help h usage ? |      | print this message.}"
        "{@input        |<none>| input images directory.}"
        "{@output        |<none>| filename for output file.}";

int
main(int argc, char *const *argv) {
    int retCode = EXIT_SUCCESS;

    try {
        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Stereo Calibration");
        if (parser.has("help")) {
            parser.printMessage();
            return EXIT_SUCCESS;
        }
        if (argc != 3) {
            std::cerr
                    << "Usage: \n\t ./stereo_calibrate image_directory calibration_result.yml"
                    << std::endl;
            return EXIT_FAILURE;
        }
        int cols = 8;
        int rows = 6;
        std::string files_dir = parser.get<cv::String>("@input");
        std::string output_fname = parser.get<cv::String>("@output");

        if (!IsPathExist(files_dir)) {
            std::cerr << "Directory of images doesn't exist <" << files_dir << ">" << std::endl;
            return EXIT_FAILURE;
        }

        const cv::Size board_size = cv::Size(cols - 1, rows - 1);
        float square_size = 0.02875;
        std::string wname = "Stereo";
        cv::namedWindow(wname);
        std::vector<std::vector<cv::Point3f>> _3d_points;
        std::vector<std::vector<cv::Point2f>> _2d_points_left;
        std::vector<std::vector<cv::Point2f>> _2d_points_right;
        cv::Size camera_size = cv::Size(0, 0);
        cv::Size camera_size_old = cv::Size(0, 0);
        DirReader Dir;
        auto files = Dir.read(files_dir, ".jpg", DirReader::Params(true));
        std::vector<cv::Point3f> _3d_corners = generate_3d_calibration_points(board_size, square_size);
        for (auto & file : files) {
            cv::Mat img = cv::imread(file);
            cv::Mat img_left = img(cv::Range(0, img.rows), cv::Range(0, round(img.cols / 2)));
            camera_size = cv::Size(img_left.cols, img_left.rows);
            cv::Mat img_right = img(cv::Range(0, img.rows), cv::Range(round(img.cols / 2), img.cols));
            camera_size = cv::Size(img_left.cols, img_left.rows);
            if ((camera_size_old != cv::Size(0, 0)) && camera_size_old != camera_size) {
                std::cerr << "Images must have the same dimensions" << std::endl;
                return EXIT_FAILURE;
            }
            camera_size_old = camera_size;
            std::vector<cv::Point2f> corners_left;
            std::vector<cv::Point2f> corners_right;
            if (find_chessboard_corners(img_left, board_size, corners_left, wname.c_str()) &&
                find_chessboard_corners(img_right, board_size, corners_right, wname.c_str())) {
                _2d_points_left.push_back(corners_left);
                _2d_points_right.push_back(corners_right);
                _3d_points.push_back(_3d_corners);
            }
        }
        cv::Mat cam_mat_left, cam_mat_right, dist_coef_left, dist_coef_right, R, T, E, F;
        float error = cv::stereoCalibrate(_3d_points, _2d_points_left, _2d_points_right, cam_mat_left, cam_mat_right,
                                          dist_coef_left, dist_coef_right, camera_size, R, T, E, F, 0.0,
                                          cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 60, 1e-6));
        auto fs = cv::FileStorage();
        fs.open(output_fname, cv::FileStorage::WRITE);
        save_calibration_parameters(fs, camera_size, error, cam_mat_left, dist_coef_left, cam_mat_right,
                                    dist_coef_right, R, T, E, F);
    }
    catch (std::exception &e) {
        std::cerr << "Exception caught: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}