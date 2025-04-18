#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <vector>
#include <iostream>
#include <optional>

namespace algorithms {

    class ArucoDetector {
    public:

        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;
        };

        ArucoDetector();

        // Detect markers and return their IDs and corners
        std::vector<Aruco> detect(const cv::Mat& frame);

        // Draw markers on a frame
        void drawDetected(cv::Mat& frame, const std::vector<Aruco>& arucos);

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };

}
