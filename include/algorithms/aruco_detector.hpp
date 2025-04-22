#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace algorithms {

    class ArucoDetector {
    public:
        struct Aruco {
            int id;
            std::vector<cv::Point2f> corners;
        };

        ArucoDetector();
        ~ArucoDetector() = default;

        std::vector<Aruco> detect(const cv::Mat& frame);
        void drawDetected(cv::Mat& frame, const std::vector<Aruco>& arucos);

    private:
        cv::Ptr<cv::aruco::Dictionary> dictionary_;
    };
}
