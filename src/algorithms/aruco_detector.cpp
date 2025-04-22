#include "algorithms/aruco_detector.hpp"
#include <iostream>

namespace algorithms {

    ArucoDetector::ArucoDetector() {
        dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    }

    std::vector<ArucoDetector::Aruco> ArucoDetector::detect(const cv::Mat& frame) {
        std::vector<Aruco> arucos;
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        cv::aruco::detectMarkers(frame, dictionary_, corners, ids);

        if (!ids.empty()) {
            std::cout << "Detected ArUco IDs: ";
            for (size_t i = 0; i < ids.size(); ++i) {
                std::cout << ids[i] << " ";
                arucos.push_back({ids[i], corners[i]});
            }
            std::cout << std::endl;
        }

        return arucos;
    }

    void ArucoDetector::drawDetected(cv::Mat& frame, const std::vector<Aruco>& arucos) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;

        for (const auto& a : arucos) {
            ids.push_back(a.id);
            corners.push_back(a.corners);
        }

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);
        }
    }

}
