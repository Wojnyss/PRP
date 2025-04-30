#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "algorithms/aruco_detector.hpp"

class CameraNode : public rclcpp::Node, public std::enable_shared_from_this<CameraNode> {
public:
    static std::shared_ptr<CameraNode> create();
    CameraNode();
    void initTransport();
    void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    const std::vector<algorithms::ArucoDetector::Aruco>& get_last_detected() const {
        return last_detected_;
    }


private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;
    algorithms::ArucoDetector detector_;
    std::vector<algorithms::ArucoDetector::Aruco> last_detected_;
    cv::Mat last_frame_;
};


#endif // CAMERA_NODE_HPP