#ifndef CAMERA_NODE_HPP
#define CAMERA_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include "aruco_detector.hpp"

class CameraNode : public rclcpp::Node, public std::enable_shared_from_this<CameraNode> {
public:
    static std::shared_ptr<CameraNode> create();

private:
    CameraNode();
    void initTransport();
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    algorithms::ArucoDetector detector_;
    std::vector<algorithms::ArucoDetector::Aruco> last_detected_;
};

#endif // CAMERA_NODE_HPP
