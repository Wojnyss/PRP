#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "aruco_detector.hpp"

using std::placeholders::_1;

class CameraNode : public rclcpp::Node {
public:
    explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("camera_node", options), detector_(), it_(std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){}))
    {
        image_sub_ = it_.subscribe("/camera/image_raw", 10, std::bind(&CameraNode::imageCallback, this, _1));
        image_pub_ = it_.advertise("/camera/image_marked", 10);
    }

private:
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        auto detected = detector_.detect(cv_ptr->image);
        last_detected_ = detected;

        detector_.drawDetected(cv_ptr->image, detected);

        image_pub_.publish(cv_ptr->toImageMsg());
    }

    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    algorithms::ArucoDetector detector_;
    std::vector<algorithms::ArucoDetector::Aruco> last_detected_;
};