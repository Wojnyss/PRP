#include "nodes/camera_node.hpp"

CameraNode::CameraNode() : Node("camera_node"), detector_() {}

std::shared_ptr<CameraNode> CameraNode::create() {
    struct MakeSharedEnabler : public CameraNode {
        MakeSharedEnabler() : CameraNode() {}
    };
    auto node = std::make_shared<MakeSharedEnabler>();
    node->initTransport();
    return node;
}

void CameraNode::initTransport() {
    image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/bpc_prp_robot/camera/compressed", 10,
        std::bind(&CameraNode::imageCallback, this, std::placeholders::_1)
    );
}

void CameraNode::imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
        cv::Mat raw_data(msg->data);
        last_frame_ = cv::imdecode(raw_data, cv::IMREAD_COLOR);

        if (last_frame_.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty image frame.");
            return;
        }

        auto detected = detector_.detect(last_frame_);
        last_detected_ = detected;

        // if (!detected.empty()) {
        // std::cout << "Detected ArUco IDs: ";
        //     for (const auto& marker : detected) {
        //         std::cout << marker.id << " ";
        //     }
        //     std::cout << std::endl;
        // }

    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "Image decode exception: %s", e.what());
    }
}
