// ultrasound_node.cpp
#include "nodes/ultrasound_node.hpp"

namespace nodes {

UltrasoundNode::UltrasoundNode() : Node("ultrasound_node") {
    front_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/ultrasound/front", 10,
        std::bind(&UltrasoundNode::front_callback, this, std::placeholders::_1));

    left_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/ultrasound/left", 10,
        std::bind(&UltrasoundNode::left_callback, this, std::placeholders::_1));

    right_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/ultrasound/right", 10,
        std::bind(&UltrasoundNode::right_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "UltrasoundNode initialized and listening to front, left, and right ultrasound topics.");
}

void UltrasoundNode::front_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    front_ = msg->range;
}

void UltrasoundNode::left_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    left_ = msg->range;
}

void UltrasoundNode::right_callback(const sensor_msgs::msg::Range::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    right_ = msg->range;
}

float UltrasoundNode::get_front_distance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return front_;
}

float UltrasoundNode::get_left_distance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return left_;
}

float UltrasoundNode::get_right_distance() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return right_;
}

} // namespace nodes
