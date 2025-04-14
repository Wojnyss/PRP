// ultrasound_node.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <mutex>

namespace nodes {

class UltrasoundNode : public rclcpp::Node {
public:
    UltrasoundNode();

    float get_front_distance() const;
    float get_left_distance() const;
    float get_right_distance() const;

private:
    void front_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void left_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void right_callback(const sensor_msgs::msg::Range::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr front_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr left_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr right_sub_;

    mutable std::mutex mutex_;
    float front_{NAN};
    float left_{NAN};
    float right_{NAN};
};

} // namespace nodes
