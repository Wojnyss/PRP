#include "nodes/lidar_node.hpp"
#include <cmath>

namespace nodes {

    LidarNode::LidarNode() : Node("lidar_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", rclcpp::SensorDataQoS(),
            std::bind(&LidarNode::on_lidar_msg, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "LidarNode initialized and listening to /scan");
    }

    float LidarNode::get_range_at_angle(const sensor_msgs::msg::LaserScan::SharedPtr& msg, float angle_rad) {
        if (!msg || msg->ranges.empty()) return NAN;

        if (angle_rad < msg->angle_min || angle_rad > msg->angle_max) return NAN;

        int index = static_cast<int>((angle_rad - msg->angle_min) / msg->angle_increment);

        if (index < 0 || index >= static_cast<int>(msg->ranges.size())) return NAN;

        float value = msg->ranges[index];
        return (std::isfinite(value) && value > 0.01f) ? value : NAN;
    }

    void LidarNode::on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        forward_ = get_range_at_angle(msg, 3.14f);
        left_ = get_range_at_angle(msg, -1.57f);   // ~90°
        right_ = get_range_at_angle(msg, 1.57f); // ~-90°
        back_ = get_range_at_angle(msg, 0.0f);   // ~180°
    }

    float LidarNode::get_forward_distance() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return forward_;
    }
    float LidarNode::get_back_distance() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return back_;
    }
    float LidarNode::get_left_distance() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return left_;
    }
    float LidarNode::get_right_distance() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return right_;
    }

} // namespace nodes
