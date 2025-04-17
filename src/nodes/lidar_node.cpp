#include "nodes/lidar_node.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace nodes {

    constexpr int NUM_BEAMS = 5; // počet paprskov na priemerovanie pre každý smer
    constexpr float INVALID_RANGE = NAN;

    LidarNode::LidarNode() : Node("lidar_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", rclcpp::SensorDataQoS(),
            std::bind(&LidarNode::on_lidar_msg, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "LidarNode initialized and listening to /bpc_prp_robot/lidar");
    }

    float LidarNode::average_range_at_angle(const sensor_msgs::msg::LaserScan::SharedPtr& msg, float angle_rad) {
        if (!msg || msg->ranges.empty()) return INVALID_RANGE;
        if (angle_rad < msg->angle_min || angle_rad > msg->angle_max) return INVALID_RANGE;

        int center_index = static_cast<int>((angle_rad - msg->angle_min) / msg->angle_increment);
        int half_window = NUM_BEAMS / 2;

        std::vector<float> valid_ranges;
        for (int i = center_index - half_window; i <= center_index + half_window; ++i) {
            if (i >= 0 && i < static_cast<int>(msg->ranges.size())) {
                float val = msg->ranges[i];
                if (std::isfinite(val) && val > 0.01f) {
                    valid_ranges.push_back(val);
                }
            }
        }

        if (valid_ranges.empty()) return INVALID_RANGE;

        float sum = std::accumulate(valid_ranges.begin(), valid_ranges.end(), 0.0f);
        return sum / valid_ranges.size();
    }

    void LidarNode::on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        forward_ = average_range_at_angle(msg, M_PI);    // predok
        left_ = average_range_at_angle(msg, -M_PI_2);     // ľavá strana
        right_ = average_range_at_angle(msg, M_PI_2);     // pravá strana
        back_ = average_range_at_angle(msg, 0.0f);        // vzadu
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
