#include "nodes/lidar_node.hpp"
#include <cmath>
#include <numeric>
#include <algorithm>

namespace nodes
{
    constexpr int NUM_BEAMS = 5; // počet paprskov na priemerovanie pre každý smer
    constexpr float INVALID_RANGE = NAN;

    LidarNode::LidarNode() : Node("lidar_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/bpc_prp_robot/lidar", rclcpp::SensorDataQoS(),
            std::bind(&LidarNode::on_lidar_msg, this, std::placeholders::_1));
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
        forward_ = get_range_at_angle(msg, M_PI);
        left_ = get_range_at_angle(msg, -M_PI_2);        // 90°
        right_ = get_range_at_angle(msg, M_PI_2);      // -90°
        back_ = get_range_at_angle(msg, 0);          // 180°
        diagonal_left_ = get_range_at_angle(msg, -M_PI_4);   // 45°
        diagonal_right_ = get_range_at_angle(msg, M_PI_4); // -45°
        last_scan_ = msg;
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
    float LidarNode::get_diagonal_left_distance() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return diagonal_left_;
    }
    float LidarNode::get_diagonal_right_distance() const {
        std::lock_guard<std::mutex> lock(data_mutex_);
        return diagonal_right_;

    }

    std::vector<float> LidarNode::get_average_distances_around_angle(float center_angle, int num_beams_each_side) {
        std::vector<float> distances;
        std::lock_guard<std::mutex> lock(data_mutex_);

        if (!last_scan_ || last_scan_->ranges.empty()) return distances;

        const auto& msg = last_scan_;
        float increment = msg->angle_increment;
        int center_index = static_cast<int>((center_angle - msg->angle_min) / increment);
        int total_beams = static_cast<int>(msg->ranges.size());

        for (int i = -num_beams_each_side; i <= num_beams_each_side; ++i) {
            int idx = center_index + i;
            if (idx >= 0 && idx < total_beams) {
                float val = msg->ranges[idx];
                if (std::isfinite(val) && val > 0.01f) {
                    distances.push_back(val);
                }
            }
        }

        return distances;
    }

}
// namespace nodes
