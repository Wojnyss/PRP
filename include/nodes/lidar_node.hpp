#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mutex>

namespace nodes {

    class LidarNode : public rclcpp::Node {
    public:
        LidarNode();

        float get_forward_distance() const;
        float get_back_distance() const;
        float get_left_distance() const;
        float get_right_distance() const;

    private:
        void on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        float average_range_at_angle(const sensor_msgs::msg::LaserScan::SharedPtr& msg, float angle_rad);

        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
        mutable std::mutex data_mutex_;

        float forward_{NAN};
        float back_{NAN};
        float left_{NAN};
        float right_{NAN};
    };

} // namespace nodes
