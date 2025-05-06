#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <mutex>
#include <limits>
#include <cmath>

namespace nodes {

    class LidarNode : public rclcpp::Node {
    public:
        LidarNode();

        float get_forward_distance() const;
        float get_back_distance() const;
        float get_left_distance() const;
        float get_right_distance() const;
        float get_diagonal_left_distance() const;
        float get_diagonal_right_distance() const;
        // std::vector<float>get_average_distances_around_angle(float center_angle, int num_beams_each_side);
        float get_average_range_at_angle(float angle_rad, int num_beams_each_side);

    private:
        void on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg);
        float get_range_at_angle(const sensor_msgs::msg::LaserScan::SharedPtr& msg, float angle_rad);
        sensor_msgs::msg::LaserScan::SharedPtr last_scan_;



        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;

        mutable std::mutex data_mutex_;
        float forward_ = std::numeric_limits<float>::quiet_NaN();
        float back_ = std::numeric_limits<float>::quiet_NaN();
        float left_ = std::numeric_limits<float>::quiet_NaN();
        float right_ = std::numeric_limits<float>::quiet_NaN();
        float diagonal_left_ = std::numeric_limits<float>::quiet_NaN();
        float diagonal_right_ = std::numeric_limits<float>::quiet_NaN();

    };

} // namespace nodes
