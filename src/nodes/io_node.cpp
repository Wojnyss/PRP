//#include "nodes/io_node.hpp"

#include <nodes/io_node.hpp>
#include <rclcpp/node.hpp>

float l_max = 280.0f;
float r_max = 400.0f;

namespace nodes{
    IoNode::IoNode() : Node("io_node") {
        button_subscriber_ = this->create_subscription<std_msgs::msg::UInt8>(
            "/bpc_prp_robot/buttons", 10, std::bind(&IoNode::on_button_callback, this, std::placeholders::_1));
        encoder_subscriber_ = this->create_subscription<std_msgs::msg::UInt32MultiArray>(
            "/bpc_prp_robot/encoders", 10, [this](const std_msgs::msg::UInt32MultiArray::SharedPtr msg) { on_encoder_callback(msg); });
        rgb_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/rgb_leds", 10);
        RCLCPP_INFO(this->get_logger(), "Node setup complete");
        motor_publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>("/bpc_prp_robot/set_motor_speeds", 10);
        RCLCPP_INFO(this->get_logger(), "Node setup complete");
    }

    int IoNode::get_button_pressed() const {
        return button_pressed_;
    }

    std::array<uint32_t, 2> IoNode::get_encoder_values() const {
        std::lock_guard<std::mutex> lock(encoder_mutex_);
        return encoder_values_;
    }

    void IoNode::turn_on_leds(const std::vector<uint8_t>& values) {
        if (values.size() != 12) {
            RCLCPP_WARN(this->get_logger(), "Wrong number of values provided");
            return;
        }
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = values;
        rgb_publisher_->publish(msg);
       // RCLCPP_INFO(this->get_logger(), "Turned on");

    }

    void IoNode::on_button_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
        button_pressed_ = msg->data;
        std::cout << "Button "<<button_pressed_<<" pressed" << std::endl;
        RCLCPP_INFO(this->get_logger(),"Zmacknute tlacitko: %d", button_pressed_);
    }

    void IoNode::on_encoder_callback(const std_msgs::msg::UInt32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            std::lock_guard<std::mutex> lock(encoder_mutex_);
            encoder_values_ = {msg->data[0], msg->data[1]};
            // RCLCPP_INFO(this->get_logger(), "Encoder Values: [%d, %d]", encoder_values_[0], encoder_values_[1]);
            //std::cout << "Hodnoty enkoderu "<<encoder_values_[0]<<", " <<encoder_values_[1]<< std::endl;
        } else {
            RCLCPP_WARN(this->get_logger(), "Received an invalid encoder message.");
        }
    }

    void IoNode::set_motor_speeds(const std::vector<uint8_t>& values) {
        if (values.size() != 2) {
            RCLCPP_WARN(this->get_logger(), "Wrong number of values provided");
            return;
        }
        std_msgs::msg::UInt8MultiArray msg;
        msg.data = values;
        motor_publisher_->publish(msg);
        // RCLCPP_INFO(this->get_logger(), "Turned on");

    }

    LineNode::LineNode() : Node("line_node") {
    line_sensors_subscriber_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
        "/bpc_prp_robot/line_sensors", 10,
        std::bind(&LineNode::on_line_sensors_msg, this, std::placeholders::_1));
    }

    LineNode::~LineNode() {}

    void LineNode::on_line_sensors_msg(std::shared_ptr<std_msgs::msg::UInt16MultiArray> msg) {
        if (msg->data.size() < 2) {
            RCLCPP_WARN(this->get_logger(), "Received line sensor message with insufficient data");
            return;
        }

        float left_value = static_cast<float>(msg->data[1]);
        float right_value = static_cast<float>(msg->data[0]);

        //std::cout << "Left value: " << left_value << std::endl;
        // std::cout << "Right value: " << right_value << std::endl;


        float l_norm = left_value / l_max;  // Normalizace hodnot (levý senzor má max 200)
        float r_norm = right_value / r_max;  // Normalizace hodnot (pravý senzor má max 50)

        DiscreteLinePose discrete_pose = estimate_descrete_line_pose(l_norm, r_norm);
        last_discrete_pose_ = discrete_pose;
        last_continuous_line_pose = estimate_continuous_line_pose(left_value, right_value);

        //RCLCPP_INFO(this->get_logger(), "Discrete Pose: %d", static_cast<int>(discrete_pose));
    }

    float LineNode::estimate_continuous_line_pose(float left_value, float right_value) {
        float total = left_value + right_value;
        if (total == 0) return 0.0f; // Pokud žádná čára není detekována
        return (right_value / r_max - left_value / l_max); // Normalizovaná pozice upravená podle rozsahu senzorů
    }

    DiscreteLinePose LineNode::estimate_descrete_line_pose(float l_norm, float r_norm) {
        const float threshold_l = 0.1f; // Práh detekce
        const float threshold_r = 0.1f;
        bool left_detected = l_norm > threshold_l;
        bool right_detected = r_norm > threshold_r;

        if (left_detected && right_detected) return DiscreteLinePose::LineBoth;
        if (left_detected) return DiscreteLinePose::LineOnLeft;
        if (right_detected) return DiscreteLinePose::LineOnRight;
        return DiscreteLinePose::LineNone;
    }

    DiscreteLinePose LineNode::get_discrete_line_pose() const {
        return last_discrete_pose_;
    }


    float LineNode::get_continuous_line_pose() const {
        return last_continuous_line_pose;
    }

    /*LidarNode::LidarNode() : Node("lidar_node") {
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&LidarNode::on_lidar_msg, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "LidarNode initialized and listening to /scan");
    }

    float LidarNode::get_range_at_angle(const sensor_msgs::msg::LaserScan::SharedPtr& msg, float angle_rad) {
        // Úprava: omezit na rozsah skeneru
        if (angle_rad < msg->angle_min || angle_rad > msg->angle_max - msg->angle_increment) {
            RCLCPP_WARN(this->get_logger(), "Úhel %.2f mimo rozsah [%.2f, %.2f]", angle_rad, msg->angle_min, msg->angle_max);
            return NAN;
        }

        int index = static_cast<int>((angle_rad - msg->angle_min) / msg->angle_increment);

        if (index >= 0 && index < static_cast<int>(msg->ranges.size())) {
            float r = msg->ranges[index];
            return std::isfinite(r) ? r : NAN;
        }

        return NAN;
    }

    void LidarNode::on_lidar_msg(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        forward_ = get_range_at_angle(msg, 0.0f);
        left_ = get_range_at_angle(msg, 1.57);
        right_ = get_range_at_angle(msg, -1.57);
        back_ = get_range_at_angle(msg, 3.13);
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
    }*/

}



