#include "nodes/imu_node.hpp"

namespace nodes {

    ImuNode::ImuNode() : Node("imu_node") {
        imu_subscriber_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/bpc_prp_robot/imu", 10,
            std::bind(&ImuNode::on_imu_msg, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "IMU node initialized.");
    }

    void ImuNode::setMode(const ImuNodeMode setMode) {
        mode = setMode;
        RCLCPP_INFO(this->get_logger(), "IMU mode changed to: %s",
                    setMode == ImuNodeMode::CALIBRATE ? "CALIBRATE" : "INTEGRATE");
    }

    ImuNodeMode ImuNode::getMode() {
        return mode;
    }

    float ImuNode::getIntegratedResults() {
        return planar_integrator_.getYaw();
    }

    void ImuNode::reset_imu() {
        planar_integrator_.reset();
        gyro_calibration_samples_.clear();
    }

    void ImuNode::on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg) {
        float gyro_z = static_cast<float>(msg->angular_velocity.z);
        static rclcpp::Time last_time = msg->header.stamp;
        rclcpp::Time current_time = msg->header.stamp;

        double dt = (current_time - last_time).seconds();
        last_time = current_time;

        if (mode == ImuNodeMode::CALIBRATE) {
            gyro_calibration_samples_.push_back(gyro_z);
            if (gyro_calibration_samples_.size() >= 100) {  // napr. po 100 vzorkoch kalibrácia
                calibrate();
            }
        } else if (mode == ImuNodeMode::INTEGRATE) {
            integrate();
            planar_integrator_.update(gyro_z, dt);
        }
    }

    void ImuNode::calibrate() {
        planar_integrator_.setCalibration(gyro_calibration_samples_);
        gyro_calibration_samples_.clear();
        RCLCPP_INFO(this->get_logger(), "IMU gyroscope calibrated.");
    }

    void ImuNode::integrate() {
        // Momentálne žiadna extra logika, ale môžeš sem pridať ďalšie výpočty v budúcnosti
    }

} // namespace nodes
