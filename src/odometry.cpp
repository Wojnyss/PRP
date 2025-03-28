#include "odometry.hpp"

Odometry::Odometry(double wheel_diameter, double wheel_base, int pulses_per_rev_l_, int pulses_per_rev_r_)
    : wheel_diameter_(wheel_diameter),
      wheel_base_(wheel_base),
      pulses_per_rev_l_(pulses_per_rev_l_),
      pulses_per_rev_r_(pulses_per_rev_r_) {
    current_pose_ = {0.0, 0.0, 0.0};
}

double Odometry::normalizeAngle(double angle_deg) {
    while (angle_deg > 180.0)
        angle_deg -= 360.0;
    while (angle_deg < -180.0)
        angle_deg += 360.0;
    return angle_deg;
}

void Odometry::update(int pulses_left, int pulses_right) {
    double wheel_circumference = wheel_diameter_ * M_PI;
    double left_distance = (static_cast<double>(pulses_left) / pulses_per_rev_l_) * wheel_circumference;
    double right_distance = (static_cast<double>(pulses_right) / pulses_per_rev_r_) * wheel_circumference;

    double distance = (left_distance + right_distance) / 2.0;
    double delta_theta_rad = (right_distance - left_distance) / wheel_base_;
    double delta_theta_deg = delta_theta_rad * (180.0 / M_PI); // převod na stupně

    double theta_mid_deg = current_pose_.theta + delta_theta_deg / 2.0;
    double theta_mid_rad = theta_mid_deg * (M_PI / 180.0);

    current_pose_.x += distance * std::cos(theta_mid_rad);
    current_pose_.y += distance * std::sin(theta_mid_rad);
    current_pose_.theta = normalizeAngle(current_pose_.theta + delta_theta_deg);
}

Pose Odometry::getPose() const {
    return current_pose_;
}

void Odometry::resetPose() {
    current_pose_ = {0.0, 0.0, 0.0};
}

void Odometry::drive(double forward_speed, double turn_rate_deg) {
    double turn_clamped = std::clamp(turn_rate_deg, -90.0, 90.0);
    double forward_clamped = std::clamp(forward_speed, -1.0, 1.0);

    // Korekce rychlosti na základě počtu pulsů
    double left_correction = static_cast<double>(pulses_per_rev_r_) / pulses_per_rev_l_;
    double right_correction = static_cast<double>(pulses_per_rev_l_) / pulses_per_rev_r_;

    int16_t base = 127;
    double max_offset = 128.0;

    int16_t left_pwm = static_cast<int16_t>(
        (base + forward_clamped * max_offset - (turn_clamped / 90.0) * max_offset) * left_correction
    );

    int16_t right_pwm = static_cast<int16_t>(
        (base + forward_clamped * max_offset + (turn_clamped / 90.0) * max_offset) * right_correction
    );

    uint8_t left_motor = static_cast<uint8_t>(std::clamp(left_pwm, static_cast<int16_t>(0), static_cast<int16_t>(255)));
    uint8_t right_motor = static_cast<uint8_t>(std::clamp(right_pwm, static_cast<int16_t>(0), static_cast<int16_t>(255)));

   // RCLCPP_INFO(io_node_->get_logger(), "PWM OUT: L=%d R=%d | L_corr=%.2f R_corr=%.2f", left_motor, right_motor, left_correction, right_correction);

    io_node_->set_motor_speeds({left_motor, right_motor});
}


