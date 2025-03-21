#include "odometry.hpp"

Odometry::Odometry(double wheel_diameter, double wheel_base, int pulses_per_rev_l_, int pulses_per_rev_r_)
    : wheel_diameter_(wheel_diameter),
      wheel_base_(wheel_base),
      pulses_per_rev_l_(pulses_per_rev_l_),
      pulses_per_rev_r_(pulses_per_rev_r_) {
    current_pose_ = {0.0, 0.0, 0.0};
}

double Odometry::normalizeAngle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

void Odometry::update(int pulses_left, int pulses_right) {
    double wheel_circumference = wheel_diameter_ * M_PI;
    double left_distance = (static_cast<double>(pulses_left) / pulses_per_rev_l_) * wheel_circumference;
    double right_distance = (static_cast<double>(pulses_right) / pulses_per_rev_r_) * wheel_circumference;

    double distance = (left_distance + right_distance) / 2.0;
    double delta_theta = (right_distance - left_distance) / wheel_base_;

    double theta_mid = current_pose_.theta + delta_theta / 2.0;
    current_pose_.x += distance * std::cos(theta_mid);
    current_pose_.y += distance * std::sin(theta_mid);
    current_pose_.theta = normalizeAngle(current_pose_.theta + delta_theta);
}

Pose Odometry::getPose() const {
    return current_pose_;
}

void Odometry::resetPose() {
    current_pose_ = {0.0, 0.0, 0.0};
}

void Odometry::drive(double forward_speed, double turn_rate_deg) {
    // forward_speed: 0.0–1.0
    // turn_rate_deg: -90 až 90 stupňů
    double turn_clamped = std::clamp(turn_rate_deg, -90.0, 90.0);
    double forward_clamped = std::clamp(forward_speed, 0.0, 1.0);

    uint8_t base_speed = static_cast<uint8_t>(127 + (128 * forward_clamped));
    int8_t delta = static_cast<int8_t>((turn_clamped / 90.0) * 30);  // max ±30

    uint8_t left_motor = std::clamp(base_speed - delta, 0, 255);
    uint8_t right_motor = std::clamp(base_speed + delta, 0, 255);

    io_node_->set_motor_speeds({left_motor, right_motor});
}
