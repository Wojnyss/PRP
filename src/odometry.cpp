#include "odometry.hpp"

// #define NEW_DRIVE
#define OLD_DRIVE


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

uint8_t last_left_motor_ = 127;
uint8_t last_right_motor_ = 127;
constexpr double TURN_SCALE = 0.9; // sníží dopad otočky


#ifdef NEW_DRIVE
void Odometry::drive(double forward_speed, double turn_rate_deg) {
    double turn_clamped = std::clamp(turn_rate_deg, -90.0, 90.0);
    double forward_clamped = std::clamp(forward_speed, -1.0, 1.0);

    double left_correction = static_cast<double>(pulses_per_rev_r_) / pulses_per_rev_l_;
    double right_correction = static_cast<double>(pulses_per_rev_l_) / pulses_per_rev_r_;

    double turn_norm = (turn_clamped / 90.0) * TURN_SCALE;
    double left_speed = std::clamp(forward_clamped - turn_norm, -1.0, 1.0);
    double right_speed = std::clamp(forward_clamped + turn_norm, -1.0, 1.0);

    auto speed_to_pwm = [](double speed) -> uint8_t {
        if (std::abs(speed) < 0.01) return 127;

        const double min_effective_pwm = 30.0;
        double pwm = 127.0 + speed * (128.0 - min_effective_pwm);

        if (speed > 0)
            pwm = std::max(pwm, 127.0 + min_effective_pwm);
        else
            pwm = std::min(pwm, 127.0 - min_effective_pwm);

        return static_cast<uint8_t>(std::clamp(pwm, 0.0, 255.0));
    };

    uint8_t left_motor = speed_to_pwm(left_speed * left_correction);
    uint8_t right_motor = speed_to_pwm(right_speed * right_correction);

    // 🛑 DEAD-TIME OCHRANA: pokud změna směru → zastav krátce oba motory
    auto sign = [](uint8_t pwm) -> int {
        if (pwm > 127) return 1;
        if (pwm < 127) return -1;
        return 0;
    };

    bool left_reversed = sign(left_motor) != sign(last_left_motor_) && sign(left_motor) != 0;
    bool right_reversed = sign(right_motor) != sign(last_right_motor_) && sign(right_motor) != 0;

    if (left_reversed || right_reversed) {
        io_node_->set_motor_speeds({127, 127}); // stop
        std::this_thread::sleep_for(std::chrono::milliseconds(20)); // dead-time delay
    }

    io_node_->set_motor_speeds({left_motor, right_motor});

    last_left_motor_ = left_motor;
    last_right_motor_ = right_motor;
}
#endif

#ifdef OLD_DRIVE
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
#endif


void Odometry::setIoNode(const std::shared_ptr<nodes::IoNode>& node) {
    io_node_ = node;
}
