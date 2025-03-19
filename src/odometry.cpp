#include "odometry.hpp"

// Konstruktor – inicializuje parametry odometrie.
Odometry::Odometry(double wheel_diameter, double wheel_base, int pulses_per_rev)
    : wheel_diameter_(wheel_diameter),
      wheel_base_(wheel_base),
      pulses_per_rev_(pulses_per_rev) {
    current_pose_ = {0.0, 0.0, 0.0};
}

// Funkce pro normalizaci úhlu do intervalu [-pi, pi].
double Odometry::normalizeAngle(double angle) {
    while (angle > M_PI)
        angle -= 2.0 * M_PI;
    while (angle < -M_PI)
        angle += 2.0 * M_PI;
    return angle;
}

// Aktualizace polohy na základě počtu pulzů z levého a pravého kola.
void Odometry::update(int pulses_left, int pulses_right) {
    // Obvod kola = průměr * pi
    double wheel_circumference = wheel_diameter_ * M_PI;
    
    // Vzdálenost, kterou ušlo kolo
    double left_distance = (static_cast<double>(pulses_left) / pulses_per_rev_) * wheel_circumference;
    double right_distance = (static_cast<double>(pulses_right) / pulses_per_rev_) * wheel_circumference;

    // Průměrná vzdálenost a změna úhlu
    double distance = (left_distance + right_distance) / 2.0;
    double delta_theta = (right_distance - left_distance) / wheel_base_;

    // Aktualizace pozice s využitím přibližného středního úhlu
    double theta_mid = current_pose_.theta + delta_theta / 2.0;
    current_pose_.x += distance * std::cos(theta_mid);
    current_pose_.y += distance * std::sin(theta_mid);
    current_pose_.theta = normalizeAngle(current_pose_.theta + delta_theta);
}

// Vrací aktuální polohu.
Pose Odometry::getPose() const {
    return current_pose_;
}

// Resetuje polohu na výchozí hodnoty.
void Odometry::resetPose() {
    current_pose_ = {0.0, 0.0, 0.0};
}
