#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <cmath>
#include <vector>
#include <cstdint>
#include "nodes/io_node.hpp"

// Struktura představující aktuální polohu robota.
struct Pose {
    double x;      // souřadnice x (v metrech)
    double y;      // souřadnice y (v metrech)
    double theta;  // orientace (ve stupních)
};

// Třída pro výpočet odometrie.
class Odometry {
public:
    Odometry(double wheel_diameter, double wheel_base, int pulses_per_rev_l_, int pulses_per_rev_r_);

    void update(int pulses_left, int pulses_right);
    Pose getPose() const;
    void resetPose();
    void setIoNode(const std::shared_ptr<nodes::IoNode>& node);


    // Rychlost ve směru vpřed (0.0–1.0), zatáčení ve stupních (-90 až +90)
    void drive(double forward_speed, double turn_rate_deg);

private:
    double wheel_diameter_;
    double wheel_base_;
    int pulses_per_rev_l_;
    int pulses_per_rev_r_;
    Pose current_pose_;

    std::shared_ptr<nodes::IoNode> io_node_ = std::make_shared<nodes::IoNode>();

    double normalizeAngle(double angle_deg);  // Nyní pracuje ve stupních
};

#endif // ODOMETRY_HPP
