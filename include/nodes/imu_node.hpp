#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>
#include <cmath>

#include "algorithms/planar_imu_integrator.hpp"

namespace nodes {

    enum class ImuNodeMode {
        CALIBRATE,
        INTEGRATE
    };

    class ImuNode : public rclcpp::Node {
    public:
        ImuNode();

        void setMode(ImuNodeMode setMode);
        ImuNodeMode getMode();
        float getIntegratedResults();
        void reset_imu();

        static float normalize_angle(float angle);  // teraz public

    private:
        void on_imu_msg(const sensor_msgs::msg::Imu::SharedPtr msg);
        void calibrate();
        void integrate();

        ImuNodeMode mode = ImuNodeMode::CALIBRATE;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber_;
        std::vector<float> gyro_calibration_samples_;
        algorithms::PlanarImuIntegrator planar_integrator_;  // opravené meno
    };

} // namespace nodes

// TODO: Hodnota chybejici steny je vetsi jak sirka bunky ale mensi jak inf, upravit podle toho program

