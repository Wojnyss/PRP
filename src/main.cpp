#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "odometry.hpp"
#include "algorithms/pid.hpp"
#include <cmath>
#include <thread>

//Doplnit aby robot nemohl jet jinak nez +-90, +-180

//#define debug
#define PID

constexpr float RAD90 = M_PI_2;
constexpr float RAD180 = M_PI;
constexpr float FRONT_STOP_DIST = 0.2f;
constexpr float TURN_SPEED = 10.0f;
constexpr float FORWARD_SPEED_PID = 0.1f;
constexpr float FORWARD_SPEED_BB = 0.04f;
constexpr float SIDE_CLEAR_DIST = 0.35f;
constexpr float MIN_VALID_SIDE_DIST = 0.1f;
constexpr float ANGLE_TOLERANCE = 0.05f;

enum class State {
    FOLLOW_CORRIDOR,
    ROTATE
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto io_node = std::make_shared<nodes::IoNode>();
    auto lidar_node = std::make_shared<nodes::LidarNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>();

    executor->add_node(io_node);
    executor->add_node(lidar_node);
    executor->add_node(imu_node);

    std::thread executor_thread([&executor]() { executor->spin(); });

    rclcpp::Rate rate(50);
    const float dt = 1.0f / 50.0f;

    Odometry odom(0.07082, 0.12539, 570, 570.7);

#ifdef PID
    algorithms::Pid pid(6.0f, 0.68f, 0.013f);
    const float forward_speed = FORWARD_SPEED_PID;
    const float max_turn_angle = 10.0f;
#else
    const float forward_speed = FORWARD_SPEED_BB;
    const float max_turn_angle = 4.0f;
#endif

    State state = State::FOLLOW_CORRIDOR;
    float yaw_start = 0.0f;
    float target_yaw = 0.0f;
    float turn_direction = 0.0f;
    float planned_turn_angle = 0.0f;

    io_node->turn_on_leds({100, 100, 0, 100, 100, 0, 100, 100, 0, 0, 0, 0});
    RCLCPP_INFO(io_node->get_logger(), "Čekám na tlačítko 2 pro kalibraci IMU...");

    while (rclcpp::ok()) {
        if (io_node->get_button_pressed() == 2) {
            io_node->turn_on_leds({0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 0, 0});
            RCLCPP_INFO(io_node->get_logger(), "🧭 Kalibrace IMU spuštěna – nechte robota v klidu...");
            imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
            rclcpp::sleep_for(std::chrono::seconds(2));
            imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);
            RCLCPP_INFO(io_node->get_logger(), "✅ Kalibrace dokončena. Stiskněte tlačítko 1 pro spuštění.");
            break;
        }
        rate.sleep();
    }

    while (rclcpp::ok()) {
        if (io_node->get_button_pressed() == 1) {
            io_node->turn_on_leds({0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 0, 0});
            RCLCPP_INFO(io_node->get_logger(), "Start jízdy v koridoru...");
            break;
        }
        rate.sleep();
    }

    while (rclcpp::ok()) {
        float front = lidar_node->get_forward_distance();
        float left = lidar_node->get_left_distance();
        float right = lidar_node->get_right_distance();
        float back = lidar_node->get_back_distance();
        float yaw = imu_node->getIntegratedResults();

        RCLCPP_INFO(io_node->get_logger(), "🔄 IMU yaw: %.2f rad", yaw);

        if (io_node->get_button_pressed() == 0) {
            odom.drive(0.0, 0.0);
            RCLCPP_INFO(io_node->get_logger(), "🚩 Ukončuji program (tlačítko 0).");
            break;
        }

        if (state == State::FOLLOW_CORRIDOR) {
            if (std::isfinite(front) && front < FRONT_STOP_DIST) {
                odom.drive(0.0, 0.0);
                yaw_start = yaw;

                if (std::isfinite(left) && left > SIDE_CLEAR_DIST) {
                    planned_turn_angle = RAD90;
                    target_yaw = yaw_start + planned_turn_angle;
                    turn_direction = +TURN_SPEED;
                    RCLCPP_INFO(io_node->get_logger(), "⬅️ Volno vlevo – točím doleva.");
                } else if (std::isfinite(right) && right > SIDE_CLEAR_DIST) {
                    planned_turn_angle = RAD90;
                    target_yaw = yaw_start - planned_turn_angle;
                    turn_direction = -TURN_SPEED;
                    RCLCPP_INFO(io_node->get_logger(), "➡️ Volno vpravo – točím doprava.");
                } else {
                    planned_turn_angle = RAD180;
                    target_yaw = yaw_start + planned_turn_angle;
                    turn_direction = +TURN_SPEED;
                    RCLCPP_INFO(io_node->get_logger(), "🔄 Slepa ulička – otáčím o 180°.");
                }

                state = State::ROTATE;
                continue;
            }

            bool left_valid = std::isfinite(left) && left > MIN_VALID_SIDE_DIST;
            bool right_valid = std::isfinite(right) && right > MIN_VALID_SIDE_DIST;
            bool front_clear = !std::isfinite(front) || front >= FRONT_STOP_DIST;

            if (!left_valid && !right_valid && front_clear) {
                odom.drive(forward_speed, 0.0);
                RCLCPP_WARN(io_node->get_logger(), "🔹 Boční data chybí – jedu rovne");
                rate.sleep();
                continue;
            }

#ifdef PID
            float error = 0.0f;
            if (left_valid && right_valid) error = left - right;
            else if (left_valid) error = +0.2f;
            else if (right_valid) error = -0.2f;

            float correction = pid.step(error, dt);
            correction = std::clamp(correction, -max_turn_angle, max_turn_angle);
            odom.drive(forward_speed, correction);
#else
            float turn = 0.0f;
            if (left_valid && right_valid) {
                float error = left - right;
                if (error > 0.1f) turn = +max_turn_angle;
                else if (error < -0.1f) turn = -max_turn_angle;
            } else if (left_valid) turn = +max_turn_angle;
            else if (right_valid) turn = -max_turn_angle;

            odom.drive(forward_speed, turn);
#endif

        } else if (state == State::ROTATE) {
            float current_yaw = imu_node->getIntegratedResults();
            float diff = std::fabs(current_yaw - target_yaw);

            if (diff >= planned_turn_angle - ANGLE_TOLERANCE) {
                RCLCPP_INFO(io_node->get_logger(), "✅ Otočení dokončeno – pokračuji vpřed.");
                state = State::FOLLOW_CORRIDOR;
                continue;
            }

            odom.drive(0.0, turn_direction);
        }

        rate.sleep();
    }

    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}