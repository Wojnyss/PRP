#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "odometry.hpp"
#include "algorithms/pid.hpp"
#include <cmath>
#include <thread>
#include <chrono>
#include <ctime>

#define PID

constexpr float RAD90 = M_PI_2;
constexpr float RAD180 = M_PI;
constexpr float FRONT_STOP_DIST = 0.24f;
constexpr float TURN_SPEED = 10.0f;
constexpr float FORWARD_SPEED_PID = 0.1f;
constexpr float MIN_VALID_SIDE_DIST = 0.1f;
constexpr float ANGLE_TOLERANCE = 0.01f;
constexpr float WALL_MISSING_THRESHOLD = 0.35f;


enum class State {
    CALIBRATION,
    WAIT_FOR_START,
    CORRIDOR_FOLLOWING,
    TURNING
};

std::string get_time_string() {
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    char buffer[26];
    ctime_r(&now_time, buffer);
    buffer[24] = '\0'; // Remove trailing newline
    return std::string(buffer);
}

float round_to_nearest_rad90(float angle_rad) {
    int quadrant = static_cast<int>(std::round(angle_rad / RAD90));
    return quadrant * RAD90;
}

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
#endif

    State state = State::CALIBRATION;
    float yaw_start = 0.0f;
    float target_yaw = 0.0f;
    float turn_direction = 0.0f;
    float planned_turn_angle = 0.0f;

    io_node->turn_on_leds({100, 100, 0, 100, 100, 0, 100, 100, 0, 0, 0, 0});
    RCLCPP_INFO(io_node->get_logger(), "[%s] Čekám na tlačítko 2 pro kalibraci IMU...", get_time_string().c_str());

    while (rclcpp::ok()) {
        float yaw = imu_node->getIntegratedResults();
        RCLCPP_INFO(io_node->get_logger(), "[%s] Aktuální yaw: %.1f °", get_time_string().c_str(), yaw * 180.0f / M_PI);
        RCLCPP_INFO(io_node->get_logger(), "[%s] Stav: %s", get_time_string().c_str(),
            (state == State::CALIBRATION ? "CALIBRATION" :
             state == State::WAIT_FOR_START ? "WAIT_FOR_START" :
             state == State::CORRIDOR_FOLLOWING ? "CORRIDOR_FOLLOWING" :
             state == State::TURNING ? "TURNING" : "UNKNOWN"));

        switch (state) {
            case State::CALIBRATION:
                if (io_node->get_button_pressed() == 2) {
                    io_node->turn_on_leds({0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 0, 0});
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Kalibrace IMU spuštěna...", get_time_string().c_str());
                    imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
                    rclcpp::sleep_for(std::chrono::seconds(2));
                    imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Kalibrace dokončena. Stiskněte tlačítko 1 pro start.", get_time_string().c_str());
                    state = State::WAIT_FOR_START;
                }
                if (io_node->get_button_pressed() == 0) {
                    odom.drive(0.0, 0.0);
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Zastavení robota (tlačítko 0).", get_time_string().c_str());
                    break;
                }

                break;

            case State::WAIT_FOR_START:
                if (io_node->get_button_pressed() == 1) {
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Start...", get_time_string().c_str());
                    state = State::CORRIDOR_FOLLOWING;
                }
                if (io_node->get_button_pressed() == 0) {
                    odom.drive(0.0, 0.0);
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Zastavení robota (tlačítko 0).", get_time_string().c_str());
                    break;
                }

                break;

            case State::CORRIDOR_FOLLOWING: {
                float front = lidar_node->get_forward_distance();
                float left = lidar_node->get_left_distance();
                float right = lidar_node->get_right_distance();

                RCLCPP_INFO(io_node->get_logger(), "[%s] Lidar vzdálenosti - Front: %.2f, Left: %.2f, Right: %.2f",
                get_time_string().c_str(), front, left, right);

                bool front_blocked = std::isfinite(front) && front < FRONT_STOP_DIST;
                bool left_clear = std::isfinite(left) && left > WALL_MISSING_THRESHOLD;
                bool right_clear = std::isfinite(right) && right > WALL_MISSING_THRESHOLD;

                if (front_blocked && !(left_clear && right_clear)) {
                    yaw_start = imu_node->getIntegratedResults();
                    if (left_clear && !right_clear) {
                        planned_turn_angle = RAD90;
                        target_yaw = round_to_nearest_rad90(yaw_start + planned_turn_angle);
                        turn_direction = +TURN_SPEED;
                    } else if (right_clear && !left_clear) {
                        planned_turn_angle = RAD90;
                        target_yaw = round_to_nearest_rad90(yaw_start - planned_turn_angle);
                        turn_direction = -TURN_SPEED;
                    } else {
                        planned_turn_angle = RAD180;
                        target_yaw = round_to_nearest_rad90(yaw_start + planned_turn_angle);
                        turn_direction = +TURN_SPEED;
                    }
                    state = State::TURNING;
                    odom.drive(0.0, 0.0);
                    break;
                }

                bool left_valid = std::isfinite(left) && left > MIN_VALID_SIDE_DIST;
                bool right_valid = std::isfinite(right) && right > MIN_VALID_SIDE_DIST;

                bool front_clear = !std::isfinite(front) || front >= FRONT_STOP_DIST;
                if ((left_clear || right_clear) && front_clear) {
                    RCLCPP_WARN(io_node->get_logger(), "[%s] Chybí boční stěny, jedu rovně bez PID.", get_time_string().c_str());
                    odom.drive(forward_speed, 0.0f);
                    if (io_node->get_button_pressed() == 0) {
                        odom.drive(0.0, 0.0);
                        RCLCPP_INFO(io_node->get_logger(), "[%s] Zastavení robota (tlačítko 0).", get_time_string().c_str());
                        break;
                    }

                    break;
                }

                float error = 0.0f;
                if (left_valid && right_valid) error = left - right;
                else if (left_valid) error = +0.2f;
                else if (right_valid) error = -0.2f;

                float correction = pid.step(error, dt);
                correction = std::clamp(correction, -max_turn_angle, max_turn_angle);
                odom.drive(forward_speed, correction);

                if (io_node->get_button_pressed() == 0) {
                    odom.drive(0.0, 0.0);
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Zastavení robota (tlačítko 0).", get_time_string().c_str());
                    break;
                }

                break;
            }

            case State::TURNING: {
                float current_yaw = imu_node->getIntegratedResults();
                float delta_yaw = nodes::ImuNode::normalize_angle(current_yaw - yaw_start);
                float yaw_error = nodes::ImuNode::normalize_angle(target_yaw - current_yaw);
                float abs_delta = std::fabs(delta_yaw);
                RCLCPP_INFO(io_node->get_logger(), "[%s] Otáčení: cíl = %.1f ° | aktuální = %.1f ° | odchylka = %.1f °",
                    get_time_string().c_str(), target_yaw * 180.0f / M_PI, current_yaw * 180.0f / M_PI, yaw_error * 180.0f / M_PI);
                if (abs_delta >= planned_turn_angle - ANGLE_TOLERANCE) {
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Otáčka dokončena. Pokračuji v jízdě.", get_time_string().c_str());
                    state = State::CORRIDOR_FOLLOWING;
                } else {
                    odom.drive(0.0, turn_direction);
                }


                if (io_node->get_button_pressed() == 0) {
                    odom.drive(0.0, 0.0);
                    RCLCPP_INFO(io_node->get_logger(), "[%s] Zastavení robota (tlačítko 0).", get_time_string().c_str());
                    break;
                }

                break;
            }
        }



        rate.sleep();
    }

    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}


