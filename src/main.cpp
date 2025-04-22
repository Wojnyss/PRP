#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "odometry.hpp"
#include "algorithms/pid.hpp"
#include <thread>
#include <chrono>
#include <cmath>

// #define LINE
#define RING

#ifdef LINE
constexpr float FRONT_STOP_DIST = 0.35f;
constexpr float FORWARD_SPEED = 0.3f;
#endif

#ifdef RING
constexpr float FRONT_STOP_DIST = 0.25f;
constexpr float FORWARD_SPEED = 0.2f;
#endif

constexpr float MAX_TURN_ANGLE = 10.0f;
constexpr float MIN_VALID_SIDE_DIST = 0.1f;
constexpr float TURN_RADIUS = 0.2f;
constexpr float CELL_WIDTH = 0.40f;
constexpr float ANGLE_TOLERANCE = 0.2f;

float round_to_nearest_rad90(float angle_rad) {
    int quadrant = static_cast<int>(std::round(angle_rad / M_PI_2));
    return quadrant * M_PI_2;
}

enum class RingState {
    FOLLOW_WALL,
    INITIATE_TURN,
    TURNING
};

#if defined(LINE) || defined(RING)
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
    algorithms::Pid pid(10.f, 2.2f, 5.3f);
    algorithms::Pid pid_turn(5.0f, 0.0f, 2.0f);

#ifdef RING
    RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Čekám na tlačítko 2 pro kalibraci IMU...");
    while (rclcpp::ok() && io_node->get_button_pressed() != 2) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Kalibrace spuštěna...");
    imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
    rclcpp::sleep_for(std::chrono::seconds(2));
    imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);
#endif

    RCLCPP_INFO(io_node->get_logger(), "[MODE] Čekám na tlačítko 1 pro start...");
    while (rclcpp::ok() && io_node->get_button_pressed() != 1) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }

    RCLCPP_INFO(io_node->get_logger(), "[MODE] Start...");

    float turn_start_yaw = 0.0f;
    float target_yaw = 0.0f;
    RingState ring_state = RingState::FOLLOW_WALL;

    while (rclcpp::ok()) {
#ifdef LINE
        float front = lidar_node->get_forward_distance();

        std::vector<float> left_beams = lidar_node->get_average_distances_around_angle(-M_PI_2, 5);
        std::vector<float> right_beams = lidar_node->get_average_distances_around_angle(M_PI_2, 5);

        float left = std::accumulate(left_beams.begin(), left_beams.end(), 0.0f) / std::max(1ul, left_beams.size());
        float right = std::accumulate(right_beams.begin(), right_beams.end(), 0.0f) / std::max(1ul, right_beams.size());

        bool front_blocked = std::isfinite(front) && front < FRONT_STOP_DIST;
        if (front_blocked) {
            odom.drive(0.0f, 0.0f);
            RCLCPP_INFO(io_node->get_logger(), "[LINE MODE] Konec koridoru detekován. Robot zastavuje.");
            break;
        }

        bool left_valid = std::isfinite(left) && left > MIN_VALID_SIDE_DIST;
        bool right_valid = std::isfinite(right) && right > MIN_VALID_SIDE_DIST;

        float error = 0.0f;
        if (left_valid && right_valid) error = left - right;
        else if (left_valid) error = (CELL_WIDTH - left) - left;
        else if (right_valid) error = right - (CELL_WIDTH - right);
        else error = 0.0f;

        float correction = pid.step(error, dt);
        correction = std::clamp(correction, -MAX_TURN_ANGLE, MAX_TURN_ANGLE);

        RCLCPP_INFO(io_node->get_logger(), "[LINE MODE] Chyba: %.3f | Korekce: %.3f", error, correction);

        odom.drive(FORWARD_SPEED, correction);
#endif

#ifdef RING
        float front = lidar_node->get_forward_distance();
        switch (ring_state) {
            case RingState::FOLLOW_WALL: {
                std::vector<float> left_beams = lidar_node->get_average_distances_around_angle(-M_PI_2, 5);
                std::vector<float> right_beams = lidar_node->get_average_distances_around_angle(M_PI_2, 5);

                float left = std::accumulate(left_beams.begin(), left_beams.end(), 0.0f) / std::max(1ul, left_beams.size());
                float right = std::accumulate(right_beams.begin(), right_beams.end(), 0.0f) / std::max(1ul, right_beams.size());

                RCLCPP_INFO(io_node->get_logger(), "[RING MODE] L: %.3f | P: %.3f | F: %.3f", left, right, front);

                bool front_blocked = std::isfinite(front) && front < FRONT_STOP_DIST;
                if (front_blocked) {
                    odom.drive(0.0f, 0.0f);
                    turn_start_yaw = imu_node->getIntegratedResults();
                    target_yaw = round_to_nearest_rad90(turn_start_yaw + M_PI_2);
                    ring_state = RingState::TURNING;
                    RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Zahajuji otáčení. Aktuální yaw: %.2f°", turn_start_yaw * 180.0f / M_PI);
                    break;
                }

                float error = 0.0f;
                if (std::isfinite(left) && std::isfinite(right)) error = left - right;
                else if (std::isfinite(left)) {
                    error = (CELL_WIDTH - left) - left;
                    RCLCPP_WARN(io_node->get_logger(), "[RING MODE] Pravá stěna nedostupná, dopočítávám chybu z levé.");
                } else if (std::isfinite(right)) {
                    error = right - (CELL_WIDTH - right);
                    RCLCPP_WARN(io_node->get_logger(), "[RING MODE] Levá stěna nedostupná, dopočítávám chybu z pravé.");
                } else {
                    error = 0.0f;
                    RCLCPP_WARN(io_node->get_logger(), "[RING MODE] Obě stěny nedostupné, chyba nastavena na 0.");
                }

                float correction = pid.step(error, dt);
                correction = std::clamp(correction, -MAX_TURN_ANGLE, MAX_TURN_ANGLE);

                odom.drive(FORWARD_SPEED, correction);
                RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Chyba: %.3f | Korekce: %.3f", error, correction);
                break;
            }

            case RingState::TURNING: {
                float current_yaw = imu_node->getIntegratedResults();
                float yaw_error = nodes::ImuNode::normalize_angle(target_yaw - current_yaw);
                float correction = pid_turn.step(yaw_error, dt);
                correction = std::clamp(correction, -MAX_TURN_ANGLE, MAX_TURN_ANGLE);
                odom.drive(0.0f, correction);

                RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Otáčení | Cíl: %.2f° | Aktuální: %.2f° | Chyba: %.2f° | Korekce: %.2f",
                            target_yaw * 180.0f / M_PI, current_yaw * 180.0f / M_PI, yaw_error * 180.0f / M_PI, correction);

                if (std::fabs(yaw_error) < ANGLE_TOLERANCE) {
                    odom.drive(0.0f, 0.0f);
                    ring_state = RingState::FOLLOW_WALL;
                    RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Otáčení dokončeno.");
                }
                break;
            }
        }
#endif

        if (io_node->get_button_pressed() == 0) {
            odom.drive(0.0f, 0.0f);
            RCLCPP_INFO(io_node->get_logger(), "[MODE] Zastavení robota (tlačítko 0).");
            break;
        }

        rate.sleep();
    }

    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
#endif


