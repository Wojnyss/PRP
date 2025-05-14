#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "nodes/lidar_node.hpp"
#include "nodes/imu_node.hpp"
#include "nodes/camera_node.hpp"
#include "odometry.hpp"
#include "algorithms/pid.hpp"
#include <thread>
#include <chrono>
#include <cmath>

//-------------------DEBUG MODES----------------------
// #define DEBUG_ROT_R //prepis direktivu na NEW_DRIVE V odometry.cpp
// #define DEBUG_ROT_L //prepis direktivu na NEW_DRIVE V odometry.cpp
// #define ARUCO_DEBUG
#define RING //MAIN PROGRAM PRO MAZE ESCAPE //prepis direktivu na OLD_DRIVE V odometry.cpp

//------------------Type of escape---------------------
// #define EXIT
#define POKLAD

//-----------------Choose robot-----------------------
#define ORANGE
// #define BLUE


// #define RECALIBRATION

#ifdef ORANGE
#ifdef LINE
constexpr float FRONT_STOP_DIST = 0.35f;
constexpr float FORWARD_SPEED = 0.3f;
#endif

#ifdef RING
constexpr float FRONT_STOP_DIST = 0.27f;
constexpr float FORWARD_SPEED = 0.20f; //0.20f
#endif

constexpr float MIN_VALID_CENTER_DIST = 0.05f;
constexpr float MAX_TURN_ANGLE = 20.0f;
constexpr float MIN_VALID_SIDE_DIST = 0.05f;
constexpr float TURN_RADIUS = 0.2f; //0.2
constexpr float CELL_WIDTH = 0.40f;
constexpr float MAX_TURN_ANGLE_ROT = 10.0f;
constexpr float ANGLE_TOLERANCE = 0.15f;

algorithms::Pid pid_turnR(5.0f, 7.0f, 1.3f);
algorithms::Pid pid_turnL(5.0f, 7.0f, 1.50f); //(5.0f, 7.0f, 0.0f);
algorithms::Pid pid(12.f, 5.3f, 9.8f);
#endif


#ifdef BLUE
#ifdef LINE
constexpr float FRONT_STOP_DIST = 0.35f;
constexpr float FORWARD_SPEED = 0.3f;
#endif

#ifdef RING
constexpr float FRONT_STOP_DIST = 0.27f;
constexpr float FORWARD_SPEED = 0.20f;
#endif

constexpr float MIN_VALID_CENTER_DIST = 0.05f;
constexpr float MAX_TURN_ANGLE = 20.0f;
constexpr float MIN_VALID_SIDE_DIST = 0.05f;
constexpr float TURN_RADIUS = 0.2f; //0.2
constexpr float CELL_WIDTH = 0.40f;
constexpr float MAX_TURN_ANGLE_ROT = 10.0f;
constexpr float ANGLE_TOLERANCE = 0.15f;

algorithms::Pid pid_turnR(18.0f, 19.6f, 0.0f);
algorithms::Pid pid_turnL(18.0f, 19.0f, 0.0f);
algorithms::Pid pid(12.f, 5.3f, 9.8f);
#endif

float round_to_nearest_rad90(float angle_rad) {
    int quadrant = static_cast<int>(std::round(angle_rad / M_PI_2));
    return quadrant * M_PI_2;
}

enum class RingState {
    FOLLOW_WALL,
    INITIATE_TURN,
    INITIATE_INTERSECTION,
    TURNING
};

#if defined(LINE) || defined(RING)
int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);
    int aruco = -1;
    int tmp_aruco = -1;
    int turn = 0;

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto io_node = std::make_shared<nodes::IoNode>();
    auto lidar_node = std::make_shared<nodes::LidarNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>();
    auto camera_node = CameraNode::create();

    executor->add_node(camera_node);
    executor->add_node(io_node);
    executor->add_node(lidar_node);
    executor->add_node(imu_node);

    RCLCPP_INFO(camera_node->get_logger(), "Camera node started. Listening for ArUco markers...");


    std::thread executor_thread([&executor]() { executor->spin(); });

    rclcpp::Rate rate(50);
    const float dt = 1.0f / 50.0f;

#ifdef ORANGE
    Odometry odom(0.07082, 0.12539, 570, 570.7);
#endif
#ifdef BLUE
    Odometry odom(0.06684, 0.12889, 584, 584);
#endif


#ifdef RING
    io_node->turn_on_leds({100, 100, 0, 100, 100, 0, 100, 100, 0, 0, 0, 100});
    RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Čekám na tlačítko 2 pro kalibraci IMU...");
    while (rclcpp::ok() && io_node->get_button_pressed() != 1) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
    io_node->turn_on_leds({100, 100, 0, 100, 100, 0, 100, 100, 0, 0, 0, 100});
    RCLCPP_INFO(io_node->get_logger(), "[RING MODE] Kalibrace spuštěna...");
    imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
    rclcpp::sleep_for(std::chrono::seconds(2));
    imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);
#endif

    io_node->turn_on_leds({100, 100, 0, 100, 100, 0, 100, 100, 0, 100, 100, 0});
    RCLCPP_INFO(io_node->get_logger(), "[MODE] Čekám na tlačítko 1 pro start...");
    // while (rclcpp::ok() && io_node->get_button_pressed() != 1) {
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    // }

    RCLCPP_INFO(io_node->get_logger(), "[MODE] Start...");

    float turn_start_yaw = 0.0f;
    float target_yaw = 0.0f;
    RingState ring_state = RingState::FOLLOW_WALL;

    bool stop_requested = false;
    bool autostop = false;
    bool in_intersection = false;

    while (rclcpp::ok()) {
        float front = lidar_node->get_forward_distance();
        float back = lidar_node->get_back_distance();

        float left = lidar_node->get_left_distance();
        float right = lidar_node->get_right_distance();

        if (io_node->get_button_pressed() == 0) stop_requested = true;

        // std::vector<float> left_beams = lidar_node->get_average_distances_around_angle(-M_PI_2, 5);
        // std::vector<float> right_beams = lidar_node->get_average_distances_around_angle(M_PI_2, 5);

        // float left = std::accumulate(left_beams.begin(), left_beams.end(), 0.0f) / std::max(1ul, left_beams.size());
        // float right = std::accumulate(right_beams.begin(), right_beams.end(), 0.0f) / std::max(1ul, right_beams.size());

        switch (ring_state) {
            case RingState::FOLLOW_WALL: {
                std::cout <<  "STATE: FOLLOW_WALL" << std::endl;
                float virtual_left = left;
                float virtual_right = right;

                if (!std::isfinite(right) || right > CELL_WIDTH) virtual_right = CELL_WIDTH - left;
                if (!std::isfinite(left) || left > CELL_WIDTH) virtual_left = CELL_WIDTH - right;

                float error = 0.0f;
                if (std::isfinite(left) && left <= CELL_WIDTH && std::isfinite(right) && right <= CELL_WIDTH) error = left - right;
                else if (std::isfinite(left) && left <= CELL_WIDTH) error = left - (CELL_WIDTH - left);
                else if (std::isfinite(right) && right <= CELL_WIDTH) error = (CELL_WIDTH - right) - right;

                float correction = pid.step(error, dt);
                correction = std::clamp(correction, -MAX_TURN_ANGLE, MAX_TURN_ANGLE);
                odom.drive(FORWARD_SPEED, correction);

                bool left_free = std::isfinite(left) && left > CELL_WIDTH;
                bool right_free = std::isfinite(right) && right > CELL_WIDTH;
                bool front_free = std::isfinite(front) && front > CELL_WIDTH;
                bool back_free = std::isfinite(back) && back > CELL_WIDTH;

                    const auto& detected = camera_node->get_last_detected();
                    if (!detected.empty())
                    {
                        for (const auto& tag : detected)
                        {
                            int id = tag.id;
#ifdef EXIT
                            if (id < 10)
                            {
                                aruco = id;
                                RCLCPP_INFO(camera_node->get_logger(), "[ARUCO][EXIT] Detekováno ID: %d, uloženo jako aruco = %d", id, aruco);
                                break;
                            }
#endif
#ifdef POKLAD
                            if (id >= 10){
                                if (in_intersection == true){
                                tmp_aruco = id % 10;
                                RCLCPP_INFO(camera_node->get_logger(), "[ARUCO][POKLAD] Detekováno ID: %d, uloženo jako tmp_aruco = %d", id, tmp_aruco);
                                break;
                                }
                                else
                                {
                                    aruco = id % 10;
                                    RCLCPP_INFO(camera_node->get_logger(), "[ARUCO][POKLAD] Detekováno ID: %d, uloženo jako aruco = %d", id, aruco);
                                    break;
                                }
                            }
#endif
                        }
                    }

                //DETEKCE KRIZOVATEK
                    float pos_in_cell = fmod(front, CELL_WIDTH); // pozice v rámci jedné buňky
                    bool centered = fabs(pos_in_cell - CELL_WIDTH/2) <= MIN_VALID_CENTER_DIST;
                    int num_of_exits = (front_free + left_free + right_free + back_free);

                // if (centered)
                    std::cout << "ARUCO_VAL: "<< aruco << "TMP_VAL: "<< tmp_aruco << ", EXITS: " << num_of_exits << std::endl;
                    // std::cout << "F: " << front_free << "B: " << back_free << "R: " << right_free << "L: " << left_free <<  std::endl;
                    // std::cout << "F: " << front << "B: " << back << "R: " << right << "L: " << left <<  std::endl;

                if (front < FRONT_STOP_DIST)
                    odom.drive(0.0f, 0.0f);

                if (centered && num_of_exits == 2 && !front_free)//zatacka L R
                {
                    std::cout << "TURN"<<  std::endl;
                    std::cout << "F: " << front_free << "B: " << back_free << "R: " << right_free << "L: " << left_free <<  std::endl;
                    in_intersection = false;
                    if (right_free)
                        turn = 1;
                    else if (left_free)
                        turn = -1;
                    ring_state = RingState::INITIATE_TURN;
                }
                else if (centered && num_of_exits == 1 && !front_free) //slepa ulicka
                {
                    odom.drive(0.0f, 0.0f);
                    std::cout << "TURN 180"<< std::endl;
                    in_intersection = false;
                    aruco = tmp_aruco;
                    turn = 2;
                    ring_state = RingState::INITIATE_TURN;
                }
                else if (centered && num_of_exits == 3 && aruco != 0 ) //T
                {
                    odom.drive(0.0f, 0.0f);
                    std::cout << "T INTERSECTION"<< std::endl;
                    in_intersection = true;
                    turn = 0;
                    ring_state = RingState::INITIATE_INTERSECTION;
                }
                else if (centered && num_of_exits == 3 && aruco == 0 ) //T rovne
                {
                    std::cout << "T INTERSECTION through"<< std::endl;
                    in_intersection = true;
                }
                else if (centered && num_of_exits == 4 && aruco != 0) //+
                {
                    odom.drive(0.0f, 0.0f);
                    std::cout << "+ INTERSECTION"<< std::endl;
                    in_intersection = true;
                    turn = 0;
                    ring_state = RingState::INITIATE_INTERSECTION;
                }
                else if (centered && num_of_exits == 3 && aruco == 0 && !front_free) //T do zdi
                {
                    odom.drive(0.0f, 0.0f);
                    stop_requested = true;
                    RCLCPP_INFO(io_node->get_logger(), "[MODE] CRITICAL ERROR: WALL AHEAD (AUTOSTOP).\n");
                    autostop = true;
                }
                else
                    in_intersection = false;

                io_node->turn_on_leds({0, 0, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0});
                break;
            }
            case RingState::INITIATE_TURN: {
                std::cout <<  "STATE: INITIATE_TURN" << std::endl;
                turn_start_yaw = imu_node->getIntegratedResults();
                float direction = 0.0f;
                if (turn == -1) {       //VLEVO
                    direction = 1.0f;
                    ring_state = RingState::TURNING;
                    io_node->turn_on_leds({0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 100, 0});
                } else if (turn == 1) { //VPRAVO
                    direction = -1.0f;
                    ring_state = RingState::TURNING;
                    io_node->turn_on_leds({0, 100, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0});
                } else if (turn == 2) { //DOZADU
                    direction = -2.0f;
                    ring_state = RingState::TURNING;
                    io_node->turn_on_leds({0, 100, 0, 0, 100, 0, 0, 100, 0, 0, 0, 0});
                } else {
                    ring_state = RingState::FOLLOW_WALL;
                }
                target_yaw = round_to_nearest_rad90(nodes::ImuNode::normalize_angle(turn_start_yaw + direction * M_PI_2));
                break;
            }
            case RingState::INITIATE_INTERSECTION: {
                        std::cout <<  "STATE: INITIATE_INTERSECTION" << std::endl;
                        turn_start_yaw = imu_node->getIntegratedResults();
                        float direction = 0.0f;
                        if (aruco == 1) {       //VLEVO
                            direction = 1.0f;
                            ring_state = RingState::TURNING;
                            io_node->turn_on_leds({0, 100, 0, 0, 0, 0, 0, 100, 0, 0, 100, 0});
                        } else if (aruco == 2) { //VPRAVO
                            direction = -1.0f;
                            ring_state = RingState::TURNING;
                            io_node->turn_on_leds({0, 100, 0, 0, 100, 0, 0, 0, 0, 0, 100, 0});
                        } else {
                            ring_state = RingState::FOLLOW_WALL;
                        }
                        target_yaw = round_to_nearest_rad90(nodes::ImuNode::normalize_angle(turn_start_yaw + direction * M_PI_2));
                        break;
            }
            case RingState::TURNING: {
                std::cout <<  "STATE: TURNING" << std::endl;

                float current_yaw = imu_node->getIntegratedResults();
                float yaw_error = nodes::ImuNode::normalize_angle(target_yaw - current_yaw);
                    float correction = 0.f;
                    if (yaw_error > 0) {
                        correction = pid_turnL.step(yaw_error, dt);
                    } else {
                        correction = pid_turnR.step(yaw_error, dt);
                    }


                correction = std::clamp(correction, -MAX_TURN_ANGLE_ROT, MAX_TURN_ANGLE_ROT);
                odom.drive(0.0f, correction);

                RCLCPP_INFO(io_node->get_logger(), "[TURN] Current: %.2f° | Target: %.2f° | Error: %.2f°",
                            current_yaw * 180.0f / M_PI, target_yaw * 180.0f / M_PI, yaw_error * 180.0f / M_PI);

                if (std::fabs(yaw_error) < ANGLE_TOLERANCE) {
                    odom.drive(0.0f, 0.0f);
                    RCLCPP_INFO(io_node->get_logger(), "[TURN] Otocka dokoncena. Spoustim rekalibraci...");
                    if (turn == 0)
                        aruco = 0;
                    // if (in_intersection == true)

#ifdef RECALIBRATION
                    imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
                    rclcpp::sleep_for(std::chrono::seconds(1));
                    imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);
#endif
                    ring_state = RingState::FOLLOW_WALL;
                    rclcpp::sleep_for(std::chrono::milliseconds(200));
                }
                break;
            }
        }

        if (stop_requested) {
            odom.drive(0.0f, 0.0f);
            io_node->turn_on_leds({100, 0, 0, 100, 0, 0, 100, 0, 0, 100, 0, 0});
            if (!autostop)
                RCLCPP_INFO(io_node->get_logger(), "[MODE] Zastaveni robota (tlacitko 0).\n");
            break;
        }

        rate.sleep();
    }

    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
#endif

#ifdef ARUCO_DEBUG
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto camera_node = CameraNode::create();

    int aruco = -1;

    RCLCPP_INFO(camera_node->get_logger(), "Camera node started. Listening for ArUco markers...");

    rclcpp::Rate rate(10);
    while (rclcpp::ok()) {
        rclcpp::spin_some(camera_node);

        const auto& detected = camera_node->get_last_detected();
        if (!detected.empty()) {
            int id = detected[0].id;
            RCLCPP_INFO(camera_node->get_logger(), "[ARUCO] Detekováno ID: %d", id);
#ifdef EXIT
            if (id == 0) aruco = 0;
            else if (id == 1) aruco = 1;
            else if (id == 2) aruco = 2;
#endif
#ifdef POKLAD
            if (id == 10) aruco = 0;
            else if (id == 11) aruco = 1;
            else if (id == 12) aruco = 2;
#endif
            RCLCPP_INFO(camera_node->get_logger(), "[ARUCO] Uloženo jako aruco = %d", aruco);
        }

        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
#endif

#ifdef DEBUG_ROT_L
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto io_node = std::make_shared<nodes::IoNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>();

    if (!io_node || !imu_node) {
        std::cerr << "[CRITICAL] Nelze inicializovat IO nebo IMU node." << std::endl;
        return 1;
    }

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(io_node);
    executor->add_node(imu_node);

    std::thread executor_thread([&executor]() { executor->spin(); });

    rclcpp::Rate rate(50);
    const float dt = 1.0f / 50.0f;

#ifdef ORANGE
    Odometry odom(0.07082, 0.12539, 570, 570);
#endif
#ifdef BLUE
    Odometry odom(0.06684, 0.12889, 584, 584);
#endif
    odom.setIoNode(io_node);

    RCLCPP_INFO(io_node->get_logger(), "[TUNE] PID otáčení VLEVO o 90°...");

    imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
    rclcpp::sleep_for(std::chrono::seconds(2));
    imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);

    float current_yaw = imu_node->getIntegratedResults();
    float target_yaw = round_to_nearest_rad90(nodes::ImuNode::normalize_angle(current_yaw + M_PI_2));

    while (rclcpp::ok()) {
        current_yaw = imu_node->getIntegratedResults();
        float yaw_error = nodes::ImuNode::normalize_angle(target_yaw - current_yaw);

        if (!std::isfinite(yaw_error)) {
            io_node->set_motor_speeds({127, 127});
            RCLCPP_ERROR(io_node->get_logger(), "[TUNE] Neplatná korekce. Zastavuji.");
            break;
        }

        float correction = pid_turnL.step(yaw_error, dt);
        correction = std::clamp(correction, -MAX_TURN_ANGLE_ROT, MAX_TURN_ANGLE_ROT);

        if (std::abs(correction) < 0.1f) correction = 0.0f;

        try {
            odom.drive(0.0f, correction);
        } catch (...) {
            io_node->set_motor_speeds({127, 127});
            RCLCPP_ERROR(io_node->get_logger(), "[TUNE] Výjimka při řízení.");
            break;
        }

        RCLCPP_INFO(io_node->get_logger(), "[TUNE] Yaw: %.2f° → %.2f° | Err: %.2f° | Corr: %.2f",
                    current_yaw * 180.0 / M_PI,
                    target_yaw * 180.0 / M_PI,
                    yaw_error * 180.0 / M_PI,
                    correction);

        if (std::fabs(yaw_error) < ANGLE_TOLERANCE) {
            odom.drive(0.0f, 0.0f);
            RCLCPP_INFO(io_node->get_logger(), "[TUNE] Otáčka dokončena.");
            rclcpp::sleep_for(std::chrono::milliseconds(1000));

            current_yaw = imu_node->getIntegratedResults();
            target_yaw = round_to_nearest_rad90(nodes::ImuNode::normalize_angle(current_yaw + M_PI_2));
            pid_turnL.reset();
        }

        rate.sleep();
    }

    odom.drive(0.0f, 0.0f);
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
#endif


#ifdef DEBUG_ROT_R
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto io_node = std::make_shared<nodes::IoNode>();
    auto imu_node = std::make_shared<nodes::ImuNode>();

    if (!io_node || !imu_node) {
        std::cerr << "[CRITICAL] Nelze inicializovat IO nebo IMU node." << std::endl;
        return 1;
    }

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(io_node);
    executor->add_node(imu_node);

    std::thread executor_thread([&executor]() { executor->spin(); });

    rclcpp::Rate rate(50);
    const float dt = 1.0f / 50.0f;

#ifdef ORANGE
    Odometry odom(0.07082, 0.12539, 570, 570.7);
#endif
#ifdef BLUE
    Odometry odom(0.06684, 0.12889, 584, 584);
#endif
    odom.setIoNode(io_node);

    RCLCPP_INFO(io_node->get_logger(), "[TUNE] PID otáčení VPRAVO o 90°...");

    imu_node->setMode(nodes::ImuNodeMode::CALIBRATE);
    rclcpp::sleep_for(std::chrono::seconds(2));
    imu_node->setMode(nodes::ImuNodeMode::INTEGRATE);

    float current_yaw = imu_node->getIntegratedResults();
    float target_yaw = round_to_nearest_rad90(nodes::ImuNode::normalize_angle(current_yaw - M_PI_2));

    while (rclcpp::ok()) {
        current_yaw = imu_node->getIntegratedResults();
        float yaw_error = nodes::ImuNode::normalize_angle(target_yaw - current_yaw);

        if (!std::isfinite(yaw_error)) {
            io_node->set_motor_speeds({127, 127});
            RCLCPP_ERROR(io_node->get_logger(), "[TUNE-R] Neplatná korekce. Zastavuji.");
            break;
        }

        float correction = pid_turnR.step(yaw_error, dt);
        correction = std::clamp(correction, -MAX_TURN_ANGLE_ROT, MAX_TURN_ANGLE_ROT);

        if (std::abs(correction) < 0.1f) correction = 0.0f;

        try {
            odom.drive(0.0f, correction);
        } catch (...) {
            io_node->set_motor_speeds({127, 127});
            RCLCPP_ERROR(io_node->get_logger(), "[TUNE-R] Výjimka při řízení.");
            break;
        }

        RCLCPP_INFO(io_node->get_logger(), "[TUNE-R] Yaw: %.2f° → %.2f° | Err: %.2f° | Corr: %.2f",
                    current_yaw * 180.0 / M_PI,
                    target_yaw * 180.0 / M_PI,
                    yaw_error * 180.0 / M_PI,
                    correction);

        if (std::fabs(yaw_error) < ANGLE_TOLERANCE) {
            odom.drive(0.0f, 0.0f);
            RCLCPP_INFO(io_node->get_logger(), "[TUNE-R] Otáčka dokončena.");
            rclcpp::sleep_for(std::chrono::milliseconds(1000));

            current_yaw = imu_node->getIntegratedResults();
            target_yaw = round_to_nearest_rad90(nodes::ImuNode::normalize_angle(current_yaw - M_PI_2));
            pid_turnR.reset();
        }

        rate.sleep();
    }

    odom.drive(0.0f, 0.0f);
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
#endif
