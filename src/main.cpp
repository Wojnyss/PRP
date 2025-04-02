#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "odometry.hpp"
#include "algorithms/pid.hpp"
#include <cmath>


int main(int argc, char* argv[]) {

    rclcpp::init(argc, argv);

    // Vytvoření instance LidarNode
    auto lidar_node = std::make_shared<nodes::LidarNode>();

    // Spuštění vláken ROS executor
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(lidar_node);

    rclcpp::Rate rate(10);  // 10 Hz

    // Spuštění vlákna pro executor (zpracovává zprávy ze scan)
    std::thread executor_thread([&executor]() {
        executor->spin();
    });

    // Smyčka pro periodické výpisy vzdáleností
    while (rclcpp::ok()) {
        float front = lidar_node->get_forward_distance();
        float back = lidar_node->get_back_distance();
        float left = lidar_node->get_left_distance();
        float right = lidar_node->get_right_distance();

        RCLCPP_INFO(lidar_node->get_logger(),
            "LiDAR vzdálenosti [m] → Vpřed: %.2f | Vzad: %.2f | Vlevo: %.2f | Vpravo: %.2f",
            front, back, left, right);

        rate.sleep();
    }

    executor_thread.join();
    rclcpp::shutdown();
    return 0;
    /*
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto io_node = std::make_shared<nodes::IoNode>();
    auto line_node = std::make_shared<nodes::LineNode>();

    executor->add_node(io_node);
    executor->add_node(line_node);

    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    rclcpp::Rate rate(50); // 50 Hz pro jemnější řízení
    const float dt = 1.0f / 50.0f; // vzorkovací perioda

    Odometry odom(0.07082, 0.12539, 570, 570.7);

    // Parametry PID regulátoru (lze si pohrát s laděním)
    algorithms::Pid pid(5.0f, 0.0f, 1.5f); // KP, KI, KD //18 0 3

    const double forward_speed = 0.08 //0.08
    ; // stálá dopředná rychlost
    const double max_turn_angle = 10.0; // maximální úhel zatáčení //2.0

    // Čekání na stisk tlačítka 1
    io_node->turn_on_leds({100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}); // žlutá LED
    RCLCPP_INFO(io_node->get_logger(), "Čekám na stisk tlačítka 1 pro spuštění sledování čáry...");

    while (rclcpp::ok()) {
        if (io_node->get_button_pressed() == 1) {
            io_node->turn_on_leds({0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}); // zelená LED
            RCLCPP_INFO(io_node->get_logger(), "Zahajuji sledování čáry pomocí PID regulátoru.");
            break;
        }
        rate.sleep();
    }

    // Smyčka sledování čáry
    while (rclcpp::ok()) {
        auto encoders = io_node->get_encoder_values();
        odom.update(encoders[0], encoders[1]);

        // Hodnota pozice čáry: -1.0 = vlevo, 0 = uprostřed, 1.0 = vpravo
        float line_error = line_node->get_continuous_line_pose(); // (float)

        // PID výstup – řízení zatáčení
        float correction = pid.step(line_error, dt);

        // Omezíme výstup PID na rozsah řízení otáčení (např. ±max_turn_angle)
        correction = std::clamp(correction, static_cast<float>(-max_turn_angle), static_cast<float>(max_turn_angle));


        odom.drive(forward_speed, correction); // řízení vpřed + odklon

        RCLCPP_INFO(io_node->get_logger(), "Čára: %.2f | Korekce: %.2f°", line_error, correction);

        // Nouzové ukončení – tlačítko 0
        if (io_node->get_button_pressed() == 0) {
            odom.drive(0.0, 0.0);
            RCLCPP_INFO(io_node->get_logger(), "Zastavuji program.");
            io_node->turn_on_leds({100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}); // červená LED
            break;
        }

        rate.sleep();
    }/*


    odom.resetPose();
    RCLCPP_INFO(io_node->get_logger(), "Jedu rovně s konstantní rychlostí 0.1 m/s");
    while (rclcpp::ok()) {
        odom.drive(0.1, 0.0);
        rate.sleep();
    }
    executor_thread.join();
    rclcpp::shutdown();
    return 0;*/
}
