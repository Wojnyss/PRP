#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "odometry.hpp"
#include "algorithms/pid.hpp"
#include <cmath>

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto io_node = std::make_shared<nodes::IoNode>();
    auto line_node = std::make_shared<nodes::LineNode>();

    executor->add_node(io_node);
    executor->add_node(line_node);

    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    rclcpp::Rate rate(50); // 50 Hz pro jemnější řízení
    const float dt = 1.0f / 50.0f; // vzorkovací perioda

    Odometry odom(0.07082, 0.12539, 570, 580);

    // Parametry PID regulátoru (lze si pohrát s laděním)
    algorithms::Pid pid(500.0f, 0.0f, 0.0f); // KP, KI, KD

    const double forward_speed = 0.10; // stálá dopředná rychlost
    const double max_turn_angle = 2.0; // maximální úhel zatáčení

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
    }

    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
