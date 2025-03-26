#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "odometry.hpp"
#include "algorithms/pid.hpp"
#include <cmath>

//#define debug
//#define PID

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

    Odometry odom(0.07082, 0.12539, 570, 570.7);

    // Parametry PID regulátoru (lze si pohrát s laděním)
    algorithms::Pid pid(5.0f, 0.0f, 1.5f); // KP, KI, KD //18 0 3

    const double forward_speed = 0.04 //0.04
    ; // stálá dopředná rychlost
    const double max_turn_angle = 2.0; // maximální úhel zatáčení //2.0

    // Čekání na stisk tlačítka 1
    io_node->turn_on_leds({100, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}); // žlutá LED
    RCLCPP_INFO(io_node->get_logger(), "Čekám na stisk tlačítka 1 pro spuštění sledování čáry...");


#ifdef debug

    odom.resetPose();
    RCLCPP_INFO(io_node->get_logger(), "Jedu rovně s konstantní rychlostí 0.1 m/s");
    while (rclcpp::ok()) {
        odom.drive(0.04, 2.0);
        rate.sleep();
    }

#elif defined(PID)

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
#else

    // Čekání na tlačítko 1
    while (rclcpp::ok()) {
        if (io_node->get_button_pressed() == 1) {
            io_node->turn_on_leds({0, 100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
            RCLCPP_INFO(io_node->get_logger(), "Zahajuji bang-bang sledování čáry pomocí diskrétní funkce.");
            break;
        }
        rate.sleep();
    }

    while (rclcpp::ok()) {
        auto encoders = io_node->get_encoder_values();
        odom.update(encoders[0], encoders[1]);

        auto pose = line_node->get_discrete_line_pose();

        double turn = 0.0;

        switch (pose) {
        case nodes::DiscreteLinePose::LineOnLeft:
            turn = -max_turn_angle;  // zatáčej doleva
            break;
        case nodes::DiscreteLinePose::LineOnRight:
            turn = max_turn_angle;   // zatáčej doprava
            break;
        case nodes::DiscreteLinePose::LineBoth:
            turn = 0.0;              // čára pod oběma → rovně
            break;
        case nodes::DiscreteLinePose::LineNone:
            turn = 0.0;              // čára ztracena → rovně
            break;
        }

        odom.drive(forward_speed, turn);

        RCLCPP_INFO(io_node->get_logger(), "Diskrétní pozice čáry: %d | Otáčím: %.2f°", static_cast<int>(pose), turn);

        if (io_node->get_button_pressed() == 0) {
            odom.drive(0.0, 0.0);
            io_node->turn_on_leds({100, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0});
            RCLCPP_INFO(io_node->get_logger(), "Zastavuji bang-bang cyklus.");
            break;
        }

        rate.sleep();
    }

#endif
    executor_thread.join();
    rclcpp::shutdown();
    return 0;
}
