#include <rclcpp/rclcpp.hpp>
#include "RosExampleClass.hpp"
#include "nodes/io_node.hpp"
#include "odometry.hpp"


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto io_node = std::make_shared<nodes::IoNode>();

    auto line_node = std::make_shared<nodes::LineNode>();

    executor->add_node(io_node);

    auto executor_thread = std::thread([&executor]() { executor->spin(); });


    std::vector<uint8_t> rgb_values = {0,0,0,0,0,0,0,0,0,0,0,0};

    std::vector<uint8_t> motor_speed = {127,127};

    rclcpp::Rate rate(10); // 10 Hz loop to print encoder values

    // std::array<uint32_t, 2> encoders_basic;
    // do {
    //     encoders_basic = io_node->get_encoder_values();
    //     RCLCPP_INFO(io_node->get_logger(), "Waiting for encoder values...");
    //     std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // } while (encoders_basic[0] == 0 && encoders_basic[1] == 0);
    //
    // RCLCPP_INFO(io_node->get_logger(), "Initial encoder values set: [%u, %u]", encoders_basic[0], encoders_basic[1]);

    // while (rclcpp::ok()) {
    //     switch (io_node->get_button_pressed()) {
    //         case 0:
    //             rgb_values = {0,100,0,0,0,0,0,0,0,0,0,0};
    //         motor_speed = {255,255};
    //         break;
    //         case 1:
    //             rgb_values = {100,0,0,0,0,0,0,0,0,0,0,0};
    //         motor_speed = {127,127};
    //         break;
    //         case 2:
    //             break;
    //     }
    //
    //     std::array<uint32_t, 2> encoders = io_node->get_encoder_values();
    //     RCLCPP_INFO(io_node->get_logger(), "Current Encoder Values: [%u, %u]", encoders[0], encoders[1]);
    //
    //     if (encoders[0] > (encoders_basic[0]+2788) || encoders[1] > (encoders_basic[1]+2806)) {
    //         motor_speed = {127,127};
    //         io_node->set_motor_speeds(motor_speed);
    //         encoders_basic = io_node->get_encoder_values();
    //         std::cout << "STOP" << std::endl;
    //         break;
    //     }
    //
    //     io_node->turn_on_leds(rgb_values);
    //     io_node->set_motor_speeds(motor_speed);
    //     rate.sleep();
    // }
/*
    //Odometrie ---------------------------------------------------------------------
    Odometry odom(0.1, 0.5, 360);
    
    // Simulované pulzy z enkodérů
    int pulses_left = 180;   // např. 180 pulzů z levého kola
    int pulses_right = 200;  // např. 200 pulzů z pravého kola

    odom.update(pulses_left, pulses_right);

    // Získání a vypsání aktuální polohy
    Pose current_pose = odom.getPose();
    std::cout << "Aktuální poloha: " << std::endl;
    std::cout << "x = " << current_pose.x << " m" << std::endl;
    std::cout << "y = " << current_pose.y << " m" << std::endl;
    std::cout << "theta = " << current_pose.theta << " rad" << std::endl;

    //-------------------------------------------------------------------------------
    
    bool waiting_for_reset = false;
    std::array<int32_t, 2> encoders_basic = io_node->get_encoder_values();

    while (rclcpp::ok()) {
        auto encoders = io_node->get_encoder_values();
        RCLCPP_INFO(io_node->get_logger(), "Current Encoder Values: [%d, %d]", encoders[0], encoders[1]);

        if (!waiting_for_reset && (encoders[0] > (encoders_basic[0] + 2788) || encoders[1] > (encoders_basic[1] + 2806))) {
            RCLCPP_INFO(io_node->get_logger(), "STOP - Limit reached");
            waiting_for_reset = true;
        }

        if (waiting_for_reset) {
            if (io_node->get_button_pressed() == 2) {
                RCLCPP_INFO(io_node->get_logger(), "System reset after button 2 press");
                encoders_basic = io_node->get_encoder_values();
                waiting_for_reset = false;
            }
        } else {
            if (io_node->get_button_pressed() == 1) {
                RCLCPP_INFO(io_node->get_logger(), "Moving forward");
            }
        }

        rclcpp::spin_some(io_node);
        rate.sleep();
    }
*/
    while (rclcpp::ok()) {
        rclcpp::spin_some(line_node);
        nodes::DiscreteLinePose discrete_pose = line_node->get_discrete_line_pose();

        switch (discrete_pose) {
            case nodes::DiscreteLinePose::LineOnLeft:
                RCLCPP_INFO(line_node->get_logger(), "Čára vlevo");
            break;
            case nodes::DiscreteLinePose::LineOnRight:
                RCLCPP_INFO(line_node->get_logger(), "Čára vpravo");
            break;
            case nodes::DiscreteLinePose::LineNone:
                RCLCPP_INFO(line_node->get_logger(), "Čára nenalezena");
            break;
            default:
                break;
        }
        rate.sleep();
    }

    //executor->spin();

    rclcpp::shutdown();
    return 0;
}
