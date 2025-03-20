#include <rclcpp/rclcpp.hpp>
#include "nodes/io_node.hpp"
#include "odometry.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    // Vytvoření ROS executoru
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    // Vytvoření ROS uzlů
    auto io_node = std::make_shared<nodes::IoNode>();

    // Přidání uzlů do executoru
    executor->add_node(io_node);

    // Spuštění executor ve vlákně
    auto executor_thread = std::thread([&executor]() { executor->spin(); });

    // Inicializace motorových rychlostí
    std::vector<uint8_t> motor_speed = {127,127};

    rclcpp::Rate rate(10); // 10 Hz loop

    // Odometrie setup (parametry: průměr kola, rozteč kol, pulzy na otáčku)
    // Odometry odom(65.57, 128.5, 572);
    Odometry odom(0.0006557, 0.1285, 2000);

    bool moving = false;  // Indikátor pohybu
    Pose start_pose;      // Výchozí poloha
    const double target_distance = 2.0; // Cíl = 1 metr

    while (rclcpp::ok()) {
        auto encoders = io_node->get_encoder_values();
        RCLCPP_INFO(io_node->get_logger(), "Encoder Values: [%d, %d]", encoders[0], encoders[1]);

        int button = io_node->get_button_pressed();
        RCLCPP_INFO(io_node->get_logger(), "Stisknuté tlačítko: %d", button);

        // Resetování odometrie po stisku tlačítka 0
        if (button == 0) {
            odom.resetPose();
            RCLCPP_INFO(io_node->get_logger(), "Resetování odometrie - připraveno na další jízdu.");
            moving = false;
        }

        // Spustíme jízdu po stisku tlačítka 1
        if (button == 1 && !moving) {
            odom.resetPose(); // Reset odometrie
            start_pose = odom.getPose();
            moving = true;
            motor_speed = {255, 255}; // Nastavíme maximální rychlost
            io_node->set_motor_speeds(motor_speed);
            RCLCPP_INFO(io_node->get_logger(), "Začínám jízdu na 1 metr...");
        }

        if (moving) {
            // Aktualizace odometrie
            odom.update(encoders[0], encoders[1]);

            // Výpočet ujeté vzdálenosti
            Pose current_pose = odom.getPose();
            double distance_traveled = std::sqrt(
                std::pow(current_pose.x - start_pose.x, 2) +
                std::pow(current_pose.y - start_pose.y, 2)
            );

            RCLCPP_INFO(io_node->get_logger(), "Ujeto: %.2f m", distance_traveled);

            if (distance_traveled >= target_distance) {
                motor_speed = {127, 127}; // Zastavení robota
                io_node->set_motor_speeds(motor_speed);
                moving = false;
                RCLCPP_INFO(io_node->get_logger(), "Cíl dosažen - zastavuji. Stiskněte tlačítko 0 pro nový pokus.");
            }
        }

        rate.sleep();
    }

    executor_thread.join(); // Po ukončení ROS uzavřeme executor
    rclcpp::shutdown();
    return 0;
}


/*
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

    auto io_node = std::make_shared<nodes::IoNode>();

    //auto line_node = std::make_shared<nodes::LineNode>();

    executor->add_node(io_node);

    auto executor_thread = std::thread([&executor]() { executor->spin(); });


    std::vector<uint8_t> rgb_values = {0,0,0,0,0,0,0,0,0,0,0,0};

    std::vector<uint8_t> motor_speed = {127,127};

    rclcpp::Rate rate(10); // 10 Hz loop to print encoder values

    while (rclcpp::ok())
    {
        switch (io_node->get_button_pressed()) {
        case 0:
            rgb_values = {0,100,0,0,0,0,0,0,0,0,0,0};
            motor_speed = {255,255};
            break;
        case 1:
            rgb_values = {100,0,0,0,0,0,0,0,0,0,0,0};
            motor_speed = {127,127};
            break;
        case 2:
            rgb_values = {0,0,100,0,0,0,0,0,0,0,0,0};
            motor_speed = {2,2};
            break;
        }
        io_node->turn_on_leds(rgb_values);
        io_node->set_motor_speeds(motor_speed);
        rate.sleep();
    }
}

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
/*
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


// std::array<uint32_t, 2> encoders_basic;
// do {
//     encoders_basic = io_node->get_encoder_values();
//     RCLCPP_INFO(io_node->get_logger(), "Waiting for encoder values...");
//     std::this_thread::sleep_for(std::chrono::milliseconds(500));
// } while (encoders_basic[0] == 0 && encoders_basic[1] == 0);
//
// RCLCPP_INFO(io_node->get_logger(), "Initial encoder values set: [%u, %u]", encoders_basic[0], encoders_basic[1]);


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


*/