#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "GalilMotionController.h"
#include "ctr_kinematics/srv/init_motors.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
    // node class constructor
    TestNode::TestNode() : Node("Test")
    {
        this->_gmc = GalilMotionController();
        this->_init_motors_cli = this->create_client<ctr_kinematics::srv::InitMotors>("InitMotors");
        // this->_timer = this->create_wall_timer(
        //     3000ms,
        //     std::bind(&TestNode::_timerCB, this));
    }

    //timer callback function
    void _timerCB()
    {
        while (!this->_init_motors_cli->wait_for_service(3s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting...");
                rclcpp::shutdown();
                break;
            }
            RCLCPP_INFO(this->get_logger(), "motor_init service not available, waiting again...");
        }

        // init parameters
        auto request = std::make_shared<ctr_kinematics::srv::InitMotors::Request>();
        request->motor_speed_lin = 10;
        request->motor_speed_rot = 10;
        request->motor_speed_coeff = 10;
        request->ip_address.resize(6UL);
        request->ip_address[0] = ("192.168.42.0");
        request->ip_address[1] = ("192.168.42.97");
        request->ip_address[2] = ("192.168.42.2");
        request->ip_address[3] = ("192.168.42.98");
        request->ip_address[4] = ("192.168.42.96");
        request->ip_address[5] = ("192.168.42.99");
        auto response = this->_init_motors_cli->async_send_request(request);
        std::cout << "request sent" << std::endl; // test code
        if (response.get()->is_connect)
        {
            std::cout << "drive motors" << std::endl; // test code
            // this->_gmc.driveMotor(0);
            // this->_gmc.driveMotor(1);
            // this->_gmc.driveMotor(2);
            // this->_gmc.driveMotor(3);
            // this->_gmc.driveMotor(4);
            // this->_gmc.driveMotor(5);
        }
        std::cout << "drive motors???" << std::endl; // test code

        this->_timer->cancel();
        rclcpp::shutdown();
    }

    // galil controller object
    GalilMotionController _gmc;

    // server client
    rclcpp::Client<ctr_kinematics::srv::InitMotors>::SharedPtr _init_motors_cli;

    // action timer
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto test_node = std::make_shared<TestNode>();

    while (!test_node->_init_motors_cli->wait_for_service(3s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_INFO(test_node->get_logger(), "Interrupted while waiting for the service. Exiting...");
            rclcpp::shutdown();
            break;
        }
        RCLCPP_INFO(test_node->get_logger(), "motor_init service not available, waiting again...");
    }

    // init parameters
    auto request = std::make_shared<ctr_kinematics::srv::InitMotors::Request>();
    request->motor_speed_lin = 10;
    request->motor_speed_rot = 10;
    request->motor_speed_coeff = 10;
    request->ip_address.resize(6UL);
    request->ip_address[0] = ("192.168.42.0");
    request->ip_address[1] = ("192.168.42.97");
    request->ip_address[2] = ("192.168.42.2");
    request->ip_address[3] = ("192.168.42.98");
    request->ip_address[4] = ("192.168.42.96");
    request->ip_address[5] = ("192.168.42.99");
    auto response = test_node->_init_motors_cli->async_send_request(request);
    std::cout << "request sent" << std::endl; // test code
    if (rclcpp::spin_until_future_complete(test_node, response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "drive motors" << std::endl; // test code
        // galil_node->_gmc.driveMotor(0);
        // galil_node->_gmc.driveMotor(1);
        // galil_node->_gmc.driveMotor(2);
        // galil_node->_gmc.driveMotor(3);
        // galil_node->_gmc.driveMotor(4);
        // galil_node->_gmc.driveMotor(5);
    }
    std::cout << "drive motors???" << std::endl; // test code
    rclcpp::shutdown();
    return 0;
}