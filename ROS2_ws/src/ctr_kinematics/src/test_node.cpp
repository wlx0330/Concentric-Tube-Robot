#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "GalilMotionController.h"
#include "ctr_kinematics/srv/init_motors.hpp"
#include "ctr_kinematics/srv/drive_motors.hpp"
#include "ctr_kinematics/action/robot_teleop.hpp"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
    // node class constructor
    TestNode::TestNode() : Node("Test")
    {
        this->_gmc = GalilMotionController();
        this->_init_motors_cli = this->create_client<ctr_kinematics::srv::InitMotors>("InitMotors");
        this->teleop_action_cli = rclcpp_action::create_client<ctr_kinematics::action::RobotTeleop>(
            this, "TeleOperation");

        // this->_timer = this->create_wall_timer(
        //     3000ms,
        //     std::bind(&TestNode::_timerCB, this));
    }

    // galil controller object
    GalilMotionController _gmc;

    // server client
    rclcpp::Client<ctr_kinematics::srv::InitMotors>::SharedPtr _init_motors_cli;

    // drive motor client
    rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr _drive_motors_cli;

    // action timer
    rclcpp::TimerBase::SharedPtr _timer;

    //timer callback function
    //void _timerCB() {}

    /////////////////////////// Action Stuff /////////////////////
    // action client object
    rclcpp_action::Client<ctr_kinematics::action::RobotTeleop>::SharedPtr teleop_action_cli;

    // action client send function
    void teleop_action_send()
    {
        if (!this->teleop_action_cli->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }

        auto goal_msg = ctr_kinematics::action::RobotTeleop::Goal();
        goal_msg.init_teleop = true;

        RCLCPP_INFO(this->get_logger(), "Sending goal");
    }

    // action client goal response CB
    void TeleopActionGoalResponseCb(
        std::shared_future<rclcpp_action::ClientGoalHandle<ctr_kinematics::action::RobotTeleop>::SharedPtr>
            future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    // action client feedback CB
    void TeleopActionFeedbackCb(
        rclcpp_action::ClientGoalHandle<ctr_kinematics::action::RobotTeleop>::SharedPtr,
        // const std::shared_ptr<const ctr_kinematics::action::RobotTeleop::Feedback> f,
        const ctr_kinematics::action::RobotTeleop::Feedback::ConstPtr feedback)
    {
        for (int i = 0; i < 3; i++)
        {
            std::cout << feedback->joint_error_rot[0] << std::endl; // test code
        }
    }

    // action result callback
    void TeleopActionResultCb(
        const rclcpp_action::ClientGoalHandle<ctr_kinematics::action::RobotTeleop>::WrappedResult
            &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        }
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
    RCLCPP_INFO(test_node->get_logger(), "motor_inti request has been sent. ");
    if (rclcpp::spin_until_future_complete(test_node, response) == rclcpp::FutureReturnCode::SUCCESS)
    {
        if (response.get()->is_connect)
        {
            RCLCPP_INFO(test_node->get_logger(), "All motors are initialized. Starting motor motion...");
            test_node->_drive_motors_cli = test_node->create_client<ctr_kinematics::srv::DriveMotors>("DriveMotors");
            auto request2 = std::make_shared<ctr_kinematics::srv::DriveMotors::Request>();
            request2->motor_step_lin = {10.f, 10.f, 10.f};
            request2->motor_step_rot = {360.f, 360.f, 360.f};
            auto response2 = test_node->_drive_motors_cli->async_send_request(request2);
            RCLCPP_INFO(test_node->get_logger(), "drive_motor request has been sent. ");
            while (!test_node->_drive_motors_cli->wait_for_service(3s))
            {
                if (!rclcpp::ok())
                {
                    RCLCPP_INFO(test_node->get_logger(), "Interrupted while waiting for the service. Exiting...");
                    rclcpp::shutdown();
                    break;
                }
                RCLCPP_INFO(test_node->get_logger(), "motor_init service not available, waiting again...");
            }
            rclcpp::spin_until_future_complete(test_node, response2);
        }
    }
    rclcpp::shutdown();
    return 0;
}