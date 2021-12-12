#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "GalilMotionController.h"
// #include "ctr_kinematics/srv/init_motors.hpp"
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
        // this->_init_motors_cli = this->create_client<ctr_kinematics::srv::InitMotors>("InitMotors");
        this->teleop_action_cli = rclcpp_action::create_client<ctr_kinematics::action::RobotTeleop>(
            this, "TeleOperation");

        // this->_timer = this->create_wall_timer(
        //     3000ms,
        //     std::bind(&TestNode::_timerCB, this));
    }

    // galil controller object
    GalilMotionController _gmc;

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
        goal_msg.init_config_tran = {0, 0, 0};
        goal_msg.init_config_rot = {90, 90, 90};
        RCLCPP_INFO(this->get_logger(), "Sending goal");

        // set send goal options
        auto send_goal_options = rclcpp_action::Client<ctr_kinematics::action::RobotTeleop>::SendGoalOptions();
        send_goal_options.goal_response_callback = std::bind(
            &TestNode::TeleopActionGoalResponseCb, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(
            &TestNode::TeleopActionFeedbackCb, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback = std::bind(
            &TestNode::TeleopActionResultCb, this, std::placeholders::_1);

        // send goal
        this->teleop_action_cli->async_send_goal(goal_msg, send_goal_options);
    }

    // action client goal response CB
    void TeleopActionGoalResponseCb(
        std::shared_future<rclcpp_action::ClientGoalHandle<ctr_kinematics::action::RobotTeleop>::SharedPtr>
            future)
    {
        std::cout << "action client goal response cb" << std::endl; //test code
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
        this->teleop_action_cli->async_cancel_all_goals();
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

    if (1)
    {
        RCLCPP_INFO(test_node->get_logger(), "All motors are initialized. Starting motor motion...");
        test_node->_drive_motors_cli = test_node->create_client<ctr_kinematics::srv::DriveMotors>("DriveMotors");
        auto request2 = std::make_shared<ctr_kinematics::srv::DriveMotors::Request>();
        request2->motor_step_lin = {0.f, 0.f, 0.f};
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

        // action test
        test_node->teleop_action_send();
        rclcpp::spin(test_node);
    }
    rclcpp::shutdown();
    return 0;
}