#include <chrono>
#include <iostream>
#include <string>

#include "GalilMotionController.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ctr_kinematics/msg/robot_config.hpp"
#include "ctr_kinematics/srv/solve_kinematics.hpp"
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

        // this->teleop_action_cli = rclcpp_action::create_client<ctr_kinematics::action::RobotTeleop>(
        //     this, "TeleOperation");

        // this->_timer = this->create_wall_timer(
        //     3000ms,
        //     std::bind(&TestNode::_timerCB, this));
    }

    // galil controller object
    GalilMotionController _gmc;

    // drive motor client
    rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr _drive_motors_cli;

    // set motor home client
    rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr set_motor_home_cli_;

    // ctr fk client
    rclcpp::Client<ctr_kinematics::srv::SolveKinematics>::SharedPtr solve_ctr_fk_cli_;

    // ctr ik client
    rclcpp::Client<ctr_kinematics::srv::SolveKinematics>::SharedPtr solve_ctr_ik_cli_;

    // action timer
    rclcpp::TimerBase::SharedPtr timer;

    //timer callback function
    void CbTimer()
    {
        auto message = ctr_kinematics::msg::RobotConfig();
        message.rot_joint = {};
        message.tran_joint = {0, 0, 0};
    }

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

    // set speed
    auto param_cli = std::make_shared<rclcpp::SyncParametersClient>(
        test_node, "GalilMotionController");
    while (!param_cli->wait_for_service(3s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor not ready, waiting again...");
    }
    std::vector<rclcpp::Parameter> motor_speed;
    motor_speed.push_back(rclcpp::Parameter("robot_speed_lin", 10));
    motor_speed.push_back(rclcpp::Parameter("robot_speed_rot", 60));
    //motor_speed.push_back(rclcpp::Parameter("robot_speed_coeff", 10)); //
    auto results = param_cli->set_parameters(motor_speed);

    // motor motion
    RCLCPP_INFO(test_node->get_logger(), "All motors are initialized. Starting motor motion...");
    test_node->_drive_motors_cli = test_node->create_client<ctr_kinematics::srv::DriveMotors>("DriveMotors");
    while (!test_node->_drive_motors_cli->wait_for_service(3s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor not ready, waiting again...");
    }
    auto request = std::make_shared<ctr_kinematics::srv::DriveMotors::Request>();
    request->motor_step_lin = {0.f, 0.f, 0.f};
    request->motor_step_rot = {360.f, 360.f, 360.f};
    auto response = test_node->_drive_motors_cli->async_send_request(request);
    RCLCPP_INFO(test_node->get_logger(), "drive_motor request has been sent. ");
    rclcpp::spin_until_future_complete(test_node, response);

    // set motor current position
    test_node->set_motor_home_cli_ = test_node->create_client<ctr_kinematics::srv::DriveMotors>("SetHome");
    while (!test_node->set_motor_home_cli_->wait_for_service(3s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor not ready, waiting again...");
    }
    auto req2 = std::make_shared<ctr_kinematics::srv::DriveMotors::Request>();
    req2->motor_step_lin = {0, 0, 0};
    req2->motor_step_rot = {0, 0, 0};
    auto res2 = test_node->set_motor_home_cli_->async_send_request(req2);
    RCLCPP_INFO(test_node->get_logger(), "set motor home request has been sent. ");
    rclcpp::spin_until_future_complete(test_node, res2);

    // motor config publisher
    auto target_config_pub_ =
        test_node->create_publisher<ctr_kinematics::msg::RobotConfig>("TargetConfig", 10);
    auto message = ctr_kinematics::msg::RobotConfig();

    rclcpp::Rate loop_rate(100ms);
    int end = 360;
    int step = 3;
    for (int start = 0; start <= end; start += step)
    {
        message.rot_joint = {(float)start, (float)start, (float)start};
        message.tran_joint = {0, 0, 0};
        target_config_pub_->publish(message);
        loop_rate.sleep();
    }

    //test kineamtics node fk server
    test_node->solve_ctr_fk_cli_ =
        test_node->create_client<ctr_kinematics::srv::SolveKinematics>("SolveCtrFk");
    while (!test_node->solve_ctr_fk_cli_->wait_for_service(3s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor not ready, waiting again...");
    }
    auto req3 = std::make_shared<ctr_kinematics::srv::SolveKinematics::Request>();
    req3->fk_config_rot = {90.0, 0, 0}; //deg
    req3->fk_config_tran = {0, 0, 0};
    auto res3 = test_node->solve_ctr_fk_cli_->async_send_request(req3);
    RCLCPP_INFO(test_node->get_logger(), "ctr fk request has been sent. ");
    rclcpp::spin_until_future_complete(test_node, res3);
    auto tip = res3.get()->tip_coord;
    std::cout << "x = " << tip[0] << " y = " << tip[1] << " z = " << tip[2] << std::endl; //tc

    //test kineamtics node ik server
    test_node->solve_ctr_ik_cli_ =
        test_node->create_client<ctr_kinematics::srv::SolveKinematics>("SolveCtrIk");
    while (!test_node->solve_ctr_ik_cli_->wait_for_service(3s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor not ready, waiting again...");
    }
    req3->ik_target_coord = tip;
    req3->ik_target_coord[1] -= 0.001;
    auto res4 = test_node->solve_ctr_ik_cli_->async_send_request(req3);
    RCLCPP_INFO(test_node->get_logger(), "ctr ik request has been sent. ");
    rclcpp::spin_until_future_complete(test_node, res4);
    auto tran_new = res4.get()->config_tran;
    auto rot_new = res4.get()->config_rot;
    std::cout << "tube 1 tran = " << tran_new[0] << ", rot = " << rot_new[0] << std::endl;
    std::cout << "tube 2 tran = " << tran_new[1] << ", rot = " << rot_new[1] << std::endl;
    std::cout << "tube 3 tran = " << tran_new[2] << ", rot = " << rot_new[2] << std::endl;
    std::cout << "x = " << tip[0] << " y = " << tip[1] << " z = " << tip[2] << std::endl; //tc

    //

    //test_node->timer = rclcpp::create_wall_timer(100ms, std::bind(&TestNode::))

    // action test
    // test_node->teleop_action_send();
    // rclcpp::spin(test_node);

    rclcpp::shutdown();
    return 0;
}