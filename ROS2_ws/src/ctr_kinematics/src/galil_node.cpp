#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <array>
#include <vector>
#include <algorithm>
#include <functional>
#include <memory>
#include <future>
#include <thread>

#include "GalilMotionController.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ctr_kinematics/msg/robot_config.hpp"
#include "ctr_kinematics/srv/init_motors.hpp"
#include "ctr_kinematics/srv/drive_motors.hpp"
#include "ctr_kinematics/srv/enable_actions.hpp"
#include "ctr_kinematics/action/robot_teleop.hpp"

using namespace std::chrono_literals;

class GalilNode : public rclcpp::Node
{
public:
    // node class constructor
    GalilNode::GalilNode() : Node("GalilMotionController")
    {
        // init galil motors
        std::vector<std::string> ip_address(6);
        ip_address[0] = "192.168.42.0";  // inner tube tran motor
        ip_address[1] = "192.168.42.97"; // middle tube tran motor
        ip_address[2] = "192.168.42.2";  // outer tube tran motor
        ip_address[3] = "192.168.42.98"; // inner tube rot motor
        ip_address[4] = "192.168.42.96"; // middle tube rot motor
        ip_address[5] = "192.168.42.99"; // outer tube rot motor
        bool is_ready = this->InitGalilMotors(ip_address);

        // set default galil motor parameters
        this->_param_cb_handle = this->add_on_set_parameters_callback(
            std::bind(&GalilNode::_setParamCB, this, std::placeholders::_1));
        this->declare_parameter("robot_speed_lin", 10);
        this->declare_parameter("robot_speed_rot", 10);
        this->declare_parameter("robot_speed_coeff", 10);

        // TODO start sub and pub
        // TODO init calib srv/action
        // this->declare_parameter("is_motor_ready", false);
        // this->declare_parameter("is_robot_ready", is_ready);

        // initialize motor setup service server
        // this->_init_motors_srv = this->create_service<ctr_kinematics::srv::InitMotors>(
        //     "InitMotors",
        //     std::bind(&GalilNode::_initMotorSrvCB, this, std::placeholders::_1, std::placeholders::_2),
        //     rmw_qos_profile_services_default);
    }

private:
    // GMC object
    GalilMotionController _gmc;

    // init motors function
    bool InitGalilMotors(const std::vector<std::string> &ip_vec)
    {
        // initialize galil motion controller
        this->_gmc = GalilMotionController();
        // async motor init
        std::vector<std::future<bool>> fut_vec(ip_vec.size());
        for (int i = 0; i < ip_vec.size(); ++i)
        {
            // set lambda function
            fut_vec[i] = std::async(
                std::launch::async, [this, &ip_vec](int i) -> bool
                {
                    if (this->_gmc.connectMotor(i, ip_vec[i]))
                    {
                        return this->_gmc.initMotor(i);
                    }
                    else
                    {
                        return false;
                    }
                },
                i);
        }
        for (int i = 0; i < ip_vec.size(); ++i)
        {
            if (fut_vec[i].get())
            {
                RCLCPP_INFO(this->get_logger(), "Motor %d initialization SUCCESS!", i);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Motor %d initialization FAIL!", i);
                return false;
            }
        }
        return true;
    }

    /* ROS2 Topics */

    // motor target configuration subscriber
    rclcpp::Subscription<ctr_kinematics::msg::RobotConfig>::SharedPtr _joint_config_sub;
    void _jointConfigSubCB(const ctr_kinematics::msg::RobotConfig::SharedPtr target_config)
    {
        std::cout << "_jointConfigSubCB" << std::endl; // test code
    }

    /* ROS2 Parameters */

    // parameter related service
    OnSetParametersCallbackHandle::SharedPtr _param_cb_handle;
    rcl_interfaces::msg::SetParametersResult _setParamCB(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "";
        for (const auto &p : parameters)
        {
            // set speed for linear motors
            if (p.get_name() == "robot_speed_lin" &&
                p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO(this->get_logger(), "Parameter robot_speed_lin is set to %s", p.value_to_string().c_str());
                this->_gmc.setMotorSpeedMax(0, p.as_int());
                this->_gmc.setMotorSpeedMax(1, p.as_int());
                this->_gmc.setMotorSpeedMax(2, p.as_int());
                result.successful = true;
                result.reason = "translation motor speed is set";
            }
            // set speed for rotation motor
            else if (p.get_name() == "robot_speed_rot" &&
                     p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO(this->get_logger(), "Parameter robot_speed_rot is set to %s", p.value_to_string().c_str());
                this->_gmc.setMotorSpeedMax(3, p.as_int());
                this->_gmc.setMotorSpeedMax(4, p.as_int());
                this->_gmc.setMotorSpeedMax(5, p.as_int());
                result.successful = true;
                result.reason = "rotation motor speed is set";
            }
            // set speed change rate
            else if (p.get_name() == "robot_speed_coeff" &&
                     p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO(this->get_logger(), "Parameter robot_speed_coeff is set to %s", p.value_to_string().c_str());
                for (int i = 0; i < 6; i++)
                {
                    this->_gmc.setMotorSpeedCoeff(i, p.as_int());
                }
                result.successful = true;
                result.reason = "motor speed change rate is set";
            }
            // enable services and actions when robot is ready
            else if (p.get_name() == "is_robot_ready" &&
                     p.get_type() == rclcpp::ParameterType::PARAMETER_BOOL)
            {
                if (p.as_bool())
                {
                    RCLCPP_INFO(this->get_logger(), "robot is ready, enabling features...");

                    // initialize motor drive service server
                    this->_drive_motors_srv = this->create_service<ctr_kinematics::srv::DriveMotors>(
                        "DriveMotors",
                        std::bind(&GalilNode::_driveMotorsSrvCB, this, std::placeholders::_1, std::placeholders::_2),
                        rmw_qos_profile_services_default);
                    RCLCPP_INFO(this->get_logger(), "galil_node drive_motor service is enabled.");

                    // teleop action
                    this->_teleop_action_srv = rclcpp_action::create_server<ctr_kinematics::action::RobotTeleop>(
                        this, "TeleOperation",
                        std::bind(&GalilNode::_TeleopActionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
                        std::bind(&GalilNode::_TeleopActionHandleCancel, this, std::placeholders::_1),
                        std::bind(&GalilNode::_TeleopActionHandleAccept, this, std::placeholders::_1));
                    RCLCPP_INFO(this->get_logger(), "galil_node teleop action is enabled.");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "robot is NOT ready");
                }
            }
        }
        return result;
    }

    /* ROS2 Services */

    // motor init service
    rclcpp::Service<ctr_kinematics::srv::InitMotors>::SharedPtr _init_motors_srv;
    void _initMotorSrvCB(
        const ctr_kinematics::srv::InitMotors::Request::SharedPtr request,
        ctr_kinematics::srv::InitMotors::Response::SharedPtr response)
    {
        // connect motors to ip
        auto address = request->ip_address;
        std::vector<std::future<bool>> fut(address.size());

        // async motor init
        for (int i = 0; i < address.size(); ++i)
        {
            // set lambda function
            fut[i] = std::async(
                std::launch::async, [this, &address](int i) -> bool
                {
                    if (this->_gmc.connectMotor(i, address[i]))
                    {
                        return this->_gmc.initMotor(i);
                    }
                    else
                    {
                        return false;
                    }
                },
                i);
        }
        for (int i = 0; i < address.size(); ++i)
        {
            if (fut[i].get())
            {
                RCLCPP_INFO(this->get_logger(), "Motor %d initialization SUCCESS!", i);
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Motor %d initialization FAIL!", i);
                response->is_connect = false;
                return;
            }
        }

        // set motor speed parameter after init motors
        this->set_parameter(rclcpp::Parameter("robot_speed_lin", request->motor_speed_lin));
        this->set_parameter(rclcpp::Parameter("robot_speed_rot", request->motor_speed_rot));
        this->set_parameter(rclcpp::Parameter("robot_speed_coeff", request->motor_speed_coeff));
        this->set_parameter(rclcpp::Parameter("is_robot_ready", true));

        // update service response
        response->is_connect = true;
    }

    // drive motor service
    rclcpp::Service<ctr_kinematics::srv::DriveMotors>::SharedPtr _drive_motors_srv;
    void _driveMotorsSrvCB(
        const ctr_kinematics::srv::DriveMotors::Request::SharedPtr request,
        ctr_kinematics::srv::DriveMotors::Response::SharedPtr response)
    {
        std::vector<float> step(6);
        for (int i = 0; i < request->motor_step_lin.size(); ++i)
        {
            step[i] = request->motor_step_lin[i];
        }
        for (int i = 0; i < request->motor_step_rot.size(); ++i)
        {
            step[3 + i] = request->motor_step_rot[i];
        }

        // set up async execution
        std::vector<std::future<void>> fut(6);
        for (int i = 0; i < step.size(); ++i)
        {
            fut[i] = std::async(
                std::launch::async, [this, &step](int i) -> void
                { this->_gmc.driveMotor(i, step[i]); },
                i);
        }
        for (int i = 0; i < step.size(); ++i)
        {
            fut[i].wait();
        }

        //update response
        response->is_motion_complete = true;
    }

    // enable teleop action service
    rclcpp::Service<ctr_kinematics::srv::EnableActions>::SharedPtr enable_actions_srv_;
    void EnableActionsSrvCb(
        const ctr_kinematics::srv::EnableActions::Request::SharedPtr request,
        ctr_kinematics::srv::EnableActions::Response::SharedPtr response)
    {
    }

    /* ROS2 Actions */

    // teleop action
    rclcpp_action::Server<ctr_kinematics::action::RobotTeleop>::SharedPtr _teleop_action_srv;
    rclcpp_action::GoalResponse _TeleopActionHandleGoal(
        const rclcpp_action::GoalUUID &uuid,
        // std::shared_ptr<const ctr_kinematics::action::RobotTeleop::Goal> goal
        ctr_kinematics::action::RobotTeleop::Goal::ConstPtr goal)
    {
        // move motor to start position
        RCLCPP_INFO(this->get_logger(), "Request accepted, moving motors to start location...");
        std::array<float, 6> step;
        for (int i = 0; i < 3; ++i)
        {
            step[i] = goal->init_config_tran[i];
            step[i + 3] = goal->init_config_rot[i];
        }

        // set up async execution
        std::vector<std::future<void>> fut(6);
        for (int i = 0; i < step.size(); ++i)
        {
            fut[i] = std::async(
                std::launch::async, [this, &step](int i) -> void
                { this->_gmc.driveMotor(i, step[i]); },
                i);
        }
        for (int i = 0; i < step.size(); ++i)
        {
            fut[i].wait();
        }

        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse _TeleopActionHandleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::RobotTeleop>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request, stopping teleop...");
        this->_gmc.stopAllMotors(); // TODO stop all motor motion

        // TODO stop the motor pos subscriber
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void _TeleopActionHandleAccept(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::RobotTeleop>> goal_handle)
    {
        std::thread{std::bind(&GalilNode::_TeleopActionExecute, this, std::placeholders::_1),
                    goal_handle}
            .detach();
    }

    void _TeleopActionExecute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::RobotTeleop>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "running teleop");
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<ctr_kinematics::action::RobotTeleop::Feedback>();
        auto result = std::make_shared<ctr_kinematics::action::RobotTeleop::Result>();
        feedback->joint_error_rot = {0, 1, 2};
        feedback->joint_error_tran = {0, 1, 2};

        // init clock
        rclcpp::Rate loop_clock(1s);

        while (rclcpp::ok())
        {

            // TODO check if cancel
            if (goal_handle->is_canceling())
            {
                //result->start_teleop = false;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "goal canceled");
                return;
            }

            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "publish feedback");

            loop_clock.sleep();

            // TODO create motor pos subscriber
            // TODO init feedback
        }

        // check if goal is done
        if (rclcpp::ok())
        {
            // result->start_teleop = false;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "goal succeeded");
        }
    }

    // other actions
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GalilNode>());
    std::cout << "after spin" << std::endl; //tc
    rclcpp::shutdown();
    return 0;
}