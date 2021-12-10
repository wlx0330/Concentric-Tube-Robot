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
#include "ctr_kinematics/action/robot_teleop.hpp"

using namespace std::chrono_literals;

class GalilNode : public rclcpp::Node
{
public:
    // node class constructor
    GalilNode::GalilNode() : Node("GalilMotionController")
    {
        // initialize private members
        this->_gmc = GalilMotionController();

        // initialize motor setup service server
        this->_init_motors_srv = this->create_service<ctr_kinematics::srv::InitMotors>(
            "InitMotors",
            std::bind(&GalilNode::_initMotorSrvCB, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);
    }

private:
    // GMC object
    GalilMotionController _gmc;

    // motor init service server
    rclcpp::Service<ctr_kinematics::srv::InitMotors>::SharedPtr _init_motors_srv;

    // motor init service callback function
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
                std::launch::async,
                [this, &address](int i) -> bool
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

        // initialize parameter callback handle for all params
        this->_param_cb_handle = this->add_on_set_parameters_callback(
            std::bind(&GalilNode::_setParamCB, this, std::placeholders::_1));

        // declare motor speed parameter after init motors
        std::map<std::string, int> speed;
        speed["lin"] = request->motor_speed_lin;     // default linear motor speed 10 mm/s
        speed["rot"] = request->motor_speed_rot;     // default rotation motor speed 10 deg/s
        speed["coeff"] = request->motor_speed_coeff; // default speed change rate 10 s^-1
        this->declare_parameters("robot_speed", speed);

        // initialize motor drive service server
        this->_drive_motors_srv = this->create_service<ctr_kinematics::srv::DriveMotors>(
            "DriveMotors",
            std::bind(&GalilNode::_driveMotorsSrvCB, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);

        // init track motor action
        this->_teleop_action_srv = rclcpp_action::create_server<ctr_kinematics::action::RobotTeleop>(
            this, "TeleOperation",
            std::bind(&GalilNode::_TeleopActionHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GalilNode::_TeleopActionHandleCancel, this, std::placeholders::_1),
            std::bind(&GalilNode::_TeleopActionHandleAccept, this, std::placeholders::_1));

        // update service response
        response->is_connect = true;
    }

    // parameter set callback handle
    OnSetParametersCallbackHandle::SharedPtr _param_cb_handle;

    // parameter set callback function
    rcl_interfaces::msg::SetParametersResult _setParamCB(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "";
        for (const auto &p : parameters)
        {
            // set speed for linear motors
            if (p.get_name() == "robot_speed.lin" &&
                p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO(this->get_logger(), "robot_speed.lin parameter value is %s", p.value_to_string().c_str());
                this->_gmc.setMotorSpeedMax(0, p.as_int());
                this->_gmc.setMotorSpeedMax(1, p.as_int());
                this->_gmc.setMotorSpeedMax(2, p.as_int());
                result.successful = true;
                result.reason = "translation motor speed is set";
            }
            // set speed for rotation motor
            else if (p.get_name() == "robot_speed.rot" &&
                     p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO(this->get_logger(), "robot_speed.rot parameter value is %s", p.value_to_string().c_str());
                this->_gmc.setMotorSpeedMax(3, p.as_int());
                this->_gmc.setMotorSpeedMax(4, p.as_int());
                this->_gmc.setMotorSpeedMax(5, p.as_int());
                result.successful = true;
                result.reason = "rotation motor speed is set";
            }
            // set speed change rate
            else if (p.get_name() == "robot_speed.coeff" &&
                     p.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
            {
                RCLCPP_INFO(this->get_logger(), "robot_speed.coeff parameter value is %s", p.value_to_string().c_str());
                for (int i = 0; i < 6; i++)
                {
                    this->_gmc.setMotorSpeedCoeff(i, p.as_int());
                }
                result.successful = true;
                result.reason = "motor speed change rate is set";
            }
        }
        return result;
    }

    // drive motor service server
    rclcpp::Service<ctr_kinematics::srv::DriveMotors>::SharedPtr _drive_motors_srv;

    // drive motor service callback function
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
                std::launch::async,
                [this, &step](int i) -> void
                {
                    this->_gmc.driveMotor(i, step[i]);
                },
                i);
        }
        for (int i = 0; i < step.size(); ++i)
        {
            fut[i].wait();
        }

        //update response
        response->is_motion_complete = true;
    }

    // motor target configuration subscriber
    rclcpp::Subscription<ctr_kinematics::msg::RobotConfig>::SharedPtr _joint_config_sub;

    // motor target configuration sub callback
    void _jointConfigSubCB(const ctr_kinematics::msg::RobotConfig::SharedPtr target_config)
    {
        std::cout << "_jointConfigSubCB" << std::endl; // test code
    }

    //teleop action server object
    rclcpp_action::Server<ctr_kinematics::action::RobotTeleop>::SharedPtr _teleop_action_srv;

    // teleop action goal server
    rclcpp_action::GoalResponse _TeleopActionHandleGoal(
        const rclcpp_action::GoalUUID &uuid,
        // std::shared_ptr<const ctr_kinematics::action::RobotTeleop::Goal> goal
        ctr_kinematics::action::RobotTeleop::Goal::ConstPtr goal)
    {
        if (goal->init_teleop)
        {
            RCLCPP_INFO(this->get_logger(), "Received teleop request, checking motors before action...");

            // TODO move motor to home position

            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        else
        {
            return rclcpp_action::GoalResponse::REJECT;
        }
    }

    // teleop action cancel server
    rclcpp_action::CancelResponse _TeleopActionHandleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::RobotTeleop>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received cancel request, stopping teleop...");

        // TODO stop all motor motion
        // TODO stop the motor pos subscriber

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // teleop action accept server
    void _TeleopActionHandleAccept(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::RobotTeleop>> goal_handle)
    {
        std::thread{std::bind(&GalilNode::_TeleopActionExecute, this, std::placeholders::_1),
                    goal_handle}
            .detach();
    }

    // teleop action execution function
    void _TeleopActionExecute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::RobotTeleop>> goal_handle)
    {
        // TODO check if cancel
        // TODO create motor pos subscriber
        // TODO init feedback
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GalilNode>());
    std::cout << "node ending" << std::endl; //test code
    rclcpp::shutdown();
    return 0;
}