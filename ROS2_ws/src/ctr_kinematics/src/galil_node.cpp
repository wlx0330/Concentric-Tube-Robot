#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <array>
#include <vector>
#include <functional>
#include <memory>
#include <future>

#include "GalilMotionController.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ctr_kinematics/srv/init_motors.hpp"
#include "ctr_kinematics/srv/drive_motors.hpp"

using namespace std::chrono_literals;

class GalilNode : public rclcpp::Node
{
public:
    // node class constructor
    GalilNode::GalilNode() : Node("GalilMotionController")
    {
        // initialize private members
        this->_gmc = GalilMotionController();

        // initialize parameter callback handle for all params
        this->_param_cb_handle = this->add_on_set_parameters_callback(
            std::bind(&GalilNode::_setParamCB, this, std::placeholders::_1));

        // initialize motor setup service server
        this->_init_motors_srv = this->create_service<ctr_kinematics::srv::InitMotors>(
            "InitMotors",
            std::bind(&GalilNode::_initMotorSrvCB, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);

        // initialize motor drive service server
        this->_drive_motors_srv = this->create_service<ctr_kinematics::srv::DriveMotors>(
            "DriveMotors",
            std::bind(&GalilNode::_driveMotorsSrvCB, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);
    }

private:
    // GMC object
    GalilMotionController _gmc;

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

        // init result
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

        // declare motor speed parameter after init motors
        std::map<std::string, int> speed;
        speed["lin"] = request->motor_speed_lin;     // default linear motor speed 10 mm/s
        speed["rot"] = request->motor_speed_rot;     // default rotation motor speed 10 deg/s
        speed["coeff"] = request->motor_speed_coeff; // default speed change rate 10 s^-1
        this->declare_parameters("robot_speed", speed);

        // update service response
        response->is_connect = true;
    }

    // drive motor service server
    rclcpp::Service<ctr_kinematics::srv::DriveMotors>::SharedPtr _drive_motors_srv;

    // drive motor service callback function
    void _driveMotorsSrvCB(
        const ctr_kinematics::srv::DriveMotors::Request::SharedPtr request,
        ctr_kinematics::srv::DriveMotors::Response::SharedPtr response)
    {
        auto temp = request->motor_step_lin;
        //std::vector<int> step;
        // std::vector<int> step;
        // step.reserve(request->motor_step_lin.size() + request->motor_step_rot.size());
        // step.insert(step.end(), request->motor_step_lin.begin(), request->motor_step_lin.end());
        // step.insert(step.end(), request->motor_step_rot.begin(), request->motor_step_rot.end());
        std::vector<std::future<void>> fut(6);

        // const float motor_step = 0.f;

        for (int i = 0; i < 6; ++i)
        {
            fut[i] = std::async(
                std::launch::async,
                [this, &temp](int i) -> void
                {
                    this->_gmc.driveMotor(i);
                    this->_gmc.driveMotor(i, temp[0]);
                },
                i);
        }

        for (int i = 0; i < 6; ++i)
        {
            fut[i].wait();
        }
        response->is_motion_complete = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GalilNode>());
    rclcpp::shutdown();
    return 0;
}