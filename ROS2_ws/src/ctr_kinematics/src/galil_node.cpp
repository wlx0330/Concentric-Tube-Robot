#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <vector>
#include <functional>
#include <memory>

#include "GalilMotionController.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ctr_kinematics/srv/init_motors.hpp"

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
    }

private:
    // GMC object
    GalilMotionController _gmc;

    // parameter set callback handle
    OnSetParametersCallbackHandle::SharedPtr _param_cb_handle;

    // robot init service server
    rclcpp::Service<ctr_kinematics::srv::InitMotors>::SharedPtr _init_motors_srv;

    // robot speed parameter set callback function
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

    // motor init service callback function
    void _initMotorSrvCB(
        const ctr_kinematics::srv::InitMotors::Request::SharedPtr request,
        ctr_kinematics::srv::InitMotors::Response::SharedPtr response)
    {
        // connect motors to ip
        auto address = request->ip_address;
        for (int i = 0; i < address.size(); ++i)
        {
            this->_gmc.connectMotor(i, address[i]);
        }

        // init motors
        if (this->_gmc.initMotors())
        {
            RCLCPP_INFO(this->get_logger(), "Motor initialization SUCCESS! ");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Motor initialization FAIL! ");
            response->is_connect = false;
            return;
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GalilNode>());
    rclcpp::shutdown();
    return 0;
}