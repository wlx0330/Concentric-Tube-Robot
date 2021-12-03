#include <iostream>
#include <string>
#include <chrono>
#include <map>
#include <vector>
#include <functional>

#include "GalilMotionController.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

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
        

        // // declare motor speed parameter after init motors
        // std::map<std::string, int> default_speed;
        // default_speed["lin"] = 10;                          // default linear motor speed 10 mm/s
        // default_speed["rot"] = 10;                          // default rotation motor speed 10 deg/s
        // default_speed["coeff"] = 10;                        // default speed change rate 10 s^-1
        // this->declare_parameters("robot_speed", default_speed);
    }

private:
    // GMC object
    GalilMotionController _gmc;

    // robot speed set callback handle
    OnSetParametersCallbackHandle::SharedPtr _param_cb_handle;

    // robot speed parameter set callback function
    rcl_interfaces::msg::SetParametersResult _setParamCB(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "";
        std::cout << "lol param call back" << std::endl; //test code
        for (const auto &p : parameters)
        {
            std::cout << "lol param call back" << std::endl; //test code
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
    //void _initMotorCB(const ){}
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GalilNode>());
    rclcpp::shutdown();
    return 0;
}