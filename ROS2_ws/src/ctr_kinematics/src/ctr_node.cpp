#include <iostream>
#include <string>
#include <chrono>

#include "Controller.h"
#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ctr_kinematics/msg/robot_config.hpp"
#include "ctr_kinematics/srv/drive_motors.hpp"

using namespace std::chrono_literals;

class CTRNode : public rclcpp::Node
{
public:
  // node class constructor
  CTRNode::CTRNode() : Node("ConcentricTubeRobot")
  {
  }

  void InitCtrNode()
  {
    this->SetRobotSpeed(10, 60, 10);
    this->DriveMotor({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
  }

private:
  /* ROS2 Parameters */

  // set parameter function

  /* ROS2 Topics */

  // motor target config publisher
  rclcpp::Publisher<ctr_kinematics::msg::RobotConfig>::SharedPtr target_config_pub_;
  void PubMotorConfig()
  {
  }

  /* ROS2 Services */

  // set motor speed (tranlastion speed mm/s, rotation speed deg/s, speed coefficient)
  rclcpp::SyncParametersClient::SharedPtr param_cli_;
  void SetRobotSpeed(const int &tran, const int &rot, const int &coeff)
  {
    this->param_cli_ = std::make_shared<rclcpp::SyncParametersClient>(
        this->shared_from_this(), "GalilMotionController"); //this shared ptr
    while (!param_cli_->wait_for_service(3s))
    {
      if (!rclcpp::ok())
      {
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Parameter motor_speed is not found, waiting for decleration...");
    }
    std::vector<rclcpp::Parameter> motor_speed;
    motor_speed.push_back(rclcpp::Parameter("robot_speed_lin", 10));
    motor_speed.push_back(rclcpp::Parameter("robot_speed_rot", 60));
    motor_speed.push_back(rclcpp::Parameter("robot_speed_coeff", 10));
    std::cout << motor_speed.size() << std::endl; //tc
    std::vector<rcl_interfaces::msg::SetParametersResult> result =
        param_cli_->set_parameters(motor_speed);
    std::cout << "??" << std::endl; //tc
    //std::cout << "???" << std::endl; //tc
    RCLCPP_INFO(this->get_logger(), "Sending request to change robot_speed parameters.");
    std::cout << "?????" << std::endl; //tc
  }

  // drive motor with step
  rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr drive_motor_cli_;
  void DriveMotor(const std::array<double, 3> &motor_tran,
                  const std::array<double, 3> &motor_rot)
  {
    this->drive_motor_cli_ =
        this->create_client<ctr_kinematics::srv::DriveMotors>("DriveMotors");
    while (!this->drive_motor_cli_->wait_for_service(3s))
    {
      if (!rclcpp::ok())
      {
        rclcpp::shutdown();
      }
      RCLCPP_INFO(this->get_logger(),
                  "Motors are not ready, waiting for Initialization...");
    }
    auto request = std::make_shared<ctr_kinematics::srv::DriveMotors::Request>();
    request->motor_step_lin = {motor_tran[0], motor_tran[1], motor_tran[2]};
    request->motor_step_rot = {motor_rot[0], motor_rot[1], motor_rot[2]};
    auto response = this->drive_motor_cli_->async_send_request(request);
    RCLCPP_INFO(this->get_logger(), "Sending request to drive motors.");
    response.wait();
    std::cout << "okok" << std::endl; //tc
  }

  /* ROS2 Actions */

  // CTR FK action server

  // CTR IK action server
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<CTRNode>());
  auto ctr_node = std::make_shared<CTRNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(ctr_node);
  ctr_node->InitCtrNode();
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
