#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
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
    // this->param_cli_ = std::make_shared<rclcpp::SyncParametersClient>(
    //     this, "GalilMotionController"); //this shared ptr
  }

  void InitCtrNode()
  {
    std::cout << "init ctr" << std::endl; //tc
    this->SetRobotSpeed(10, 60, 10);
    this->DriveMotor({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
  }

  // set motor speed (tranlastion speed mm/s, rotation speed deg/s, speed coefficient)
  void SetRobotSpeed(const int &tran, const int &rot, const int &coeff)
  {
    auto param_cli = std::make_shared<rclcpp::SyncParametersClient>(
        this, "GalilMotionController");
    while (!param_cli->wait_for_service(3s))
    {
      if (!rclcpp::ok())
      {
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Parameter motor_speed is not found, waiting for decleration...");
    }
    std::vector<rclcpp::Parameter> motor_speed;
    motor_speed.push_back(rclcpp::Parameter("robot_speed_lin", tran));
    motor_speed.push_back(rclcpp::Parameter("robot_speed_rot", rot));
    motor_speed.push_back(rclcpp::Parameter("robot_speed_coeff", coeff));
    RCLCPP_INFO(this->get_logger(), "Sending request to change robot_speed parameters.");
    auto results = param_cli->set_parameters(motor_speed);
  }

  void DriveMotor(const std::array<double, 3> &motor_tran,
                  const std::array<double, 3> &motor_rot)
  {
    this->drive_motor_cli_ =
        this->create_client<ctr_kinematics::srv::DriveMotors>("DriveMotors");
    while (!this->drive_motor_cli_->wait_for_service(3s))
    {
      if (!rclcpp::ok())
      {
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Motors are not ready, waiting for Initialization...");
    }
    auto request = std::make_shared<ctr_kinematics::srv::DriveMotors::Request>();
    request->motor_step_lin = {motor_tran[0], motor_tran[1], motor_tran[2]};
    request->motor_step_rot = {motor_rot[0], motor_rot[1], motor_rot[2]};
    RCLCPP_INFO(this->get_logger(), "Sending request to drive motors.");
    auto response = this->drive_motor_cli_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->shared_from_this(), response);
  }

  void SetMotorHome(const std::array<double, 3> &motor_tran,
                    const std::array<double, 3> &motor_rot)
  {
    this->set_motor_home_cli_ =
        this->create_client<ctr_kinematics::srv::DriveMotors>("SetHome");
    while (!this->set_motor_home_cli_->wait_for_service(3s))
    {
      if (!rclcpp::ok())
      {
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service SetMotorHome not ready, waiting...");
    }
    auto request = std::make_shared<ctr_kinematics::srv::DriveMotors::Request>();
    request->motor_step_lin = {motor_tran[0], motor_tran[1], motor_tran[2]};
    request->motor_step_rot = {motor_rot[0], motor_rot[1], motor_rot[2]};
    RCLCPP_INFO(this->get_logger(), "Sending request to set home configuration.");
    auto response = this->set_motor_home_cli_->async_send_request(request);
    rclcpp::spin_until_future_complete(this->shared_from_this(), response);
  }

private:
  /* ROS2 Parameters */

  /* ROS2 Topics */

  // motor target config publisher
  rclcpp::Publisher<ctr_kinematics::msg::RobotConfig>::SharedPtr target_config_pub_;
  void PubMotorConfig()
  {
  }

  /* ROS2 Services */

  // drive motor with step
  rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr drive_motor_cli_;

  // set motor home configuration service client
  rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr set_motor_home_cli_;

  /* ROS2 Actions */

  // CTR FK action server

  // CTR IK action server
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<CTRNode>());
  auto ctr_node = std::make_shared<CTRNode>();
  ctr_node->SetRobotSpeed(10, 60, 10);
  ctr_node->DriveMotor({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
  ctr_node->SetMotorHome({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});

  // action clients for FK and IK

  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(ctr_node);
  // rclcpp::spin(ctr_node);

  rclcpp::shutdown();
  return 0;
}
