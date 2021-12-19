#include <iostream>
#include <string>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ctr_kinematics/msg/robot_config.hpp"
#include "ctr_kinematics/srv/drive_motors.hpp"
#include "ctr_kinematics/action/kinematics.hpp"

using namespace std::chrono_literals;

class CTRNode : public rclcpp::Node
{
public:
  // node class constructor
  CTRNode::CTRNode() : Node("ConcentricTubeRobot")
  {
    this->CreateFkActionClient();
    this->CreateIkActionClient();
    this->CreateTargetConfigPublisher();
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

  // drive motors with specific step size
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

  // set motor home position
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

  // publish motor target config
  void PubMotorConfig(const std::array<double, 3> &motor_tran,
                      const std::array<double, 3> &motor_rot)
  {
    auto config = ctr_kinematics::msg::RobotConfig();
    config.tran_joint = motor_tran;
    config.rot_joint = motor_rot;
    RCLCPP_INFO(this->get_logger(), "Sending target configurations to motors...");
    this->target_config_pub_->publish(config);
  }

  // solve CTR FK
  void SolveCtrFk(const std::array<double, 3> &ctr_tran,
                  const std::array<double, 3> &ctr_rot)
  {
    while (!this->fk_action_cli_->wait_for_action_server(3s))
    {
      if (!rclcpp::ok())
      {
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Action server forward kinematics not ready, waiting...");
    }
    auto goal_msg = ctr_kinematics::action::Kinematics::Goal();
    goal_msg.fk_tran = {ctr_tran[0], ctr_tran[1], ctr_tran[2]};
    goal_msg.fk_rot = {ctr_rot[0], ctr_rot[1], ctr_rot[2]};
    auto goal_options =
        rclcpp_action::Client<ctr_kinematics::action::Kinematics>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(
        &CTRNode::CbFkActionGoalResponse, this, std::placeholders::_1);
    goal_options.feedback_callback = std::bind(
        &CTRNode::CbFkActionFeedback, this, std::placeholders::_1, std::placeholders::_2);
    goal_options.result_callback = std::bind(
        &CTRNode::CbFkActionResult, this, std::placeholders::_1);
    RCLCPP_INFO(this->get_logger(), "Sending goal to CTR FK action server.");
    auto f_goal_handle = this->fk_action_cli_->async_send_goal(goal_msg, goal_options);
    rclcpp::spin_until_future_complete(this->shared_from_this(), f_goal_handle);
    auto f_result_handle = f_goal_handle.get()->async_result();
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), f_result_handle, 1s) ==
        rclcpp::FutureReturnCode::TIMEOUT)
    {
      this->fk_action_cli_->async_cancel_all_goals();
      std::cout << "time out" << std::endl; //tc
    }
  }

  // solve CTR IK
  void SolveCtrIk(const std::array<double, 3> &init_tran,
                  const std::array<double, 3> &init_rot,
                  const std::array<double, 3> &goal_coord)
  {
    while (!this->ik_action_cli_->wait_for_action_server(3s))
    {
      if (!rclcpp::ok())
      {
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Action server inverse kinematics not ready, waiting...");
    }
    auto goal_msg = ctr_kinematics::action::Kinematics::Goal();
    goal_msg.fk_tran = {init_tran[0], init_tran[1], init_tran[2]};
    goal_msg.fk_rot = {init_rot[0], init_rot[1], init_rot[2]};
    goal_msg.ik_goal = {goal_coord[0], goal_coord[1], goal_coord[2]};
    auto goal_options =
        rclcpp_action::Client<ctr_kinematics::action::Kinematics>::SendGoalOptions();
    goal_options.goal_response_callback = std::bind(
        &CTRNode::CbIkActionGoalResponse, this, std::placeholders::_1);
    goal_options.feedback_callback = std::bind(
        &CTRNode::CbIkActionFeedback, this, std::placeholders::_1, std::placeholders::_2);
    goal_options.result_callback = std::bind(
        &CTRNode::CbIkActionResult, this, std::placeholders::_1);
    RCLCPP_INFO(this->get_logger(), "Sending goal to CTR IK action server.");
    auto f_goal_handle = this->ik_action_cli_->async_send_goal(goal_msg, goal_options);
    rclcpp::spin_until_future_complete(this->shared_from_this(), f_goal_handle);
    auto f_result_handle = f_goal_handle.get()->async_result();
    if (rclcpp::spin_until_future_complete(this->shared_from_this(), f_result_handle, 1s) ==
        rclcpp::FutureReturnCode::TIMEOUT)
    {
      this->ik_action_cli_->async_cancel_all_goals();
      std::cout << "time out" << std::endl; //tc
    }
  }

private:
  // kinematics timer
  rclcpp::TimerBase::SharedPtr kinematics_timer_;
  void CbKinematicsTimer()
  {
  }

  /* ROS2 Parameters */

  /* ROS2 Topics */

  // motor target config publisher
  rclcpp::Publisher<ctr_kinematics::msg::RobotConfig>::SharedPtr target_config_pub_;
  void CreateTargetConfigPublisher()
  {
    this->target_config_pub_ = this->create_publisher<ctr_kinematics::msg::RobotConfig>(
        "TargetConfig", 10);
  }

  /* ROS2 Services */

  // drive motor with step
  rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr drive_motor_cli_;

  // set motor home configuration service client
  rclcpp::Client<ctr_kinematics::srv::DriveMotors>::SharedPtr set_motor_home_cli_;

  /* ROS2 Actions */

  // CTR FK action server
  rclcpp_action::Client<ctr_kinematics::action::Kinematics>::SharedPtr fk_action_cli_;
  void CreateFkActionClient()
  {
    this->fk_action_cli_ = rclcpp_action::create_client<
        ctr_kinematics::action::Kinematics>(this, "ForwardKinematics");
  }
  void CbFkActionGoalResponse(
      std::shared_future<rclcpp_action::ClientGoalHandle<ctr_kinematics::action::Kinematics>::SharedPtr>
          future)
  {
    if (!future.get())
    {
      RCLCPP_ERROR(this->get_logger(), "Goal request is rejected by the action server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal request is accepted by the action server");
    }
  }
  void CbFkActionFeedback(
      rclcpp_action::ClientGoalHandle<ctr_kinematics::action::Kinematics>::SharedPtr,
      const ctr_kinematics::action::Kinematics::Feedback::ConstPtr feedback) {}
  void CbFkActionResult(
      const rclcpp_action::ClientGoalHandle<ctr_kinematics::action::Kinematics>::WrappedResult
          &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
      auto ret = result.result->tip_coord;
      std::cout << ret[0] << " " << ret[1] << " " << ret[2] << std::endl; //tc
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

  // CTR IK action server
  rclcpp_action::Client<ctr_kinematics::action::Kinematics>::SharedPtr ik_action_cli_;
  void CreateIkActionClient()
  {
    this->ik_action_cli_ = rclcpp_action::create_client<
        ctr_kinematics::action::Kinematics>(this, "InverseKinematics");
  }
  void CbIkActionGoalResponse(
      std::shared_future<rclcpp_action::ClientGoalHandle<ctr_kinematics::action::Kinematics>::SharedPtr>
          future)
  {
    if (!future.get())
    {
      RCLCPP_ERROR(this->get_logger(), "Goal request is rejected by the action server");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal request is accepted by the action server");
    }
  }
  void CbIkActionFeedback(
      rclcpp_action::ClientGoalHandle<ctr_kinematics::action::Kinematics>::SharedPtr,
      const ctr_kinematics::action::Kinematics::Feedback::ConstPtr feedback) {}
  void CbIkActionResult(
      const rclcpp_action::ClientGoalHandle<ctr_kinematics::action::Kinematics>::WrappedResult
          &result)
  {
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
    {
      auto tran = result.result->robot_tran;
      auto rot = result.result->robot_rot;
      auto ret = result.result->tip_coord;
      std::cout << tran[0] << " " << tran[1] << " " << tran[2] << std::endl; //tc
      std::cout << rot[0] << " " << rot[1] << " " << rot[2] << std::endl;    //tc
      std::cout << ret[0] << " " << ret[1] << " " << ret[2] << std::endl;    //tc

      // TODO pub coord
      this->PubMotorConfig(tran, rot);

      break;
    }
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
  // rclcpp::spin(std::make_shared<CTRNode>());
  auto ctr_node = std::make_shared<CTRNode>();
  ctr_node->SetRobotSpeed(10, 60, 10);
  ctr_node->SetMotorHome({0.0, 0.0, 0.0}, {0, 0.0, 0.0});
  ctr_node->DriveMotor({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
  rclcpp::Rate gap(100ms);
  ctr_node->SolveCtrFk({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
  gap.sleep();
  ctr_node->SolveCtrIk({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0}, {0.010, -0.0218, 0.0523});
  gap.sleep();
  ctr_node->SolveCtrIk({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0}, {0.010, -0.0218, 0.06});
  gap.sleep();
  ctr_node->SolveCtrIk({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0}, {0.012, -0.0218, 0.06});
  gap.sleep();
  ctr_node->PubMotorConfig({0.0, 0.0, 0.0}, {90.0, 0.0, 0.0});
  gap.sleep();

  // action clients for FK and IK

  // rclcpp::executors::MultiThreadedExecutor executor;
  // executor.add_node(ctr_node);
  // rclcpp::spin(ctr_node);

  rclcpp::shutdown();
  return 0;
}
