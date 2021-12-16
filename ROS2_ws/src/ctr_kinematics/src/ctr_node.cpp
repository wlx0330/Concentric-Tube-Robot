#include <iostream>
#include <string>
#include <chrono>

#include "Controller.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class CTRNode : public rclcpp::Node
{
public:
  // node class constructor
  CTRNode::CTRNode() : Node("ConcentricTubeRobot")
  {
  }

private:
  /* ROS2 Parameters */

  // set parameter function

  /* ROS2 Topics */

  // motor target config publisher

  /* ROS2 Services */

  // set motor home

  /* ROS2 Actions */

  // CTR FK action server

  // CTR IK action server
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CTRNode>());
  rclcpp::shutdown();
  return 0;
}
