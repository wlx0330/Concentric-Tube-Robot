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
    // init ctr robot member
    this->ctr_robot_ = Controller();
    this->ctr_robot_.ControllerMenu();
  }

private:
  //
  Controller ctr_robot_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CTRNode>());
  rclcpp::shutdown();
  return 0;
}
