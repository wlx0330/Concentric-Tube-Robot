#include <iostream>

#include "Controller.h"
#include "rclcpp/rclcpp.hpp"

class CTRNode : public rclcpp::Node
{
public:
  CTRNode::CTRNode() : Node("CTR")
  {
    this->robot_ = Controller();
    this->robot_.ControllerMenu();
  }

private:
  Controller robot_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CTRNode>());
  rclcpp::shutdown();
  return 0;
}
