#include <iostream>

#include "MyTest.h"
#include "Controller.h"
#include "rclcpp/rclcpp.hpp"

class TestNode : public rclcpp::Node
{
public:
  TestNode::TestNode() : Node("Test")
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
  rclcpp::spin(std::make_shared<TestNode>());
  rclcpp::shutdown();
  return 0;
}
