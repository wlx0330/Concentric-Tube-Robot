#include <iostream>
// #include <blaze/Math.h>

#include "MyTest.h"
#include "Controller.h"
// #include "rclcpp/rclcpp.hpp"

// class TestNode : public rclcpp::Node
// {
// public:
//   TestNode::TestNode() : Node("Test")
//   {
//     this->robot_ = Controller();
//     this->robot_.ControllerMenu();
//   }

// private:
//   Controller robot_;
// };

int main(int argc, char **argv)
{
  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<TestNode>());
  // rclcpp::shutdown();
  std::cout << 123131231 << std::endl;
  // system("pause");
  if (true)
  {
    MyTest test;
    test.MymyTest();

    // Controller robot = Controller();
    // Controller robot;
    //robot.ControllerMenu();
  }
  // robot.ControllerMenu();
  system("pause");
  return 0;
}
