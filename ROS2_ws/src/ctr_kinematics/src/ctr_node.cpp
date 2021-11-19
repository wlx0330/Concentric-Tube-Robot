#include <iostream>
#include <blaze/Math.h>

#include "Controller.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;



  Controller robot;
  robot.ControllerMenu();
  std::cout << 1 << std::endl;
  system("pause");
  return 0;
}
