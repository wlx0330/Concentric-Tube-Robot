#include <iostream>
#include <blaze/Math.h>

#include "Controller.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  Controller robot;
  // robot.ControllerMenu();
  blaze::StaticMatrix<double, 3UL, 3UL> temp;
  temp = {{1, 2, 3}, {4, 5, 6}, {7, 8, 9}};
  std::cout << 1 << std::endl;
  std::cout << temp << std::endl;
  system("pause");
  std::cout << blaze::inv(temp) << std::endl;


  system("pause");
  return 0;
}
