#include <iostream>
#include <string>
#include <chrono>

#include "Controller.h"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class KinematicsNode : public rclcpp::Node
{
public:
    // node class constructor
    KinematicsNode::KinematicsNode() : Node("CTRKinematics")
    {
        // init ctr robot member
        this->robot_ = Controller();
        this->robot_.ControllerMenu();
    }

private:
    Controller robot_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
