#include <chrono>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "GalilMotionController.h"

using namespace std::chrono_literals;

class TestNode : public rclcpp::Node
{
public:
    // node class constructor
    TestNode::TestNode() : Node("Test")
    {
        this->_gmc = GalilMotionController();
        this->_timer = this->create_wall_timer(
            3000ms,
            std::bind(&TestNode::_timerCB, this));
    }

private:
    //timer callback function
    void _timerCB()
    {
        this->_gmc.connectMotor(0, "192.168.42.0");
        this->_gmc.connectMotor(1, "192.168.42.97");
        this->_gmc.connectMotor(2, "192.168.42.2");
        this->_gmc.connectMotor(3, "192.168.42.98");
        this->_gmc.connectMotor(4, "192.168.42.96");
        this->_gmc.connectMotor(5, "192.168.42.99");
        // std::cout << "ip assigned" << std::endl; // test code
        auto ret = this->_gmc.initMotors();
        if (ret)
        {
            std::cout << "init successful" << std::endl;
        }
        else
        {
            std::cout << "init fail" << std::endl;
        }

        for (int i = 0; i < 6; ++i)
        {
            this->_gmc.setMotorSpeedMax(i, 10);
            this->_gmc.setMotorSpeedCoeff(i, 10);
        }

        this->_gmc.driveMotor(0);
        this->_gmc.driveMotor(1);
        this->_gmc.driveMotor(2);
        this->_gmc.driveMotor(3);
        this->_gmc.driveMotor(4);
        this->_gmc.driveMotor(5);

        this->_timer->cancel();
    }

    // galil controller object
    GalilMotionController _gmc;

    // action timer
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestNode>());
    rclcpp::shutdown();
    return 0;
}