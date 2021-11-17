#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ctr_sim/msg/motor_pos.hpp"
#include "ctr_sim/msg/robot_pos.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class KinematicsNode : public rclcpp::Node
{
public:
    //constructor
    KinematicsNode::KinematicsNode() : Node("RobotKinematics")
    {
        //initialize members
        this->motor_target = {0, 0, 0, 0, 0, 0};
        this->motor_target_pub_ = this->create_publisher<ctr_sim::msg::MotorPos>("MotorTarget", 10);
        this->robot_pos_sub_ = this->create_subscription<ctr_sim::msg::RobotPos>("RobotTargetPos", 10, std::bind(&KinematicsNode::robot_target_callback, this, _1));
        this->timer_ = this->create_wall_timer(100ms, std::bind(&KinematicsNode::timer_callback, this));
    }

private:
    //publisher callback
    void timer_callback()
    {
        //test code
        auto message = ctr_sim::msg::MotorPos();
        if (this->motor_target[0] % 20 == 0)
        {
            this->step = -this->step;
        }
        for (int i = 0; i < this->motor_target.size(); i++)
        {
            this->motor_target[i] += this->step;
        }
        message.motor_position = this->motor_target;
        RCLCPP_INFO(this->get_logger(), "Current step is %d", this->motor_target[0]);
        this->motor_target_pub_->publish(message);
    }

    //subscriber callback
    void robot_target_callback(const ctr_sim::msg::RobotPos::SharedPtr RobotTarget)
    {
    }

    //publishing timer
    rclcpp::TimerBase::SharedPtr timer_;

    //motor target position publisher
    rclcpp::Publisher<ctr_sim::msg::MotorPos>::SharedPtr motor_target_pub_;

    //motor target position
    std::vector<int> motor_target;

    //robot position subscriber
    rclcpp::Subscription<ctr_sim::msg::RobotPos>::SharedPtr robot_pos_sub_;

    ////////test code///////////
    int step = 1;
    ///////////////////
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}