#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "ctr_sim/msg/robot_pos.hpp"
#include "ctr_sim/srv/motor_speed.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class TeleopNode : public rclcpp::Node
{
public:
    //constructor
    TeleopNode::TeleopNode() : Node("TeleOperation")
    {
        this->motor_speed_cb_group_ = this->create_callback_group(rclcpp::callback_group::CallbackGroupType::MutuallyExclusive);
        //this->cli_timer_ = this->create_wall_timer(10ms, std::bind(&TeleopNode::cli_timer_callback, this));

        //this->motor_speed_cli_ = this->create_client<ctr_sim::srv::MotorSpeed>("MotorSpeed");
        this->robot_target_pub_ = this->create_publisher<ctr_sim::msg::RobotPos>("RobotTarget", 10);
        //this->timer_ = this->create_wall_timer(100ms, std::bind(&TeleopNode::timer_callback, this));
    }

private:
    //timer callback function for target pub
    void timer_callback()
    {
        //auto message = ctr_sim::msg::RobotPos();
    }

    //callback function group
    rclcpp::callback_group::CallbackGroup::SharedPtr motor_speed_cb_group_;

    //publishing timer
    rclcpp::TimerBase::SharedPtr timer_;

    //robot target position publisher
    rclcpp::Publisher<ctr_sim::msg::RobotPos>::SharedPtr robot_target_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(std::make_shared<TeleopNode>());
    executor.spin();
    //rclcpp::spin(std::make_shared<TeleopNode>());

    rclcpp::shutdown();
    return 0;
}