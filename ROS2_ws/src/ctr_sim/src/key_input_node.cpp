#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <windows.h> //only for windows platforms
#undef max
#undef min

#include "rclcpp/rclcpp.hpp"
#include "ctr_sim/msg/key_input.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class KeyInputNode : public rclcpp::Node
{
public:
    KeyInputNode::KeyInputNode() : Node("KeyboardInput")
    {
        this->key_pub_ = this->create_publisher<ctr_sim::msg::KeyInput>("KeyInput", 10);
        this->timer_ = this->create_wall_timer(10ms, std::bind(&KeyInputNode::timer_callback, this)); //100hz timer
    }

private:
    // timer callback function
    void timer_callback()
    {
        //exit key
        if ((GetAsyncKeyState(VK_RETURN) & 0x8000) && (this->key_msg.motor_index == 0))
        {
            this->key_msg.exit = true;
            RCLCPP_INFO(this->get_logger(), "Exiting...");
        }
        else
        {
            this->key_msg.exit = false;
        }

        //toggle motor selection
        if ((GetAsyncKeyState(0x30) & 0x8000) || (GetAsyncKeyState(VK_NUMPAD0) & 0x8000)) //0 key
        {
            this->key_msg.motor_index = 0;
            RCLCPP_INFO(this->get_logger(), "No motor is selected. Press ENTER to exit. ");
        }
        else if ((GetAsyncKeyState(0x31) & 0x8000) || (GetAsyncKeyState(VK_NUMPAD1) & 0x8000)) //1 key
        {
            this->key_msg.motor_index = 1;
            RCLCPP_INFO(this->get_logger(), "Motor %d is selected. ", this->key_msg.motor_index);
        }
        else if ((GetAsyncKeyState(0x32) & 0x8000) || (GetAsyncKeyState(VK_NUMPAD2) & 0x8000)) //2 key
        {
            this->key_msg.motor_index = 2;
            RCLCPP_INFO(this->get_logger(), "Motor %d is selected. ", this->key_msg.motor_index);
        }
        else if ((GetAsyncKeyState(0x33) & 0x8000) || (GetAsyncKeyState(VK_NUMPAD3) & 0x8000)) //3 key
        {
            this->key_msg.motor_index = 3;
            RCLCPP_INFO(this->get_logger(), "Motor %d is selected. ", this->key_msg.motor_index);
        }
        else if ((GetAsyncKeyState(0x34) & 0x8000) || (GetAsyncKeyState(VK_NUMPAD4) & 0x8000)) //4 key
        {
            this->key_msg.motor_index = 4;
            RCLCPP_INFO(this->get_logger(), "Motor %d is selected. ", this->key_msg.motor_index);
        }
        else if ((GetAsyncKeyState(0x35) & 0x8000) || (GetAsyncKeyState(VK_NUMPAD5) & 0x8000)) //5 key
        {
            this->key_msg.motor_index = 5;
            RCLCPP_INFO(this->get_logger(), "Motor %d is selected. ", this->key_msg.motor_index);
        }
        else if ((GetAsyncKeyState(0x36) & 0x8000) || (GetAsyncKeyState(VK_NUMPAD6) & 0x8000)) //6 key
        {
            this->key_msg.motor_index = 6;
            RCLCPP_INFO(this->get_logger(), "Motor %d is selected. ", this->key_msg.motor_index);
        }

        //control move direction
        if ((GetAsyncKeyState(VK_ADD) & 0x8000) || (GetAsyncKeyState(VK_OEM_PLUS) & 0x8000)) //+ key
        {
            this->key_msg.motor_move = 1;
        }
        else if ((GetAsyncKeyState(VK_SUBTRACT) & 0x8000) || (GetAsyncKeyState(VK_OEM_MINUS) & 0x8000)) //- key
        {
            this->key_msg.motor_move = -1;
        }
        else
        {
            this->key_msg.motor_move = 0;
        }

        //log message
        if (this->key_msg.motor_move != 0 && this->key_msg.motor_index != 0)
        {
            RCLCPP_INFO(this->get_logger(), "Motor %d is moving in the %s direction. ", this->key_msg.motor_index, this->key_msg.motor_move > 0 ? "positive" : "negative");
        }
        this->key_pub_->publish(this->key_msg);
    }

    //wall timer
    rclcpp::TimerBase::SharedPtr timer_;

    //key publisher
    rclcpp::Publisher<ctr_sim::msg::KeyInput>::SharedPtr key_pub_;

    //key pub message
    ctr_sim::msg::KeyInput key_msg;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KeyInputNode>());
    rclcpp::shutdown();
    FlushConsoleInputBuffer(GetStdHandle(STD_INPUT_HANDLE)); // clear console inputs
    return 0;
}