#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
//#include "ctr_sim/msg/motor_pos.hpp"
#include "ctr_sim/msg/key_input.hpp"
//#include "ctr_sim/srv/motor_speed.hpp"
#include "ctr_sim/srv/motor_connect.hpp"
#include "ctr_sim/srv/motor_jog.hpp"
#include "ctr_sim/action/motor_calib.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

#define LINEAR_MOTOR 1
#define ROTATION_MOTOR 2

class MotorSetupNode : public rclcpp::Node
{
public:
    MotorSetupNode::MotorSetupNode() : Node("MotorSetup")
    {
        this->motor_init_cli_ = this->create_client<ctr_sim::srv::MotorConnect>("MotorInit");
        this->parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "GalilController");
        this->motor_jog_cli_ = this->create_client<ctr_sim::srv::MotorJog>("MotorJog");
        this->jog_request_ = std::make_shared<ctr_sim::srv::MotorJog::Request>();
        this->jog_response_ = std::make_shared<ctr_sim::srv::MotorJog::Response>();
        //this->action_client_ = rclcpp_action::create_client<ctr_sim::action::MotorCalib>(this, "motor_calib");
    }

    //motor init function
    void InitMotors()
    {
        while (!this->motor_init_cli_->wait_for_service(5s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting...");
                rclcpp::shutdown();
                break;
            }
            RCLCPP_INFO(this->get_logger(), "motor_init service not available, waiting again...");
        }

        for (int i = 0; i < this->requests_.size(); i++)
        {
            auto response = this->motor_init_cli_->async_send_request(this->requests_[i]);
            this->responses_.push_back(response);
            response.wait_for(200ms);
        }

        for (int i = 0; i < this->responses_.size(); i++)
        {
            if (rclcpp::spin_until_future_complete(std::dynamic_pointer_cast<MotorSetupNode>(this->shared_from_this()), this->responses_[i]) != rclcpp::FutureReturnCode::SUCCESS)
            {
                auto message = this->responses_[i].get();
                RCLCPP_INFO(this->get_logger(), "Motor %d failed to initialize!", message->index);
            }
        }
        RCLCPP_INFO(this->get_logger(), "Motor initialization finished.");
    }

    //set motor speed
    void SetSpeed(int lin_speed, int rot_speed, int acc_coe)
    {
        while (!this->parameters_client_->wait_for_service(5s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting...");
                rclcpp::shutdown();
                break;
            }
            RCLCPP_INFO(this->get_logger(), "speed_param service not available, waiting again...");
        }

        std::vector<rclcpp::Parameter> new_speed;
        new_speed.push_back(rclcpp::Parameter("motor_speed.lin", lin_speed));
        new_speed.push_back(rclcpp::Parameter("motor_speed.rot", rot_speed));
        new_speed.push_back(rclcpp::Parameter("motor_speed.coe", acc_coe));
        auto result = this->parameters_client_->set_parameters(new_speed);
        if (rclcpp::spin_until_future_complete(std::dynamic_pointer_cast<MotorSetupNode>(this->shared_from_this()), result) != rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "set_parameters service call failed.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Motor speed is successfully set. ");
    }

    //add motor function
    void AddMotor(std::string address, int type)
    {
        auto request = std::make_shared<ctr_sim::srv::MotorConnect::Request>();
        request->address = address;
        request->type = type;
        this->requests_.push_back(request);
    }

    //motor calibration function
    void CalibMotors()
    {
        while (!this->motor_jog_cli_->wait_for_service(5s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Interrupted while waiting for the service. Exiting...");
                rclcpp::shutdown();
                break;
            }
            RCLCPP_INFO(this->get_logger(), "motor_jog service not available, waiting again...");
        }

        //calibration info
        RCLCPP_INFO(this->get_logger(), "Starting motor calibration. Please run the key input node. ");
        this->jog_request_->motor_index = 0;
        this->jog_request_->motor_move = 0;
        this->jog_response_->motor_ready = true;
        RCLCPP_INFO(this->get_logger(), "Press a number (1-6) to select motor. Press 0 to exit. ");
        this->key_sub_ = this->create_subscription<ctr_sim::msg::KeyInput>("KeyInput", 10, std::bind(&MotorSetupNode::key_sub_callbk, this, _1));
        rclcpp::spin(std::dynamic_pointer_cast<MotorSetupNode>(this->shared_from_this()));
        RCLCPP_INFO(this->get_logger(), "Calibration completed. Setting home position...");
    }

private:
    /*
    //action goal response callback function
    void goal_response_callbk(std::shared_future<rclcpp_action::ClientGoalHandle<ctr_sim::action::MotorCalib>::SharedPtr> future)
    {
        auto goal_handle = future.get();
        if (!goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Goal was rejected by the server");
        }
        else
        {
            //RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    //action feedback callback function
    void feedback_callbk(rclcpp_action::ClientGoalHandle<ctr_sim::action::MotorCalib>::SharedPtr, const std::shared_ptr<const ctr_sim::action::MotorCalib::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "feedback_callback");
    }

    //action result callback function
    void result_callbk(const rclcpp_action::ClientGoalHandle<ctr_sim::action::MotorCalib>::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            //RCLCPP_INFO(this->get_logger(), "Goal was completed");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            return;
        }
        //rclcpp::shutdown();
    }
    */

    //keyboard input subscriber callback function
    void key_sub_callbk(const ctr_sim::msg::KeyInput::SharedPtr input)
    {
        //exit code
        if (input->motor_index == 0)
        {
            if (input->exit)
            {
                rclcpp::shutdown();
                return;
            }
            return;
        }

        //move motor code
        if (this->jog_response_->motor_ready)
        {
            if (input->motor_index != this->jog_request_->motor_index)
            {
                if (this->jog_request_->motor_move != 0)
                {
                    this->jog_request_->motor_move = 0; //stop the current moving motor
                    auto response = this->motor_jog_cli_->async_send_request(this->jog_request_, std::bind(&MotorSetupNode::motor_jog_response_callback, this, _1));
                    this->jog_response_->motor_ready = false;
                }
                this->jog_request_->motor_index = input->motor_index;
                RCLCPP_INFO(this->get_logger(), "Motor %d is selected. ", input->motor_index);
            }

            if (input->motor_move != this->jog_request_->motor_move)
            {
                this->jog_request_->motor_move = input->motor_move;
                if (input->motor_move > 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Motor %d is moving in the positive direction. ", input->motor_index);
                }
                else if (input->motor_move < 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Motor %d is moving in the negative direction. ", input->motor_index);
                }
                else if (input->motor_move == 0)
                {
                    RCLCPP_INFO(this->get_logger(), "Motor %d is stopping. ", input->motor_index);
                }
                auto response = this->motor_jog_cli_->async_send_request(this->jog_request_, std::bind(&MotorSetupNode::motor_jog_response_callback, this, _1));
                this->jog_response_->motor_ready = false;
            }
        }
    }

    //motor jog service response callback function
    void motor_jog_response_callback(rclcpp::Client<ctr_sim::srv::MotorJog>::SharedFuture future)
    {
        auto response = future.get();
        if (response->motor_ready)
        {
            this->jog_response_->motor_ready = true;
        }
    }

    //motor init service request
    std::vector<rclcpp::Client<ctr_sim::srv::MotorConnect>::SharedRequest> requests_;

    //motor init service response (shared future object)
    std::vector<rclcpp::Client<ctr_sim::srv::MotorConnect>::SharedFuture> responses_;

    //async param client
    rclcpp::AsyncParametersClient::SharedPtr parameters_client_;

    //motor init service client
    rclcpp::Client<ctr_sim::srv::MotorConnect>::SharedPtr motor_init_cli_;

    //keyboard input subscriber
    rclcpp::Subscription<ctr_sim::msg::KeyInput>::SharedPtr key_sub_;

    //motor jog service client
    rclcpp::Client<ctr_sim::srv::MotorJog>::SharedPtr motor_jog_cli_;

    //motor job service request
    rclcpp::Client<ctr_sim::srv::MotorJog>::SharedRequest jog_request_;

    //motor jog service response
    rclcpp::Client<ctr_sim::srv::MotorJog>::SharedResponse jog_response_;

    //motor calib action client
    //rclcpp_action::Client<ctr_sim::action::MotorCalib>::SharedPtr action_client_;

    //action goal options
    //rclcpp_action::Client<ctr_sim::action::MotorCalib>::SendGoalOptions send_goal_options_;

    //action goal request message
    //ctr_sim::action::MotorCalib::Goal goal_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto motor_init_node = std::make_shared<MotorSetupNode>();
    motor_init_node->AddMotor("192.168.42.98", LINEAR_MOTOR);
    motor_init_node->AddMotor("192.168.42.96", LINEAR_MOTOR);
    motor_init_node->AddMotor("192.168.42.99", LINEAR_MOTOR);
    motor_init_node->AddMotor("192.168.42.0", ROTATION_MOTOR);
    motor_init_node->AddMotor("192.168.42.97", ROTATION_MOTOR);
    motor_init_node->AddMotor("192.168.42.2", ROTATION_MOTOR);
    motor_init_node->InitMotors();
    motor_init_node->SetSpeed(10, 10, 20); //lin, rot, coe
    motor_init_node->CalibMotors();        //motor calibration action

    //TODO motor home action and start position control

    //rclcpp::spin(motor_init_node);
    //rclcpp::shutdown();
    return 0;
}
