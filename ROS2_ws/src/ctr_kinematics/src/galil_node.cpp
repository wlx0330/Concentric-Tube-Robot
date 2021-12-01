#include <memory>
#include <string>
#include <chrono>
#include <functional>
#include <map>
#include <thread>

#include "galil_controller.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "ctr_sim/msg/motor_pos.hpp"
#include "ctr_sim/srv/motor_speed.hpp"
#include "ctr_sim/srv/motor_connect.hpp"
#include "ctr_sim/srv/motor_jog.hpp"
#include "ctr_sim/action/motor_calib.hpp"

//#include "rclcpp_components/register_node_macro.hpp"
//#include "ctr_sim/visibility_control.h"

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class GalilNode : public rclcpp::Node
{
public:
    //constructor
    GalilNode::GalilNode() : Node("GalilController")
    {
        //initialize members
        this->GC_ = GalilMotionController();
        this->srv_callbk_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        this->motor_init_srv_ = this->create_service<ctr_sim::srv::MotorConnect>(
            "MotorInit",
            std::bind(&GalilNode::motor_init_callback, this, _1, _2),
            rmw_qos_profile_services_default, this->srv_callbk_group_);
        this->motor_target_sub_ = this->create_subscription<ctr_sim::msg::MotorPos>(
            "MotorTarget",
            10,
            std::bind(&GalilNode::motor_target_callback, this, _1));
        this->motor_pos_publisher_ = this->create_publisher<ctr_sim::msg::MotorPos>("MotorPos", 10);
        //this->timer_ = this->create_wall_timer(3000ms, std::bind(&GalilNode::timer_callback, this));

        //jog service server
        this->motor_jog_srv_ = this->create_service<ctr_sim::srv::MotorJog>("MotorJog", std::bind(&GalilNode::motor_jog_callback, this, _1, _2)); //, rmw_qos_profile_services_default, this->srv_callbk_group_);

        //set default param value (motor position, motor speed, motor acc dec)
        motor_speed["lin"] = 1;
        motor_speed["rot"] = 1;
        motor_speed["coe"] = 1;
        this->declare_parameters("motor_speed", motor_speed);
        this->callbk_handle = this->add_on_set_parameters_callback(std::bind(&GalilNode::set_parameters_callbk, this, _1));

        //set action servers
        this->action_server_ = rclcpp_action::create_server<ctr_sim::action::MotorCalib>(
            this, "motor_calib",
            std::bind(&GalilNode::handle_goal, this, _1, _2),
            std::bind(&GalilNode::handle_cancel, this, _1),
            std::bind(&GalilNode::handle_accepted, this, _1));
    }

private:
    //publishing motor pos
    void GalilNode::timer_callback()
    {
        //for (int i = 0; i <= this->GC_.m_pos.size(); i++)
        //{
        //  this->pos_.motor_position.push_back(this->GC_.m_pos[i]);
        //}
        //this->pos_.motor_position = this->GC_.m_pos;
        //this->motor_pos_publisher_->publish(pos_);
        RCLCPP_INFO(this->get_logger(), "publishing current motor pos...");
    }

    //motor position subscriber callback
    void GalilNode::motor_target_callback(const ctr_sim::msg::MotorPos::SharedPtr MotorTarget)
    {
        RCLCPP_INFO(this->get_logger(), "I heard motor target");
        for (int i = 0; i < MotorTarget->motor_position.size(); i++)
        {
            this->GC_.m_pos[i] = MotorTarget->motor_position[i];
            this->GC_.TrackMotor(i, this->GC_.m_pos[i]);
        }
    }

    //motor init service callback
    void GalilNode::motor_init_callback(const ctr_sim::srv::MotorConnect::Request::SharedPtr request, ctr_sim::srv::MotorConnect::Response::SharedPtr response)
    {
        string address = request->address;
        int type = request->type;
        if (this->GC_.Connect(address, type)) //need a return value return
        {
            RCLCPP_INFO(this->get_logger(), "%s is connected.", this->GC_.buf);
            response->index = this->GC_.vPair.size();
            this->GC_.InitMotor((int)(response->index) - 1);
            this->mMap.insert(make_pair(type, response->index - 1));
            response->is_connect = true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Can not connect motor at %s.", address);
        }
    }

    //motor jog service callback
    void GalilNode::motor_jog_callback(const ctr_sim::srv::MotorJog::Request::SharedPtr request, ctr_sim::srv::MotorJog::Response::SharedPtr response)
    {
        int index = request->motor_index;
        int move = request->motor_move;
        this->GC_.JogMotor(index - 1, move);
        response->motor_ready = true;
    }

    //parameter set callback
    rcl_interfaces::msg::SetParametersResult set_parameters_callbk(const std::vector<rclcpp::Parameter> &params)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = "";
        for (const auto &param : params)
        {
            if (param.get_name() == "motor_speed.lin")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    this->motor_speed["lin"] = param.as_int();
                    auto index_range = this->mMap.equal_range(1);
                    for (auto it = index_range.first; it != index_range.second; it++)
                    {
                        this->GC_.SetSpeed(it->second, this->motor_speed["lin"], this->motor_speed["coe"]);
                    }
                }
                result.successful = true;
            }
            if (param.get_name() == "motor_speed.rot")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    this->motor_speed["rot"] = param.as_int();
                    auto index_range = this->mMap.equal_range(2);
                    for (auto it = index_range.first; it != index_range.second; it++)
                    {
                        this->GC_.SetSpeed(it->second, this->motor_speed["rot"], this->motor_speed["coe"]);
                    }
                }
                result.successful = true;
            }
            if (param.get_name() == "motor_speed.coe")
            {
                if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
                {
                    this->motor_speed["coe"] = param.as_int();
                    for (auto it = this->mMap.begin(); it != this->mMap.end(); it++)
                    {
                        if (it->first == 1)
                        {
                            this->GC_.SetSpeed(it->second, this->motor_speed["lin"], this->motor_speed["coe"]);
                        }
                        else if (it->first == 2)
                        {
                            this->GC_.SetSpeed(it->second, this->motor_speed["rot"], this->motor_speed["coe"]);
                        }
                    }
                }
                result.successful = true;
                //this->GC_.MotorHome(); //test code
            }
        }
        return result;
    }

    ///////////////////////////////////ROS2 ACTIONS////////////////////////////////
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const ctr_sim::action::MotorCalib::Goal> goal)
    {
        //RCLCPP_INFO(this->get_logger(), "received goal request");
        //(void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_sim::action::MotorCalib>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "received goal cancel request");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_sim::action::MotorCalib>> goal_handle)
    {
        std::thread{std::bind(&GalilNode::action_execute, this, _1), goal_handle}.detach();
        //RCLCPP_INFO(this->get_logger(), "action goal request accepted");
    }

    void action_execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_sim::action::MotorCalib>> goal_handle)
    {
        //auto feedback = std::make_shared<ctr_sim::action::MotorCalib::Feedback>();
        auto result = std::make_shared<ctr_sim::action::MotorCalib::Result>();
        const auto goal = goal_handle->get_goal();
        int index = goal->motor_index;
        int move = goal->motor_move;

        //check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            std::cout << "cancel" << std::endl; //test code
            this->GC_.StopMotor(index);         //stop the current motor
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "action is cancelled");
        }
        else
        {
            this->GC_.JogMotor(index - 1, move);
            goal_handle->succeed(result);
        }
    }

    //////////////////////////////////////////////////////////////////////////////

    //publishing timer
    rclcpp::TimerBase::SharedPtr timer_;

    //galil controller object
    GalilMotionController GC_;

    //motor position publisher
    rclcpp::Publisher<ctr_sim::msg::MotorPos>::SharedPtr motor_pos_publisher_;

    //target position subscriber
    rclcpp::Subscription<ctr_sim::msg::MotorPos>::SharedPtr motor_target_sub_;

    //motor connect service
    rclcpp::Service<ctr_sim::srv::MotorConnect>::SharedPtr motor_init_srv_;

    //server callback group
    rclcpp::CallbackGroup::SharedPtr srv_callbk_group_;

    //motor speed parameter
    std::map<std::string, int> motor_speed;

    //motor axis map
    std::multimap<int, int> mMap;

    //param callback handle
    OnSetParametersCallbackHandle::SharedPtr callbk_handle;

    //jog service server
    rclcpp::Service<ctr_sim::srv::MotorJog>::SharedPtr motor_jog_srv_;

    //calibration action server
    rclcpp_action::Server<ctr_sim::action::MotorCalib>::SharedPtr action_server_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //rclcpp::spin(std::make_shared<GalilNode>());
    auto galil_node = std::make_shared<GalilNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(galil_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}