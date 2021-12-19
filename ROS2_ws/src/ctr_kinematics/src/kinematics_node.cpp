#include <iostream>
#include <string>
#include <chrono>

#include "CTRKinematicsController.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ctr_kinematics/msg/robot_config.hpp"
#include "ctr_kinematics/srv/solve_kinematics.hpp"
#include "ctr_kinematics/action/kinematics.hpp"

using namespace std::chrono_literals;

class KinematicsNode : public rclcpp::Node
{
public:
    // node class constructor
    KinematicsNode::KinematicsNode() : Node("CTRKinematics")
    {
        // init ctr robot member
        this->ctr_robot_ = CtrKinematicsController();
        this->ctr_ghost_ = CtrKinematicsController();

        // init callback groups
        this->cbg_timer_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        // start servers
        this->CaSolveFkSrv(); // start CTR FK server
        this->CaSolveIkSrv(); // start CTR IK server

        // start FK and IK action servers
        this->CreateFkActionSrv();
        this->CreateIkActionSrv();
    }

private:
    // CTR controller class
    CtrKinematicsController ctr_robot_;
    CtrKinematicsController ctr_ghost_;

    // ROS2 callback groups
    rclcpp::CallbackGroup::SharedPtr cbg_timer_;

    // kinematics computation timer
    rclcpp::TimerBase::SharedPtr timer_;
    void CbTimer()
    {
        this->timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "Solving CTR kinematics time out.");
        // TODO: cancel kineamtics compute actions
        rclcpp::shutdown();
    }

    /* ROS2 Parameters */

    /* ROS2 Topics */

    // motor configuration publisher
    rclcpp::Subscription<ctr_kinematics::msg::RobotConfig>::SharedPtr target_config_pub_;

    /* ROS2 Services */

    // TODO consider using ROS2 action in order to allow cancelling
    // CTR FK service
    rclcpp::Service<ctr_kinematics::srv::SolveKinematics>::SharedPtr solve_fk_srv_;
    void CaSolveFkSrv() // call "ahead" function for FK service
    {
        this->solve_fk_srv_ = this->create_service<ctr_kinematics::srv::SolveKinematics>(
            "SolveCtrFk",
            std::bind(&KinematicsNode::CbSolveFkSrv, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);
    }
    void CbSolveFkSrv(
        const ctr_kinematics::srv::SolveKinematics::Request::SharedPtr request,
        ctr_kinematics::srv::SolveKinematics::Response::SharedPtr response)
    {
        std::chrono::seconds time_out = 1s; // max computation time
        this->timer_ = this->create_wall_timer(time_out, std::bind(&KinematicsNode::CbTimer, this));
        this->ctr_robot_.SolveFK(request->fk_config_tran, request->fk_config_rot);
        std::chrono::duration<double, std::milli> time_passed =
            time_out - this->timer_->time_until_trigger();
        this->timer_->cancel();
        RCLCPP_INFO(this->get_logger(),
                    "Solving CTR forward kinematics in %lf ms.", time_passed.count());
        response->tip_coord = this->ctr_robot_.GetTipCoord();
    }

    // CTR IK service
    rclcpp::Service<ctr_kinematics::srv::SolveKinematics>::SharedPtr solve_ik_srv_;
    void CaSolveIkSrv() // call "ahead" function for IK service
    {
        this->solve_ik_srv_ = this->create_service<ctr_kinematics::srv::SolveKinematics>(
            "SolveCtrIk",
            std::bind(&KinematicsNode::CbSolveIkSrv, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);
    }
    void CbSolveIkSrv(
        const ctr_kinematics::srv::SolveKinematics::Request::SharedPtr request,
        ctr_kinematics::srv::SolveKinematics::Response::SharedPtr response)
    {
        auto time_out = 3s; // max computation time
        this->timer_ = this->create_wall_timer(
            time_out, std::bind(&KinematicsNode::CbTimer, this), this->cbg_timer_);
        // move ctr to init config
        this->ctr_ghost_.SolveFK(request->fk_config_tran, request->fk_config_rot);
        this->ctr_ghost_.SolveIK(request->ik_target_coord);
        std::chrono::duration<double, std::milli> time_passed =
            time_out - this->timer_->time_until_trigger();
        this->timer_->cancel();
        RCLCPP_INFO(this->get_logger(),
                    "Solving CTR inverse kinematics in %lf ms.", time_passed.count());
        response->config_tran = this->ctr_ghost_.GetConfigTran();
        response->config_rot = this->ctr_ghost_.GetConfigRot();
        response->tip_coord = this->ctr_ghost_.GetTipCoord();
    }

    /* ROS2 Actions*/

    // CTR FK action
    rclcpp_action::Server<ctr_kinematics::action::Kinematics>::SharedPtr fk_action_srv_;
    void CreateFkActionSrv()
    {
        this->fk_action_srv_ = rclcpp_action::create_server<ctr_kinematics::action::Kinematics>(
            this, "ForwardKinematics",
            std::bind(&KinematicsNode::FkActionHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&KinematicsNode::FkActionHandleCancel, this, std::placeholders::_1),
            std::bind(&KinematicsNode::FkActionHandleAccept, this, std::placeholders::_1));
    }
    rclcpp_action::GoalResponse FkActionHandleGoal(
        const rclcpp_action::GoalUUID &uuid,
        ctr_kinematics::action::Kinematics::Goal::ConstPtr goal)
    {
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse FkActionHandleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::Kinematics>>
            goal_handle)
    {
        std::cout << "goal cancel" << std::endl; //tc
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void FkActionHandleAccept(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::Kinematics>>
            goal_handle)
    {
        std::thread{std::bind(&KinematicsNode::FkActionExecute, this, std::placeholders::_1),
                    goal_handle}
            .detach();
    }
    void FkActionExecute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::Kinematics>>
            goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ctr_kinematics::action::Kinematics::Result>();
        // async lambda function call
        auto fut = std::async(std::launch::async, [this, &goal]()
                              { this->ctr_robot_.SolveFK(goal->fk_tran, goal->fk_rot); });
        // check future and cancel request
        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                result->tip_coord = {0, 0, 0};
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Solving CTR FK time out, request is canceled.");
                return;
            }
            if (fut.wait_for(1ms) == std::future_status::ready)
            {
                result->tip_coord = this->ctr_robot_.GetTipCoord();
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(),
                            "Solving CTR forward kinematics in %lf ms.",
                            this->ctr_robot_.timer.count());
                return;
            }
        }
    }

    // CTR IK action
    rclcpp_action::Server<ctr_kinematics::action::Kinematics>::SharedPtr ik_action_srv_;
    void CreateIkActionSrv()
    {
        this->ik_action_srv_ = rclcpp_action::create_server<ctr_kinematics::action::Kinematics>(
            this, "InverseKinematics",
            std::bind(&KinematicsNode::IkActionHandleGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            std::bind(&KinematicsNode::IkActionHandleCancel, this, std::placeholders::_1),
            std::bind(&KinematicsNode::IkActionHandleAccept, this, std::placeholders::_1));
    }
    rclcpp_action::GoalResponse IkActionHandleGoal(
        const rclcpp_action::GoalUUID &uuid,
        ctr_kinematics::action::Kinematics::Goal::ConstPtr goal)
    {
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    rclcpp_action::CancelResponse IkActionHandleCancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::Kinematics>>
            goal_handle)
    {
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }
    void IkActionHandleAccept(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::Kinematics>>
            goal_handle)
    {
        std::thread{std::bind(&KinematicsNode::IkActionExecute, this, std::placeholders::_1),
                    goal_handle}
            .detach();
    }
    void IkActionExecute(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<ctr_kinematics::action::Kinematics>>
            goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<ctr_kinematics::action::Kinematics::Result>();
        //async lambda function call
        auto fut = std::async(std::launch::async, [this, &goal]()
                              { this->ctr_robot_.SolveFK(goal->fk_tran, goal->fk_rot); });
        bool is_fk = true;
        while (rclcpp::ok())
        {
            if (goal_handle->is_canceling())
            {
                result->tip_coord = {0, 0, 0};
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Solving CTR IK time out, request is canceled.");
                return;
            }
            if (fut.wait_for(1ms) == std::future_status::ready)
            {
                if (is_fk)
                {
                    is_fk = false;
                    fut = std::async(std::launch::async, [this, &goal]()
                                     { this->ctr_robot_.SolveIK(goal->ik_goal); });
                }
                else
                {
                    result->robot_tran = this->ctr_robot_.GetConfigTran();
                    result->robot_rot = this->ctr_robot_.GetConfigRot();
                    result->tip_coord = this->ctr_robot_.GetTipCoord();
                    goal_handle->succeed(result);
                    RCLCPP_INFO(this->get_logger(),
                                "Solving CTR inverse kinematics in %lf ms.",
                                this->ctr_robot_.timer.count());
                    return;
                }
            }
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<KinematicsNode>());
    auto kinematics_node = std::make_shared<KinematicsNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(kinematics_node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
