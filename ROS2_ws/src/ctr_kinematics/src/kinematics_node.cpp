#include <iostream>
#include <string>
#include <chrono>

#include "CTRKinematicsController.h"
#include "rclcpp/rclcpp.hpp"
#include "ctr_kinematics/msg/robot_config.hpp"
#include "ctr_kinematics/srv/solve_kinematics.hpp"

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

        // start CTR FK server
        this->solve_fk_srv_ = this->create_service<ctr_kinematics::srv::SolveKinematics>(
            "SolveCtrFk",
            std::bind(&KinematicsNode::CbSolveFkSrv, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);

        this->CaSolveIkSrv(); // start CTR IK server
    }

private:
    // CTR controller class
    CtrKinematicsController ctr_robot_;
    CtrKinematicsController ctr_ghost_;

    // kinematics computation timer
    // TODO may need to set up different callback groups
    rclcpp::TimerBase::SharedPtr timer_;
    void CbTimer()
    {
        this->timer_->cancel();
        // kinematics
        RCLCPP_INFO(this->get_logger(), "Solving CTR kinematics time out.");
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
    // TODO use multi thread?
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
        std::chrono::seconds time_out = 5s; // max computation time
        this->timer_ = this->create_wall_timer(time_out, std::bind(&KinematicsNode::CbTimer, this));
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
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicsNode>());
    rclcpp::shutdown();
    return 0;
}
