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

        // start CTR FK server
        this->solve_fk_srv_ = this->create_service<ctr_kinematics::srv::SolveKinematics>(
            "SolveKinematics",
            std::bind(&KinematicsNode::CbSolveFkSrv, this, std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default);

        //...
    }

private:
    // CTR controller class
    CtrKinematicsController ctr_robot_;

    // kinematics computation timer
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

    // CTR FK service
    rclcpp::Service<ctr_kinematics::srv::SolveKinematics>::SharedPtr solve_fk_srv_;
    void CbSolveFkSrv(
        const ctr_kinematics::srv::SolveKinematics::Request::SharedPtr request,
        ctr_kinematics::srv::SolveKinematics::Response::SharedPtr response)
    {
        this->timer_ = this->create_wall_timer(1s, std::bind(&KinematicsNode::CbTimer, this));
        this->ctr_robot_.SolveFK(request->fk_config_tran, request->fk_config_rot);
        auto time_passed = 1s - this->timer_->time_until_trigger();
        this->timer_->cancel();
        RCLCPP_INFO(this->get_logger(),
                    "Solving CTR forward kinematics in %d ms.", time_passed.count());
        response->tip_coord = this->ctr_robot_.GetTipCoord();
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
