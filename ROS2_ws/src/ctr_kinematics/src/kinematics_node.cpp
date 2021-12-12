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
    }

private:
    // CTR controller class
    CtrKinematicsController ctr_robot_;

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
