#include "CTRKinematicsController.h"
using KC = CtrKinematicsController;

// class constructor
KC::CtrKinematicsController()
{
    this->ctr_ = CTR();
    this->kinematics_ = Kinematics();
}

// return CTR tip coordinates
std::array<double, 3> KC::GetTipCoord()
{
    auto tip_coord = this->ctr_.GetDist();
    std::array<double, 3> coord = {tip_coord[0], tip_coord[1], tip_coord[2]};
    return coord;
}

// calculate CTR FK
void KC::SolveFK(const std::array<double, 3> &config_tran,
                 const std::array<double, 3> &config_rot)
{
    blaze::StaticVector<double, 6> input;
    blaze::subvector(input, 0UL, 3UL) = blaze::StaticVector<double, 3>(config_tran);
    blaze::subvector(input, 3UL, 3UL) = blaze::StaticVector<double, 3>(config_rot);
    this->kinematics_.CTRFK(this->ctr_, input);
}

// calculate CTR IK
void KC::SolveIK(const std::array<double, 3> &target_coord)
{
    this->kinematics_.CTRIK(this->ctr_, blaze::StaticVector<double, 3>(target_coord));
}
