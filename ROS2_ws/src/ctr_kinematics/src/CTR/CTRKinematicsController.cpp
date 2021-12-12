#include "CTRKinematicsController.h"
using KC = CtrKinematicsController;

// class constructor
KC::CtrKinematicsController()
{
    this->ctr_ = CTR();
    this->kinematics_ = Kinematics();
}

// calculate CTR FK
void KC::SolveFK(const std::array<double, 3> &config_tran,
                 const std::array<double, 3> &config_rot)
{
    // blaze::StaticVector<double, 3> tran(config_tran);
    // blaze::StaticVector<double, 3> rot(config_rot);
    blaze::StaticVector<double, 6> input;
    blaze::subvector(input, 0UL, 3UL) = blaze::StaticVector<double, 3>(config_tran);
    blaze::subvector(input, 3UL, 3UL) = blaze::StaticVector<double, 3>(config_rot);
    this->kinematics_.CTRFK(this->ctr_, input);
}

// return CTR tip coordinates
std::array<double, 3> KC::GetTipCoord()
{
    std::array<double, 3> temp = {0, 0, 0};
    return temp;
}