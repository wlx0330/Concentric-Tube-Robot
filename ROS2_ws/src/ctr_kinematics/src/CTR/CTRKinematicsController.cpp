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
    auto temp = this->ctr_.GetDist();
    this->tip_coord_ = {temp[0], temp[1], temp[2]};
    return this->tip_coord_;
}

// calculate CTR FK
void KC::SolveFK(const std::array<double, 3> &config_tran,
                 const std::array<double, 3> &config_rot) // deg to rad
{
    blaze::StaticVector<double, 6> input;
    blaze::subvector(input, 0UL, 3UL) = blaze::StaticVector<double, 3>(config_tran);
    blaze::subvector(input, 3UL, 3UL) = blaze::StaticVector<double, 3>(config_rot) * M_PI / 180.0;
    this->kinematics_.CTRFK(this->ctr_, input);
}

// calculate CTR IK
void KC::SolveIK(const std::array<double, 3> &target_coord)
{
    this->kinematics_.CTRIK(this->ctr_, blaze::StaticVector<double, 3>(target_coord));
}

// get CTR translation motor config
std::array<double, 3> KC::GetConfigTran()
{
    auto temp = this->ctr_.GetTran();
    std::array<double, 3UL> tran = {temp[0], temp[1], temp[2]};
    return tran;
}

// get CTR rotation motor config
std::array<double, 3> KC::GetConfigRot()
{
    auto temp = this->ctr_.GetRot() * 180.0 / M_PI;
    std::array<double, 3UL> rot = {temp[0], temp[1], temp[2]};
    return rot;
}