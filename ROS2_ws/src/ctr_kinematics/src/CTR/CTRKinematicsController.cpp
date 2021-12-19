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
    auto t1 = std::chrono::high_resolution_clock::now();
    blaze::StaticVector<double, 6> input;
    blaze::subvector(input, 0UL, 3UL) = blaze::StaticVector<double, 3>(config_tran) / 1000.0;
    blaze::subvector(input, 3UL, 3UL) = blaze::StaticVector<double, 3>(config_rot) * M_PI / 180.0;
    this->kinematics_.CTRFK(this->ctr_, input);
    auto t2 = std::chrono::high_resolution_clock::now();
    this->timer = t2 - t1;
}

// calculate CTR IK
void KC::SolveIK(const std::array<double, 3> &target_coord)
{
    auto t1 = std::chrono::high_resolution_clock::now();
    this->kinematics_.CTRIK(this->ctr_, blaze::StaticVector<double, 3>(target_coord));
    auto t2 = std::chrono::high_resolution_clock::now();
    this->timer = t2 - t1;
}

// get CTR translation motor config
std::array<double, 3> KC::GetConfigTran()
{
    auto temp = this->ctr_.GetTran() * 1000.0;
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