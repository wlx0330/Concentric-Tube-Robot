#pragma once
#include <iostream>
#include <chrono>
#include <string>
#include <blaze/Math.h>

#include "CTR.h"
#include "Kinematics.h"

class CtrKinematicsController
{
public:
    // class constructor
    CtrKinematicsController();

    // return CTR tip coordinates
    std::array<double, 3> GetTipCoord();

    // calculate CTR FK
    void SolveFK(const std::array<double, 3> &config_tran,
                 const std::array<double, 3> &config_rot);

    // calculate CTR IK
    void SolveIK(const std::array<double, 3> &target_coord);

    // get CTR translation motor config
    std::array<double, 3> GetConfigTran();

    // get CTR rotation motor config
    std::array<double, 3> GetConfigRot();

private:
    // ctr class
    CTR ctr_;

    // kinematics class
    Kinematics kinematics_;

    // CTR tip coordinate
    std::array<double, 3> tip_coord_;
};