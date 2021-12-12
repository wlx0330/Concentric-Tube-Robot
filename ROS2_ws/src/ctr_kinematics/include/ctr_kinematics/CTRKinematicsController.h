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

    // calculate CTR FK
    void SolveFK(const std::array<double, 3> &config_tran,
                 const std::array<double, 3> &config_rot);

    // return CTR tip coordinates
    std::array<double, 3> GetTipCoord();

private:
    // ctr class
    CTR ctr_;

    // kinematics class
    Kinematics kinematics_;
};