#pragma once
#include <iostream>
#include <string>
#include <cmath>

#include "gclibo.h"

typedef unsigned int MType;
#define M_TYPE_ELECTRO 1
#define M_TYPE_HARMONIC 2

class GalilMotors
{
public:
    // default constructor
    GalilMotors();

    // init class constructor
    GalilMotors(const MType &motor_type);

    // convert unit length/rotation inputs to encoder pulse counts for each motor
    int unitToPulse(const float &val);

    // convert encoder cnts back to metric motor actuation units
    float pulseToUnit(const int &val);

    // get encoder pulses per unit actuation
    float getPPU();

    // get brushless modules value
    std::string getBM();

    // motor IP address
    std::string ip;

    // motor position
    int pos;

    // motor connection
    GCon gcon;

    // initialization flag if motor is ready to move
    bool is_ready;

    // motor max speed
    int speed_max;

    // motor speed change rate
    int speed_coeff;

private:
    //object type identifier
    MType _motor;
};