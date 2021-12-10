#pragma once
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <future>
#include <array>

#include "gclibo.h"
#include "GalilMotors.h"

//galil motion controller class
class GalilMotionController
{
public:
    // class constructor
    GalilMotionController();

    // connect motor
    bool connectMotor(const int &i, const std::string &address);

    // initialize motor
    bool initMotor(const int &i);

    // set motor current location
    void setMotorLocation(const int &i, const int &pos_val);

    // set motor max speed
    void setMotorSpeedMax(const int &i, const int &speed);

    // set motor speed change rate
    void setMotorSpeedCoeff(const int &i, const int &rate);

    // drive motor
    void driveMotor(const int &i);

    // drive motor with step
    void driveMotor(const int &i, const float &step);

    // enable motor position tracking mode
    void trackMotor(const int &i, const float &pos_val);

private:
    // check command error
    inline void _errTest(const GReturn &rc);

    // check if motor key exist
    inline bool _keyTest(const int &key_val);

    // stop motor motion
    inline void _stopMotor(const int &i);

    // message buffer
    char _buf[G_SMALL_BUFFER];

    // buffer size
    GSize _buf_size = G_SMALL_BUFFER;

    // rotation motor type
    MType _rot_motor;

    // translation motor type
    MType _lin_motor;

    // motors container
    std::map<int, GalilMotors> _motors;
};