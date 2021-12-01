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
    bool connectMotor(const std::string &address, const int &i);

    // initialize motor
    bool initMotor(const int &i);

    // initialize all motor
    bool initMotors();

private:
    // check command error
    inline void _errTest(const GReturn &rc);

    // check if motor key exist
    inline bool _keyTest(const int &key_val);

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