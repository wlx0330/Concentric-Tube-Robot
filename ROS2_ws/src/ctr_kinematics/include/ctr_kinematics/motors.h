#pragma once
#include <iostream>
#include <string>
#include <cmath>
using namespace std;

class AbstractMotor
{
public:
    //convert unit length/rotation inputs to encoder pulse counts for each motor
    int getCnts(int val);

    //convert encoder cnts back to metric motor actuation units
    int getUnits(int cnts);

    //get encoder pulses per unit actuation
    virtual float getPPU() = 0;

    //get brushless modules value
    virtual string getBM() = 0;

    //motor ip address
    string ip_address;
};

//Rotation motor
class HarmonicMotor : public AbstractMotor
{
public:
    //get encoder pulses per unit revolution (360 degrees)
    float getPPU();

    //get brushless modules value
    string getBM();
};

//Translation motor
class ElectroMotor : public AbstractMotor
{
public:
    //get encoder pulses per unit length (inch)
    float getPPU();

    //get brushless modules value
    string getBM();
};
