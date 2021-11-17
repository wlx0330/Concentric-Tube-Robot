#pragma once
#include <iostream>
#include <string>
#include <math.h>
using namespace std;

class AbstractMotor
{
public:
    //convert unit length/rotation inputs to encoder pulse counts for each motor
    int GetCnts(int val);

    //convert encoder cnts back to metric motor actuation units
    int GetUnits(int cnts);

    //get encoder pulses per unit actuation
    virtual float GetPPU() = 0;

    //get brushless modules value
    virtual string GetBM() = 0;

    string m_address;
};

//Rotation motor
class HarmonicMotor : public AbstractMotor
{
public:
    //get encoder pulses per unit revolution (360 degrees)
    float GetPPU();

    //get brushless modules value
    string GetBM();

    string m_address;
};

//Translation motor
class ElectroMotor : public AbstractMotor
{
public:
    //get encoder pulses per unit length (inch)
    float GetPPU();

    //get brushless modules value
    string GetBM();

    string m_address;
};
