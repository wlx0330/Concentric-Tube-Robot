#include "motors.h"

//convert unit length/rotation inputs to encoder pulse counts for each motor
int AbstractMotor::getCnts(int val)
{
    return (int)round((float)val * this->getPPU());
}

//convert encoder cnts back to metric motor actuation units
int AbstractMotor::getUnits(int cnts)
{
    return (int)round((float)cnts / this->getPPU());
}

//get brushless modules value
string HarmonicMotor::getBM()
{
    return "1600";
}

//get encoder pulses per unit rotation (degree)
float HarmonicMotor::getPPU() //800000 PPR / 360 degrees
{
    return 800000.f / 360.f;
}

//get brushless modules value
string ElectroMotor::getBM()
{
    return "2000";
}

//get encoder pulses per unit length (mm)
float ElectroMotor::getPPU() //1000 PPR, 4000 CPR, 2 pair of poles, 8 rev/inch
{
    return 32000.f / 25.4f;
}