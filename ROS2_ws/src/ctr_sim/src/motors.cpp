#include "motors.h"

//convert unit length/rotation inputs to encoder pulse counts for each motor
int AbstractMotor::GetCnts(int val)
{
    val = (int)round((float)val * this->GetPPU()); //convert to encoder pulse speed
    return val;
}

//convert encoder cnts back to metric motor actuation units
int AbstractMotor::GetUnits(int cnts)
{
    cnts = (int)round((float)cnts / this->GetPPU());
    return cnts;
}

//get brushless modules value
string HarmonicMotor::GetBM()
{
    return "1600";
}

//get encoder pulses per unit rotation (degree)
float HarmonicMotor::GetPPU() //800000 PPR / 360 degrees
{
    return 800000.f / 360.f;
}

//get brushless modules value
string ElectroMotor::GetBM()
{
    return "2000";
}

//get encoder pulses per unit length (mm)
float ElectroMotor::GetPPU() //1000 PPR, 4000 CPR, 2 pair of poles, 8 rev/inch
{
    return 32000.f / 25.4f;
}