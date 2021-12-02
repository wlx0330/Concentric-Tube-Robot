#include "GalilMotors.h"

// default constructor
GalilMotors::GalilMotors() {}

// class constructor
GalilMotors::GalilMotors(const MType &motor_type) : _motor(motor_type) {}

//convert unit length/rotation inputs to encoder pulse counts for each motor
int GalilMotors::unitToPulse(const float &val)
{
    return (int)round(val * this->getPPU());
}

//convert encoder cnts back to metric motor actuation units
float GalilMotors::pulseToUnit(const int &val)
{
    return (float)round((float)val / this->getPPU());
}

//get encoder pulses per unit actuation
float GalilMotors::getPPU()
{
    switch (this->_motor)
    {
    case M_TYPE_ELECTRO:
        return 32000.f / 25.4f;
        break;
    case M_TYPE_HARMONIC:
        return 800000.f / 360.f;
        break;
    default:
        break;
    }
}

//get brushless modules value
std::string GalilMotors::getBM()
{
    switch (this->_motor)
    {
    case M_TYPE_ELECTRO:
        return "2000";
        break;
    case M_TYPE_HARMONIC:
        return "1600";
        break;
    default:
        break;
    }
}