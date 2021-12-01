#include "GalilMotionController.h"
using GMC = GalilMotionController;

// class constructor
GMC::GalilMotionController()
    : // init motor types
      _rot_motor(M_TYPE_HARMONIC),
      _lin_motor(M_TYPE_ELECTRO)
{
    // init motor container 1-3 lin 4-6 rot
    this->_motors[0] = GalilMotors(this->_lin_motor);
    this->_motors[1] = GalilMotors(this->_lin_motor);
    this->_motors[2] = GalilMotors(this->_lin_motor);
    this->_motors[3] = GalilMotors(this->_rot_motor);
    this->_motors[4] = GalilMotors(this->_rot_motor);
    this->_motors[5] = GalilMotors(this->_rot_motor);
}

// check command error
inline void GMC::_errTest(const GReturn &rc)
{
    if (rc != G_NO_ERROR)
    {
        std::cout << rc << std::endl;
        GError(rc, this->_buf, sizeof(this->_buf));
        std::cout << this->_buf << std::endl;
    }
}

// check if motor key exist
inline bool GMC::_keyTest(const int &key_val)
{
    auto iter = this->_motors.find(key_val);
    if (iter == this->_motors.end())
    {
        return false;
    }
    else
    {
        return true;
    }
}

// connect motor
bool GMC::connectMotor(const std::string &address, const int &i)
{
    if (this->_keyTest(i))
    {
        GCon g = 0;
        std::string address = "--address " + address + " --subscribe ALL";
        this->_errTest(GOpen(address.c_str(), &g));
        if (g)
        {
            this->_motors[i].ip = address;
            this->_motors[i].gcon = g;
            this->_motors[i].pos = 0;
            return true;
        }
    }
    return false;
}

// initialize motor
bool GMC::initMotor(const int &i)
{
    if (this->_keyTest(i))
    {
        GCon g = this->_motors[i].gcon;
        if (g)
        {
            std::string BM = "BM " + this->_motors[i].getBM();
            this->_errTest(GCmd(g, "BA A"));
            this->_errTest(GCmd(g, BM.c_str()));
            this->_errTest(GCmd(g, "BZ <1000>1500"));
            this->_errTest(GCmd(g, "BZA = 3"));
            this->_errTest(GCmd(g, "SHA"));
            return true;
        }
    }
    return false;
}

// initialize all motor
bool GMC::initMotors()
{
    std::array<std::future<bool>, 6UL> fut;
    for (int i = 0; i < 6; ++i)
    {
        fut[i] = std::async(
            std::launch::async,
            [this](int i) -> bool
            { auto ret = this->initMotor(i); },
            i);
    }

    bool ret = true;
    for (int i = 0; i < 6; ++i)
    {
        ret = ret && fut[i].get();
    }
    return ret;
}