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
bool GMC::connectMotor(const int &i, const std::string &address)
{
    if (this->_keyTest(i))
    {
        GCon g = 0;
        std::string ip_command = "--address " + address + " --subscribe ALL";
        this->_errTest(GOpen(ip_command.c_str(), &g));
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
            {
                auto ret = this->initMotor(i);
                return ret;
            },
            i);
    }

    bool ret = true;
    for (int i = 0; i < 6; ++i)
    {
        ret = ret && fut[i].get();
    }
    return ret;
}

// set motor current location
void GMC::setMotorLocation(const int &i, const int &pos_val)
{
    if (this->_keyTest(i))
    {
        GCon g = this->_motors[i].gcon;
        this->_stopMotor(i);
        std::string s = "DPA=" + std::to_string(this->_motors[i].unitToPulse(pos_val));
        this->_errTest(GCmd(g, s.c_str()));
        this->_motors[i].pos = pos_val;
    }
}

// set motor max speed and speed change rate
void GMC::setMotorSpeed(const int &i, const int &speed, const int &rate)
{
    if (this->_keyTest(i))
    {
        GCon g = this->_motors[i].gcon;
        this->_stopMotor(i);
        auto pulse_speed = this->_motors[i].unitToPulse(speed); //convert to encoder pulse speed
        std::string s = "SPA=" + std::to_string(pulse_speed);   //set speed
        this->_errTest(GCmd(g, s.c_str()));
        s = "ACA=" + std::to_string(pulse_speed * rate); //set acc
        this->_errTest(GCmd(g, s.c_str()));
        s = "DCA=" + std::to_string(pulse_speed * rate); //set de-acc
        this->_errTest(GCmd(g, s.c_str()));
    }
}

// stop motor motion
inline void GMC::_stopMotor(const int &i)
{
    GCon g = this->_motors[i].gcon;
    this->_errTest(GCmd(g, "STA"));
    this->_errTest(GMotionComplete(g, "A"));
}

// drive motor
void GMC::driveMotor(const int &i)
{
    if (this->_keyTest(i))
    {
        GCon g = this->_motors[i].gcon;
        while (1)
        {
            std::cout << "Enter a number to move or enter a non-number to exit:" << std::endl;
            int step = 0;
            std::cin >> step;    //enter new position
            if (std::cin.fail()) //A non-number was entered
            {
                std::cin.clear();
                std::cin.ignore();
                return;
            }
            else
            {
                // this->_errTest(GCmd(g, "PTA=1")); // enable position tracking
                this->_stopMotor(i);         // stop motor
                std::cout << i << std::endl; // test code
                std::string s = "IPA=" + std::to_string(this->_motors[i].unitToPulse(step));
                this->_errTest(GCmd(g, s.c_str()));      // Go to new position
                this->_errTest(GMotionComplete(g, "A")); // wait for motion to complete
                this->_motors[i].pos += step;            //update position
            }
        }
    }
}