#pragma once

#include <string>
#include <vector>
#include <set>
#include <fstream>

#include "gclibo.h"
#include "motors.h"

class GalilMotionController
{
public:
    //controller class constructor
    GalilMotionController();

    //check gclib error code
    inline void errTest(GReturn rc);

    //connect motor and controller
    bool Connect(string IP, int MotorName);

    //drive one motor
    void MoveMotor(int n);

    //position tracking of one motor
    void TrackMotor(int i, int pos);

    //position tracking for all motors
    void TrackMotor(vector<int> pos);

    //initialize motor i
    void InitMotor(int i);

    //initialize all motors
    void InitMotor();

    //check ith motor connection
    bool ConTest(int i);

    //check motor type
    AbstractMotor *MotorType(int MotorName);

    //set ith motor speed
    void SetSpeed(int i, int speed, int acc_coe);

    //set speed for all motors with specific speed
    void SetSpeed(int speed, int acc_coe);

    //set speed for all motors
    void SetSpeed();

    //get speed
    int GetSpeed(int i);

    //get acc_coe
    int GetAcc(int i);

    //get deacc_coe
    int GetDec(int i);

    //motor calibration
    void Calibration(int i);

    //move motor to home position
    void MotorHome();

    //set current position
    void SetPosition(int i, int pos);

    //get current position
    int GetPosition(int i);

    //update the current position to file
    void SavePos();

    //jog one motor
    bool JogMotor(int i, int direction);

    //stop all motor
    void StopMotor(int i);

    //std::multimap<int, pair<GCon, AbstractMotor *>> mMap;
    std::vector<pair<GCon, AbstractMotor *>> vPair; //use pointer instead??
    std::vector<int> m_pos;                         //motor position member
    GSize buf_size = G_SMALL_BUFFER;
    char buf[G_SMALL_BUFFER];
};