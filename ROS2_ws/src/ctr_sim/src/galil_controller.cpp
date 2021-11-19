#include "galil_controller.h"

//controller class constructor
GalilMotionController::GalilMotionController()
{
    //this->mMap.clear();
    this->vPair.clear();
    this->m_pos.clear();
}

//error code test
inline void GalilMotionController::errTest(GReturn rc)
{
    if (rc != G_NO_ERROR)
    {
        cout << rc << endl;
        GError(rc, this->buf, sizeof(this->buf));
        cout << this->buf << endl;
        //GSize size = sizeof(buf);

        //if (g)
        //{
        //	GUtility(g, G_UTIL_ERROR_CONTEXT, buf, &size);
        //	cout << buf << '\n'; //further context
        //}
    }
}

//check motor type
AbstractMotor *GalilMotionController::MotorType(int MotorName)
{
    switch (MotorName)
    {
    case 1:
        return new HarmonicMotor;
    case 2:
        return new ElectroMotor;
    default:
        return NULL;
    }
}

//check ith motor connection
bool GalilMotionController::ConTest(int i)
{
    if ((i >= 0) && (i < this->vPair.size()))
    {
        return true;
    }
    else
    {
        cout << "Invalid Motor Index." << endl;
        return false;
    }
}

//connect motor and controller
bool GalilMotionController::Connect(string IP, int MotorName)
{
    GCon g = 0;
    string address = "--address " + IP + " --subscribe ALL";
    errTest(GOpen(address.c_str(), &g));
    if (g)
    {
        errTest(GInfo(g, this->buf, sizeof(this->buf)));
        // cout << this->buf << " is connected!" << endl;
        AbstractMotor *Motor = this->MotorType(MotorName); // may require a delete at the end
        Motor->m_address = IP;
        this->vPair.push_back(make_pair(g, Motor));
        this->m_pos.push_back(0);
        return true;
    }
    else
    {
        //cout << IP << " failed to connect!" << endl;
        return false;
    }
}

//initialize motor
void GalilMotionController::InitMotor(int i)
{
    GCon g = this->vPair[i].first;
    string BM = "BM " + this->vPair[i].second->GetBM();
    errTest(GCmd(g, "BA A"));
    errTest(GCmd(g, BM.c_str()));
    errTest(GCmd(g, "BZ <1000>1500"));
    errTest(GCmd(g, "BZA = 3"));
    errTest(GCmd(g, "SHA"));
}

//initialize all motors
void GalilMotionController::InitMotor()
{
    for (int i = 0; i < this->vPair.size(); i++)
    {
        this->InitMotor(i); //rewrite to support parallel operations
    }
}

//position motor to
void GalilMotionController::MoveMotor(int i)
{
    GCon g = this->vPair[i].first;
    string s;
    int move = 0;
    cout << "Enter a number to move or enter a non-number to exit:" << endl;
    while (1)
    {
        cin >> move;    //enter new position
        if (cin.fail()) //A non-number was entered
        {
            cin.clear();
            cin.ignore();
            break;
        }
        else
        {
            errTest(GCmd(g, "STA"));                                      // Go to new position
            errTest(GMotionComplete(g, "A"));                             //Wait for motion to complete
            s = "IPA=" + to_string(this->vPair[i].second->GetCnts(move)); //convert to encoder pulse speed
            errTest(GCmd(g, s.c_str()));                                  // Go to new position
        }
    }
    errTest(GCmd(g, "STA"));          //stop motor
    errTest(GMotionComplete(g, "A")); //Wait for motion to complete
    return;
}

//position tracking of one motor
void GalilMotionController::TrackMotor(int i, int pos)
{
    GCon g = this->vPair[i].first;
    string s;
    //cout << "Moving motor " << i + 1 << " to position " << pos << "..." << endl;
    s = "PAA=" + to_string(this->vPair[i].second->GetCnts(pos)); //convert to encoder pulse speed
    errTest(GCmd(g, s.c_str()));                                 //Go to new
    //errTest(GMotionComplete(g, "A"));
}

//position tracking for all motors
void GalilMotionController::TrackMotor(vector<int> pos)
{
    for (int i = 0; i < pos.size(); i++)
    {
        this->TrackMotor(i, pos[i]);
    }

    //don't wait for motion to complete??
    for (int i = 0; i < pos.size(); i++)
    {
        //GCon g = this->vPair[i].first;
        //errTest(GMotionComplete(g, "A"));
        this->m_pos[i] = pos[i];
    }
}

//set ith motor speed (mm/s and deg/s) and acc (mm/s^2 and deg/s^2)
void GalilMotionController::SetSpeed(int i, int speed, int acc_coe)
{
    GCon g = this->vPair[i].first;
    string s;
    errTest(GCmd(g, "STA"));
    errTest(GMotionComplete(g, "A"));
    speed = this->vPair[i].second->GetCnts(speed); //convert to encoder pulse speed
    s = "SPA=" + to_string(speed);                 //set speed
    errTest(GCmd(g, s.c_str()));
    s = "ACA=" + to_string(speed * acc_coe); //set acc
    errTest(GCmd(g, s.c_str()));
    s = "DCA=" + to_string(speed * acc_coe); //set de-acc
    errTest(GCmd(g, s.c_str()));
}

//set speed for all motors
void GalilMotionController::SetSpeed()
{
    for (int i = 0; i < this->vPair.size(); i++)
    {
        this->SetSpeed(i, 10, 10);
    }
}

//set speed for all motors with specific speed
void GalilMotionController::SetSpeed(int speed, int acc_coe)
{
    for (int i = 0; i < this->vPair.size(); i++)
    {
        this->SetSpeed(i, speed, acc_coe);
    }
}

//get speed
int GalilMotionController::GetSpeed(int i)
{
    GCon g = this->vPair[i].first;
    //errTest(GCmd(g, "STA"));
    //errTest(GMotionComplete(g, "A"));
    errTest(GCmdT(g, "SPA=?", this->buf, this->buf_size, NULL));
    int speed = std::strtol(this->buf, NULL, 0);
    speed = this->vPair[i].second->GetUnits(speed);
    return speed;
}

//get acc
int GalilMotionController::GetAcc(int i)
{
    GCon g = this->vPair[i].first;
    errTest(GCmdT(g, "ACA=?", this->buf, this->buf_size, NULL));
    int acc = std::strtol(this->buf, NULL, 0);
    acc = this->vPair[i].second->GetUnits(acc);
    return acc;
}

//get dec
int GalilMotionController::GetDec(int i)
{
    GCon g = this->vPair[i].first;
    errTest(GCmdT(g, "DCA=?", this->buf, this->buf_size, NULL));
    int dec = std::strtol(this->buf, NULL, 0);
    dec = this->vPair[i].second->GetUnits(dec);
    return dec;
}

//motor calibration
void GalilMotionController::Calibration(int i)
{
    cout << "Starting calibration for motor " << i + 1 << "...";
    this->MoveMotor(i);
}

//check the calibration file and set the home position
void GalilMotionController::MotorHome()
{
    vector<int> v_pos;
    ifstream ifs;
    ifs.open("MotorCalib.txt", ios::in);
    if (!ifs.is_open()) //if the calibration file does not exist
    {
        cout << "No calibration file! " << endl;
    }
    else
    {
        string IP;
        int pos;
        cout << endl
             << "Find the following calibration data: " << endl;
        while (ifs >> IP >> pos)
        {
            cout << "Motor Address: " << IP << "\tCurrent Position: " << pos << endl;
            v_pos.push_back(pos);
        }
        cout << endl;
    }
    cout << "Please select the next step: " << endl;
    cout << "1. New calibration" << endl;
    cout << "2. Use calibration" << endl;
    cout << "3. Reset calibration" << endl;
    int select;
    cin >> select;
    if (select == 1) //new calibration
    {
        for (int i = 0; i < this->vPair.size(); i++)
        {
            this->Calibration(i);
            this->SetPosition(i, 0);
        }
    }
    else if (select == 2) //use calibration
    {
        for (int i = 0; i < this->vPair.size(); i++)
        {
            this->SetPosition(i, v_pos[i]);
            this->TrackMotor(i, 0);
        }
        //wait for all motors to complete motion
        for (int i = 0; i < this->vPair.size(); i++)
        {
            errTest(GMotionComplete(this->vPair[i].first, "A")); //Wait for motion to complete
        }
        cout << "Motor homing complete!" << endl;
    }
    else if (select == 3) //no calibration
    {
        for (int i = 0; i < this->vPair.size(); i++)
        {
            this->SetPosition(i, 0);
        }
    }
    else
    {
        cin.clear();
        cin.ignore();
        cout << "Please enter a proper number. " << endl;
    }
    ifs.close();
    this->SavePos();
}

//set current position
void GalilMotionController::SetPosition(int i, int pos)
{
    GCon g = this->vPair[i].first;
    string s;
    errTest(GCmd(g, "STA"));
    errTest(GMotionComplete(this->vPair[i].first, "A"));
    errTest(GMotionComplete(g, "A"));
    s = "DPA=" + to_string(this->vPair[i].second->GetCnts(pos));
    errTest(GCmd(g, s.c_str()));
    errTest(GCmd(g, "PTA=1"));
    this->m_pos[i] = pos;
}

//get current position
int GalilMotionController::GetPosition(int i)
{
    GCon g = this->vPair[i].first;
    string s;
    return 0;
}

//update the current position to file
void GalilMotionController::SavePos()
{
    ofstream ofs; //write file
    ofs.open("MotorCalib.txt", ios::out | ios::trunc);
    for (int i = 0; i < this->vPair.size(); i++)
    {
        ofs << this->vPair[i].second->m_address << " " << this->m_pos[i] << endl;
    }
    ofs.close();
}

//jog one motor
bool GalilMotionController::JogMotor(int i, int direction)
{
    GCon g = this->vPair[i].first;
    string s;
    if (direction == 0)
    {
        errTest(GCmd(g, "STA"));
        errTest(GMotionComplete(g, "A"));
        return true;
    }
    else if (direction > 0)
    {
        int speed = this->vPair[i].second->GetCnts(GetSpeed(i)); //convert to encoder pulse speed
        s = "JGA=" + to_string(speed);
    }
    else if (direction < 0)
    {
        int speed = this->vPair[i].second->GetCnts(GetSpeed(i)); //convert to encoder pulse speed
        s = "JGA=-" + to_string(speed);
    }
    errTest(GCmd(g, "STA"));
    errTest(GMotionComplete(g, "A"));
    errTest(GCmd(g, s.c_str()));
    errTest(GCmd(g, "BG A"));
    return true;
}

//stop all motor
void GalilMotionController::StopMotor(int i)
{
    GCon g = this->vPair[i].first;
    errTest(GCmd(g, "STA"));
}