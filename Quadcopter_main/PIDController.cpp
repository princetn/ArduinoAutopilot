// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to provide a generic PID controller. 
//          for the Arduino board.
#include "PIDController.h"
#include <math.h>

Control::PID::PID(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _ei = 0.0;
    _ep0 = 0.0;
    _rng[0] = -200; // default
    _rng[1] = 200; // default

}

Control::PID::~PID()
{
}

void Control::PID::setRange(int min, int max)
{
    _rng[0] = min;
    _rng[1] = max;
    
}

int Control::PID::compensate(float desired, float current, float dt)
{
    float ep = round(desired - current);
    float ed = (ep-_ep0)/ dt;
    _ep0 = ep;

    _ei = _ei + ep * dt;

    float res = _kp * ep + _kd * ed + _ki * _ei;

    int r = round(res);
    
    if(r < _rng[0])
    {
        r = _rng[0];
    }

    if(r > _rng[1])
    {
        r = _rng[1];
    }    
    

    return r;
}
