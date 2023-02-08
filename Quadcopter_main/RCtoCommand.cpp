// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to convert RC duty cycle values for different channels 
//          to a control command (Roll, Pitch, Yaw rate, Altitude Rate, Switch ON/OFF motors, etc). 
//          for the Arduino board.

#include "RCtoCommand.h"
using namespace RC;

RCtoCommand::RCtoCommand()
{
    _pitchCalibrated = false;
    _rollCalibrated = false;
    _yawCalibrated = false;
    _pitchCenter = 1500;
    _rollCenter = 1500;
    _yawCenter = 1500;
}

RCtoCommand::~RCtoCommand()
{
}

void RCtoCommand::setRollLimits(float min, float max)
{
    _rollLimits[0] = min;
    _rollLimits[1] = max;
}

void RCtoCommand::setPitchLimits(float min, float max)
{
    _pitchLimits[0] = min;
    _pitchLimits[1] = max;
}

void RCtoCommand::setYawRateLimits(float min, float max)
{
    _yawRateLimits[0] = min;
    _yawRateLimits[1] = max;
}

void RCtoCommand::setAltitudeRateLimits(float min, float max)
{
    _altitudeRateLimits[0] = min;
    _altitudeRateLimits[1] = max;
}

float RCtoCommand::getRoll(unsigned int v)
{
    if (_rollCalibrated)
    return ((float)v -(float)_rollCenter)/1000.0f*(_rollLimits[1] - _rollLimits[0]);

    return ((float)v - 1000.0f)/1000.0f * (_rollLimits[1] - _rollLimits[0])+_rollLimits[0];
}

float RCtoCommand::getPitch(unsigned int v)
{
    if (_pitchCalibrated)
    return ((float)v -(float)_rollCenter)/1000.0f* (_pitchLimits[1] - _pitchLimits[0]);
    return ((float)v - 1000.0f)/1000.0f * (_pitchLimits[1] - _pitchLimits[0])+ _pitchLimits[0];
}

float RCtoCommand::getYawRate(unsigned int v)
{
    if( _yawCalibrated)
    return ((float)v - _yawCenter)/1000.0f * (_yawRateLimits[1] - _yawRateLimits[0]);
    return ((float)v - 1000.0f)/1000.0f * (_yawRateLimits[1] - _yawRateLimits[0])+_yawRateLimits[0];
}

float RCtoCommand::getAltitudeRate(unsigned int v)
{
    return ((int)v - 1000)/(float)1000 * (_altitudeRateLimits[1] - _altitudeRateLimits[0])+_altitudeRateLimits[0];
}

bool RCtoCommand::urgentMotorKill(unsigned int v)
{
    return v < 1500?true:false;
}

float RC::RCtoCommand::getPIDTune(unsigned int v)
{
    const float v1 = 0.0f;
    const float v2 = 100.0f;
    return ((float)v - 1000.0f)*(v2-v1)/1000.0f + v1;
}

void RC::RCtoCommand::calibratePitch(unsigned int c)
{
    _pitchCalibrated = true;
    _pitchCenter = c;
}

void RC::RCtoCommand::calibrateRoll(unsigned int c)
{
    _rollCalibrated = true;
    _rollCenter = c;
}

void RC::RCtoCommand::calibrateYawRate(unsigned int c)
{
    _yawCalibrated = true;
    _yawCenter = c;
}
