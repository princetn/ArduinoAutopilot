// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to convert RC duty cycle values for different channels 
//          to a control command (Roll, Pitch, Yaw rate, Altitude Rate, Switch ON/OFF motors, etc). 
//          for the Arduino board.
#pragma once

namespace RC
{

class RCtoCommand
{
    public:
    /// @brief Constructor of converter from RC channel 1000-2000 to orientation & altitude & arm/disarm motors.    
    RCtoCommand();
    /// @brief Destructor.
    ~RCtoCommand();

    /// @brief This sets rollLimits
    /// @param min is the the minimum roll corresponding to channel 1 lowest stick
    /// @param max is the the maximum roll corresponding to channel 1 lowest stick
    void setRollLimits(float min, float max);
    void setPitchLimits(float min, float max);
    void setYawRateLimits(float min, float max);
    void setAltitudeRateLimits(float min, float max);

    /// @brief  Calculates the roll angle based on Channel 1 stick input
    /// @param v channel 1 value
    /// @return desired roll angle
    float getRoll(unsigned int v);
    /// @brief Calculate the pitch angle based on channel 2 stick input
    /// @param v channel 2 value
    /// @return desired pitch angle
    float getPitch(unsigned int v);
    /// @brief Calculates the yaw angle rate (rotation speed deg/s) based on channel 4 stick input
    /// @param v channel 4 value
    /// @return 
    float getYawRate(unsigned int v);
    /// @brief Calculates Atitude Rate (Ascent/Descent speed m/s)
    /// @param v channel 3 value
    /// @return altitude rate
    float getAltitudeRate(unsigned int v);

    /// @brief Checks for channel 6 of RC switch if urgent motor kill was requested to set all motors to 0 save props.
    /// @param v channel 6
    /// @return True: kill motors regarless of other channels values. False: normal operation.
    bool urgentMotorKill(unsigned int v);

    /// @brief used to tune the PID control value through channel 5 of radio.
    /// @param  
    /// @return returns the mapped value from channel 1000-2000 to a smaller range.
    float getPIDTune(unsigned int v);


    

    




    private:
    float _yawRate;
    float _roll;
    float _pitch;
    float _altitutdeRate;
    float _yawRateLimits[2];
    float _pitchLimits[2];
    float _rollLimits[2];
    float _altitudeRateLimits[2];



};
}
