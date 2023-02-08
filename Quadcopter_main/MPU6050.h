// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 7, 2023
// purpose: This class is designed to obtain the inital pitch & roll of quadcopter when at started up. 
//          for the Arduino board.


#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define MPU6050_I2C 0x68

namespace Sensors
{
    class MPU6050
    {
        public:
        /// @brief Constructor
        MPU6050();
        /// @brief Destructor
        ~MPU6050();
        /// @brief This is used to initialize the MPU6050 registers (configuration of measurement unit)
        void setup();
        /// @brief This is used to read current values of accelerometer from MPU registers.
        void readAccelData();
        /// @brief returns the pitch angle from acceleromter useful for initializing datafusion angles.
        /// @return pitch angle in degrees.
        float getPitch();
        /// @brief returns the Roll angle from acceleromter useful for initializing datafusion angles.
        /// @return roll angle in degrees.
        float getRoll();
        /// @brief this is used to initiate a calibration of accelerometer three axes.
        void calibration();
        private:
        float _accelCal[3][2] = {{-0.99,0.96}, {-0.98,1.01},{-1.02,0.98}};// these values are for my gy-87.
        float _roll;
        float _x;
        float _y;
        float _z;
        float _rawx;
        float _rawy;
        float _rawz;
        const float accel_gain = 16384.0;

        /// @brief This is used to obtain raw acceleration values in G unit from MPU registers.
        void _readRawAccel();
        /// @brief This is used to calibrate the raw acceleration values to compensate for acceleration values 
        ///        not being exactly 1G in each of the axis
        void _calibrateAccel();

    };
    
}
