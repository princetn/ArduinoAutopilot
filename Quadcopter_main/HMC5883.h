// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to provide a basic compass for for the HMC5883. 
//          for the Arduino board.

#pragma once
#include <math.h>
#include <Arduino.h>
#include <Wire.h>


#define MPU6050_I2C 0x68
#define HMC5883_I2C  0x1E

#define RadToDeg 57.29578f

namespace Sensors
{
    class HMC5883
    {
        public:
        /// @brief Constructor
        HMC5883();
        /// @brief Destructor
        ~HMC5883();

        /// @brief Sets up the HMC5883 registers via aux I2C bypass through MPU6050 
        ///        for the GY-87 10DOF. 
        /// @param  
        void setup(void);
        /// @brief Reads raw data from compass via I2C
        /// @param  
        void readRawData(void);
        /// @brief Starts calibration loop to determines ranges of 3 axes.
        /// @param  
        void startCalibration(void);
        /// @brief Computes the offsets and scales using the calibration defaults values.
        /// @param  
        void compute_offsets_scales(void);
        /// @brief Calibrates raw reads by offsetting and scaling all axes.
        /// @param  
        void calibrateData(void);
        /// @brief Calculates the x, y z magnetic field after calibration and applying the gain.
        /// @param  
        void processData(void);
        /// @brief estimates the absolute yaw with respect to the magnetic North.
        /// @param pitch pitch angle in degrees 
        /// @param roll roll angle in degrees 
        /// @return Yaw angle in degrees.
        float getYaw(float pitch=0.0, float roll = 0.0);



        private:
        int _rawx = 0;
        int _rawy = 0;
        int _rawz = 0;
        unsigned char _overflow_status;
        float _calx=0.0f;
        float _caly=0.0f;
        float _calz=0.0f;
        float _x = 0;
        float _y = 0;
        float _z = 0;
        float _yaw = 0;
        const int gain = 1090;

        int _defaults[3][2]  = //{{-1019, 165}, {-251, 253}, {-550, 1021}};
                //{{-1013, 509}, {-241, 247}, {-762, 836}};
                {{-1271, 301}, {-600, 873}, {-251, 254}};
        
        int _offset[3]={0};
        float _scale[3]={0};

    




        

    };
}
