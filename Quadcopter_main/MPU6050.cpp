// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 7, 2023
// purpose: This class is designed to obtain the inital pitch & roll of quadcopter when at started up. 
//          for the Arduino board.


#include "MPU6050.h"

Sensors::MPU6050::MPU6050()
{
}

Sensors::MPU6050::~MPU6050()
{
}

void Sensors::MPU6050::setup()
{
  Wire.begin();
  Wire.setClock(400000);
    // power management
  Wire.beginTransmission(MPU6050_I2C);          // Start the communication by using address of MPU
  Wire.write(0x6B);                           // Access the power management register
  Wire.write(0b00000000);                     // Set sleep = 0
  Wire.endTransmission();                     // End the communication

  // configure gyro
  Wire.beginTransmission(MPU6050_I2C);
  Wire.write(0x1B);                           // Access the gyro configuration register
  Wire.write(0b00000000);
  Wire.endTransmission();

  // configure accelerometer
  Wire.beginTransmission(MPU6050_I2C);
  Wire.write(0x1C);                           // Access the accelerometer configuration register
  Wire.write(0b00000000);                     // set to +/-2G full scale. It cannot measure higher than 2G.
  Wire.endTransmission(); 

  // Configure INTA
  Wire.beginTransmission(MPU6050_I2C);
  Wire.write(0x38);                           // Access the accelerometer Interrupt register
  Wire.write(0b00000001);
  Wire.endTransmission();
  // configure gyro samplerate divider to 7 
  Wire.beginTransmission(MPU6050_I2C);          // Start the communication by using address of MPU
  Wire.write(0x19);                           // Access the power management register
  Wire.write(7);                     // Set sampling frequency to 8000Hz/(1+7) = 1000Hz
  Wire.endTransmission();
}

void Sensors::MPU6050::readAccelData()
{
    _readRawAccel();
    _calibrateAccel();
}

float Sensors::MPU6050::getPitch()
{
    return 180.0/M_PI * atan2(_y, _z);
}

float Sensors::MPU6050::getRoll()
{
    return 180.0/M_PI * atan2(-_x,  sqrt(_y*_y + _z * _z));
}

void Sensors::MPU6050::calibration()
{
    Serial.println("Accel. Calibration...");
    Serial.println("Accel. X calibration put sensor on X+ when done press enter");
    while(true){
    if(Serial.available()>0)
    {
        _readRawAccel();
        
        _accelCal[0][0] = _rawx;
        Serial.print(_accelCal[0][0]);
        if(Serial.read() == 10)
        {
        Serial.println("Done X-");
        break;
        
        
        }
    }
    }

    Serial.println("Accel. X calibration put sensor on X+ when done press enter");
    while(true){
    if(Serial.available()>0)
    {
        _readRawAccel();
        _accelCal[0][1] = _rawx;
        Serial.print(_accelCal[0][1]);
        if(Serial.read() == 10)
        {
        Serial.println("Done X+");
        break;
        
        
        }
    }
    }
    Serial.println("Accel. Y calibration put sensor on Y- when done press enter");
  while(true){
  if(Serial.available()>0)
  {
    _readRawAccel();
    _accelCal[1][0] = _rawy;
    Serial.print(_accelCal[1][0]);
    if(Serial.read() == 10)
    {
      Serial.println("Done Y-");
      break;
      
      
    }
  }
  }
  Serial.println("Accel. Y calibration put sensor on Y+ when done press enter");
  while(true){
  if(Serial.available()>0)
  {
    _readRawAccel();
    _accelCal[1][1] = _rawy;
    Serial.print(_accelCal[1][1]);
    if(Serial.read() == 10)
    {
      Serial.println("Done Y+");
      break;
      
      
    }
  }
  }

  Serial.println("Accel. Z calibration put sensor on Z- when done press enter");
  while(true){
  if(Serial.available()>0)
  {
    _readRawAccel();
    _accelCal[2][0] = _rawz;
    Serial.print(_accelCal[2][0]);
    if(Serial.read() == 10)
    {
      Serial.println("Done Z-");
      break;
      
      
    }
  }
  }
  Serial.println("Accel. Z calibration put sensor on Z+ when done press enter");
  while(true){
  if(Serial.available()>0)
  {
    _readRawAccel();
    _accelCal[2][1] = _rawz;
    Serial.print(_accelCal[2][1]);
    if(Serial.read() == 10)
    {
      Serial.println("Done Z+");
      break;
      
      
    }
  }
  }

}

void Sensors::MPU6050::_readRawAccel()
{
    long accx, accy, accz;

    Wire.beginTransmission(MPU6050_I2C);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(0b1101000,6);
    while(Wire.available() < 6);
    accx = Wire.read()<<8|Wire.read();
    accy = Wire.read()<<8|Wire.read();
    accz = Wire.read()<<8|Wire.read();
    
    _rawx = accx/accel_gain;
    _rawy = accy/accel_gain;
    _rawz = accz/accel_gain;

}

void Sensors::MPU6050::_calibrateAccel()
{
    _x = (_rawx - _accelCal[0][0])*2.0 / (_accelCal[0][1] - _accelCal[0][0]) - 1.0f;
    _y = (_rawy - _accelCal[1][0])*2.0 / (_accelCal[1][1] - _accelCal[1][0]) - 1.0f;
    _z = (_rawz - _accelCal[2][0])*2.0 / (_accelCal[2][1] - _accelCal[2][0]) - 1.0f;
}
