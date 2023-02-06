// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to provide a basic compass for for the HMC5883. 
//          for the Arduino board.

#include "HMC5883.h"

Sensors::HMC5883::HMC5883()
{
}

Sensors::HMC5883::~HMC5883()
{
}

void Sensors::HMC5883::setup(void)
{


    int err;
    Wire.begin();
    Wire.setClock(400000);
    
    


    {
        // power management
//        Wire.beginTransmission(MPU6050_I2C);          // Start the communication by using address of MPU
//        Wire.write(0x6B);                           // Access the power management register
//        Wire.write(0b00000000);                     // Set sleep = 0
//        Wire.endTransmission();                     // End the communication
        
        Wire.beginTransmission(MPU6050_I2C);
        Wire.write(0x6A);
        Wire.write(0x00);// disable I2C master mode
        Wire.endTransmission();
        Wire.beginTransmission(MPU6050_I2C);
        Wire.write(0x37);
        Wire.write(0x02); //enable I2C bypass
        Wire.endTransmission();
    
    }

 



    // now let's configure the HMC5883 chip
    // config Register A
    Wire.beginTransmission(HMC5883_I2C);          // Start the communication by using address of MPU
    Wire.write(0x00);                           // control Register  
    Wire.write(0b00011000);                     // 1bit reserved|num of sample per avg meas|Data rate|measurement mode
                                                // (0)| (00=1, 01=2, 10=4, 11=8)| (000= 0.75Hz 110= 75Hz)| (00 default normal (no bias),01 x,yz positive bias 10=negative bias)  
    err = Wire.endTransmission();
    Serial.println(err);
    // Config Register B
    Wire.beginTransmission(HMC5883_I2C);          // Start the communication by using address of MPU
    Wire.write(0x01);                           // set/reset perdio FBR 0-7 bits. 
    Wire.write(0x00100000);                           // 3bit for Gain |the rest bits must be 0
                                                // 011 = +/-2.5Gauss.. 100 = +/-4Gauss. 
    err = Wire.endTransmission();
    Serial.println(err);
    // Mode Register
    Wire.beginTransmission(HMC5883_I2C);          // Start the communication by using address of MPU
    Wire.write(0x02);                           // set/reset perdio FBR 0-7 bits. 
    Wire.write(0x00000000);                     // bit7-bit2 = High speed I2C enable| bit1-bit0 (operation mode)
                                                // op mode 00=continous measurement, 01=single mode, 10/11=idle mode.
    err = Wire.endTransmission();
    Serial.println(err);
    

 
}

void Sensors::HMC5883::readRawData(void)
{

    Wire.beginTransmission(HMC5883_I2C);
    Wire.write(0x03);
    auto err = Wire.endTransmission();
    //Serial.print("err=");
    //Serial.println(err);
    
    
    
    auto val = Wire.requestFrom(HMC5883_I2C, 7) ;
    {          
        _rawx = (int16_t)(Wire.read()<<8 | Wire.read());
        _rawz = (int16_t)(Wire.read()<<8 | Wire.read());
        _rawy = (int16_t)(Wire.read()<<8 | Wire.read());
        _overflow_status = Wire.read() & 0x03;
    }
}

void Sensors::HMC5883::startCalibration(void)
{
    auto changed = false;
    auto done = false;
    unsigned long t=0, c=0;

    while(!done)
    {
        readRawData();
        if(_rawx < _defaults[0][0]) {
            _defaults[0][0] = _rawx;
            changed = true;
        }
        if(_rawx > _defaults[0][1]) {
            _defaults[0][1] = _rawx;
            changed = true;
        }

        if(_rawy < _defaults[1][0]) {
            _defaults[1][0] = _rawy;
            changed = true;
        }
        if(_rawy > _defaults[1][1]) {
            _defaults[1][1] = _rawy;
            changed = true;
        }

        if(_rawz < _defaults[2][0]) {
            _defaults[2][0] = _rawz;
            changed = true;
        }
        if(_rawz > _defaults[2][1]) {
            _defaults[2][1] = _rawz;
            changed = true;
        }

        if (changed && !done) {
            Serial.println("CALIBRATING... Keep moving your sensor around.");
            c = millis();
        }
            t = millis();
        
        
        if ( (t - c > 20000) ) {
            done = true;
            Serial.println("DONE calibrating");
            Serial.println();
            
            Serial.print("new calibration set ={");
            Serial.print(_defaults[0][0]);
            Serial.print(", ");
            Serial.print(_defaults[0][1]);
            Serial.print(", ");
            Serial.print(_defaults[1][0]);
            Serial.print(", ");
            Serial.print(_defaults[1][1]);
            Serial.print(", ");
            Serial.print(_defaults[2][0]);
            Serial.print(", ");
            Serial.print(_defaults[2][1]);
            Serial.println("};");
        }
    }
}

void Sensors::HMC5883::compute_offsets_scales(void)
{
    float scalexyz = 1.0;
    int rng[3];

    for(int i = 0; i < 3; i++)
    {
        _offset[i] = (_defaults[i][1] + _defaults[i][0])/2;
        rng[i] = (_defaults[i][1] - _defaults[i][0]);
        scalexyz *= (float)rng[i];
    
    }
    scalexyz = pow(scalexyz, 1.0/3.0);
    for(int i = 0; i < 3; i++)
    {
        _scale[i] = scalexyz/(float)rng[i];
        Serial.print("scale[i]=");
        Serial.println(_scale[i]);
    }
   
}

void Sensors::HMC5883::calibrateData(void)
{
    _calx = (float)(_rawx - _offset[0]) * _scale[0];
    _caly = (float)(_rawy - _offset[1]) * _scale[1];
    _calz = (float)(_rawz - _offset[2]) * _scale[2];
}

void Sensors::HMC5883::processData(void)
{
    _x = (float)_calx / (float)gain;
    _y = (float)_caly / (float)gain;
    _z = (float)_calz / (float)gain;
}

float Sensors::HMC5883::getYaw(float pitch, float roll)
{
    _yaw = RadToDeg * atan2(-_y, _x);
    return _yaw;
}
