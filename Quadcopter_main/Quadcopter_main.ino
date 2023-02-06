
// PID controller for quadcopter

#include "PPMReader.h"
#include <Servo.h>
#include "PIDController.h"
#include "RCtoCommand.h"
#include "HMC5883.h"



#define M1 5
#define M2 6
#define M3 9
#define M4 10


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector






// Sensors:
Sensors::HMC5883 compass;

auto ppmReader = RC::PPMReader::getInstance();
RC::RCtoCommand rctoCommand;
Servo s1, s2, s3, s4;
Control::PID pid_roll(5,1,0.5), pid_pitch(5,1,0.5), pid_altitude(250,150,50), pid_yaw(50,10,20);


unsigned long t0 = 0;
unsigned long t1 = 0;
float dt;
bool once = true;


unsigned int M1_throttle=0, M2_throttle=0, M3_throttle=0, M4_throttle =0;






void setup() {
  // put your setup code here, to run once:
  ppmReader->Start();
  Serial.begin(115200);
  s1.attach(M1);
  s2.attach(M2);
  s3.attach(M3);
  s4.attach(M4);
  s1.writeMicroseconds(000);
  s2.writeMicroseconds(000);
  s3.writeMicroseconds(000);
  s4.writeMicroseconds(000);
  

  rctoCommand.setRollLimits(-10.0f, 10.0f); // limits roll range to -10/10deg
  rctoCommand.setPitchLimits(-10.0f, 10.0f); // limits pitch range to -10/10deg
  rctoCommand.setYawRateLimits(-1.0f, 1.0f); // limits yaw range to -1/1deg/s
  constexpr float altmin = -5.0f/3.6f; // top speed of descent.
  constexpr float altmax = 5.0f/3.6f; // top speed of ascent.
  rctoCommand.setAltitudeRateLimits(altmin, altmax);

  pid_roll.setRange(-450,450);
  pid_pitch.setRange(-450,450);
  pid_altitude.setRange(-500,500);
  pid_yaw.setRange(-300,300);



  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  if (devStatus == 0) {
      // Calibration Time: generate offsets and calibrate our MPU6050
      mpu.CalibrateAccel(6);
      mpu.CalibrateGyro(6);
      mpu.PrintActiveOffsets();
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu.setDMPEnabled(true);
      mpuIntStatus = mpu.getIntStatus();
      dmpReady = true;
  }











  
  //compass.setup();
  compass.compute_offsets_scales();

}




long t = 0;
long Dtm = 1000/20; //10Hz update to motors.
long tm = 0;


float rolld, rolls, pitchd, pitchs, yawd, yaws, altd, alts;

unsigned int* ch;

void loop() {
  
  t1=micros();
  if(once)
  {
    once = false;
    t0 = t1;
  }
  dt = dt * 0.999 + 0.001* (float)(t1 - t0)/1000000.0;
  t0 = t1;

  



  // 1) read sensor data (roll, pitch, yaw, height)
  compass.readRawData();
  compass.calibrateData();
  compass.processData();
  yaws = yaws * 0.95 + 0.05*compass.getYaw();
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 
     mpu.dmpGetQuaternion(&q, fifoBuffer);
     mpu.dmpGetGravity(&gravity, &q);
     //mpu.dmpGetEuler(euler, &q);
     mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
     pitchs = ypr[1]* 180/M_PI;
     rolls = ypr[2]* 180/M_PI;   
     
            
          
    
  }
  
  // 2) read incoming RC channels for command. 
  ch = ppmReader->read();

  
  

  // 3) convert RC channels to (desired roll & pitch &, motor throttle level, 
  pitchd = rctoCommand.getPitch(ch[1]);
  rolld  = rctoCommand.getRoll(ch[0]);
  yawd   = rctoCommand.getYawRate(ch[3]);
  altd   = rctoCommand.getAltitudeRate(ch[2]);
  

  //4) compute integral, derivative, proportional errors:
  auto roll_pid = pid_roll.compensate(rolld, rolls, dt);
  auto pitch_pid = pid_pitch.compensate(pitchd, pitchs, dt);
  auto yaw_pid = pid_yaw.compensate(yawd, yaws, dt);
  auto alt_pid = pid_altitude.compensate(altd, alts, dt);

  

  

  // 5) apply result to current 4pwm throttle of 4 motors (4servo objs).

  M2_throttle = ch[2] - pitch_pid + 0         + 0*(yaw_pid>0?yaw_pid:0);
  M1_throttle = ch[2] + 0         + roll_pid  + 0*(yaw_pid<0?-yaw_pid:0)+100; // some problem with pwm? 100 offset.
  M4_throttle = ch[2] + pitch_pid + 0         + 0*(yaw_pid<0?-yaw_pid:0);
  M3_throttle = ch[2] + 0         - roll_pid + 0*(yaw_pid>0?yaw_pid:0)+100;

  // clamp the throttles
  M1_throttle = M1_throttle>2000?2000:M1_throttle;
  M1_throttle = M1_throttle<1000?1000:M1_throttle;
  M2_throttle = M2_throttle>2000?2000:M2_throttle;
  M2_throttle = M2_throttle<1000?1000:M2_throttle;
  M3_throttle = M3_throttle>2000?2000:M3_throttle;
  M3_throttle = M3_throttle<1000?1000:M3_throttle;
  M4_throttle = M4_throttle>2000?2000:M4_throttle;
  M4_throttle = M4_throttle<1000?1000:M4_throttle;
  if(millis()> t + 1000)
  {
    
    t = millis();
    Serial.println("####################PPM output###################");
    Serial.print(ch[0]);Serial.print("\t  ");
    Serial.print(ch[1]);Serial.print("\t ");
    Serial.print(ch[2]);Serial.print("\t ");
    Serial.print(ch[3]);Serial.print("\t ");
    Serial.print(ch[4]);Serial.print("\t ");
    Serial.print(ch[5]);Serial.println("\t ");
    Serial.println("####################RC to command output###################");
    Serial.print("dt= "); Serial.println(dt,10);
    Serial.print(pitchd);Serial.print("\t");
    Serial.print(rolld);Serial.print("\t");
    Serial.print(yawd);Serial.print("\t");
    Serial.println(altd);
    Serial.println("####################Control output###################");
    Serial.print(pitch_pid);Serial.print("\t");
    Serial.print(roll_pid);Serial.print("\t");
    Serial.print(yaw_pid);Serial.print("\t");
    Serial.println(alt_pid);
    Serial.print("Yaw= ");
    Serial.println(yaws);
    Serial.print("euler\t");
//    Serial.print(euler[0] * 180/M_PI);
//    Serial.print("\t");
//    Serial.print(euler[1] * 180/M_PI);
//    Serial.print("\t");
//    Serial.println(euler[2] * 180/M_PI);
    Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);  
    Serial.println("####################Motor Throttles###################");
    
    Serial.print(M1_throttle);Serial.print("\t");
    Serial.print(M2_throttle);Serial.print("\t");
    Serial.print(M3_throttle);Serial.print("\t");
    Serial.println(M4_throttle);
    
    
  }

  
  if(millis()> tm+Dtm)
  {
    if(rctoCommand.urgentMotorKill(ch[5]))// channel 6 of RC kills motors when switch is below 1500.
    {
      
      s1.writeMicroseconds(000);
      s2.writeMicroseconds(000);
      s3.writeMicroseconds(000);
      s4.writeMicroseconds(000);
    }
    else
    {
      s1.writeMicroseconds(M1_throttle);
      s2.writeMicroseconds(M2_throttle);
      s3.writeMicroseconds(M3_throttle);
      s4.writeMicroseconds(M4_throttle);
    
    }
    
    
    tm =millis();
    
  }
  

}
