
// PID controller for quadcopter

#include "PPMReader.h"
#include <Servo.h>
#include "PIDController.h"
#include "RCtoCommand.h"
#include "Sensors/HMC5883.h"



#define M1 5
#define M2 6
#define M3 9
#define M4 10


auto ppmReader = RC::PPMReader::getInstance();
RC::RCtoCommand rctoCommand;
Servo s1, s2, s3, s4;
Control::PID pid_roll(50,10,20), pid_pitch(50,10,20), pid_altitude(250,50,150), pid_yaw(50,10,20);


unsigned long t0 = 0;
unsigned long t1 = 0;
float dt;
bool once = true;









void setup() {
  // put your setup code here, to run once:
  ppmReader->Start();
  Serial.begin(115200);
  s1.attach(M1);
  s2.attach(M2);
  s3.attach(M3);
  s4.attach(M4);

  rctoCommand.setRollLimits(-10.0f, 10.0f); // limits roll range to -10/10deg
  rctoCommand.setPitchLimits(-10.0f, 10.0f); // limits pitch range to -10/10deg
  rctoCommand.setYawRateLimits(-1.0f, 1.0f); // limits yaw range to -1/1deg/s
  constexpr float altmin = -5.0f/3.6f; // top speed of descent.
  constexpr float altmax = 5.0f/3.6f; // top speed of ascent.
  rctoCommand.setAltitudeRateLimits(altmin, altmax);

  pid_roll.setRange(-300,300);
  pid_pitch.setRange(-300,300);
  pid_altitude.setRange(-500,500);
  pid_yaw.setRange(-300,300);

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
  dt = (float)(t1 - t0)/1000000.0;
  t0 = t1;

  



  // 1) read sensor data (roll, pitch, yaw, height)
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

  if(millis()> t + 500)
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
  }

  

  // 5) apply result to current 4pwm throttle of 4 motors (4servo objs).

  unsigned int M1_throttle = ch[2] + pitch_pid + 0         + yaw_pid>0?yaw_pid:0;
  unsigned int M2_throttle = ch[2] + 0         + roll_pid  + yaw_pid<0?-yaw_pid:0;
  unsigned int M3_throttle = ch[2] - pitch_pid + 0         + yaw_pid<0?-yaw_pid:0;
  unsigned int M4_throttle = ch[2] + 0         - roll_pid + yaw_pid>0?yaw_pid:0;

  // clamp the throttles
  M1_throttle = M1_throttle>2000?2000:M1_throttle;
  M1_throttle = M1_throttle<1000?1000:M1_throttle;
  M2_throttle = M2_throttle>2000?2000:M1_throttle;
  M2_throttle = M1_throttle<1000?1000:M1_throttle;
  M3_throttle = M3_throttle>2000?2000:M1_throttle;
  M3_throttle = M3_throttle<1000?1000:M1_throttle;
  M4_throttle = M4_throttle>2000?2000:M1_throttle;
  M4_throttle = M4_throttle<1000?1000:M1_throttle;

  
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
