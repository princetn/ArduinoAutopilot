# ArduinoAutopilot
Autopilot for a quadcopter for the Arduino board.
## Description
The autopilot will be limited to a single Arduino board.
## Hardware:
- Quadcopter frame kit, 4 brushless motors, 4 ESCs, 2 pairs of CCW, CW props.
- HW-290 or GY-87 10DOF sensor (MPU6050 (gyro +accel), BMP085, HMC5883L(GY-87) or QMC5883L(HW-290))
- GPS module Neo-6M
## Functionality
- Provide Auto-leveling (stabilize the quadcopter so pitch and roll stay constant without any pilot input).
- Maintain Altitude.
- Follow preprogrammed waypoints.

## PID Control Tuning
Lesson learned:
1) Set all PID coefficients to 0. 
2) Start by tuning only one angle at a time i.e. pitch then roll then yaw. So that involves setting to 0 the throttles of the remaining two motors responsible the other angles.
2) Use the variometer of the RC channel 5 for live tuning each of the PID coefficients once at at time.
3) Start by tuning the derivative Kd using the method of PIDControl class setPID(0,0,0.01*rctoCommand.getPIDTune(ch[4])) inside the loop().pid_pitch.setPID(2.58,0.4,0.8);
  You can print the value to serial monitor as you tune the value so you can save it for later to set it at the constructor level once you are satisfied with the PID response.
  '''
  Serial.println(0.01*rctoCommand.getPIDTune(ch[4]),7);
  '''
    - turn the motor kill off.
    - start increasing channel 3 of rc up to some level where the two motors are spinning the props to a close to hovering but a little below. 
    - next start with lowest value and turn the knob up slowly until the quadcopter starts quickly swinging.
    - lower it slightly below that value then save the value so you use it and move to Proportional and derivative. say the value was 1.0
 4) Start tuning the proportional Kp in the same way as above.
'''
 pid_pitch.setPID(0.1*rctoCommand.getPIDTune(ch[4]),0,0.8)
'''
    - start to dial up the channel 5 knob until the quadcopter reaches a nice balance.    
    - take note of that value and then move on to Integrator. 
5) for the integrator only a very small Ki value will be sufficient so you can even for smaller granularity you can mutliply getPIDTune by 0.001x 
Obviously between step 3 & 4 and 4 & 5 you will need to cut off the motors and return to arduino IDE to modify the code and that's it.

## Sensor Data fusion
I am actually not doing any data fusion in arduino the MPU6050 has a nice DMP(digital motion processor) I am using a library I found that dumps the DMP program from motion pitch by InvenSense the hardware company which provides very accurate measurement of orientation. I looked into Kalman filter, Madgwick filter and complementary filter which all can also be used by all of this is avoided as it will overwelem the arduino microcontroller atmega.
