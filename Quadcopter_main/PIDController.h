// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to provide a generic PID controller. 
//          for the Arduino board.
#pragma once

namespace Control
{
    class PID
    {
        public:
        /// @brief PID controller constructor
        /// @param kp proportional ceoff.
        /// @param ki integral coeff.
        /// @param kd derivative coeff.
        PID(float kp, float ki, float kd);
        ~PID();
        /// @brief sets the control output range.
        /// @param min minimum the control can throttle the system.
        /// @param max maximum the control can throttle the system.
        void setRange(int min, int max);
        /// @brief Calculates the new control throttle level based on new observed measurement.
        /// @param desired desired value the sytem wants to get to.
        /// @param current last sensor value
        /// @param dt delta time
        /// @return control throttle output.
        int compensate(float desired, float current, float dt);
        /// @brief Resets the PID coefficients.
        /// @param kp 
        /// @param ki 
        /// @param kd 
        void setPID(float kp, float ki, float kd);
        /// @brief Resets the integrator error to zero when motors are at rest so the value does not keep on creeping.
        /// @param  
        void resetIntegrator(void);

        private:
        float _kp;
        float _ki;
        float _kd;
        int _rng[2];
        float _ei;
        float _ep0;
    };
    
} // namespace Control

