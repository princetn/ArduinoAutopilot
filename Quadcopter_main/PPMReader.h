// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to provide ppm decoding for an RC radio 
//          for the Arduino board.
#pragma once
#include <Arduino.h>

// PPM signal must be connected to this PIN for 
// Arduino UNO.
#define InterruptPin 2
#define n 13 // number of readings before transfering data for processing (data frame)

namespace RC
{
    // This is a singleton class that can be instantiated only once so you can only use one RC control per arduino project.
    class PPMReader
    {
        public:
        /// @brief Returns a single instance of PPMReader.
        /// @return 
        static PPMReader* getInstance();

        /// @brief Singletons should not be copiable.
        /// @param other 
        PPMReader(PPMReader &other) = delete;
        /**
         * Singletons should not be assignable.
         */
        void operator=(const PPMReader &) = delete;
        
        /// @brief Destructor.
        ~PPMReader();

        /// @brief Starts receiving and processing PPM RC signal.
        void Start();

        
        /// @brief reads new incoming RC channel values.
        /// @return channel unsigned int values.
        unsigned int* read();
        

        private:
        static unsigned long _a;
        static unsigned long _b;
        static unsigned int x[n];
        static unsigned int _ch1[n];
        static unsigned int _ch[6];
        static unsigned char i;
        static PPMReader* _ppmReader;
        static bool reading;
        

        /// @brief interrupt function will be called each time interrupt was caused by a rise in ppm signal on digital pin 2.
        static void _Iread_ppm();

        protected:
        /// @brief Constructor of PPM Reader.
        PPMReader();
        

    };

} // namespace RC.
