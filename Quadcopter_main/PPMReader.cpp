// Author: Amir Gasmi <argasmi@gmail.com>
// Date: Feb 4, 2023
// purpose: This class is designed to provide ppm decoding for RC radio 
//          for the Arduino.
#include "PPMReader.h"

RC::PPMReader::PPMReader()
{
    
}

RC::PPMReader *RC::PPMReader::getInstance()
{
    if(_ppmReader==nullptr){
        _ppmReader = new PPMReader();
    }
    return _ppmReader;
}

RC::PPMReader::~PPMReader()
{

}

void RC::PPMReader::Start()
{

    pinMode(InterruptPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(2), RC::PPMReader::_Iread_ppm,   RISING);
}

unsigned int *RC::PPMReader::read()
{
    int k=n;
    bool found = false;
    reading = true; // syncronization of interrupt copying

      for(int ii = 0; ii < n; ii++)
      {
          if(_ch1[ii]> 5000){
          k = ii;
          found = true;
          break;
          }
      }
//     for(int ii =k-1;ii -k >=0 &&(5-(ii-k+1)) >=0; ii--)
//     {
//         if(_ch1[ii] <=2050 && _ch1[ii] >950)
//         _ch[5-(ii-k+1)] = _ch1[ii]; 
//     }
      if(found)
      {
        
        for(int ii =k+1;ii <n && ii-k-1<6 ; ii++)
        {
            if(_ch1[ii] <=2050 && _ch1[ii] >950)
            _ch[(ii-k-1)] = _ch1[ii]; 
        }
        
      }
      reading = false;// now interrupt can copy.



    return _ch;
}



void RC::PPMReader::_Iread_ppm()
{
    //unsigned int x[n];

    _a=micros(); //store time value a when pin value falling
    unsigned int c=((long)_a-(long)_b);      //calculating   time inbetween two peaks
    _b=_a;        // 
    x[i]=c;     //storing n value in   array
    i=i+1;
    if(i ==n)
    { 
        i = 0; 
        
        if(!reading)
        {
          
          for(int ii =0; ii <n; ii++)
          {
              _ch1[ii] = x[ii];
          }
          
        }
    }
} 

RC::PPMReader* RC::PPMReader::_ppmReader = nullptr;
unsigned long RC::PPMReader::_a = 0;
unsigned long RC::PPMReader::_b = 0;
unsigned char RC::PPMReader::i = 0;
unsigned int RC::PPMReader::_ch1[n];
unsigned int RC::PPMReader::x[n];
unsigned int RC::PPMReader::_ch[6];
bool RC::PPMReader::reading = false;

