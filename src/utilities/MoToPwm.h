#ifndef MOTOPWM_H
#define MOTOPWM_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Definitions and declarations for the pwm part of MobaTools ( only for ESP8266 )
*/
#ifdef ESP8266

///////////////////////////////////////////////////////////////////////////////////////////////
    #define PWMCYC  1000    // default analogWrite cycletime in µs ( = 1000Hz )
    const uint16_t MIN_PULSE = 50;
    
class MoToPwm
{ // create pwm pulses
  // 
  public:
    MoToPwm();
    void attach( uint8_t pin );             // set Gpio to create pulses on
    void analogWrite ( uint16_t duty1000 ); // create pwm pulse with defined dutycycle 0...1000 ( promille)
    void setFreq(float freq);               // set frequency for following analogWrite commands
    void tone(float freq);                  // create tone with dutycycle 50%
    void setPwm( long high, long low );     // set pem with free defined hig and low times ( in microseconds )
    void stop()                             // stop creating pulses
  private:
    uint8_t     pinNbr;                     // 255 means not attached
    uint32_t    pwmCycle                    // cycletime for analogWrite command
    
};
#endif

#endif