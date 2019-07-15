#ifndef MOTOSERVO_H
#define MOTOSERVO_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Definitions and declarations for the servo part of MobaTools
*/

#define WITHSERVO

////////////////////////////////////////////////////////////////////////////////////
// global servo data ( used in ISR )
typedef struct servoData_t {
  struct servoData_t* prevServoDataP;
  uint8_t servoIx :6 ;  // Servo number
  uint8_t on   :1 ;     // True: create pulse
  uint8_t noAutoff :1;  // don't switch pulses off automatically
  int soll;             // Position, die der Servo anfahren soll ( in Tics ). -1: not initialized
  volatile int ist;     // Position, die der Servo derzeit einnimt ( in Tics )
  int inc;              // Schrittweite je Zyklus um Ist an Soll anzugleichen
  uint8_t offcnt;       // counter to switch off pulses if length doesn't change
  #ifdef FAST_PORTWRT
  uint8_t* portAdr;     // port adress related to pin number
  uint8_t  bitMask;     // bitmask related to pin number
  #endif
  uint8_t pin     ;     // pin 
} servoData_t ;

////////////////////////////////////////////////////////////////////////////////////////
class Servo8
{
  private:
    int16_t lastPos;     // startingpoint of movement
    uint8_t pin;
    uint8_t angle;       // in degrees
    uint8_t min16;       // minimum pulse, 16uS units  (default is 34)
    uint8_t max16;       // maximum pulse, 16uS units, (default is 150)
    servoData_t servoData;  // Servo data to be used in ISR

	public:
    Servo8();
    uint8_t attach(int pin); // attach to a pin, sets pinMode, returns 0 on failure, won't
                             // position the servo until a subsequent write() happens
    uint8_t attach( int pin, bool autoOff );        // automatic switch off pulses with constant length
    uint8_t attach(int pin, int pos0, int pos180 ); // also sets position values (in us) for angele 0 and 180
    uint8_t attach(int pin, int pos0, int pos180, bool autoOff );
    void detach();
    void write(int);         // specify the angle in degrees, 0 to 180. Values obove 180 are interpreted
                             // as microseconds, limited to MaximumPulse and MinimumPulse
    void setSpeed(int);      // Set Movement speed, the higher the faster
                             // Zero means no speed control (default)
    void setSpeed(int,bool); // Set compatibility-Flag (true= compatibility with version V08 and earlier)
    #define HIGHRES 0
    #define SPEEDV08 1
    
    uint8_t moving();        // returns the remaining Way to the angle last set with write() in
                             // in percentage. '0' means, that the angle is reached
    uint8_t read();          // current position in degrees (0...180)
    int   readMicroseconds();// current pulsewidth in microseconds
    uint8_t attached();
    void setMinimumPulse(uint16_t);  // pulse length for 0 degrees in microseconds, 700uS default
    void setMaximumPulse(uint16_t);  // pulse length for 180 degrees in microseconds, 2300uS default
};

#endif