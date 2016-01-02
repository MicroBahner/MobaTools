#ifndef MOBATOOLS_H
#define MOBATOOLS_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2015 All right reserved.

  Library to drive the Stepper Motor 28BYJ-48
  connected to SPI (MOSI,CLK,SS) Interface via shift register
  or 4 pins directly
   
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include <inttypes.h>
#include <Arduino.h>

#define Servo2	Servo8		// Kompatibilität zu Version 01 und 02
//defines used in user programs
#define HALFSTEP    1
#define FULLSTEP    2
#define NOSTEP      0   // invalid-flag

#define NO_OUTPUT   0
#define PIN8_11     1
#define PIN4_7      2
#define SPI_1        3
#define SPI_2        4
#define SPI_3        5
#define SPI_4        6
#define SINGLE_PINS  7

// for formatted printing to Serial( just like fprintf )
// you need to define txtbuf with proper length to use this
#define SerialPrintf( ... ) sprintf( txtbuf,  __VA_ARGS__ ); Serial.print( txtbuf );

//#define FIXED_POSITION_SERVO_PULSES
////////////////// END OF 'PUBLIC' PARAMETERS ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// internal defines
#define FAST_PORTWRT        // if this is defined, ports are written directly in IRQ-Routines,
                            // not with 'digitalWrite' functions
#define TICS_PER_MICROSECOND (clockCyclesPerMicrosecond() / 8 ) // prescaler is 8

// defines for the stepper motor
#define MAX_STEPPER  4    // 
#define MIN_STEPTIME 800  // minimum steptime between 2 steps
#define CYCLETIME   200     // Irq-periode in us. Step time is an integer multiple
                            // of this value
#define CYCLETICS   CYCLETIME*TICS_PER_MICROSECOND

// defines for soft-leds
#define MAX_LEDS    4     // Soft On/Off of 4 LEDs

// defines for servos
    #define MINPULSEWIDTH   700     // don't make it shorter
    #define MAXPULSEWIDTH   2300    // don't make it longer
#ifdef FIXED_POSITION_SERVO_PULSES
    #define MAX_SERVOS  8
#else
    #define OVLMARGIN           180     // Overlap margin ( Overlap is MINPULSEWIDTH - OVLMARGIN )
    #define OVL_TICS       ( ( MINPULSEWIDTH - OVLMARGIN ) * TICS_PER_MICROSECOND )
    #define MARGINTICS      ( OVLMARGIN * TICS_PER_MICROSECOND )
    #define MAX_SERVOS  16  
#endif               
#define MINPULSETICS    MINPULSEWIDTH * TICS_PER_MICROSECOND
#define MAXPULSETICS    MAXPULSEWIDTH * TICS_PER_MICROSECOND
#define OFF_COUNT       50  // if autoOff is set, a pulse is switched off, if it length does not change for
                            // OFF_COUNT cycles ( = OFF_COUNT * 20ms )
#define FIRST_PULSE     100 // first pulse starts 200 tics after timer overflow, so we do not compete
                            // with overflow IRQ

///////////////////////////////////////////////////////////////////////////////////////////////
// 
typedef struct {    // portaddress and bitmask for direkt pin set/reset
   byte* Adr;
   byte Mask;
} portBits_t;

// global stepper data ( used in ISR )
typedef struct {
  volatile long stepCnt;        // nmbr of steps to take
  volatile int8_t patternIx;    // Pattern-Index of actual Step (0-7)
  int8_t patternIxInc;          // halfstep: +/-1, fullstep: +/-2, sign defines direction
  uint16_t cycSteps;            // nbr of IRQ cycles per step ( speed of motor )
  volatile uint16_t cycCnt;     // counting cycles until cycStep
  volatile long stepsFromZero;  // distance from last reference point ( always as steps in HALFSTEP mode )
                                // in FULLSTEP mode this is twice the real step number
  byte output  :6 ;             // PORTB(pin8-11), PORTD (pin4-7), SPI0,SPI1,SPI2,SPI3
  byte activ :1;  
  byte endless :1;              // turn endless
  #ifdef FAST_PORTWRT
  portBits_t portPins[4];       // Outputpins as Portaddress and Bitmask for faster writing
  #else
  byte pins[4];                 // Outputpins as Arduino numbers
  #endif
  byte lastPattern;             // only changed pins are updated ( is faster )
} stepperData_t ;

typedef union { // used output channels as bit und byte
      struct {
        uint8_t pin8_11 :1;
        uint8_t pin4_7  :1;
        uint8_t spi1    :1;
        uint8_t spi2    :1;
        uint8_t spi3    :1;
        uint8_t spi4    :1;
      };
      uint8_t outputs;
    
} outUsed_t;

// global servo data ( used in ISR )
typedef struct {
  int soll = -1;     // Position, die der Servo anfahren soll ( in Tics )
  volatile int ist;      // Position, die der Servo derzeit einnimt ( in Tics )
  byte inc;     // Schrittweite je Zyklus um Ist an Soll anzugleichen
  byte offcnt;  // counter to switch off pulses if length doesn't change
  #ifdef FAST_PORTWRT
  byte* portAdr; // port adress related to pin number
  byte  bitMask; // bitmask related to pin number
  #endif
  byte pin  :6 ; // pin 
  byte on   :1 ; // True: create pulse
  byte noAutoff :1;  // don't switch pulses off automatically
} servoData_t ;

// global Data to softleds ( used in ISR )
typedef struct {    // globale Infos zu den SoftLeds
  byte brightStep = 20; // Stufe je 20ms bei auf/abblenden
  volatile byte bright; // Derzeitige Helligkeitsstufe
  byte pin    :6 ;      // Pin-Nummer ( Muss PWM-fahig sein )
  byte on     :1 ;      // True: Led ist an
  byte aktiv  :1 ;      // Led ist aktiv
} ledData_t;


//////////////////////////////////////////////////////////////////////////////
class Stepper4
{
  private:
    uint8_t stepperIx;              // Index in Structure
    int stepsRev;                   // steps per full rotation
    int stepsToMove;                // from last point
    uint8_t stepMode;               // FULLSTEP or HALFSTEP
    uint8_t minCycSteps;            // minimum time between 2 steps
    static outUsed_t outputsUsed;
    long getSFZ();                  // get step-distance from last reference point
    void initialize(int,uint8_t,uint8_t);
  public:
    Stepper4(int steps);            // steps per 360 degree in FULLSTEP mode
    Stepper4(int steps, uint8_t mode ); 
                                    // mode means HALFSTEP or FULLSTEP
    Stepper4(int steps, uint8_t mode, uint8_t minStepTime ); // min StepTim in ms
    
    uint8_t attach( byte,byte,byte,byte); //single pins definition for output
    uint8_t attach(byte outArg);    // stepMode defaults to halfstep
    uint8_t attach(byte outArg, byte*  ); 
                                    // returns 0 on failure
    void detach();                  // detach from output, motor will not move anymore
    void write(long angle);         // specify the angle in degrees, mybe pos or neg. angle is
                                    // measured from last 'setZero' point
    void write(long angle, byte factor);        // factor specifies resolution of parameter angle
                                    // e.g. 10 means, 'angle' is angle in .1 degrees
	void writeSteps( long stepPos );// Go to position stepPos steps from zeropoint
    void setZero();                 // actual position is set as 0 angle (zeropoint)
    int setSpeed(int rpm10 );       // Set movement speed, rpm*10
    void doSteps(long count);       // rotate count steps. May be positive or negative
                                    // angle is updated internally, so the next call to 'write'
                                    // will move to the correct angle
    void rotate(int8_t direction ); // rotate endless until 'stop',
    void stop();                    // stops moving immediately
    uint8_t moving();               // returns the remaining way to the angle last set with write() in
                                    // in percentage. '0' means, that the angle is reached
                                    // 255 means the motor is rotating endlessly
    long read();                    // actual angle from zeropoint (setZero)
    long readSteps();               // actual distance to zeropoint in steps
    uint8_t attached();
};

////////////////////////////////////////////////////////////////////////////////////////
class Servo8
{
  private:
    int lastPos;     // startingpoint of movement
    uint8_t pin;
    uint8_t angle;       // in degrees
    uint8_t min16;       // minimum pulse, 16uS units  (default is 34)
    uint8_t max16;       // maximum pulse, 16uS units, (default is 150)
    uint8_t servoIndex;  //

	public:
    Servo8();
    uint8_t attach(int pin);     // attach to a pin, sets pinMode, returns 0 on failure, won't
                             // position the servo until a subsequent write() happens
                             // Only works for 9 and 10.
    uint8_t attach( int pin, bool autoOff ); // automatic switch off pulses with constant length
    uint8_t attach(int pin, int pos0, int pos180 ); // also sets position values (in us) for angele 0 and 180
    uint8_t attach(int pin, int pos0, int pos180, bool autoOff );
    void detach();
    void write(int);         // specify the angle in degrees, 0 to 180. Values obove 180 are interpreted
                             // as microseconds, limited to MaximumPulse and MinimumPulse
    void setSpeed(int);      // Set Movement speed, the higher the faster
                             // Zero means no speed control (default)
    uint8_t moving();        // returns the remaining Way to the angle last set with write() in
                             // in percentage. '0' means, that the angle is reached
    uint8_t read();          // current position in degrees (0...180)
    int   readMicroseconds();// current pulsewidth in microseconds
    uint8_t attached();
    void setMinimumPulse(uint16_t);  // pulse length for 0 degrees in microseconds, 540uS default
    void setMaximumPulse(uint16_t);  // pulse length for 180 degrees in microseconds, 2400uS default
};

//////////////////////////////////////////////////////////////////////////////////////////////
class SoftLed
{ // Switch leds on/off softly.
  // 
  public:
    SoftLed();
    uint8_t attach(uint8_t);     // Led-pin with soft on. Parameter must be a PWM-Port
    void riseTime( int );       // in millisec - falltime is the same
    void on();                   // 
    void off();                  // 
  private:
    uint8_t ledIndex;
    uint8_t ledIsOn;
    uint8_t ledBrightStep;  // PWM-steps per Interrupt with On and Off
    
};

////////////////////////////////////////////////////////////////////////////////////////////
// Timermanagement


class EggTimer
{
  public:
    EggTimer();
    void setTime(long);
    bool running();

  private:
    long timervalue;
};
#endif

