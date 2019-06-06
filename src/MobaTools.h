#ifndef MOBATOOLS_H
#define MOBATOOLS_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  MobaTools V1.0
   (C) 02-2019 fpm fpm@mnet-online.de
   
  History:
  V1.1  Anfahr- und Bremsrampe für die Stepper
  V1.0  11-2017 Use of Timer 3 if available ( on AtMega32u4 and AtMega2560 )
  V0.9  03-2017
        Better resolution for the 'speed' servo-paramter (programm starts in compatibility mode)
        outputs for softleds can be inverted
        MobaTools run on STM32F1 platform
        
  V0.8 02-2017
        Enable Softleds an all digital outputs
  V0.7 01-2017
		Allow nested Interrupts with the servos. This allows more precise other
        interrupts e.g. for NmraDCC Library.
		A4988 stepper driver IC is supported (needs only 2 ports: step and direction)

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

/*  08-02-17    start of implementing STM32 support
    03-02-17 / V0.8  Softleds now working on all digital outputs
    02-11-16 / (V0.7) Updating Stepper driver:
   - stepper motor can be connected ba means of a4988 stepper motor driver IC
     this uses only 2 pins: STEP and DIRECTION
*/
#include <inttypes.h>
#include <Arduino.h>

#ifndef  __AVR_MEGA__
#ifndef __STM32F1__
#error "Only AVR AtMega  or STM32F1 processors are supported"
#endif
#endif
#ifdef __STM32F1__
#include <libmaple/timer.h>
#include <libmaple/spi.h>
#include <libmaple/nvic.h>
#endif


#define Servo2	Servo8		// Kompatibilität zu Version 01 und 02
//defines used in user programs
#define HALFSTEP    1
#define FULLSTEP    2
#define A4988       3   // using motordriver A4988
#define NOSTEP      0   // invalid-flag

#define NO_OUTPUT   0
#ifdef __AVR_MEGA__
#define PIN8_11     1
#define PIN4_7      2
#endif
#define SPI_1        3
#define SPI_2        4
#define SPI_3        5
#define SPI_4        6
#define SINGLE_PINS  7
#define A4988_PINS  8

// for formatted printing to Serial( just like fprintf )
// you need to define txtbuf with proper length to use this
#define SerialPrintf( ... ) sprintf( txtbuf,  __VA_ARGS__ ); Serial.print( txtbuf );

//#define FIXED_POSITION_SERVO_PULSES
////////////////// END OF 'PUBLIC' PARAMETERS ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
// internal defines
#ifdef __AVR_MEGA__
    // defines only for ATMega
    #define FAST_PORTWRT        // if this is defined, ports are written directly in IRQ-Routines,
                                // not with 'digitalWrite' functions
    #define TICS_PER_MICROSECOND (clockCyclesPerMicrosecond() / 8 ) // prescaler is 8 = 0.5us
        
#elif defined __STM32F1__
    //defines only for STM32
    #define TICS_PER_MICROSECOND (clockCyclesPerMicrosecond() / 36 ) // prescaler is 36 = 0.5us
#endif
#define TIMERPERIODE    20000   // Timer Overflow in µs
#define TIMER_OVL_TICS  ( TIMERPERIODE*TICS_PER_MICROSECOND )
// defines for the stepper motor
#define MAX_STEPPER  6    // 
#define MIN_STEPTIME 800  // minimum steptime between 2 steps
#define CYCLETIME   200     // Irq-periode in us. Step time is an integer multiple
                            // of this value
#define CYCLETICS   (CYCLETIME*TICS_PER_MICROSECOND)
#define RAMPOFFSET  16       // startvaue of rampcounter

// defines for soft-leds
#define MAX_LEDS    16     // Soft On/Off defined for compatibility reasons. There is no fixed limit anymore.

// defines for servos
#define WITHSERVO
    #define MINPULSEWIDTH   700     // don't make it shorter
    #define MAXPULSEWIDTH   2300    // don't make it longer
#ifdef FIXED_POSITION_SERVO_PULSES
    #define MAX_SERVOS  8
#else
    #define OVLMARGIN           280     // Overlap margin ( Overlap is MINPULSEWIDTH - OVLMARGIN )
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
#define SPEED_RES       4   // All position values in tics are multiplied by this factor. This means, that one 
                            // 'Speed-tic' is 0,125 µs per 20ms cycle. This gives better resolution in defining the speed.
                            // Only when computing the next interrupt time the values are divided by this value again to get
                            // the real 'timer tics'

// Defines for Softleds
#define WITHSOFTLED
///////////////////////////////////////////////////////////////////////////////////////////////
// 
typedef struct {    // portaddress and bitmask for direkt pin set/reset
   uint8_t* Adr;
   uint8_t Mask;
} portBits_t;

/////////////////////////////////////////////////////////////////////////////////
// global stepper data ( used in ISR )
enum rampStats_t:byte { INACTIVE, STOPPED, RAMPSTART, RAMPACCEL, CRUISING, STARTDECEL, RAMPDECEL  };
typedef struct stepperData_t {
  struct stepperData_t *nextStepperDataP;    // chain pointer
  volatile long stepCnt;        // nmbr of steps to take
  long stepCnt2;                // nmbr of steps to take after automatic reverse
  volatile int8_t patternIx;    // Pattern-Index of actual Step (0-7)
  int8_t   patternIxInc;        // halfstep: +/-1, fullstep: +/-2, A4988 +1/-1  the sign defines direction
  uint16_t tCycSteps;           // nbr of IRQ cycles per step ( target value of motorspeed  )
  uint16_t tCycRemain;          // Remainder of division when computing tCycSteps
  uint16_t aCycSteps;           // nbr of IRQ cycles per step ( actual motorspeed  )
  uint16_t aCycRemain;          // accumulate tCycRemain when cruising
  uint16_t cyctXramplen;        // precompiled  tCycSteps*rampLen*RAMPOFFSET
  int16_t  stepRampLen;         // Length of ramp in steps
  uint16_t stepsInRamp;         // stepcounter within ramp ( counting from stop: incrementing in startramp, decrementing in stopramp
  rampStats_t rampState;        // State of acceleration/deceleration
  volatile uint16_t cycCnt;     // counting cycles until cycStep
  volatile long stepsFromZero;  // distance from last reference point ( always as steps in HALFSTEP mode )
                                // in FULLSTEP mode this is twice the real step number
  uint8_t output  :6 ;             // PORTB(pin8-11), PORTD (pin4-7), SPI0,SPI1,SPI2,SPI3, SINGLE_PINS, A4988_PINS
  uint8_t activ :1;  
  //uint8_t endless :1;              // turn endless
  #ifdef FAST_PORTWRT
  portBits_t portPins[4];       // Outputpins as Portaddress and Bitmask for faster writing
  #else
  uint8_t pins[4];                 // Outputpins as Arduino numbers
  #endif
  uint8_t lastPattern;             // only changed pins are updated ( is faster )
} stepperData_t ;

typedef union { // used output channels as bit and uint8_t
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
#ifdef WITHSOFTLED // Constants and variables for softleds
//////////////////////////////////////////////////////////////////////////////////
// global data for softleds ( used in ISR )
// the PWM pulses are created together with stepper pulses
//
// table of pwm-steps for soft on/off in CYCLETIME units ( bulb simulation). The last value means pwm cycletime
//14ms cycletime
//const uint8_t iSteps[] = { 2, 3 , 4, 6, 8, 11, 14, 17, 21, 25, 30, 36, 43, 55, 70 };
//16ms cycletime ( 60Hz )
//const uint8_t iSteps[] = {2, 8 ,14 ,20, 25,30, 35, 39, 43, 47, 50, 53, 56, 58, 60, 62, 64,66,68,70,72,73,74,75,76,78,79,80 };
const uint8_t iSteps[] = {9, 16 ,23 ,29, 35,41, 45, 49, 53, 56, 59, 62, 64, 66, 68, 70, 71,72,73,74,75,76,77,77,78,78,79,80 };
// 20ms cycletime ( 50Hz )
//const uint8_t iSteps[] = { 2,4, 6, 9, 12, 15, 20, 25, 30, 35, 40, 45,50,55,65,80,100 };

#define LED_STEP_MAX    (sizeof(iSteps) -1)
#define LED_CYCLE_MAX   (iSteps[LED_STEP_MAX])
#define LED_PWMTIME     (iSteps[LED_STEP_MAX] / 5)  // PWM refreshrate in ms
                                        // todo: dies gilt nur bei einer CYCLETIME von 200us (derzeit default)
typedef struct ledData_t {            // global led values ( used in IRQ )
  struct ledData_t*   nextLedDataP;   // chaining the active Leds
  struct ledData_t**  backLedDataPP;    // adress of pointer, that points to this led (backwards reference)
  int8_t speed;             // > 0 : steps per cycle ( more steps = more speed )
                            // < 0 : cycles per step ( more cycles = less speed )
                            // 0: led is inactive (not attached)
    // uint8_t invert=false;   // false: ON ist HIGH, true: ON is LOW
  int8_t aStep;      // actual step (brightness)
  int8_t aCycle;  // actual cycle ( =length of PWM pule )
  int8_t stpCnt;     // counter for PWM cycles on same step (for low speed)
  uint8_t actPulse;    // PWM pulse is active
  uint8_t state;	// actual state: steady or incementing/decrementing
    #define NOTATTACHED 0
    #define STATE_OFF   1       // using #defines here is a little bit faster in the ISR than enums
    #define STATE_ON    2
    #define ACTIVE      3       // state >= ACTIVE means active in ISR routine
    #define INCFAST     3
    #define DECFAST     4
    #define INCSLOW     5
    #define DECSLOW     6
    #define INCLIN      7
    #define DECLIN      8
    
  volatile uint8_t invFlg;
  #ifdef FAST_PORTWRT
  portBits_t portPin;       // Outputpin as portaddress and bitmask for faster writing
  #else
  uint8_t pin;                 // Outputpins as Arduino numbers
  #endif
} ledData_t;
#endif


//////////////////////////////////////////////////////////////////////////////
class Stepper4
{
  private:
    stepperData_t _stepperData;      // Variables that are used in IRQ
    uint8_t stepperIx;              // Objectnumber ( 0 ... MAX_STEPPER )
    int stepsRev;                   // steps per full rotation
    uint16_t _stepSpeed10;          // speed in steps/10sec
    long stepsToMove;                // from last point
    uint8_t stepMode;               // FULLSTEP or HALFSTEP
    //uint8_t minCycSteps;            // minimum time between 2 steps without ramp
                                    // ramp starts with this speed if wanted speed ist faster
    //uint8_t minrCycSteps;           // absolute minimum time between 2 steps even with ramp
    static outUsed_t outputsUsed;
    long getSFZ();                  // get step-distance from last reference point
    bool _chkRunning();             // check if stepper is running
    void initialize(int,uint8_t,uint8_t);
    uint16_t  _setRampValues();
  public:
    Stepper4(int steps);            // steps per 360 degree in FULLSTEP mode
    Stepper4(int steps, uint8_t mode ); 
                                    // mode means HALFSTEP or FULLSTEP
    Stepper4(int steps, uint8_t mode, uint8_t minStepTime ); // min StepTim in ms
    
    uint8_t attach( uint8_t,uint8_t,uint8_t,uint8_t); //single pins definition for output
    uint8_t attach( uint8_t stepP, uint8_t dirP); // Port for step and direction in A4988 mode
    uint8_t attach(uint8_t outArg);    // stepMode defaults to halfstep
    uint8_t attach(uint8_t outArg, uint8_t*  ); 
                                    // returns 0 on failure
    void detach();                  // detach from output, motor will not move anymore
    void write(long angle);         // specify the angle in degrees, mybe pos or neg. angle is
                                    // measured from last 'setZero' point
    void write(long angle, uint8_t factor);        // factor specifies resolution of parameter angle
                                    // e.g. 10 means, 'angle' is angle in .1 degrees
	void writeSteps( long stepPos );// Go to position stepPos steps from zeropoint
    void setZero();                 // actual position is set as 0 angle (zeropoint)
    int setSpeed(int rpm10 );       // Set movement speed, rpm*10
    uint16_t setSpeedSteps( uint16_t speed10 ); // set speed withput changing ramp, returns ramp length
    uint16_t setSpeedSteps( uint16_t speed10, int16_t rampLen ); // set speed and ramp, returns ramp length
    uint16_t setRampLen( uint16_t rampLen ); // set new ramplen in steps without changing speed
    //int setAcceleration(int rpm10Ps ); // Set Acceleration in rpm*10 per second ( 0=no acceleration ramp )
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

//////////////////////////////////////////////////////////////////////////////////////////////
class SoftLed
{ // Switch leds on/off softly.
  // 
  public:
    SoftLed();
    uint8_t attach(uint8_t pinArg, uint8_t invArg = false );     // Led-pin with soft on
    void riseTime( int );       // in millisec - falltime is the same
    void on();                   // 
    void off();                  // 
	void write( uint8_t );			// is ON or OFF
    void write( uint8_t time, uint8_t type  ); //whether it is a linear or bulb type
    void toggle( void ); 
  private:
    void mount( uint8_t state );
    ledData_t ledData;
    uint8_t	setpoint;
    #define OFF 0
    #define ON  1
    uint8_t ledType;        // Type of lamp (linear or bulb)
    #define LINEAR  0
    #define BULB    1
    uint8_t ledIx;
    uint8_t ledValid;       // Flag that this is a valid instance
    #define LEDVALID 0x55
    uint8_t ledSpeed;       // speed with IRQ based softleds
    
};

////////////////////////////////////////////////////////////////////////////////////////////
// Timermanagement


class EggTimer
{
  public:
    EggTimer();
    void setTime( long);
    bool running();
    long getTime();

  private:
    bool active;
    long endtime;
};
#endif

