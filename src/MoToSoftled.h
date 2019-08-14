#ifndef MOTOSOFTLED_H
#define MOTOSOFTLED_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Definitions and declarations for the softled part of MobaTools
*/
// Defines for Softleds
///////////////////////////////////////////////////////////////////////////////////////////////
// 
//////////////////////////////////////////////////////////////////////////////////
// global data for softleds ( used in ISR )
// the PWM pulses are created together with stepper pulses
//
// table of pwm-steps for soft on/off in CYCLETIME units ( bulb simulation). The last value means pwm cycletime
//14ms cycletime
//const uint8_t iSteps[] = { 2, 3 , 4, 6, 8, 11, 14, 17, 21, 25, 30, 36, 43, 55, 70 };
//16ms cycletime ( 60Hz )
//const uint8_t iSteps[] = {2, 8 ,14 ,20, 25,30, 35, 39, 43, 47, 50, 53, 56, 58, 60, 62, 64,66,68,70,72,73,74,75,76,78,79,80 };
//const uint8_t iSteps[] = {9, 16 ,23 ,29, 35,41, 45, 49, 53, 56, 59, 62, 64, 66, 68, 70, 71,72,73,74,75,76,77,77,78,78,79,80 };
const uint8_t iSteps[] PROGMEM = { 1, 4, 6,10,13,15,17,19,21,23,25,27,29,31,33,35,36,37,38,39,
                          40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,
                          60,61,62,63,64,65,65,66,66,67,67,68,68,69,69,70,70,71,71,72,
                          72,73,73,74,74,75,75,76,76,77,77,77,78,78,78,78,79,79,79,80 };

// 20ms cycletime ( 50Hz )
//const uint8_t iSteps[] = { 2,4, 6, 9, 12, 15, 20, 25, 30, 35, 40, 45,50,55,65,80,100 };
#define DELTASTEPS 128  // this MUST be a power of 2
#define LED_IX_MAX    ((int16_t)sizeof(iSteps) -1) // int16_t to suppress warnings when comparing to aCycle
#define LED_CYCLE_MAX   (iSteps[(int16_t)sizeof(iSteps) -1])
#define LED_STEP_MAX    LED_CYCLE_MAX *DELTASTEPS
#define LED_PWMTIME     (iSteps[(int16_t)sizeof(iSteps) -1] / 5)  // PWM refreshrate in ms
                                        // todo: dies gilt nur bei einer CYCLETIME von 200us (derzeit default)
typedef struct ledData_t {            // global led values ( used in IRQ )
  struct ledData_t*   nextLedDataP;   // chaining the active Leds
  struct ledData_t**  backLedDataPP;    // adress of pointer, that points to this led (backwards reference)
  int16_t speed;             // > 0 : steps per cycle switching on
                            // < 0 : steps per cycle switching off
                            // 0: led is inactive (not attached)
    // uint8_t invert=false;   // false: ON ist HIGH, true: ON is LOW
  int16_t aStep;      // actual step (brightness)
  int8_t aCycle;  // actual cycle ( =length of PWM pule )
  //int8_t stpCnt;     // counter for PWM cycles on same step (for low speed)
  uint8_t actPulse;    // PWM pulse is active
  uint8_t state;	// actual state: steady or incementing/decrementing
    #define NOTATTACHED 0
    #define STATE_OFF   1       // using #defines here is a little bit faster in the ISR than enums
    #define STATE_ON    2
    #define ACTIVE      3       // state >= ACTIVE means active in ISR routine
    #define INCBULB     4
    #define DECBULB     5
    #define INCLIN      6
    #define DECLIN      7
    
  volatile uint8_t invFlg;
  #ifdef FAST_PORTWRT
  portBits_t portPin;       // Outputpin as portaddress and bitmask for faster writing
  #else
  uint8_t pin;                 // Outputpins as Arduino numbers
  #endif
} ledData_t;


//////////////////////////////////////////////////////////////////////////////////////////////
class SoftLed
{ // Switch leds on/off softly.
  // 
  public:
    SoftLed();
    uint8_t attach(uint8_t pinArg, uint8_t invArg = false );     // Led-pin with soft on
    void riseTime( uint16_t );       // in millisec - falltime is the same
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


#endif