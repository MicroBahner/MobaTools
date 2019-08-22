#ifndef MOTOSOFTLED_H
#define MOTOSOFTLED_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Definitions and declarations for the softled part of MobaTools
*/
// defines for soft-leds
#define MAX_LEDS    16     // Soft On/Off defined for compatibility reasons. There is no fixed limit anymore.


///////////////////////////////////////////////////////////////////////////////////////////////
// 
//////////////////////////////////////////////////////////////////////////////////
// global data for softleds ( used in ISR )
// the PWM pulses are created together with stepper pulses
//
// table of pwm-steps for soft on/off in CYCLETIME units ( bulb simulation). The first value means pwm cycletime
const uint8_t iSteps[] PROGMEM = { 80, 1, 4, 6,10,13,15,17,19,21,23,25,27,29,31,33,35,36,37,38,39,
                          40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,
                          60,61,62,63,64,65,65,66,66,67,67,68,68,69,69,70,70,71,71,72,
                          72,73,73,74,74,75,75,76,76,77,77,77,78,78,78,78,79,79,79 };

#define DELTASTEPS 128  // this MUST be a power of 2
#define LED_IX_MAX    ((int16_t)sizeof(iSteps) -1) // int16_t to suppress warnings when comparing to aCycle
#define LED_CYCLE_MAX   (iSteps[0])
#define LED_PWMTIME     LED_CYCLE_MAX * CYCLETIME / 1000  // PWM refreshrate in ms
                                        
enum LedStats_t:byte { NOTATTACHED, STATE_OFF, STATE_ON, ACTIVE, INCBULB, DECBULB, INCLIN, DECLIN };
                        // values >= ACTIVE means active in ISR routine
                        
typedef struct ledData_t {          // global led values ( used in IRQ )
  struct ledData_t*  nextLedDataP;  // chaining the active Leds
  struct ledData_t** backLedDataPP; // adress of pointer, that points to this led (backwards reference)
  int16_t speed;                    // > 0 : steps per cycle switching on
                                    // < 0 : steps per cycle switching off
                                    // 0: led is inactive (not attached)
  int16_t   aStep;                  // actual step (brightness)
  int8_t    aCycle;                 // actual cycle ( =length of PWM pule )
  uint8_t   actPulse;               // PWM pulse is active
  LedStats_t state;	                // actual state: steady or incementing/decrementing
    
  volatile uint8_t invFlg;
  #ifdef FAST_PORTWRT
  portBits_t portPin;               // Outputpin as portaddress and bitmask for faster writing
  #else
  uint8_t pin;                      // Outputpins as Arduino numbers
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
    void mount( LedStats_t state );
    ledData_t ledData;
    uint8_t	setpoint;
    #define OFF 0
    #define ON  1
    uint8_t ledType;        // Type of lamp (linear or bulb)
    #define LINEAR  0
    #define BULB    1
    int16_t ledSpeed;       // speed with IRQ based softleds
    
};


#endif