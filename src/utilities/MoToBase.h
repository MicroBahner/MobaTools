#ifndef MOTOBASE_H
#define MOTOBASE_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2020 All right reserved.

  Base definitions and declarations for all parts of MobaTools
*/
#include <inttypes.h>
#include <Arduino.h>


//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
#ifdef __AVR_MEGA__ //vvvvvvvvvvvvvvvvvvvvvvvvvvvvv  AVR ATMega vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    #include <avr/interrupt.h>
    #define FAST_PORTWRT        // if this is defined, ports are written directly in IRQ-Routines,
                                // not with 'digitalWrite' functions
    #define TICS_PER_MICROSECOND (clockCyclesPerMicrosecond() / 8 ) // prescaler is 8 = 0.5us
        
    // define timer to use
    #ifdef TCNT3
        // Timer 3 is available, use it
        // #warning "Timer 3 used"
        #define TCNTx       TCNT3
        #define GET_COUNT   TCNT3
        #define TIMERx_COMPB_vect TIMER3_COMPB_vect
        #define TIMERx_COMPA_vect TIMER3_COMPA_vect
        #define OCRxB      OCR3B
        #define OCRxA      OCR3A
        #define TCCRxA     TCCR3A
        #define TCCRxB     TCCR3B
        #define WGMx3      WGM33
        #define WGMx2      WGM32
        #define ICRx       ICR3
        #define OCIExA     OCIE3A
        #define OCIExB     OCIE3B
        #define TIMSKx     TIMSK3
    #else
        // Timer 1 benutzen
        #define TCNTx       TCNT1
        #define GET_COUNT   TCNT1
        #define TIMERx_COMPB_vect TIMER1_COMPB_vect
        #define TIMERx_COMPA_vect TIMER1_COMPA_vect
        #define OCRxB      OCR1B
        #define OCRxA      OCR1A
        #define TCCRxA     TCCR1A
        #define TCCRxB     TCCR1B
        #define WGMx3      WGM13
        #define WGMx2      WGM12
        #define ICRx       ICR1
        #define OCIExA     OCIE1A
        #define OCIExB     OCIE1B
        #define TIMSKx     TIMSK1
    #endif    
    extern bool timerInitialized;

    void seizeTimer1();
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ AVR ATMega ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //--------------------------------------------------------------------------------------------------------------
#elif defined ( __STM32F1__ ) //vvvvvvvvvvvvvvvvvvvvvvvvvv STM32F1 processors vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    #define IS_32BIT
    #include <libmaple/timer.h>
    #include <libmaple/spi.h>
    #include <libmaple/nvic.h>

    #define TICS_PER_MICROSECOND (CYCLES_PER_MICROSECOND / 36 ) // prescaler is 36 = 0.5us

    #define MT_TIMER TIMER4     // Timer used by MobaTools
    #define STEP_CHN    1       // OCR channel for Stepper and Leds
    #define TIMER_STEPCH_IRQ TIMER_CC1_INTERRUPT
    #define SERVO_CHN   2       // OCR channel for Servos
    #define TIMER_SERVOCH_IRQ TIMER_CC2_INTERRUPT
    #define GET_COUNT timer_get_count(MT_TIMER)
    
    void seizeTimer1();
    //#define USE_SPI2          // Use SPI1 if not defined
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ STM32F1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //--------------------------------------------------------------------------------------------------------------
#elif defined ( ESP8266 ) //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv ESP8266 vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    #define IS_32BIT
    #define IS_ESP  8266
    #include <utilities/ESP8266_waveform.h>
    //on ESP8266 all values are in µsec
    #define TICS_PER_MICROSECOND 1
    bool gpioUsed( unsigned int gpio );
    void setGpio( unsigned int gpio ) ;
    void clrGpio( unsigned int gpio ) ;

    // struct for gpio ISR Routines ( needs one struct elemet per GPIO
    typedef struct {
        void (*gpioISR)();
        void (*MoToISR)(void *Data);
        void *IsrData;
    }gpioISR_t;

    extern gpioISR_t gpioTab[MAX_GPIO];
    //convert gpio nbr to index in gpioTab:
    #define gpio2ISRx(gpio) (gpio>5?gpio-6:gpio) // gpio 6..11 are not allowed
    #define pin2Ix(gpio) (gpio>5?gpio-6:gpio) // gpio 6..11 are not allowed
    #define chkGpio(gpio) ( (gpio<=5) || ( gpio>11 && gpio<=16 ) );
    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ ESP8266 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //--------------------------------------------------------------------------------------------------------------
#elif defined ( ESP32 ) //vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv  ESP32  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
    #define IS_32BIT
    #warning "Kompilieren für ESP32"
    #define IS_ESP  32
    // stepper related defines
    #ifdef USE_VSPI
        #define SPI_USED    VSPI
        #define MOSI        23
        #define SCK         18
        #define SS          5
    #else
        // HSPI is used by default
        #define SPI_USED    HSPI
        #define MOSI        13
        #define SCK         14
        #define SS          15
    #endif
    
    struct timerConfig_t {
      union {
        struct {
          uint32_t reserved0:   10;
          uint32_t alarm_en:     1;             /*When set  alarm is enabled*/
          uint32_t level_int_en: 1;             /*When set  level type interrupt will be generated during alarm*/
          uint32_t edge_int_en:  1;             /*When set  edge type interrupt will be generated during alarm*/
          uint32_t divider:     16;             /*Timer clock (T0/1_clk) pre-scale value.*/
          uint32_t autoreload:   1;             /*When set  timer 0/1 auto-reload at alarming is enabled*/
          uint32_t increase:     1;             /*When set  timer 0/1 time-base counter increment. When cleared timer 0 time-base counter decrement.*/
          uint32_t enable:       1;             /*When set  timer 0/1 time-base counter is enabled*/
        };
        uint32_t val;
      };
    } ;
    extern bool timerInitialized;
    // Prescaler for 64-Bit Timer ( input is 
    #define DIVIDER     APB_CLK_FREQ/2/1000000  // 0,5µs Timertic ( 80MHz input freq )
    #define TICS_PER_MICROSECOND 2              // bei 0,5 µs Timertic
    // Mutexes für Zugriff auf Daten, die in ISR verändert werden
    extern portMUX_TYPE stepperMux;
 
    #define STEPPER_TIMER     3  // Timer 1, Group 1
    extern hw_timer_t * stepTimer;
    void seizeTimer1();
    void initSPI();             // initSPI is defined in MoToStepperAVR.inc ( it is only used with MoToStepper
    
    // defines for servo and softled ( ledPwm hardware on ESP32 is used )
    #define SERVO_FREQ  50          // 20000 
    #define SERVO_BITS  16          // bitresolution for duty cycle of servos
    #define SERVO_CYCLE ( 1000000L / SERVO_FREQ ) // Servo cycle in uS
    #define DUTY100     (( 1<<SERVOBITS )-1)
    // compute pulsewidt ( in usec ) to duty )
    #define MS2DUTY( pulse ) ( (pulse *  DUTY100) / SERVO_CYCLE )  
    // compute duty to pulsewidth ( in uS )
    #define DUTY2MS( duty )  ( (duty * SERVO_CYCLE) / DUTY100 )
    
    #define LED_FREQ    100
    #define LED_BITS    10          // bitresolution for duty cycle of leds
    extern portMUX_TYPE softledMux;
    extern portMUX_TYPE servoMux;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ end of ESP32 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    //--------------------------------------------------------------------------------------------------------------
    
#else /////////////////////////////////////////  Not supported processor  //////////////////////////////////////////
    #error "Processor not supported"

#endif

#ifndef IS_ESP
    #define  IRAM_ATTR  // Attribut IRAM_ATTR entfernen wenn nicht definiert
#endif
////////////////////////////////////////////////// general defines for all plattforms  //////////////////////////////////////////

// type definitions ( if they are different for 8bit and 32bit platforms)
#ifdef IS_32BIT
	#define uintxx_t uint32_t
	#define intxx_t	int32_t
	#define uintx8_t uint32_t
	#define intx8_t	int32_t
    extern int32_t nextCycle;   // to be used in ISR for stepper and softled
#else
	#define uintxx_t	uint16_t
	#define  intxx_t	int16_t
	#define uintx8_t uint8_t
	#define intx8_t	int8_t
    extern uint8_t nextCycle;
#endif

#define ISR_IDLETIME    5000        // max time between two Stepper/Softled ISRs ( µsec )

// old Class names ( for compatibility with former sketches )
#define Stepper4    MoToStepper
#define Servo8      MoToServo  
#define SoftLed     MoToSoftLed
#define EggTimer    MoToTimer

// defines for the stepper motor
#define HALFSTEP    1
#define FULLSTEP    2
#define A4988       3   // using motordriver A4988
#define FULL2Wire   4   // not yet used
#define NOSTEP      0   // invalid-flag


// for formatted printing to Serial( just like fprintf )
// you need to define txtbuf with proper length to use this
#define SerialPrintf( ... ) sprintf( txtbuf,  __VA_ARGS__ ); Serial.print( txtbuf );

// internal defines

#define TIMERPERIODE    20000   // Timer Overflow in µs
#define TIMER_OVL_TICS  ( TIMERPERIODE*TICS_PER_MICROSECOND )


typedef struct {    // portaddress and bitmask for direkt pin set/reset
   uint8_t* Adr;
   uint8_t Mask;
} portBits_t;


#endif



