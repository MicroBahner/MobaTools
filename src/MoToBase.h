#ifndef MOTOBASE_H
#define MOTOBASE_H
/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Base definitions and declarations for all parts of MobaTools
*/
#include <inttypes.h>
#include <Arduino.h>
#include <avr/interrupt.h>
#include <MotoDbg.h>

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


// defines for the stepper motor
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


#define CYCLETICS   (CYCLETIME*TICS_PER_MICROSECOND)
#define MIN_STEPTIME (CYCLETIME * MIN_STEP_CYCLE) 
#define MAXRAMPLEN  16000       // Do not change!

// defines for soft-leds
#define MAX_LEDS    16     // Soft On/Off defined for compatibility reasons. There is no fixed limit anymore.

// defines for servos
#define Servo2	Servo8		// Kompatibilität zu Version 01 und 02
#ifdef FIXED_POSITION_SERVO_PULSES
    #define MAX_SERVOS  8
#else
    #define OVLMARGIN           280     // Overlap margin ( Overlap is MINPULSEWIDTH - OVLMARGIN )
    #define OVL_TICS       ( ( MINPULSEWIDTH - OVLMARGIN ) * TICS_PER_MICROSECOND )
    #define MARGINTICS      ( OVLMARGIN * TICS_PER_MICROSECOND )
    #define MAX_SERVOS  16  
#endif               

#define MINPULSETICS    (MINPULSEWIDTH * TICS_PER_MICROSECOND)
#define MAXPULSETICS    (MAXPULSEWIDTH * TICS_PER_MICROSECOND)
#define OFF_COUNT       50  // if autoOff is set, a pulse is switched off, if it length does not change for
                            // OFF_COUNT cycles ( = OFF_COUNT * 20ms )
#define FIRST_PULSE     100 // first pulse starts 200 tics after timer overflow, so we do not compete
                            // with overflow IRQ
#define SPEED_RES       4   // All position values in tics are multiplied by this factor. This means, that one 
                            // 'Speed-tic' is 0,125 µs per 20ms cycle. This gives better resolution in defining the speed.
                            // Only when computing the next interrupt time the values are divided by this value again to get
                            // the real 'timer tics'

extern uint8_t nextCycle;   // to be used in ISR for stepper and softled

// for formatted printing to Serial( just like fprintf )
// you need to define txtbuf with proper length to use this
#define SerialPrintf( ... ) sprintf( txtbuf,  __VA_ARGS__ ); Serial.print( txtbuf );

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


typedef struct {    // portaddress and bitmask for direkt pin set/reset
   uint8_t* Adr;
   uint8_t Mask;
} portBits_t;


// select timer to use
#ifdef __AVR_MEGA__
    // defines only for ATMega
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
        
#elif defined __STM32F1__
    //defines only for STM32
    #define MT_TIMER TIMER4     // Timer used by MobaTools
    #define STEP_CHN    1       // OCR channel for Stepper and Leds
    #define TIMER_STEPCH_IRQ TIMER_CC1_INTERRUPT
    #define SERVO_CHN   2       // OCR channel for Servos
    #define TIMER_SERVOCH_IRQ TIMER_CC2_INTERRUPT
    #define GET_COUNT timer_get_count(MT_TIMER)
    
    //#define USE_SPI2          // Use SPI1 if not defined
#endif


                            
void seizeTimer1();

inline void _noStepIRQ() {
        #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
            TIMSK &= ~( _BV(OCIExB) );    // enable compare interrupts
        #elif defined __AVR_MEGA__
            TIMSKx &= ~_BV(OCIExB) ; 
        #elif defined __STM32F1__
            timer_disable_irq(MT_TIMER, TIMER_STEPCH_IRQ);
            //*bb_perip(&(MT_TIMER->regs).adv->DIER, TIMER_STEPCH_IRQ) = 0;
        #endif
}
inline void  _stepIRQ() {
        #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
            TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
        #elif defined __AVR_MEGA__
            TIMSKx |= _BV(OCIExB) ; 
        #elif defined __STM32F1__
            //timer_enable_irq(MT_TIMER, TIMER_STEPCH_IRQ) cannot be used, because this also clears pending irq's
            *bb_perip(&(MT_TIMER->regs).adv->DIER, TIMER_STEPCH_IRQ) = 1;
            interrupts();
        #endif
}

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



