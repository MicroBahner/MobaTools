#ifndef RP2040_DRIVER_H
#define RP2040_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
// MobaTools for  RP2040/RP2350 use the low level functionality of the RPi Pico SDK 
// Only cores that allow direct access to this SDK work with MobaTools (Earle Philhowers core )

//--------------------------------------------------------------------------------------------------------------
//vvvvvvvvvvvvvvvvvvvvvvvvvv RP2040 ( and RP2350) processors vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define RP2040
#define __RP2040__
#define IS_32BIT
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for RP2040 processors
#define DRAM_ATTR
#define MOTOSOFTLED32		// use 32-bit version of SoftLed class
#define stepperISR __not_in_flash_func(stepperISR) // stepperISR must be executed in RAM
                                                  // this is only for defining the function. in .cpp
                                                  // where it is called, it must be undef'd

#if ( F_CPU != 150000000L )
  #undef STP_TIMR_NBR
  #define STP_TIMR_NBR 0         // is obviousl not RP2350, so only timer 0 present
#endif

#include <pico/stdlib.h>		// RP2040 SDK
#include "hardware/resets.h"
#include "hardware/clocks.h"
#include "hardware/timer.h"
#include "hardware/spi.h"

#define CYCLETIME       1     // Cycle count in µs on 32Bit processors
#define TICS_PER_MICROSECOND 1 // RP2040 timer is clocked with 1MHz
#define WITH_PRINTF           // Core supports printf command
typedef struct { // TöDo für RP2040 anpasswn
    union {
        struct {
            uint32_t pin     :8;     // used pwm HW ( 0... 15
            uint32_t inUse   :1;     // 0 
            uint32_t group   :1;     // leds group ( 0/1 )
            uint32_t timer   :2;     // Timer used ( 2 for servo, 3 for softled, 0/1 unused by MobaTools
            uint32_t channel :3;     // ledc  channel ( 0..7 )
            uint32_t reserved:18;
        };
        uint32_t value;
    };
} pwmUse_t;

// Definition of used  Hw und IRQ's ( Timer/PWM/SPI )
// by default MoToStepper uses alarm 4 of timer 1
const uint8_t stepTIMER = 0; 		// RP2040 has only timer 0, maybe set to 1 on RP2350
// constexpr uint8_t stepALARM = 3;
extern uint8_t stepperAlarm;
// constexpr uint8_t stepperIRQ_NUM = TIMER_ALARM_IRQ_NUM(stepTIMER, stepALARM);
extern uint8_t stepperIRQNum; // NVIC-IRQ-number of stepper IRQ
/*
#ifdef COMPILING_MOTOSERVO_CPP
    //#warning compiling servo.cpp for RP2040
    #undef interrupts
    #undef noInterrupts
    #define interrupts()    portEXIT_CRITICAL(&servoMux);
    #define noInterrupts()  portENTER_CRITICAL(&servoMux);
#endif
#ifdef COMPILING_MOTOSOFTLED_CPP
    //#warning compiling softled.cpp for RP2040
    #undef interrupts
    #undef noInterrupts
    #define interrupts()    portEXIT_CRITICAL(&softledMux);
    #define noInterrupts()  portENTER_CRITICAL(&softledMux);
#endif
*/
#ifdef COMPILING_MOTOSTEPPER_CPP
    //#warning compiling stepper.cpp for RP2040
    #undef interrupts
    #undef noInterrupts
    #define interrupts()    irq_set_enabled (stepperIRQ, true)
    #define noInterrupts()  irq_set_enabled (stepperIRQ, false)
#endif


//extern bool timerInitialized;
void seizeTimer1();
//#define USE_SPI2          // Use SPI1 if not defined
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ RP2040 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#define ARCHITECT_INCLUDE <rp2040/MoToRP2040.h>
#endif