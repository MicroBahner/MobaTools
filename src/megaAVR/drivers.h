#ifndef AVR_DRIVER_H
#define AVR_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvv  megaAVR ATMega4809 (Nano Every) vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
//#include <avr/interrupt.h>
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for megaAVR processors
#define DRAM_ATTR


#define FAST_PORTWRT        // if this is defined, ports are written directly in IRQ-Routines,
                            // not with 'digitalWrite' functions
#define TICS_PER_MICROSECOND (clockCyclesPerMicrosecond() / 8 ) // prescaler is 8 = 0.5us

// define timer to use
// defines specially für Nano Every ( MegaAVR4808 ).
// Timer TCA0 is used
#warning "Nano Every not yet completely supported"
#define OCRxA   TCA0_SINGLE_CMP0
#define OCRxB   TCA0_SINGLE_CMP1
#define GET_COUNT   TCA0_SINGLE_CNT
#define TIMERx_COMPA_vect   TCA0_CMP0_vect
#define TIMERx_COMPB_vect   TCA0_CMP1_vect
#define TIMSKx     TCA0_SINGLE_INTCTRL


#define ARCHITECT_INCLUDE <megaAVR/MoToMegaAVR.h>
#endif
