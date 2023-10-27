#ifndef RA4M1_DRIVER_H
#define RA4M1_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
    //--------------------------------------------------------------------------------------------------------------
//vvvvvvvvvvvvvvvvvvvvvvvvvv STM32F1 processors vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define RA4M1
#define __UNOR4__
#define IS_32BIT
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for Renesas processors
#define DRAM_ATTR


#define CYCLETIME       1     // Cycle count in µs on 32Bit processors

#define TICS_PER_MICROSECOND (CYCLES_PER_MICROSECOND / 16 ) // prescaler is 16 = 0.33us
//#define TICS_PER_MICROSECOND 3 // prescaler is 16 = with 48MHz Clock

#define GET_COUNT (uint32_t)gptRegP->GTCNT

extern bool timerInitialized;
void seizeTimer1();
//#define USE_SPI2          // Use SPI1 if not defined
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ STM32F1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#define ARCHITECT_INCLUDE <ra4m1/MoToRA4M1.h>
#endif
