#ifndef RP2040_DRIVER_H
#define RP2040_DRIVER_H
//////////////////////////////////////// processor dependent defines and declarations //////////////////////////////////////////
    //--------------------------------------------------------------------------------------------------------------
//vvvvvvvvvvvvvvvvvvvvvvvvvv RP2040 ( and RP2350) processors vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define RP2040
#define __RP2040__
#define IS_32BIT
#define IRAM_ATTR       // delete in .cpp files, because it has no meaning for RP2040 processors
#define DRAM_ATTR
#define MOTOSOFTLED32		// use 32-bit version of SoftLed class

#include <timer.h>

#define CYCLETIME       1     // Cycle count in µs on 32Bit processors

#define TICS_PER_MICROSECOND 1 // RP2040 timer is clocked with 1MHz


extern bool timerInitialized;
void seizeTimer1();
//#define USE_SPI2          // Use SPI1 if not defined
//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ RP2040 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#define ARCHITECT_INCLUDE <RP2040/MoToRP2040.h>
#endif
