// STM32F1 HW-spcific Functions
#ifdef ARDUINO_ARCH_ESP32
#include <MobaTools.h>
//#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>

#warning "HW specfic - STM32F1 ---"

bool timerInitialized = false;
bool spiInitialized = false;
void ISR_Stepper(void);     // defined in MoToISR.cpp
timerConfig_t timerConfig;
portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE softledMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * stepTimer = NULL;

void seizeTimerAS() {
    // Initiieren des Stepper Timers ------------------------
    stepTimer = timerBegin(STEPPER_TIMER, DIVIDER, true); // true= countup
    timerAttachInterrupt(stepTimer, &ISR_Stepper, true);  // true= edge Interrupt
    timerAlarmWrite(stepTimer, ISR_IDLETIME*TICS_PER_MICROSECOND , false); // false = no autoreload );
    timerAlarmEnable(stepTimer);
    timerInitialized = true;  
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
}


void enableServoIsrAS() {
}


void enableSoftLedIsrAS() {

}


#endif
