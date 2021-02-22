// STM32F1 HW-spcific Functions
#ifdef ARDUINO_ARCH_STM32F1
#define bool int
#include <MobaTools.h>
#define debugTP
#define debugPrint
#include <utilities/MoToDbg.h>

#warning "HW specfic - STM32F1 ---"

void ISR_Stepper();     // defined in MoToISR.cpp
void seizeTimerAS() {
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
        timer_init( MT_TIMER );
        timer_pause(MT_TIMER);
        // IRQ-Priorität von timer 4 interrupt auf lowest (15) setzen
        nvic_irq_set_priority ( NVIC_TIMER4, 15); // Timer 4 - stmduino sets all priorities to lowest level
                                                  // To be sure we set it here agai. These long lasting IRQ's 
                                                  // MUST be lowest priority
        /*nvic_irq_set_priority ( NVIC_EXTI0, 8); // DCC-Input IRQ must be able to interrupt other long low priority IRQ's
        nvic_irq_set_priority(NVIC_SYSTICK, 0); // Systic must be able to interrupt DCC-IRQ to get correct micros() values
        */
        timer_oc_set_mode( MT_TIMER, SERVO_CHN, TIMER_OC_MODE_FROZEN, 0 );  // comparison between output compare register and counter 
                                                                    //has no effect on the outputs
        timer_oc_set_mode( MT_TIMER, STEP_CHN, TIMER_OC_MODE_FROZEN, 0 );
        timer_set_prescaler(MT_TIMER, 36-1 );    // = 0.5µs Tics at 72MHz
        timer_set_reload(MT_TIMER, TIMERPERIODE * TICS_PER_MICROSECOND );
        timer_set_compare(MT_TIMER, STEP_CHN, 400 );
        timer_attach_interrupt(MT_TIMER, TIMER_STEPCH_IRQ, (voidFuncPtr)ISR_Stepper );
        timer_set_compare(MT_TIMER, SERVO_CHN, FIRST_PULSE );
        timer_resume(MT_TIMER);
        timerInitialized = true;  
    }
}


void enableServoIsrAS() {
}




#endif
