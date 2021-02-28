#ifndef MOTOSTM32F1_H
#define MOTOSTM32F1_H
// ESP32 specific defines for Cpp files

//#warning STM32F1 specific cpp includes

void seizeTimerAS();
static inline __attribute__((__always_inline__)) void _noStepIRQ() {
            timer_disable_irq(MT_TIMER, TIMER_STEPCH_IRQ);
            // *bb_perip(&(MT_TIMER->regs).adv->DIER, TIMER_STEPCH_IRQ) = 0;
}
static inline __attribute__((__always_inline__)) void  _stepIRQ() {
    //timer_enable_irq(MT_TIMER, TIMER_STEPCH_IRQ) cannot be used, because this also clears pending irq's
    *bb_perip(&(MT_TIMER->regs).adv->DIER, TIMER_STEPCH_IRQ) = 1;
    interrupts();
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
void ISR_Servo( void );


static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
    timer_attach_interrupt(MT_TIMER, TIMER_SERVOCH_IRQ, ISR_Servo );
    timer_cc_enable(MT_TIMER, SERVO_CHN);
}

#endif // COMPILING_MOTOSERVO_CPP

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSOFTLED32_CPP
static inline __attribute__((__always_inline__)) void enableSoftLedIsrAS() {
    timer_cc_enable(MT_TIMER, STEP_CHN);
}

#endif // COMPILING_MOTOSOFTLED_CPP

//////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
    timer_cc_enable(MT_TIMER, STEP_CHN);
}

#endif // COMPILING_MOTOSTEPPER_CPP


#endif