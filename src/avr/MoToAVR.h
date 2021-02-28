#ifndef MOTOAVR_H
#define MOTOAVR_H
// AVR specific defines for Cpp files

//#warning AVR specific cpp includes
void seizeTimerAS();

// _noStepIRQ und _stepIRQ werden in servo.cpp und stepper.cpp genutzt
static inline __attribute__((__always_inline__)) void _noStepIRQ() {
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK &= ~( _BV(OCIExB) );    // enable compare interrupts
    #elif defined __AVR_MEGA__ || defined ARDUINO_AVR_ATTINYX4
        TIMSKx &= ~_BV(OCIExB) ; 
    #endif
    interrupts(); // allow other interrupts
}

static inline __attribute__((__always_inline__)) void  _stepIRQ() {
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
    #elif defined __AVR_MEGA__ || defined ARDUINO_AVR_ATTINYX4
        TIMSKx |= _BV(OCIExB) ; 
    #endif
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
    // enable compare-A interrupt
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
    TIMSK |=  _BV(OCIExA);   
    #elif defined __AVR_MEGA__  || defined ARDUINO_AVR_ATTINYX4
    TIMSKx |=  _BV(OCIExA) ; 
    #endif
}


#endif // COMPILING_MOTOSERVO_CPP

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSOFTLED_CPP

#endif // COMPILING_MOTOSOFTLED_CPP

//////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
    #elif defined __AVR_MEGA__ || defined ARDUINO_AVR_ATTINYX4
        TIMSKx |= _BV(OCIExB) ; 
    #endif
}

#endif // COMPILING_MOTOSTEPPER_CPP


#endif