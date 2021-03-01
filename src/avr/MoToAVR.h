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
static uint8_t spiInitialized = false;
#ifdef __AVR_MEGA__
    static uint8_t spiByteCount = 0;
    static inline __attribute__((__always_inline__)) void initSpiAS() {
        if ( spiInitialized ) return;
        // initialize SPI hardware.
        // MSB first, default Clk Level is 0, shift on leading edge
        uint8_t oldSREG = SREG;
        cli();
        pinMode( MOSI, OUTPUT );
        pinMode( SCK, OUTPUT );
        pinMode( SS, OUTPUT );
        SPCR = (1<<SPIE)    // Interrupt enable
             | (1<<SPE )    // SPI enable
             | (0<<DORD)    // MSB first
             | (1<<MSTR)    // Master Mode
             | (0<<CPOL)    // Clock is low when idle
             | (0<<CPHA)    // Data is sampled on leading edge
             | (0<<SPR1) | (1<<SPR0);    // fosc/16
        digitalWrite( SS, LOW );
        SREG = oldSREG;  // undo cli() 
        spiInitialized = true;  
    }
#elif defined ARDUINO_AVR_ATTINYX4
        static inline __attribute__((__always_inline__)) void initSpiAS() {
        if ( spiInitialized ) return;
        // not yet implemented
} 
#endif  // Ende unterschiedliche AVR Prozessoren für SPI
 
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
    #elif defined __AVR_MEGA__ || defined ARDUINO_AVR_ATTINYX4
        TIMSKx |= _BV(OCIExB) ; 
    #endif
}

#endif // COMPILING_MOTOSTEPPER_CPP


#endif