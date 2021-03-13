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
    uint8_t spiByteCount = 0;
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

    static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
        digitalWrite( SS, LOW );
        spiByteCount = 0;
        SPDR = spiData[1];
    }    
    
    
#elif defined ARDUINO_AVR_ATTINYX4
    static inline __attribute__((__always_inline__)) void initSpiAS() {
        if ( spiInitialized ) return;
        // set OutputPins MISO ( =DO )
        USI_SCK_PORT |= _BV(USCK_DD_PIN);   //set the USCK pin as output
        USI_DDR_PORT |= _BV(DO_DD_PIN);     //set the DO pin as output
        USI_DDR_PORT &= ~_BV(DI_DD_PIN);    //set the DI pin as input
        // set Controlregister USICR 
        USICR = 0;  //reset
        // set to 3-wire ( =SPI ) mode0,  Clock by USITC-bit, positive edge
        USICR = _BV(USIWM0) | _BV(USICS1) | _BV(USICLK);
    } 
#endif  // Ende unterschiedliche AVR Prozessoren für initSPI
 
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
    #elif defined __AVR_MEGA__ || defined ARDUINO_AVR_ATTINYX4
        TIMSKx |= _BV(OCIExB) ; 
    #endif
}

#endif // COMPILING_MOTOSTEPPER_CPP


#endif