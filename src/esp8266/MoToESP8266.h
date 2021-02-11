#ifndef MOTOESP8266_H
#define MOTOESP8266_H
// ESP8266 specific declarations for Cpp files
#warning ESP8266 specific cpp includes

#if defined COMPILING_MOTOSERVO_CPP

static inline __attribute__((__always_inline__)) void startServoPulse(servoData_t *servoDataP, uint32_t pulseWidth ) {
    startWaveformMoTo(servoDataP->pin, pulseWidth/TICS_PER_MICROSECOND/SPEED_RES, TIMERPERIODE-(pulseWidth/TICS_PER_MICROSECOND/SPEED_RES),0);
}

static inline __attribute__((__always_inline__)) void servoWrite( servoData_t *servoDataP, uint32_t pulseWidth ) {
    // the same funktion exists for ESP32 ( with another internal call )
    startWaveformMoTo(servoDataP->pin, pulseWidth/TICS_PER_MICROSECOND/SPEED_RES, TIMERPERIODE-(pulseWidth/TICS_PER_MICROSECOND/SPEED_RES),0);
}

static inline __attribute__((__always_inline__)) void servoPulseOff( servoData_t *servoDataP ) {
    stopWaveformMoTo( servoDataP->pin );
}
#endif // compiling servo

#ifdef COMPILING_MOTOSOFTLED_CPP

static inline __attribute__((__always_inline__)) uint8_t attachSoftledAS( ledData_t *ledDataP ) {
    gpioTab[gpio2ISRx(ledDataP->pin)].MoToISR = (void (*)(void*))ISR_Softled;
    gpioTab[gpio2ISRx(ledDataP->pin)].IsrData = ledDataP;
    attachInterrupt( ledDataP->pin, gpioTab[gpio2ISRx(ledDataP->pin)].gpioISR, ledDataP->invFlg?RISING:FALLING );
    return ledDataP->pin;
}

static inline __attribute__((__always_inline__)) void startLedPulseAS( uint8_t pin, uint8_t invFlg, uint32_t pulseLen ){
    // start or change the pwmpulses on the led pin.
    // with invFlg set pulseLen is lowtime, else hightime
    if ( invFlg ) {
        startWaveformMoTo(pin, PWMCYC-pulseLen, pulseLen,0);
    } else {
        startWaveformMoTo(pin, pulseLen, PWMCYC-pulseLen,0);
    }

}

static inline __attribute__((__always_inline__)) void softLedOffAS(  uint8_t pin, uint8_t invFlg ){
    stopWaveformMoTo( pin );
    digitalWrite( pin , invFlg );
}

static inline __attribute__((__always_inline__)) void softLedOnAS(  uint8_t pin, uint8_t invFlg ){
    // Bei ESP8266 findet die endgültige Einschaltung erst am Ende des Impulszyklus statt
    attachInterrupt( pin, gpioTab[gpio2ISRx(pin)].gpioISR, invFlg?FALLING:RISING ); // leading edge
}

static inline __attribute__((__always_inline__)) void softLedOn2AS(  uint8_t pin, uint8_t invFlg ){
    // Bei ESP8266 findet die endgültige Einschaltung erst hier am Ende des Impulszyklus statt
    stopWaveformMoTo( pin );
    attachInterrupt( pin, gpioTab[gpio2ISRx(pin)].gpioISR, invFlg?RISING:FALLING ); //trailing edge 
}

static inline __attribute__((__always_inline__)) void attachInterruptAS(  ledData_t *ledDataP ){
    attachInterrupt( ledDataP->pin, gpioTab[gpio2ISRx(ledDataP->pin)].gpioISR, ledDataP->invFlg?RISING:FALLING );
}
#endif // compiling softLed

#endif