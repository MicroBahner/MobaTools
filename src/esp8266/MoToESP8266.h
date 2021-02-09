#ifndef MOTOESP8266_H
#define MOTOESP8266_H
// ESP8266 specific declarations for Cpp files
#warning ESP8266 specific cpp includes

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


#endif