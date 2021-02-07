#ifndef MOTOESP8266_H
#define MOTOESP8266_H
// ESP8266 specific declarations for Cpp files
#warning ESP8266 specific cpp includes

void inline startServoPulse(servoData_t *servoDataP, uint32_t pulseWidth ) {
    startWaveformMoTo(servoDataP->pin, pulseWidth/TICS_PER_MICROSECOND/SPEED_RES, TIMERPERIODE-(pulseWidth/TICS_PER_MICROSECOND/SPEED_RES),0);
}

void inline servoWrite( servoData_t *servoDataP, uint32_t pulseWidth ) {
    // the same funktion exists for ESP32 ( with another internal call )
    startWaveformMoTo(servoDataP->pin, pulseWidth/TICS_PER_MICROSECOND/SPEED_RES, TIMERPERIODE-(pulseWidth/TICS_PER_MICROSECOND/SPEED_RES),0);
}

void inline servoPulseOff( servoData_t *servoDataP ) {
    stopWaveformMoTo( servoDataP->pin );
}


#endif