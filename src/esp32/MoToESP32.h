#ifndef MOTOESP32_H
#define MOTOESP32_H
// ESP32 specific defines for Cpp files

//#warning ESP32 specific cpp includes
#if defined COMPILING_MOTOSERVO_CPP || defined COMPILING_MOTOSOFTLED_CPP

//returns the channelnumber ( 0...15 ) of the leds channel to be used, or -1 if no channel is availabele
static inline __attribute__((__always_inline__)) int8_t servoPwmSetup( servoData_t *servoDataP ) {
    int8_t pwmNbr = initPwmChannel( servoDataP->pin, SERVO_TIMER );
    pinMode( servoDataP->pin, OUTPUT );
    attachInterruptArg( servoDataP->pin, ISR_Servo, (void*)servoDataP, FALLING );
    DB_PRINT( "PwmNbr:%d, Pin:%d, Group=%d, Channel=%d, Timer=%d", pwmNbr, pwmUse[pwmNbr].pin, pwmUse[pwmNbr].group, pwmUse[pwmNbr].channel, pwmUse[pwmNbr].timer );
    return pwmNbr;
}

//returns the channelnumber ( 0...15 ) of the leds channel to be used, or -1 if no channel is availabele
static inline __attribute__((__always_inline__)) int8_t softLedPwmSetup( servoData_t *softledDataP )  {
    int8_t pwmNbr = initPwmChannel( softledDataP->pin, LED_TIMER );
    pinMode( softledDataP->pin, OUTPUT );
    attachInterruptArg( softledDataP->pin, ISR_Servo, (void*)softledDataP, FALLING );
    return pwmNbr;
}

static inline __attribute__((__always_inline__)) void startServoPulse( servoData_t *servoDataP, uint32_t pulseWidth ) {
    setPwmPin( servoDataP->pwmNbr );
    setPwmDuty( servoDataP->pwmNbr, pulseWidth );
    attachInterruptArg( servoDataP->pin, ISR_Servo, servoDataP, FALLING );
}

static inline __attribute__((__always_inline__)) void servoWrite( servoData_t *servoDataP, uint32_t pulseWidth ) {
    // the same funktion exists for ESP8266 ( with another internal call )
    setPwmDuty( servoDataP->pwmNbr, pulseWidth );
    
}

static inline __attribute__((__always_inline__)) void servoPulseOff( servoData_t *servoDataP ) {
    //DB_PRINT("Stop Puls, ledcNr=%d", servoDataP->pwmNbr );
    setPwmDuty( servoDataP->pwmNbr, 0 );
}

static inline __attribute__((__always_inline__)) void servoDetach( servoData_t *servoDataP ) {
    detachInterrupt( servoDataP->pin );
    freePwmNbr( servoDataP->pwmNbr );
}
#endif    
#endif