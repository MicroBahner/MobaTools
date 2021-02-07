#ifndef MOTOESP32_H
#define MOTOESP32_H
// ESP32 specific defines for Cpp files

//#warning ESP32 specific cpp includes

//returns the channelnumber ( 0...15 ) of the leds channel to be used, or -1 if no channel is availabele
int8_t inline servoPwmSetup( servoData_t *servoDataP ) {
    int8_t pwmNbr = initPwmChannel( servoDataP->pin, SERVO_TIMER );
    pinMode( servoDataP->pin, OUTPUT );
    attachInterruptArg( servoDataP->pin, ISR_Servo, (void*)servoDataP, FALLING );
    return pwmNbr;
}

//returns the channelnumber ( 0...15 ) of the leds channel to be used, or -1 if no channel is availabele
int8_t inline softLedPwmSetup( servoData_t *softledDataP ) {
    int8_t pwmNbr = initPwmChannel( softledDataP->pin, LED_TIMER );
    pinMode( softledDataP->pin, OUTPUT );
    attachInterruptArg( softledDataP->pin, ISR_Servo, (void*)softledDataP, FALLING );
    return pwmNbr;
}

void inline startServoPulse( servoData_t *servoDataP, uint32_t pulseWidth ) {
    ledcAttachPin( servoDataP->pin, servoDataP->pwmNbr );
    setPwmDuty( servoDataP->pwmNbr, pulseWidth );
    attachInterruptArg( servoDataP->pin, ISR_Servo, servoDataP, FALLING );
}

void inline servoWrite( servoData_t *servoDataP, uint32_t pulseWidth ) {
    // the same funktion exists for ESP8266 ( with another internal call )
    setPwmDuty( servoDataP->pwmNbr, pulseWidth );
}

void inline servoPulseOff( servoData_t *servoDataP ) {
    ledcWrite(servoDataP->pwmNbr,0);
    //setPwmDuty( servoDataP->pwmNbr, 0 );
}
    
#endif