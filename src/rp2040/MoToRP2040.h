#ifndef MOTORP2040_H
#define MOTORP2040_H
// RP2040 specific defines for Cpp files

//#warning RP2040 specific cpp includes
bool seizeTimerAS();

void ISR_Servo( void *arg );

// defined in MoToRP2040.cpp:
extern uint8_t noStepISR_Cnt;   // Counter for nested StepISR-disable
extern uint8_t stepperIRQNum;  // dynamically assigned when timer is initialized
extern timer_hw_t *stepperTimer; // Only for test-prints in .ino
extern spi_inst_t *stepperSPI;
void initSpiAS();

inline __attribute__((__always_inline__)) void _noStepIRQ() {
  //disable stepper IRQ and count how often it has been disabled
  irq_set_enabled (stepperIRQNum, false);
  noStepISR_Cnt++;
#if defined COMPILING_MOTOSTEPPER_CPP
  //Serial.println(noStepISR_Cnt);
  SET_TP3;
#endif
}
inline __attribute__((__always_inline__)) void  _stepIRQ(bool force = true) {
  //enable stepper IRQ id disable counter is 0
  if ( force ) noStepISR_Cnt = 1;              //enable IRQ immediately
  if ( noStepISR_Cnt > 0 ) noStepISR_Cnt -= 1; // don't decrease if already 0 ( if enabling IRQ is called too often )
  if ( noStepISR_Cnt == 0 ) {
#if defined COMPILING_MOTOSTEPPER_CPP
    CLR_TP3;
#endif
    irq_set_enabled (stepperIRQNum, true);
  }
  //Serial.println(noStepISR_Cnt);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP

//returns the channelnumber ( 0...15 ) of the leds channel to be used, or -1 if no channel is availabele
static inline __attribute__((__always_inline__)) int8_t servoPwmSetup( servoData_t *servoDataP ) {
  //DB_PRINT("Search fre ledc channel");
  int8_t pwmNbr = initPwmChannel( servoDataP->pin, SERVO_TIMER );
  pinMode( servoDataP->pin, OUTPUT );
  attachInterruptArg( servoDataP->pin, ISR_Servo, (void*)servoDataP, FALLING );
  DB_PRINT( "PwmNbr:%d, Pin:%d, Group=%d, Channel=%d, Timer=%d", pwmNbr, pwmUse[pwmNbr].pin, pwmUse[pwmNbr].group, pwmUse[pwmNbr].channel, pwmUse[pwmNbr].timer );
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
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef COMPILING_MOTOSOFTLEDESP_CPP
//returns the channelnumber ( 0...15 ) of the leds channel to be used, or -1 if no channel is availabele
static inline __attribute__((__always_inline__)) int8_t softLedPwmSetupAS( servoData_t *softledDataP )  {
  int8_t pwmNbr = initPwmChannel( softledDataP->pin, LED_TIMER );
  pinMode( softledDataP->pin, OUTPUT );
  attachInterruptArg( softledDataP->pin, ISR_Servo, (void*)softledDataP, FALLING );
  return pwmNbr;
}

static inline __attribute__((__always_inline__)) uint8_t attachSoftledAS( ledData_t *ledDataP ) {
  int8_t pwmNbr = initPwmChannel( ledDataP->pin, LED_TIMER );
  if ( pwmNbr >= 0 ) {
    // freien LEDC-Slot gefunden, Pin und Interrupt einrichten
    setPwmPin(  pwmNbr );
    attachInterruptArg( ledDataP->pin, ISR_Softled, (void*)ledDataP, FALLING );
  }
  return pwmNbr;

}

static inline __attribute__((__always_inline__)) void startLedPulseAS( uint8_t pwmNbr, uint8_t invFlg, uint32_t pulseLen ) {
  // start or change the pwmpulses on the led pin.
  // with invFlg set pulseLen is lowtime, else hightime
  // compute pulselen from µs to tics
  pulseLen = slPwm2tic(pulseLen);
  if ( invFlg ) {
    setPwmDuty(pwmNbr, DUTY100 - pulseLen);
  } else {
    setPwmDuty(pwmNbr, pulseLen);
  }

}

static inline __attribute__((__always_inline__)) void softLedOffAS(  uint8_t pwmNbr, uint8_t invFlg ) {
  setPwmDuty(pwmNbr, invFlg ? DUTY100 : 0);
  //digitalWrite( pin , invFlg );
}

static inline __attribute__((__always_inline__)) void softLedOnAS(  uint8_t pwmNbr, uint8_t invFlg ) {
  setPwmDuty(pwmNbr, invFlg ? 0 : DUTY100);
  //digitalWrite( pin , invFlg );
}

static inline __attribute__((__always_inline__)) void softLedOn2AS(  uint8_t pwmNbr, uint8_t invFlg ) {
  // keine Aktion beim ESP32 notwendig
}

static inline __attribute__((__always_inline__)) void attachInterruptAS(  ledData_t *ledDataP ) {
  attachInterruptArg( ledDataP->pin, ISR_Softled, (void*)ledDataP, FALLING );
}

#endif  // Ende COMPILING_MOTOSOFTLEDESP_CPP
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#if 1 // Im Test hier immer aktiv defined COMPILING_MOTOSTEPPER_CPP
static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
  // dummy
}


static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
  // ToDo - spiWriteShortNL(spiHs, (spiData[1]<<8) + spiData[0] );
  // clear/discard rcv data ( from previous call )
    while (spi_is_readable(stepperSPI))
        (void)spi_get_hw(stepperSPI)->dr;
    while (spi_get_hw(stepperSPI)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    while (spi_is_readable(stepperSPI))
        (void)spi_get_hw(stepperSPI)->dr;

    // Don't leave overrun flag set
    spi_get_hw(stepperSPI)->icr = SPI_SSPICR_RORIC_BITS;

    uint16_t datasent = (spiData[1]<<8) + spiData[0];
        while (!spi_is_writable(stepperSPI)) tight_loop_contents(); // sent register should be free ...
        spi_get_hw(stepperSPI)->dr = (uint32_t)datasent;
    
  
}
/* from SDK
// Write len bytes directly from src to the SPI, and discard any data received back
int __not_in_flash_func(spi_write16_blocking)(spi_inst_t *spi, const uint16_t *src, size_t len) {
    invalid_params_if(HARDWARE_SPI, 0 > (int)len);
    // Deliberately overflow FIFO, then clean up afterward, to minimise amount
    // of APB polling required per halfword
    for (size_t i = 0; i < len; ++i) {
        while (!spi_is_writable(spi))
            tight_loop_contents();
        spi_get_hw(spi)->dr = (uint32_t)src[i];
    }

    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;
    while (spi_get_hw(spi)->sr & SPI_SSPSR_BSY_BITS)
        tight_loop_contents();
    while (spi_is_readable(spi))
        (void)spi_get_hw(spi)->dr;

    // Don't leave overrun flag set
    spi_get_hw(spi)->icr = SPI_SSPICR_RORIC_BITS;

    return (int)len;
}
*/

#endif

#endif