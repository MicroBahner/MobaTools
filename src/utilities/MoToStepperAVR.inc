// MoToStepper: ISR and SPI functions and Methods that are not used in ESP8266
// The difference in these functions is too big to do it with function internal ifdef's

//Variables not used with ESP8266
extern bool timerInitialized;
static uint8_t spiInitialized = false;

// constants
static const int stepPattern[8] = {0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,0b0001 };
static stepperData_t *stepperRootP = NULL;    // start of stepper data chain ( NULL if no stepper object )
static uint8_t spiData[2]; // step pattern to be output on SPI
                            // low nibble of spiData[0] is SPI_1
                            // high nibble of spiData[1] is SPI_4
                            // spiData[1] is shifted out first


// initialize SPI interface ( processordependent )
#ifdef __AVR_MEGA__
static uint8_t spiByteCount = 0;
static void initSPI() {
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
    
#elif defined __STM32F1__
static int rxData;      // dummy for STM32
static void initSPI() {
    // initialize SPI hardware.
    // MSB first, default Clk Level is 0, shift on leading edge
    #ifdef USE_SPI2// use SPI 2 interface
    spi_init(SPI2);
    spi_config_gpios(SPI2, 1,  // initialize as master
                     PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_device, PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_bit,        
                     PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_device, PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MISO_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MOSI_PIN].gpio_bit);

    uint32 flags = (SPI_FRAME_MSB | SPI_CR1_DFF_16_BIT | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(SPI2, (spi_baud_rate)SPI_BAUD_PCLK_DIV_64, (spi_mode)SPI_MODE_0, flags);
    spi_irq_enable(SPI2, SPI_RXNE_INTERRUPT);
    pinMode( BOARD_SPI2_NSS_PIN, OUTPUT);
    digitalWrite( BOARD_SPI2_NSS_PIN, LOW );

    #else// use SPI 1 interface
    spi_init(SPI1);
    spi_config_gpios(SPI1, 1,  // initialize as master
                     PIN_MAP[BOARD_SPI1_NSS_PIN].gpio_device, PIN_MAP[BOARD_SPI1_NSS_PIN].gpio_bit,        
                     PIN_MAP[BOARD_SPI1_SCK_PIN].gpio_device, PIN_MAP[BOARD_SPI1_SCK_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI1_MISO_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI1_MOSI_PIN].gpio_bit);

    uint32 flags = (SPI_FRAME_MSB | SPI_CR1_DFF_16_BIT | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(SPI1, (spi_baud_rate)SPI_BAUD_PCLK_DIV_64, (spi_mode)SPI_MODE_0, flags);
    spi_irq_enable(SPI1, SPI_RXNE_INTERRUPT);
    pinMode( BOARD_SPI1_NSS_PIN, OUTPUT);
    digitalWrite( BOARD_SPI1_NSS_PIN, LOW );
    #endif
    spiInitialized = true;  
}

#elif defined ESP32
spi_t *spiHs = NULL;
static void initSPI() {
    // initialize SPI hardware.
    // MSB first, default Clk Level is 0, shift on leading edge
    spiHs = spiStartBus(SPI_USED, SPI_CLOCK_DIV4, SPI_MODE0, SPI_MSBFIRST);
    //if ( spiHs == NULL ) Serial.println( "Init SPI failed");
    spiAttachSCK(spiHs, SCK);
    // MISO is not used, only serial output
    spiAttachMOSI(spiHs, MOSI);
    spiAttachSS(spiHs, 0, SS);
    spiSSEnable(spiHs);

    spiInitialized = true;  
}
#endif



#pragma GCC optimize "O3"   // optimize ISR for speed
#ifdef IS_32BIT
void IRAM_ATTR stepperISR(int32_t cyclesLastIRQ) {
#else
void stepperISR(uint8_t cyclesLastIRQ) {
#endif
    SET_TP4;
    stepperData_t *stepperDataP;         // actual stepper data in IRQ
    uint8_t spiChanged, changedPins, bitNr;
    //SET_TP1;SET_TP4; // Oszimessung Dauer der ISR-Routine
    spiChanged = false;
    #ifdef __AVR_MEGA__
    interrupts(); // allow nested interrupts, because this IRQ may take long
    #endif
    stepperDataP = stepperRootP;
    // ---------------Stepper motors ---------------------------------------------
    while ( stepperDataP != NULL ) {
        //CLR_TP1;    // spike for recognizing start of each stepper
        if ( stepperDataP->output == A4988_PINS ) {
            // reset step pulse - pulse is max one cycle lenght
            #ifdef FAST_PORTWRT
            noInterrupts();
            *stepperDataP->portPins[0].Adr &= ~stepperDataP->portPins[0].Mask;
            interrupts();
            #else
            digitalWrite( stepperDataP->pins[0], LOW );
            #endif
        }
        if ( stepperDataP->rampState >= rampStat::CRUISING ) {
            // only active motors
            SET_TP3;
            stepperDataP->cycCnt+=cyclesLastIRQ;
			//Serial.println( stepperDataP->cycCnt ); // Nur für TEST
            if ( stepperDataP->cycCnt >= ( stepperDataP->aCycSteps ) ) {
                // Do one step
                SET_TP2;
                stepperDataP->cycCnt = stepperDataP->cycCnt - stepperDataP->aCycSteps ;
                // update position for absolute positioning
                stepperDataP->stepsFromZero += stepperDataP->patternIxInc;
                
                // sign of patternIxInc defines direction
                int8_t _patIx;
                _patIx = stepperDataP->patternIx + stepperDataP->patternIxInc;
                if ( _patIx > 7 ) _patIx = 0;
                if ( _patIx < 0 ) _patIx += 8;
                stepperDataP->patternIx = _patIx;
                CLR_TP2;SET_TP2;
                // store pattern data
                #ifdef __AVR_MEGA__
                noInterrupts(); // because of read modify write actions in setting outputs
                #endif
                switch ( stepperDataP->output ) {
                  #ifdef __AVR_MEGA__
                  case PIN4_7:
                    PORTD = (PORTD & 0x0f) | ( stepPattern[ _patIx ] <<4 );   
                    break;
                  case PIN8_11:
                    PORTB = (PORTB & 0xf0) | ( stepPattern[ _patIx ] );   
                    break;
                  #endif
                  case SPI_1:
                    //SET_TP2;
                    spiData[0] = (spiData[0] & 0xf0) | ( stepPattern[ _patIx ] );
                    spiChanged = true; 
                    //CLR_TP2;
                    break;
                  case SPI_2:
                    spiData[0] = (spiData[0] & 0x0f) | ( stepPattern[ _patIx ] <<4 );
                    spiChanged = true;
                    //CLR_TP2;
                    break;
                  case SPI_3:
                    spiData[1] = (spiData[1] & 0xf0) | ( stepPattern[ _patIx ] );   
                    spiChanged = true;
                    break;
                  case SPI_4:
                    spiData[1] = (spiData[1] & 0x0f) | ( stepPattern[ _patIx ] <<4 );
                    spiChanged = true;
                    break;
                  case SINGLE_PINS : // Outpins are individually defined
                    changedPins = stepPattern[ _patIx ] ^ stepperDataP->lastPattern;
                    for ( bitNr = 0; bitNr < 4; bitNr++ ) {
                        if ( changedPins & (1<<bitNr ) ) {
                            // bit Changed, write to pin
                            if ( stepPattern[ _patIx ] & (1<<bitNr) ) {
                                #ifdef FAST_PORTWRT
                                *stepperDataP->portPins[bitNr].Adr |= stepperDataP->portPins[bitNr].Mask;
                                #else
                                digitalWrite( stepperDataP->pins[bitNr], HIGH );
                                #endif
                            } else {
                                #ifdef FAST_PORTWRT
                                *stepperDataP->portPins[bitNr].Adr &= ~stepperDataP->portPins[bitNr].Mask;
                                #else    
                                digitalWrite( stepperDataP->pins[bitNr], LOW );
                                #endif    
                            }
                        }
                    }
                    stepperDataP->lastPattern = stepPattern[ _patIx ];
                    break;
                  case A4988_PINS : // output step-pulse and direction
                    // direction first
                    //SET_TP2;
                    if ( stepperDataP->patternIxInc > 0 ) {
                        // turn forward 
                        #ifdef FAST_PORTWRT
                        *stepperDataP->portPins[1].Adr |= stepperDataP->portPins[1].Mask;
                        #else
                        digitalWrite( stepperDataP->pins[1], HIGH );
                        #endif
                    } else {
                        // turn backwards
                        #ifdef FAST_PORTWRT
                        *stepperDataP->portPins[1].Adr &= ~stepperDataP->portPins[1].Mask;
                        #else
                        digitalWrite( stepperDataP->pins[1], LOW );
                        #endif
                    }    
                    // Set step pulse 
                    nextCycle = MIN_STEP_CYCLE/2; // will be resettet in half of min steptime
                    #ifndef IS_32BIT // on 32 bit processors step is set at end of ste computing because auf
                                     // min lowtime between 2 pulses
                    #ifdef FAST_PORTWRT
                    *stepperDataP->portPins[0].Adr |= stepperDataP->portPins[0].Mask;
                    #else
                    digitalWrite( stepperDataP->pins[0], HIGH );
                    #endif
                    #endif
                    break;
                  default:
                    // should never be reached
                    break;
                }
                #ifdef __AVR_MEGA__
                interrupts();
                #endif
                CLR_TP2;
                // ------------------ check if last step -----------------------------------
                if ( --stepperDataP->stepCnt == 0 ) {
                    // this was the last step.
                    if (stepperDataP->stepCnt2 > 0 ) { // check if we have to start a movement backwards
                        // yes, change Direction and go stpCnt2 Steps
                        stepperDataP->patternIxInc = -stepperDataP->patternIxInc;
                        stepperDataP->stepCnt = stepperDataP->stepCnt2;
                        stepperDataP->stepCnt2 = 0;
                        stepperDataP->rampState = rampStat::RAMPACCEL;
                    } else {
                        stepperDataP->stepsInRamp = 0;      // we cannot be in ramp when stopped
                        if (stepperDataP->enablePin != 255) {
                            // enable is active, wait for disabling
                            stepperDataP->aCycSteps = stepperDataP->cycDelay;
                            stepperDataP->rampState = rampStat::STOPPING;
                        } else {    
                        stepperDataP->aCycSteps = ISR_IDLETIME;    // no more Interrupts for this stepper needed
                        stepperDataP->rampState = rampStat::STOPPED;
                        //CLR_TP2;
                        }
                    }
                }
                // --------------- compute nexte steplength ------------------------------------
                SET_TP2;
                // ramp state machine
                switch ( stepperDataP->rampState ) {
                  case  rampStat::RAMPACCEL:
                    // we are accelerating the motor
                    if (stepperDataP->stepsInRamp >= stepperDataP->stepRampLen ) {
                        // we reached the end of the ramp
                        stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                        #ifndef IS_32BIT
                        stepperDataP->aCycRemain = stepperDataP->tCycRemain;
                        #endif
                        stepperDataP->stepsInRamp = stepperDataP->stepRampLen;
                        stepperDataP->rampState = rampStat::CRUISING;
                    } else {
                        stepperDataP->aCycSteps = stepperDataP->cyctXramplen / (stepperDataP->stepsInRamp + RAMPOFFSET) ;//+1;
                        #ifndef IS_32BIT
                        stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
                       if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                            stepperDataP->aCycSteps++;
                            stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                        }
                        #endif
                        // do we have to start deceleration ( remaining steps < steps in ramp so far )
                        // Ramp must be same length in accelerating and decelerating!
                        if ( stepperDataP->stepCnt <= ( stepperDataP->stepsInRamp+1U  ) ) {
                            //CLR_TP2;
                            stepperDataP->rampState = rampStat::RAMPDECEL;
                            //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                            //SET_TP2;
                        } else {
                            // still in ramp
                            stepperDataP->stepsInRamp ++;
                        }    
                    }
                    break;
                  case rampStat::RAMPDECEL:
                  case rampStat::SPEEDDECEL:
                    if ( stepperDataP->stepsInRamp <= stepperDataP->stepRampLen ) {
                        // we are stopping the motor
                        if ( stepperDataP->stepCnt > (long)( stepperDataP->stepsInRamp ) ) {
                            //CLR_TP2; // ToDo: check whether this in necessary ( schould be done in method that changes steps to  move)
                            //steps to move has changed, accelerate again with next step
                            stepperDataP->rampState = rampStat::RAMPACCEL;
                            //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                            //SET_TP2;
                        }
                        stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( --stepperDataP->stepsInRamp + RAMPOFFSET ) ;// +1 ;
                        #ifndef IS_32BIT
                        stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
                        if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                            stepperDataP->aCycSteps++;
                            stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                        }
                        #endif
                    } else {
                        // lower speed to new value 
                        if ( (stepperDataP->stepsInRamp-stepperDataP->stepRampLen) > stepperDataP->deltaSteps ) {
                            // steps in ramp still greater than delta
                            stepperDataP->stepsInRamp -=stepperDataP->deltaSteps;
                            stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( stepperDataP->stepsInRamp + RAMPOFFSET ) ;//+1 ;
                            #ifndef IS_32BIT
                            stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
                            if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                                stepperDataP->aCycSteps++;
                                stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                            }
                            #endif
                        } else {
                            // new targetspeed reached
                            //SET_TP3;
                            stepperDataP->rampState = rampStat::CRUISING;
                            stepperDataP->stepsInRamp =  stepperDataP->stepRampLen;
                            stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                            #ifndef IS_32BIT
                            stepperDataP->aCycRemain = 0;
                            #endif
                            //CLR_TP3;
                        }
                        //ToDo - do we have to stop the motor
                    }
                    break;
                    
                  case rampStat::CRUISING:
                    // Not in ramp, targetspeed reached - or without ramp at all
                    //CLR_TP2;
                    stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                    #ifndef IS_32BIT
                    stepperDataP->aCycRemain += stepperDataP->tCycRemain;
                    if  ( stepperDataP->aCycRemain > CYCLETIME ) {
                        stepperDataP->aCycRemain -= CYCLETIME;
                        stepperDataP->aCycSteps++;
                    }
                    #endif
                    // do we have to start the deceleration
                    if ( stepperDataP->stepCnt <= stepperDataP->stepRampLen+1U ) {
                        // in mode without ramp ( stepRampLen == 0 ) , this can never be true
                        stepperDataP->rampState = rampStat::RAMPDECEL;
                    }
                    
                    break;
                    
                  default:
                    //stepper does not move -> nothing to do
                    //CLR_TP2;
                    break;
                } // End of ramp-statemachine
                #ifdef IS_32BIT // set step output
                if ( stepperDataP->output == A4988_PINS ) digitalWrite( stepperDataP->pins[0], HIGH );
                #endif
                CLR_TP2;
            } // End of do one step
			CLR_TP3;
            nextCycle = min ( nextCycle, nextCycle_t(stepperDataP->aCycSteps-stepperDataP->cycCnt) );
            //SET_TP1;
        } // end of 'if stepper active AND moving'
        else if ( stepperDataP->rampState == rampStat::STARTING ) {
            // we start with enablepin active ( cycCnt is already set to 0 )
            stepperDataP->aCycSteps = stepperDataP->cycDelay;
            if ( stepperDataP->stepRampLen > 0 ) stepperDataP->rampState = rampStat::RAMPACCEL;
            else                                stepperDataP->rampState = rampStat::CRUISING;
            nextCycle = min ( nextCycle, nextCycle_t(stepperDataP->aCycSteps) );
        } else if ( stepperDataP->rampState == rampStat::STOPPING  ) {
            stepperDataP->cycCnt+=cyclesLastIRQ;
            if ( stepperDataP->cycCnt >= stepperDataP->aCycSteps ) {
                stepperDataP->cycCnt = 0;
                digitalWrite( stepperDataP->enablePin, !stepperDataP->enable );
                stepperDataP->rampState = rampStat::STOPPED;
            }
        }

        //CLR_TP1;
        stepperDataP = stepperDataP->nextStepperDataP;
        //SET_TP1; //CLR_TP3;
    } // end of stepper-loop
    
    // shift out spiData, if SPI is active
    //SET_TP2;
    if ( spiInitialized && spiChanged ) {
        digitalWrite( SS, LOW );
        #ifdef __AVR_MEGA__
        spiByteCount = 0;
        SPDR = spiData[1];
        #elif defined __STM32F1__
        #ifdef USE_SPI2
        digitalWrite(BOARD_SPI2_NSS_PIN,LOW);
        spi_tx_reg(SPI2, (spiData[1]<<8) + spiData[0] );
        #else
        digitalWrite(BOARD_SPI1_NSS_PIN,LOW);
        spi_tx_reg(SPI1, (spiData[1]<<8) + spiData[0] );
        #endif
        #endif
    }
    CLR_TP4;
} // ==================== End of stepper ISR ======================================
#pragma GCC optimize "Os"
// ---------- SPI interupt used for output stepper motor data -------------
extern "C" {
#ifdef __AVR_MEGA__
ISR ( SPI_STC_vect ) { 
    // output step-pattern on SPI, set SS when ready
    if ( spiByteCount++ == 0 ) {
        // end of shifting out high Byte, shift out low Byte
        SPDR = spiData[0];
    } else {
        // end of data shifting
        digitalWrite( SS, HIGH );
        spiByteCount = 0;
    }
}
#elif defined __STM32F1__
    #ifdef USE_SPI2
void __irq_spi2(void) {// STM32
    static int rxData;
    rxData = spi_rx_reg(SPI2);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI2_NSS_PIN,HIGH);
}
    #else
void __irq_spi1(void) {// STM32
    rxData = spi_rx_reg(SPI1);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI1_NSS_PIN,HIGH);
}
    #endif
#endif
} // end of extern "C"

uint16_t MoToStepper::setSpeedSteps( uintxx_t speed10, int16_t rampLen ) {
    // Set speed and length of ramp to reach speed ( from stop )
    // neagtive ramplen means it was set automatically
    rampStat newRampState;      // State of acceleration/deceleration
    uintxx_t tCycSteps;         // nbr of IRQ cycles per step ( new target value of motorspeed  )
	#ifndef IS_32BIT
    uint16_t tCycRemain;        // Remainder of division when computing tCycSteps
    long     tMicroSteps;       // Microseconds per step
	#endif
    uintxx_t newCyctXramplen;      // precompiled  tCycSteps*rampLen*RAMPOFFSET
    uintxx_t newRampLen;         // new ramplen
    uintxx_t newStepsInRamp;     // new stepcounter in ramp - according to new speed and ramplen
    intxx_t  newDeltaSteps = 1;  //  only for SPEEDDECEL
    uintxx_t newSpeed10;        // new target speed

    if ( _stepperData.output == NO_OUTPUT ) return 0; // --------------->>>>>>>>>>>>>>>>not attached
    // compute new speed values, adjust length of ramp if necessary
    //actSpeed10 = oldSpeed10 = _stepSpeed10;
        
    newRampLen = abs(rampLen);    // negative values are invalid ( indicate automatically adjusted length )
    if (rampLen<0) newRampLen--;
    if (newRampLen > MAXRAMPLEN ) newRampLen = MAXRAMPLEN;
    newSpeed10 = min( uintxx_t(1000000L / MIN_STEPTIME * 10), speed10 );
	if ( newSpeed10 == 0 ) newSpeed10 = 1; // minimum speed
    
    // compute target steplength and check whether speed and ramp fit together: 
	#ifdef IS_32BIT
    tCycSteps = ( 1000000L * 10  / newSpeed10 );
    newCyctXramplen = tCycSteps * ( newRampLen + RAMPOFFSET );
	#else
    tMicroSteps = ( 1000000L * 10  / newSpeed10 );
    tCycSteps = tMicroSteps / CYCLETIME; 
    tCycRemain = tMicroSteps % CYCLETIME; 
    // tcyc * (rapmlen+RAMPOFFSET) must be less then 65000, otherwise ramplen is adjusted accordingly
    long tmp =  tMicroSteps * ( newRampLen + RAMPOFFSET ) / CYCLETIME ;
    if ( tmp > 65000L ) {
        // adjust ramplen
        newRampLen = 65000L * CYCLETIME / tMicroSteps;
        if( newRampLen > RAMPOFFSET ) newRampLen -= RAMPOFFSET; else newRampLen = 0;
        newCyctXramplen = tMicroSteps * ( newRampLen + RAMPOFFSET ) / CYCLETIME;
    } else {
        newCyctXramplen = tmp;
    }
	#endif
    if (rampLen >= 0) {
        // ramplength was set by user, update reference-values
        _lastRampSpeed = newSpeed10;
        _lastRampLen   = newRampLen;
    }
    
    // recompute all relevant rampvalues according to actual speed and ramplength
    // This needs to be done only, if a ramp is defined, the stepper is moving
    // and the speed an ramp values changed
    // In all other cases the new speed/ramp values will get active immediately
    _noStepIRQ(); SET_TP2;
    if ( (_stepperData.stepRampLen + newRampLen ) != 0
        && _chkRunning() 
        &&  newCyctXramplen != _stepperData.cyctXramplen ) {
        // local variables to hold data that might change in IRQ:
        // If there was a step during recomputing the rampvalues, we must recompute again
        // recomputing the rampvalues lasts too long to stop the IRQ during the whole time
        uint32_t        __stepCnt;
        uint32_t        __newStepCnt;
        uint32_t        __newStepCnt2;
        
         do {
            // read actual ISR values
            newRampState = _stepperData.rampState;
            __stepCnt       =  _stepperData.stepCnt; 
            __newStepCnt    = 0;    // if stepcnt is to be changed
            __newStepCnt2   = 0;
            _stepIRQ(); CLR_TP2;
            //with ramp and ramp or speed changed 
            // compute new 'steps in Ramp' according to new speed and ramp values. This maybe greater
            // than ramplen, if speed changed to slower
			#ifdef IS_32BIT
            newStepsInRamp = ( (int64_t)newCyctXramplen * (_stepperData.stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
            newStepsInRamp = newStepsInRamp<RAMPOFFSET? 0 : newStepsInRamp-RAMPOFFSET;
			#else
            newStepsInRamp = ( (long)newCyctXramplen * (_stepperData.stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
            if ( newStepsInRamp > RAMPOFFSET ) newStepsInRamp -= RAMPOFFSET; else newStepsInRamp = 0; 
			#endif
            
            if ( newSpeed10 != _stepSpeed10 ) {
                // speed changed!
                if ( newStepsInRamp > newRampLen ) {
                    //  ==========  we are too fast ============================
                        //Serial.print(" --");
                        //DB_PRINT ( "Slower: %u/%u -> %u/%u", _stepSpeed10,_stepperData.stepRampLen,  newSpeed10, newRampLen );
                        newRampState = rampStat::SPEEDDECEL;
                        newDeltaSteps = ((10L * newStepsInRamp / _stepperData.stepsInRamp)+5 )/10L;
                        if ( newDeltaSteps < 1 ) newDeltaSteps = 1;
                    
                } else  {
                    //  ==========  we are too slow ============================
                    //Serial.print(" ++"); 
                    //DB_PRINT ( "Faster: %u/%u -> %u/%u", _stepSpeed10,_stepperData.stepRampLen, newSpeed10 , newRampLen );
                    newRampState = rampStat::RAMPACCEL;
                }
            } else {
                //Serial.print(" ==");
            }

            // Check whether we can reach targetposition with new values
            if ( newStepsInRamp > (__stepCnt - _stepperData.stepCnt2) ) {
                // we cannot reach the tagetposition, so we go beyond the targetposition and than back.
                // This works even if we are already beyond the target position
                //Serial.print( " ><");
                __newStepCnt2 = newStepsInRamp - (__stepCnt - _stepperData.stepCnt2);
                __newStepCnt = newStepsInRamp;
                newRampState = rampStat::RAMPDECEL;
            }
            _noStepIRQ(); SET_TP2;
            //Serial.print(" ) ");Serial.print(_stepperData.rampState);
        } while ( __stepCnt != _stepperData.stepCnt ); // if there was a step during computing, do it again
        _stepperData.rampState = newRampState;
        _stepperData.stepsInRamp = newStepsInRamp;
        _stepperData.deltaSteps = newDeltaSteps;
        if ( __newStepCnt != 0 ) {
            _stepperData.stepCnt = __newStepCnt;
            _stepperData.stepCnt2 = __newStepCnt2;
        }
    }
    
    _stepperData.tCycSteps = tCycSteps;
	#ifndef IS_32BIT
    _stepperData.tCycRemain = tCycRemain;
	#endif
    _stepperData.cyctXramplen = newCyctXramplen;
    _stepperData.stepRampLen = newRampLen;
    _stepIRQ(); CLR_TP2;
    _stepSpeed10 = newSpeed10;
    
    prDynData();
    return _stepperData.stepRampLen;
}

extern nextCycle_t nextCycle;
//static nextCycle_t cyclesLastIRQ = 1;  // µsec since last IRQ

void MoToStepper::prDynData() {             // print actual Stepperdata
 #ifdef debugPrint
    stepperData_t printData;  // vor Ausdruck eine lokale Kopie erstellen
	intxx_t prNextCycle;
    noInterrupts();
    printData.stepCnt   = _stepperData.stepCnt;             // nmbr of steps to take
    printData.stepCnt2  = _stepperData.stepCnt2;            // nmbr of steps to take after automatic reverse
	printData.tCycSteps =   _stepperData.tCycSteps;         // nbr of IRQ cycles per step ( target value of motorspeed  )
	#ifndef IS_32BIT
	printData.tCycRemain =  _stepperData.tCycRemain;        // Remainder of division when computing tCycSteps
	printData.aCycRemain =  _stepperData.aCycRemain;        // accumulate tCycRemain when cruising
	#endif
	printData.aCycSteps =   _stepperData.aCycSteps;         // nbr of IRQ cycles per step ( actual motorspeed  )
	printData.cyctXramplen =_stepperData.cyctXramplen;     // precompiled  tCycSteps*(rampLen+RAMPOFFSET)
    printData.cycCnt =       _stepperData.cycCnt;             // counting cycles until cycStep
	printData.cycDelay =    _stepperData.cycDelay;          // delay time enable -> stepping

    printData.stepRampLen = _stepperData.stepRampLen;       // Length of ramp in steps
    printData.stepsInRamp = _stepperData.stepsInRamp;       // stepcounter within ramp ( counting from stop ( = 0 ): incrementing in startramp, decrementing in stopramp
    printData.rampState = _stepperData.rampState;           // State of acceleration/deceleration
    printData.stepsFromZero = _stepperData.stepsFromZero;   // distance from last reference point 
    printData.deltaSteps = _stepperData.deltaSteps;         // number of computed steps per real step in SPEEDDECEL
    prNextCycle = nextCycle;
    interrupts();
    uint32_t time = micros();
    uint16_t usec = time%1000; time /=1000;
    uint16_t msec = time%1000; time /= 1000;
    DB_PRINT("Time:%5lu:%03u,%03u\t stepCnt=%5lu\t stepCnt2=%5lu\t sFZ=%5ld", time, msec,usec,printData.stepCnt, printData.stepCnt2, printData.stepsFromZero );
	#ifdef IS_32BIT
    DB_PRINT("tCySteps=%5u\t # aCySteps=%5u\t ", printData.tCycSteps,printData.aCycSteps);
	#else
    DB_PRINT("tCySteps=%5u\t tCyRemain=%5u\t aCySteps=%5u\t aCyRemain=%5u", printData.tCycSteps,printData.tCycRemain,printData.aCycSteps,printData.aCycRemain);
	#endif
    DB_PRINT(" XrampL=%5u\t rampLen=%4u\t stepsInRamp=%4u\t, rampState=%s(%u)",printData.cyctXramplen,printData.stepRampLen,printData.stepsInRamp,rsC[(int)printData.rampState],(int)printData.rampState);
    DB_PRINT("dltaStp=%4d,\t cycnt=%4ld, nextCyc=%4d", printData.deltaSteps, printData.cycCnt, prNextCycle );

    Serial.println("^^^^^^^^^^^^^^ISR-Data^^^^^^^^^^^^^^^^");
    #endif
}
