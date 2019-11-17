// MobaTools for ESP8266
// Stepper ISR
void stepperISR(stepperData_t *stepperDataP) {
    SET_TP4;
    // ---------------Stepper motors ---------------------------------------------
    // Usually the ISR is fired at the start of the step pulse
    //CLR_TP1;    // spike for recognizing start of each stepper
    CLR_TP2;
    if ( digitalRead( stepperDataP->pins[0] == LOW ) {
        // was falling edge, we have to change the direction.
        digitalWrite( stepperDataP->pins[1], (stepperDataP->patternIxInc > 0) );
        // next ISR at rising edge again
        attachInterrupt( stepperDataP.pin,gpioTab[gpio2ISRx(_servoData.pin)].gpiooISR, , RISING );    
    } else {
        // ------------------ check if last step -----------------------------------
        if ( --stepperDataP->stepCnt == 0 ) {
            // this was the last step.
            if (stepperDataP->stepCnt2 > 0 ) { // check if we have to start a movement backwards
                // yes, change Direction and go stpCnt2 Steps
                stepperDataP->patternIxInc = -stepperDataP->patternIxInc;
                // change Direction at the end of the pulse
                attachInterrupt( stepperDataP.pin,gpioTab[gpio2ISRx(_servoData.pin)].gpiooISR, , FALLING ); 
                stepperDataP->stepCnt = stepperDataP->stepCnt2;
                stepperDataP->stepCnt2 = 0;
                stepperDataP->rampState = rampStat::RAMPACCEL;
            } else {    
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
            if (stepperDataP->stepsInRamp > stepperDataP->stepRampLen ) {
                // we reached the end of the ramp
                stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                stepperDataP->aCycRemain = stepperDataP->tCycRemain;
                stepperDataP->stepsInRamp = stepperDataP->stepRampLen;
                stepperDataP->rampState = rampStat::CRUISING;
            } else {
                stepperDataP->aCycSteps = stepperDataP->cyctXramplen / (stepperDataP->stepsInRamp + RAMPOFFSET) ;//+1;
                stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
               if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                    stepperDataP->aCycSteps++;
                    stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                }
            }
            // do we have to start deceleration ( remaining steps < steps in ramp so far )
            // Ramp must be same length in accelerating and decelerating!
            if ( stepperDataP->stepCnt <= (long)( stepperDataP->stepsInRamp  ) ) {
                //CLR_TP2;
                stepperDataP->rampState = rampStat::RAMPDECEL;
                //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                //SET_TP2;
            } else {
                // still in ramp
                stepperDataP->stepsInRamp ++;
            }    
            break;
          case rampStat::RAMPDECEL:
            // we are stopping the motor
            if ( stepperDataP->stepCnt > (long)( stepperDataP->stepsInRamp ) ) {
                //CLR_TP2; // ToDo: check whether this in necessary ( schould be done in method that changes steps to  move)
                //steps to move has changed, accelerate again with next step
                stepperDataP->rampState = rampStat::RAMPACCEL;
                //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                //SET_TP2;
            }
            stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( --stepperDataP->stepsInRamp + RAMPOFFSET ) ;// +1 ;
            stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
            if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                stepperDataP->aCycSteps++;
                stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
            }
            break;
        
          case rampStat::SPEEDDECEL:
            // lower speed to new value 
            stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( --stepperDataP->stepsInRamp + RAMPOFFSET ) ;//+1 ;
            stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
            if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                stepperDataP->aCycSteps++;
                stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
            }
            //if ( stepperDataP->aCycSteps >= stepperDataP->tCycSteps2 || stepperDataP->stepsInRamp == 0 ) {
            if (  stepperDataP->stepsInRamp <=  stepperDataP->stepRampLen ) {
                // new targestspeed reached
                //SET_TP3;
                stepperDataP->rampState = rampStat::CRUISING;
                stepperDataP->stepsInRamp =  stepperDataP->stepRampLen;
                //CLR_TP3;
            }
            //ToDo - do we have to stop the motor
            break;
            
          case rampStat::CRUISING:
            // Not in ramp, targetspeed reached - or without ramp at all
            //CLR_TP2;
            stepperDataP->aCycSteps = stepperDataP->tCycSteps;
            stepperDataP->aCycRemain += stepperDataP->tCycRemain;
            if  ( stepperDataP->aCycRemain > CYCLETIME ) {
                stepperDataP->aCycRemain -= CYCLETIME;
                stepperDataP->aCycSteps++;
            }
            // do we have to start the deceleration
            if ( stepperDataP->stepCnt <= stepperDataP->stepRampLen ) {
                // in mode without ramp ( stepRampLen == 0 ) , this can never be true
                stepperDataP->rampState = rampStat::RAMPDECEL;
            }
            
            break;
            
          default:
            //stepper does not move -> nothing to do
            //CLR_TP2;
            break;
        } // End of ramp-statemachine
    }
     CLR_TP2;
}    
 