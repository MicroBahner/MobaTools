// MobaTools for ESP8266
// This file contains functions that are special to the ESP Processor and are very different
// from the AVR-Versions
// Stepper ISR
// Enable is controlled by means of pulses in the dir pin ( without any step pulses )
// This is done because we have no direct time-ISR on the ESP. The dir output is always
// LOW if there are no step pulses to be created. What we need is a dedicated ISR for the dir-pin if the
// enable function is active
void ICACHE_RAM_ATTR ISR_StepperEnable( stepperData_t *stepperDataP) {
    //GPOS = (1<<2);
    if ( stepperDataP->delayActiv ) {
        // end of enable delay - start moving or switch off enable
        stepperDataP->delayActiv = false;
        if ( stepperDataP->rampState >= rampStat::CRUISING ) {
            // start moving, start creating pulses;
            digitalWrite( stepperDataP->pins[1], (stepperDataP->patternIxInc < 0) ); //set dir pin accordingly
            //delayMicroseconds( 10 );
            startWaveformISR( stepperDataP->pins[0], CYCLETIME, stepperDataP->aUsSteps-CYCLETIME, 0 ); 
        } else {
            // no more moving, switch off enable pin
            digitalWrite( stepperDataP->enablePin, !stepperDataP->enable );
        }
    }
    //GPOC = (1<<2);
}
void ICACHE_RAM_ATTR ISR_Stepper(stepperData_t *stepperDataP) {
    SET_TP1;
    //GPOS = (1<<0);

    // ---------------Stepper motors ---------------------------------------------
    // Usually the ISR is fired at the start of the step pulse
    //CLR_TP1;    // spike for recognizing start of each stepper
    //CLR_TP2;
    if ( stepperDataP->dirChange ) {
        // was falling edge, we have to change the direction.
        digitalWrite( stepperDataP->pins[1], (stepperDataP->patternIxInc < 0) );
        // next ISR at rising edge again
        stepperDataP->dirChange = false;
        attachInterrupt( stepperDataP->pins[0],gpioTab[gpio2ISRx(stepperDataP->pins[0])].gpioISR, RISING );    
    } else {
        // we did a step, update position counter
        stepperDataP->stepsFromZero += stepperDataP->patternIxInc;
        //if ( digitalRead( stepperDataP->pins[1]) ) stepperDataP->stepsFromZero--;
        //else stepperDataP->stepsFromZero++;
        // ------------------ check if last step -----------------------------------
        if ( --stepperDataP->stepCnt == 0 ) {
            // this was the last step.
            if (stepperDataP->stepCnt2 > 0 ) { // check if we have to start a movement backwards
                // yes, change Direction and go stpCnt2 Steps
                stepperDataP->patternIxInc = -stepperDataP->patternIxInc;
                // change Direction at the end of the pulse
                attachInterrupt( stepperDataP->pins[0],gpioTab[gpio2ISRx(stepperDataP->pins[0])].gpioISR, FALLING ); 
                stepperDataP->dirChange = true;
                stepperDataP->stepCnt = stepperDataP->stepCnt2;
                stepperDataP->stepCnt2 = 0;
                stepperDataP->rampState = rampStat::RAMPACCEL;
            } else {    
                stepperDataP->aUsSteps = 0;    // no more Interrupts for this stepper needed
                stepperDataP->rampState = rampStat::STOPPED;
				// stop creating pulses after the current pulse
				startWaveformISR(stepperDataP->pins[0], CYCLETIME, 10, CYCLETIME );
                if (stepperDataP->enablePin != 255) {
                    // enable is active, wait for disabling 
					// timing is done by a pulse on the dir output ( with IRQ at the end )
                    startWaveformISR( stepperDataP->pins[1], 1000*stepperDataP->usDelay, 10 , (1000*stepperDataP->usDelay)); 
                    stepperDataP->delayActiv = true;
                }  
                //CLR_TP2;
            }
        }
        // --------------- compute nexte steplength ------------------------------------
        //SET_TP2;
        // ramp state machine
        switch ( stepperDataP->rampState ) {
          case  rampStat::RAMPACCEL:
            CLR_TP1;
            // we are accelerating the motor
            if (stepperDataP->stepsInRamp > stepperDataP->stepRampLen ) {
                // we reached the end of the ramp
                stepperDataP->aUsSteps = stepperDataP->tUsSteps;
                stepperDataP->stepsInRamp = stepperDataP->stepRampLen;
                stepperDataP->rampState = rampStat::CRUISING;
            } else {
                stepperDataP->aUsSteps = stepperDataP->ustXramplen / (stepperDataP->stepsInRamp + RAMPOFFSET) ;//+1;
            }
            // do we have to start deceleration ( remaining steps < steps in ramp so far )
            // Ramp must be same length in accelerating and decelerating!
            if ( stepperDataP->stepCnt <= (long)( stepperDataP->stepsInRamp  ) ) {
                //CLR_TP2;
                stepperDataP->rampState = rampStat::RAMPDECEL;
                //SET_TP2;
            } else {
                // still in ramp
                stepperDataP->stepsInRamp ++;
            }
            SET_TP1;
            break;
          case rampStat::RAMPDECEL:
            SET_TP2;
            // we are stopping the motor
            if ( stepperDataP->stepCnt > (long)( stepperDataP->stepsInRamp ) ) {
                //CLR_TP2; // ToDo: check whether this in necessary ( schould be done in method that changes steps to  move)
                //steps to move has changed, accelerate again with next step
                stepperDataP->rampState = rampStat::RAMPACCEL;
                //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                //SET_TP2;
            }
            stepperDataP->aUsSteps = stepperDataP->ustXramplen / ( --stepperDataP->stepsInRamp + RAMPOFFSET ) ;// +1 ;
            CLR_TP2;
            break;
        
          case rampStat::SPEEDDECEL:
            // lower speed to new value 
            stepperDataP->aUsSteps = stepperDataP->ustXramplen / ( --stepperDataP->stepsInRamp + RAMPOFFSET ) ;//+1 ;
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
            stepperDataP->aUsSteps = stepperDataP->tUsSteps;
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
		// set new steplength if there are steps to do
		if ( stepperDataP->aUsSteps > 0 ) {
			startWaveformISR(stepperDataP->pins[0], CYCLETIME, stepperDataP->aUsSteps - CYCLETIME, 0 );
		}
        
    }
     CLR_TP1;
    //GPOC = (1<<0);

}    

uint16_t MoToStepper::setSpeedSteps( uint32_t speed10, int16_t rampLen ) {
    // Set speed and length of ramp to reach speed ( from stop )
    // neagtive ramplen means it was set automatically
    rampStat newRampState;      // State of acceleration/deceleration
    uint32_t tUsSteps;         // nbr of IRQ cycles per step ( new target value of motorspeed  )
    uint32_t newUstXramplen;      // precompiled  tUsSteps*rampLen*RAMPOFFSET
    int32_t newRampLen;         // new ramplen
    int32_t newStepsInRamp;     // new stepcounter in ramp - according to new speed and ramplen
    uint32_t newSpeed10;        // new target speed

    if ( _stepperData.output == NO_OUTPUT ) return 0; // --------------->>>>>>>>>>>>>>>>not attached
    DB_PRINT(">>>>>>>>>>sSS(%u,%d)", speed10, rampLen );
    // compute new speed values, adjust length of ramp if necessary
    //actSpeed10 = oldSpeed10 = _stepSpeed10;
        
    newRampLen = abs(rampLen);    // negative values are invalid ( indicate automatically adjusted length )
    if (rampLen<0) newRampLen--;
    if (newRampLen > MAXRAMPLEN ) newRampLen = MAXRAMPLEN;
    newSpeed10 = min( (uint32_t)1000000 / MIN_STEPTIME * 10, (uint32_t)speed10 );
	if ( newSpeed10 == 0 ) newSpeed10 = 1; // minimum speed
    
    //DB_PRINT( "rampLen-new=%u, ramplenParam=%u", newRampLen, rampLen );
    // compute target steplength and check whether speed and ramp fit together: 
    tUsSteps = ( 1000000L * 10  / newSpeed10 );
    // tcyc * (rapmlen+RAMPOFFSET) must be less then 65000, otherwise ramplen is adjusted accordingly
    newUstXramplen = tUsSteps * ( newRampLen + RAMPOFFSET );

    if (rampLen >= 0) {
        // ramplength was set by user, update reference-values
        _lastRampSpeed = newSpeed10;
        _lastRampLen   = newRampLen;
    }
    DB_PRINT( "rampLen-new=%u, ramplenParam=%d, _lastrampSpeed=%d, _lastRmapLen=%d", newRampLen, rampLen, _lastRampSpeed, _lastRampLen );
    
    // recompute all relevant rampvalues according to actual speed and ramplength
    // This needs to be done only, if a ramp is defined, the stepper is moving
    // and the speed an ramp values changed
    // In all other cases the new speed/ramp values will get active immediately
    DB_PRINT( "actRampLen=%u, cXr-new=%u, xCr-old=%u", newRampLen, newUstXramplen, _stepperData.ustXramplen );
    _noStepIRQ(); SET_TP2;
    if ( (_stepperData.stepRampLen + newRampLen ) != 0
        && _chkRunning() 
        &&  newUstXramplen != _stepperData.ustXramplen ) {
        // local variables to hold data that might change in IRQ:
        // If there was a step during recomputing the rampvalues, we must recompute again
        // recomputing the rampvalues lasts too long to stop the IRQ during the whole time
        long        __stepCnt;
        long        __newStepCnt;
        long        __newStepCnt2;
        
        DB_PRINT("Speed changed! New: tUs=%u, ramp=%u, cXr=%u",tUsSteps,newRampLen,newUstXramplen );
        //Serial.print(_stepperData.rampState); Serial.print(" ( ");Serial.print( _stepperData.stepsInRamp );Serial.print("->");
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
            newStepsInRamp = ( (int64_t)newUstXramplen * (_stepperData.stepsInRamp + RAMPOFFSET ) / _stepperData.ustXramplen ) -RAMPOFFSET;
            
            if ( newStepsInRamp < 0 ) newStepsInRamp = 0; 
            
            if ( newSpeed10 != _stepSpeed10 ) {
                // speed changed!
                if ( newStepsInRamp > newRampLen ) {
                    //  ==========  we are too fast ============================
                        //Serial.print(" --");
                        //DB_PRINT ( "Slower: %u/%u -> %u/%u", _stepSpeed10,_stepperData.stepRampLen,  newSpeed10, newRampLen );
                        newRampState = rampStat::SPEEDDECEL;
                        //DB_PRINT("State->%s,  actStep=%u",rsC[_stepperData.rampState], _stepperData.stepsInRamp );
                    
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
        if ( __newStepCnt != 0 ) {
            _stepperData.stepCnt = __newStepCnt;
            _stepperData.stepCnt2 = __newStepCnt2;
        }
    }
    DB_PRINT( "newStepsInRamp:%d",newStepsInRamp );
    
    _stepperData.tUsSteps = tUsSteps;
    _stepperData.ustXramplen = newUstXramplen;
    _stepperData.stepRampLen = newRampLen;
    _stepIRQ(); CLR_TP2;
    _stepSpeed10 = newSpeed10;
    
    DB_PRINT( "RampValues:, Spd=%u, rmpLen=%u, tUs=%u, aUs=%u, stX=%u", _stepSpeed10, _stepperData.stepRampLen,
                   _stepperData.tUsSteps, _stepperData.aUsSteps,_stepperData.ustXramplen );
    DB_PRINT( "   - State=%s, Rampsteps=%u" , rsC[(int)_stepperData.rampState], _stepperData.stepsInRamp );
    return _stepperData.stepRampLen;
}
 
void MoToStepper::prDynData() {             // print actual Stepperdata
 #ifdef debugPrint
    stepperData_t stepperData;  // vor Ausdruck eine lokale Kopie erstellen
    noInterrupts();
    stepperData.stepCnt = _stepperData.stepCnt;        // nmbr of steps to take
    stepperData.stepCnt2 = _stepperData.stepCnt2;                // nmbr of steps to take after automatic reverse
    stepperData.tUsSteps = _stepperData.tUsSteps;           // µseconds per step ( target value of motorspeed  )
    stepperData.aUsSteps = _stepperData.aUsSteps;  // nµseconds per step ( actual motorspeed  )
    stepperData.ustXramplen = _stepperData.ustXramplen;        // precompiled  tUsSteps*(rampLen+RAMPOFFSET)
    stepperData.usDelay = _stepperData.usDelay;            // delay time: enable -> stepping
    stepperData.dirChange = _stepperData.dirChange;          // Flag: Dir has to be changed ( at falling edge )
    stepperData.stepRampLen = _stepperData.stepRampLen;        // Length of ramp in steps
    stepperData.stepsInRamp = _stepperData.stepsInRamp;        // stepcounter within ramp ( counting from stop ( = 0 ): incrementing in startramp, decrementing in stopramp
                                // max value is stepRampLen
    stepperData.rampState = _stepperData.rampState;        // State of acceleration/deceleration
    stepperData.stepsFromZero = _stepperData.stepsFromZero;  // distance from last reference point 
    interrupts();
    Serial.println("------------ISR-Data-----------");
    Serial.printf("stepCnt=%5d\t stepCnt2=%5d\n\r", stepperData.stepCnt, stepperData.stepCnt2 );
    Serial.printf("tUsSteps=%5d\t aUsSteps=%5d\t XrampL=%5d\n\r", stepperData.tUsSteps,stepperData.aUsSteps,stepperData.ustXramplen);
    Serial.printf("rampLen=%4d\t stepsInRamp=%4d\t, rampState=%s(%d)\n\r",stepperData.stepRampLen,stepperData.stepsInRamp,rsC[(int)stepperData.rampState],stepperData.rampState);
#endif
}

