/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Functions for the stepper part of MobaTools
*/
#include <MobaTools.h>
#include <utilities/MoToDbg.h>

#ifdef ESP8266 // version for ESP8266
// Global Data for all instances and classes  --------------------------------
void startLedPulse( uint8_t pin, uint8_t invFlg, uint32_t pulseLen ){
    // start or change the pwmpulses on the led pin.
    // with invFlg set pulseLen is lowtime, else hightime
    if ( invFlg ) {
        startWaveformMoTo(pin, PWMCYC-pulseLen, pulseLen,0);
    } else {
        startWaveformMoTo(pin, pulseLen, PWMCYC-pulseLen,0);
    }

}

void ICACHE_RAM_ATTR ISR_Softled( ledData_t *_ledData ) {
    // ---------------------- softleds -----------------------------------------------
    SET_TP2;
    int changePulse = BULB; // change LINEAR or BULB ( -1: don't change )
    uint32_t pwm ;   // new value
    
    switch ( _ledData->state ) {
      case STATE_ON: // last ISR ( at leading edge )
        changePulse = -1; // nothing to change
        stopWaveformMoTo( _ledData->pin );
        attachInterrupt( _ledData->pin, gpioTab[gpio2ISRx(_ledData->pin)].gpioISR, _ledData->invFlg?RISING:FALLING ); //trailing edge 
        break;
      case INCLIN:
        changePulse = LINEAR;
      case INCBULB: // no difference in first version 
        if ( ++_ledData->stepI >= _ledData->stepMax ) {
            // full on is reached
            changePulse = -1; // nothing to change
            _ledData->stepI = _ledData->stepMax;
            if ( _ledData->tPwmOn == PWMCYC ) {
                // switch on static after pulse end (leading edge)
                 attachInterrupt( _ledData->pin, gpioTab[gpio2ISRx(_ledData->pin)].gpioISR, _ledData->invFlg?FALLING:RISING ); // leading edge
                _ledData->state = STATE_ON;
            } else {
                // pwm constant with tPwmOn
                startLedPulse(_ledData->pin, _ledData->invFlg, _ledData->tPwmOn );
                _ledData->state = ACTIVE;
                detachInterrupt( _ledData->pin );
            }
        }
        break;
      case DECLIN:
        changePulse = LINEAR;
      case DECBULB:
        if ( _ledData->stepI > 0 ) --_ledData->stepI;
        if ( _ledData->stepI == 0 ) {
            // full off is reached
             changePulse = -1; // nothing to change
           if ( _ledData->tPwmOff == 0 ) {
                // switch on static
                stopWaveformMoTo( _ledData->pin );
                digitalWrite( _ledData->pin , _ledData->invFlg );
                _ledData->state = STATE_OFF;
            } else {
                // pwm constant with tPwmOff
                startLedPulse(_ledData->pin, _ledData->invFlg, _ledData->tPwmOff );
                _ledData->state = ACTIVE;
                detachInterrupt( _ledData->pin );
            }
        }
        break;
      default:
        // nothing to do in all other cases
        ;
    }
    
    if ( changePulse >= 0 ) {
        // inc- / decreasing, set pwm according to actual step
        uint16_t pOff = max( MIN_PULSE, _ledData->tPwmOff );
        uint16_t pOn = min ( MAX_PULSE, _ledData->tPwmOn );
        if ( changePulse == LINEAR ) {
            pwm = pOff + ( (pOn - pOff) * _ledData->stepI / _ledData->stepMax );
        } else {
            pwm = _ledData->hypPo + _ledData->hypB/(stepOfs+stepRef - (stepRef * _ledData->stepI / _ledData->stepMax) );
        }
        startLedPulse(_ledData->pin,_ledData->invFlg, pwm );
    }  

    CLR_TP2;
} //=============================== End of softledISR ========================================
/////////////////////////////////////////////////////////////////////////////
//Class SoftLed - for Led with soft on / soft off ---------------------------
// Version with Software PWM

void SoftLed::_computeBulbValues() {
     // recompute parameter for bulb simulation ( hyperbolic approximation of pwm ramp )
    int hypB;
    int hypPo;
    //hypB = ( _ledData.tPwmOn - _ledData.tPwmOff )*_ledData.stepOfs*(_ledData.hypF*_ledData.stepRef+_ledData.stepOfs)/(_ledData.hypF*_ledData.stepRef);
    //hypPo = _ledData.tPwmOn - hypB/_ledData.stepOfs;
    hypB = (stepOfs*(stepRef+stepOfs)/(stepRef)) * ( _ledData.tPwmOn - _ledData.tPwmOff );
    //hypPo = _ledData.tPwmOn - hypB/stepOfs;
    hypPo = _ledData.tPwmOff - hypB/(stepOfs+stepRef);
    noInterrupts();
    _ledData.hypPo = hypPo;
    _ledData.hypB = hypB;
    interrupts();
    #ifdef debugPrint
    // print pwmval for step= 0, step=max/2, step=max
    int pwmOff = hypPo + hypB/(stepOfs+(stepRef - 0 ));
    int pwm2 = hypPo + hypB/(stepOfs+(stepRef/2 ));
    int pwmOn = hypPo + hypB/(stepOfs);
    DB_PRINT("Hyperbolic ramp ( pwmOff=%d, pwmOn=%d, stepRef=%d  .. B=%d, Po=%d", _ledData.tPwmOff,_ledData.tPwmOn,stepRef, hypB, hypPo);
    DB_PRINT("Pwm-0=%d,  Pwm1/2=%d, PwmMax=%d", pwmOff, pwm2, pwmOn);
    for( byte ix=0; ix<=20; ix++ ) {
        SET_TP2;CLR_TP2;SET_TP2;
        int step = stepRef * ix / 20;
        int pwm = hypPo + hypB/(stepOfs+stepRef - step );
        CLR_TP2;
        DB_PRINT(" Step %5d - pwm = %6d", step, pwm );
    }
    #endif
}

SoftLed::SoftLed() {
    _ledData.aPwm     = 0 ;          // initialize to OFF
    _ledData.tPwmOn = PWMCYC;      // target PWM value (µs )
    _ledData.tPwmOff  = 0;           // target PWM value (µs )
    _ledData.stepI    = 0;           // start of rising
    _ledData.stepMax  = LED_DEFAULT_RISETIME*1000/PWMCYC; // total steps for rising/falling ramp
    _ledData.state    = NOTATTACHED; // initialize 
    _setpoint = OFF ;                // initialize to off
    _ledType = LINEAR;
    _ledData.invFlg = false;
}

   

uint8_t SoftLed::attach(uint8_t pinArg, uint8_t invArg ){
    // Led-Ausgang mit Softstart. 
    
    // wrong pinnbr or pin in use?
    if ( pinArg <0 || pinArg >15 || gpioUsed(pinArg ) ) return 0;
    
    setGpio(pinArg);    // mark pin as used
    _ledData.invFlg  = invArg;
    pinMode( pinArg, OUTPUT );
    //DB_PRINT( "Led attached, ledIx = 0x%x, Count = %d", ledIx, ledCount );
    _ledData.state   = STATE_OFF ;   // initialize 
    riseTime( LED_DEFAULT_RISETIME );
    _ledData.stepMax  = LED_DEFAULT_RISETIME*1000/PWMCYC; // total steps for rising/falling ramp
    if ( _ledData.invFlg ) { 
        digitalWrite( pinArg, HIGH );
    } else {
        digitalWrite( pinArg, LOW );
    }
    _ledData.pin=pinArg ;      // Pin-Nbr 
    _computeBulbValues();
    // assign an ISR to the pin
    gpioTab[gpio2ISRx(_ledData.pin)].MoToISR = (void (*)(void*))ISR_Softled;
    gpioTab[gpio2ISRx(_ledData.pin)].IsrData = &_ledData;
    attachInterrupt( _ledData.pin, gpioTab[gpio2ISRx(_ledData.pin)].gpioISR, _ledData.invFlg?RISING:FALLING );
    
     //DB_PRINT("IX_MAX=%d, CYCLE_MAX=%d, PWMTIME=%d", LED_IX_MAX, LED_CYCLE_MAX, LED_PWMTIME );
    return true;
}

void SoftLed::on(uint8_t brightness ){
    // set brightness for on ( in percent ) and switch on
    // this brightness will stay for all succeding 'on'
    if ( _ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    uint16_t tmp;
    if ( brightness > 100 ) brightness = 100;
    tmp = PWMCYC * brightness / 100 ;
    if ( tmp <= _ledData.tPwmOff ) {
        // must be higher than value for 'off'
        _ledData.tPwmOn =_ledData.tPwmOff + MIN_PULSE;
    } else {
        _ledData.tPwmOn = tmp;
    }
    _computeBulbValues();
    on();
    DB_PRINT("On: Br=%d, PwmOn=%d ( %d ), PwmOff=%d", brightness, _ledData.tPwmOn, tmp, _ledData.tPwmOff);
}

void SoftLed::off(uint8_t brightness ){
    // set brightness for off ( in percent ) and switch off
    // this brightness will stay for all succeding 'off'
    if ( _ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    uint16_t tmp;
    if ( brightness > 100 ) brightness = 100;
    tmp = PWMCYC * brightness / 100 ;
    if ( tmp >= _ledData.tPwmOn ) {
        // must be lower than value for 'on'
        _ledData.tPwmOff =_ledData.tPwmOn - MIN_PULSE;
    } else {
        _ledData.tPwmOff = tmp;
    }
    _computeBulbValues();
    off();
}

void SoftLed::on(){
    LedStats_t oldState, stateT;
    if ( _ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    // Don't do anything if its already ON 
    if ( _setpoint != ON  ) {
        _setpoint        = ON ;
        if ( _ledType == LINEAR ) {
            stateT          = INCLIN;
        } else { // is bulb simulation
            stateT          = INCBULB;
        }
        noInterrupts();
        oldState = _ledData.state;
        _ledData.state = stateT;
        interrupts();
        if ( oldState == ACTIVE ) {
            // const pwm, no interrupt active
            attachInterrupt( _ledData.pin, gpioTab[gpio2ISRx(_ledData.pin)].gpioISR, _ledData.invFlg?RISING:FALLING );
        } else if ( oldState < ACTIVE ) {
            // no pulses at all
            _ledData.stepI++; // we will create the first pulse here
            startLedPulse( _ledData.pin, _ledData.invFlg, MIN_PULSE );
        }
    }
    //DB_PRINT( "Led %d On, state=%d", ledIx, _ledData.state);
}

void SoftLed::off(){
    LedStats_t oldState, stateT;
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    // Dont do anything if its already OFF 
    if ( _setpoint != OFF ) {
        //SET_TP3;
        _setpoint            = OFF;
        if ( _ledType == LINEAR ) {
            stateT          = DECLIN;
        } else { // is bulb simulation
            stateT          = DECBULB;
        }
        noInterrupts();
        oldState = _ledData.state;
        _ledData.state = stateT;
        interrupts();
        if ( oldState == ACTIVE ) {
            // const pwm, no interrupt active
            attachInterrupt( _ledData.pin, gpioTab[gpio2ISRx(_ledData.pin)].gpioISR, _ledData.invFlg?RISING:FALLING );
        } else if ( oldState < ACTIVE ) {
            // no pulses at all
            _ledData.stepI--; // we will create the first pulse here
            startLedPulse( _ledData.pin, _ledData.invFlg, MAX_PULSE );
        }
        //CLR_TP3;
    }
    //DB_PRINT( "Led %d Off, state=%d", ledIx, _ledData.state);
}

void SoftLed::toggle( void ) {
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    if ( _setpoint == ON  ) off();
    else on();
}

void SoftLed::write( uint8_t setpntVal, uint8_t ledPar ){
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    _ledType = ledPar;
    write( setpntVal ) ;
}

void SoftLed::write( uint8_t setpntVal ){
    //DB_PRINT( "LedWrite ix= %d, valid= 0x%x, sp=%d, lT=%d", ledIx, ledValid, setpntVal, _ledType );
    if ( _ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    if ( setpntVal == ON ) on(); else off();
    #ifdef debug
    // im Debugmode hier die Led-Daten ausgeben
    //DB_PRINT( "_ledData[%d]\n\speed=%d, Type=%d, aStep=%d, stpCnt=%d, state=%d, _setpoint= %d", ledValid, _ledSpeed, _ledType, _ledData.aStep, _ledData.stpCnt, _ledData.state, _setpoint);
    //DB_PRINT( "ON=%d, NextCyc=%d, CycleCnt=%d, StepIx=%d, NextStep=%d", 
    //         ON, ledNextCyc, ledCycleCnt, ledStepIx, ledNextStep);
    #endif
}

void SoftLed::riseTime( uint16_t riseTime ) {
    if ( _ledData.state ==  NOTATTACHED ) return;
    // length of startphase in ms (min 20ms, max 65000ms )
    // 
    uint16_t stepMax;
    if ( riseTime <= 20 ) riseTime = 20;
    _ledSpeed = riseTime;
    stepMax = riseTime * 1000L / PWMCYC; // Nbr of pwm steps from ON to OFF and vice versa
    // adjust stepnumbers for ISR
    _computeBulbValues();
    noInterrupts();
    _ledData.stepI = (long)_ledData.stepI * stepMax / _ledData.stepMax;
    _ledData.stepMax = stepMax;
    interrupts();
    DB_PRINT( "_ledSpeed = %d ( risetime=%d ), StepMax=%d", _ledSpeed, riseTime, stepMax );
}

#endif // ESP
