/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Functions for the stepper part of MobaTools
*/
#include <MobaTools.h>


// Global Data for all instances and classes  --------------------------------
extern uint8_t timerInitialized;


// variables for softLeds
static ledData_t* ledRootP = NULL; //start of ledData-chain
static uint8_t ledNextCyc = TIMERPERIODE  / CYCLETIME;     // next Cycle that is relevant for leds
static uint8_t ledCycleCnt = 0;    // count IRQ cycles within PWM cycle

static ledData_t*  ledDataP;              // pointer to active Led in ISR
    
void softledISR(uint8_t cyclesLastIRQ) {
    // ---------------------- softleds -----------------------------------------------
    SET_TP2;
    ledCycleCnt += cyclesLastIRQ;
    if ( ledCycleCnt >= ledNextCyc ) {
        // this IRQ is relevant for softleds
        ledNextCyc = LED_CYCLE_MAX; // there must be atleast one IRQ per PWM Cycle
        if ( ledCycleCnt >= LED_CYCLE_MAX ) {
            // start of a new PWM Cycle - switch all leds with rising/falling state to on
            ledCycleCnt = 0;
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP1;
                // loop over led-objects
                switch ( ledDataP->state ) {
                  case INCBULB:
                  case INCLIN:
                    // switch on led with linear characteristic
                    if (ledDataP->invFlg  ) {
                        #ifdef FAST_PORTWRT
                        *ledDataP->portPin.Adr &= ~ledDataP->portPin.Mask;
                        #else
                        digitalWrite( ledDataP->pin, LOW );
                        #endif
                    } else { 
                        #ifdef FAST_PORTWRT
                        *ledDataP->portPin.Adr |= ledDataP->portPin.Mask;
                        #else
                        digitalWrite( ledDataP->pin, HIGH );
                        #endif
                    }
                    // check if led on is reached
                    //if ( ledDataP->aStep >=  LED_STEP_MAX-1 ) {
                    if ( ledDataP->aCycle >=  LED_CYCLE_MAX ) {    // led is full on, remove from active-chain
                        SET_TP4;
                        ledDataP->state = STATE_ON;
                        *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                        if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                        ledDataP->aCycle = 0;
                        CLR_TP4;
                    } else { // switch to next PWM step
                        //ledNextCyc = min( ledDataP->aCycle, ledNextCyc);
                        if ( ledNextCyc > ledDataP->aCycle ) ledNextCyc = ledDataP->aCycle;
                        ledDataP->actPulse = true;
                    }
                    break;
                  case DECBULB:
                  case DECLIN:
                    // switching off led -> next PWM-step
                    //if ( ledDataP->aStep >=  LED_STEP_MAX-1  ) {
                    /*if ( ledDataP->aCycle ==  0  ) {
                        // led is full off, remove from active-chain
                        SET_TP4;
                        ledDataP->state = STATE_OFF;
                        *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                        if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                        CLR_TP4;
                        //ledDataP->aCycle = 0;
                    } else */{ // switch to next PWM step
                        if (ledDataP->invFlg  ) {
                            #ifdef FAST_PORTWRT
                            *ledDataP->portPin.Adr &= ~ledDataP->portPin.Mask;
                            #else
                            digitalWrite( ledDataP->pin, LOW );
                            #endif
                        } else {
                            #ifdef FAST_PORTWRT
                            *ledDataP->portPin.Adr |= ledDataP->portPin.Mask;
                            #else
                            digitalWrite( ledDataP->pin, HIGH );
                            #endif
                        }
                        if ( ledNextCyc > ledDataP->aCycle ) ledNextCyc = ledDataP->aCycle;
                        ledDataP->actPulse = true;
                    }
                    break;
                  default: ;
                } // end of 'switch'
                //CLR_TP1;
            } // end of led loop
        } else { // is switchofftime within PWM cycle
            SET_TP3;
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP4;
                if ( ledDataP->actPulse ) {
                    // led is within PWM cycle with output high
                    if ( ledDataP->aCycle <= ledCycleCnt ) {
                        uint8_t tmpIx;
                        // End of ON-time is reached
                        SET_TP4;
                        if (ledDataP->invFlg  ) {
                            #ifdef FAST_PORTWRT
                            *ledDataP->portPin.Adr |= ledDataP->portPin.Mask;
                            #else
                            digitalWrite( ledDataP->pin, HIGH );
                            #endif
                        } else {
                            #ifdef FAST_PORTWRT
                            *ledDataP->portPin.Adr &= ~ledDataP->portPin.Mask;
                            #else
                            digitalWrite( ledDataP->pin, LOW );
                            #endif
                        }
                        CLR_TP4;
                        ledDataP->actPulse = false;
                        // determine length of next PWM Cyle
                        //SET_TP1;
                        SET_TP4;
                        ledDataP->aStep += ledDataP->speed;
                        tmpIx = (ledDataP->aStep/DELTASTEPS);
                        if ( tmpIx > LED_IX_MAX ) {
                            // the end is reached
                            CLR_TP4;
                            switch ( ledDataP->state ) {
                              case DECBULB:
                              case DECLIN:
                                // led is off -> remove from chain
                                ledDataP->state = STATE_OFF;
                                *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                                if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                                break;
                              case INCBULB:
                              case INCLIN:
                                // switch permanetly on wirh next cycle
                                ledDataP->aCycle = LED_CYCLE_MAX;
                                break;
                              default:
                                ;
                            }
                            SET_TP4;
                        } else {
                            // we are still in up/down
                            CLR_TP4;
                            switch ( ledDataP->state ) {
                              case INCBULB:
                                ledDataP->aCycle = pgm_read_byte(&(iSteps[tmpIx]));
                                break;
                              case DECBULB:
                                //CLR_TP1;
                                ledDataP->aCycle = LED_CYCLE_MAX-pgm_read_byte(&(iSteps[tmpIx]));
                                //SET_TP1;
                                break;
                              case INCLIN:
                                ledDataP->aCycle = tmpIx;
                                break;
                              case DECLIN:
                                ledDataP->aCycle = LED_CYCLE_MAX - tmpIx;
                                break;
                              default:
                                // no action if state is one of NOTATTACHED, STATE_ON, STATE_OFF
                                break;
                            }
                            SET_TP4;
                        }
                        CLR_TP4;
                    } else { 
                       // End of ON-time not yet reached, compute next necessary step
                       CLR_TP3;
                       ledNextCyc = min( ledDataP->aCycle, ledNextCyc);
                       SET_TP3;
                    }
                }
                //CLR_TP4;
            }
            CLR_TP3;
        }
        //CLR_TP3;
     } // end of softleds 
    //CLR_TP3;
    nextCycle = min( nextCycle, ( ledNextCyc-ledCycleCnt ) );
    //SET_TP3;
    CLR_TP2;
} //=============================== End of softledISR ========================================
/////////////////////////////////////////////////////////////////////////////
//Class SoftLed - for Led with soft on / soft off ---------------------------
// Version with Software PWM
SoftLed::SoftLed() {
    ledData.speed    = 0;           // defines rising/falling timer
    ledData.aStep    = DELTASTEPS ;          // actual PWM step
    ledData.aCycle   = 0;           // actual cycle ( =length of PWM pule )
    ledData.actPulse = false;       // PWM pulse is active
    ledData.state    = NOTATTACHED; // initialize 
    setpoint = OFF ;                // initialize to off
    ledType = LINEAR;
    ledData.nextLedDataP = NULL;    // don't put in ISR chain
    ledData.invFlg = false;
}

void SoftLed::mount( LedStats_t stateVal ) {
    // mount softLed to ISR chain ( if not already in )
    // new active Softleds are always inserted at the beginning of the chain
    // only leds in the ISR chain are processed in ISR
    noInterrupts();
    SET_TP2;
    // check if it's not already active (mounted)
    // Leds must not be mounted twice!
    if ( ledData.state < ACTIVE ) {
        // write backward reference into the existing first entry 
        // only if the chain is not empty
        if ( ledRootP ) ledRootP->backLedDataPP = &ledData.nextLedDataP;
        CLR_TP2;
        ledData.nextLedDataP = ledRootP;
        ledRootP = &ledData;
        ledData.backLedDataPP = &ledRootP;
        SET_TP2;
    }
    ledData.state = stateVal;
    CLR_TP2;
    interrupts();
}   
    

uint8_t SoftLed::attach(uint8_t pinArg, uint8_t invArg ){
    // Led-Ausgang mit Softstart. 
    
    ledData.invFlg  = invArg;
    pinMode( pinArg, OUTPUT );
    //DB_PRINT( "Led attached, ledIx = 0x%x, Count = %d", ledIx, ledCount );
    ledData.state   = STATE_OFF ;   // initialize 
    riseTime( LED_DEFAULT_RISETIME );
    if ( ledData.invFlg ) { 
        digitalWrite( pinArg, HIGH );
    } else {
        digitalWrite( pinArg, LOW );
    }
    
    #ifdef FAST_PORTWRT
    ledData.portPin.Adr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[pinArg])]);
    ledData.portPin.Mask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pinArg]);
    #else
    ledData.pin=pinArg ;      // Pin-Nbr 
    #endif
    
    if ( !timerInitialized ) seizeTimer1();
    // enable compareB- interrupt
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
    #elif defined __AVR_MEGA__
        TIMSKx |= _BV(OCIExB) ; 
    #elif defined __STM32F1__
        timer_cc_enable(MT_TIMER, STEP_CHN);
    #endif
    DB_PRINT("IX_MAX=%d, CYCLE_MAX=%d, STEP_MAX=%d, PWMTIME=%d", LED_IX_MAX, LED_CYCLE_MAX, LED_STEP_MAX, LED_PWMTIME );
    return true;
}

void SoftLed::on(){
    if ( ledData.state ==  NOTATTACHED ) return;  // this is not a valid instance
    LedStats_t stateT;
    // Don't do anything if its already ON 
    if ( setpoint != ON  ) {
        setpoint        = ON ;
        ledData.aStep   = DELTASTEPS;
        ledData.speed   = ledSpeed;
        if ( ledType == LINEAR ) {
            stateT          = INCLIN;
            ledData.aCycle  = 1;
        } else { // is bulb simulation
            stateT          = INCBULB;
            ledData.aCycle  = iSteps[1];
        }
        mount(stateT);
    }
    //DB_PRINT( "Led %d On, state=%d", ledIx, ledData.state);
}

void SoftLed::off(){
    if ( ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    LedStats_t stateT;
    // Dont do anything if its already OFF 
    if ( setpoint != OFF ) {
        //SET_TP3;
        setpoint            = OFF;
        ledData.aStep       = DELTASTEPS;
        ledData.speed   = ledSpeed;
        if ( ledType == LINEAR ) {
            stateT          = DECLIN;
            ledData.aCycle  = LED_IX_MAX;
        } else { // is bulb simulation
            //CLR_TP3;
            stateT = DECBULB;
            ledData.aCycle = LED_CYCLE_MAX  - iSteps[1];
        }
        //CLR_TP3;
        mount(stateT);
    }
    //DB_PRINT( "Led %d Off, state=%d", ledIx, ledData.state);
}

void SoftLed::toggle( void ) {
    if ( ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    if ( setpoint == ON  ) off();
    else on();
}

void SoftLed::write( uint8_t setpntVal, uint8_t ledPar ){
    if ( ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    ledType = ledPar;
    write( setpntVal ) ;
}

void SoftLed::write( uint8_t setpntVal ){
    //DB_PRINT( "LedWrite ix= %d, valid= 0x%x, sp=%d, lT=%d", ledIx, ledValid, setpntVal, ledType );
    if ( ledData.state ==  NOTATTACHED ) return; // this is not a valid instance
    if ( setpntVal == ON ) on(); else off();
    #ifdef debug
    // im Debugmode hier die Led-Daten ausgeben
    //DB_PRINT( "LedData[%d]\n\speed=%d, Type=%d, aStep=%d, stpCnt=%d, state=%d, setpoint= %d", ledValid, ledSpeed, ledType, ledData.aStep, ledData.stpCnt, ledData.state, setpoint);
    //DB_PRINT( "ON=%d, NextCyc=%d, CycleCnt=%d, StepIx=%d, NextStep=%d", 
    //         ON, ledNextCyc, ledCycleCnt, ledStepIx, ledNextStep);
    #endif
}

void SoftLed::riseTime( uint16_t riseTime ) {
    if ( ledData.state ==  NOTATTACHED ) return;
    // length of startphase in ms (min 20ms, max 10240ms )
    // The max value ist slower, if CYCLETIME is reduced.
    // risetime is computed to a 'speed' Value with 16 beeing the slowest 
    // with speed value = 16 means risetime is (LED_CYCLE_MAX * LED_PWMTIME * DELTATIME / 16)
    // risetime = (LED_CYCLE_MAX * LED_PWMTIME * DELTATIME) / speed
    // 
    long riseMax = ((long) LED_CYCLE_MAX * DELTASTEPS * LED_PWMTIME );
    if ( riseTime <= 20 ) riseTime = 20;
    if ( riseTime >= riseMax/16 ) riseTime = riseMax/16;
    int tmp = ( ((long)riseMax  *10) / ( riseTime  ) +5 ) /10;
    ledSpeed = tmp;
    DB_PRINT( "ledSpeed[%d] = %d ( risetime=%d, riseMax=%d, PWMTIME=%d )", ledIx, ledSpeed, riseTime, riseMax, LED_PWMTIME );
}
