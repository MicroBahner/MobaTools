
/*
  MobaTools V1.0
   (C) 11-2017 fpm fpm@mnet-online.de
   
  History:
  V1.0  11-2017 Use of Timer 3 if available ( on AtMega32u4 and AtMega2560 )
  V0.91 08-2017
        Enhanced EggTimer Class. Additional method 'getTime'
        Uses only 5 byte Ram per Instance.
        No Problems with rollover of millis and if the methods are called very rarely
        ( less than once every 25 days )
  V0.9  03-2017
        Better resolution for the 'speed' paramter (programm starts in compatibility mode
        preparations for porting to STM32F1 platform
        
  V0.8 02-2017
        Enable Softleds an all digital outputs
  V0.7 01-2017
		Allow nested Interrupts with the servos. This allows more precise other
        interrupts e.g. for NmraDCC Library.
		A4988 stepper driver IC is supported (needs only 2 ports: step and direction)
        
  V1.1 05-2019
        stepper now supports ramps (accelerating, decelerating )
        stepper speed has better resolution with high steprates

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "MobaTools.h"
#include <avr/interrupt.h>
#include <Arduino.h>

// Debug-Ports
//#define debugTP
//#define debugPrint
#ifdef debugTP 
    #if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
        #define MODE_TP1 DDRF |= (1<<2) //pinA2
        #define SET_TP1 PORTF |= (1<<2)
        #define CLR_TP1 PORTF &= ~(1<<2)
        #define MODE_TP2 DDRF |= (1<<3) //pinA3
        #define SET_TP2 PORTF |= (1<<3)
        #define CLR_TP2 PORTF &= ~(1<<3)
        #define MODE_TP3 DDRF |= (1<<4) //pinA4 
        #define SET_TP3 PORTF |= (1<<4) 
        #define CLR_TP3 PORTF &= ~(1<<4) 
        #define MODE_TP4 DDRF |= (1<<5) //pinA5 
        #define SET_TP4 PORTF |= (1<<5) 
        #define CLR_TP4 PORTF &= ~(1<<5) 
    #elif defined(__AVR_ATmega32U4__)
        #define MODE_TP1 DDRF |= (1<<4) //A3
        #define SET_TP1 PORTF |= (1<<4)
        #define CLR_TP1 PORTF &= ~(1<<4)
        #define MODE_TP2 DDRF |= (1<<5) //A2
        #define SET_TP2 PORTF |= (1<<5)
        #define CLR_TP2 PORTF &= ~(1<<5)
        #define MODE_TP3 DDRD |= (1<<3) //D1
        #define SET_TP3 PORTD |= (1<<3)
        #define CLR_TP3 PORTD &= ~(1<<3)
        #define MODE_TP4 DDRD |= (1<<2) //D0
        #define SET_TP4 PORTD |= (1<<2)
        #define CLR_TP4 PORTD &= ~(1<<2)
        /*#define MODE_TP3 
        #define SET_TP3 
        #define CLR_TP3 
        #define MODE_TP4 
        #define SET_TP4 
        #define CLR_TP4 */
    #elif defined(__AVR_ATmega328P__) 
        #define MODE_TP1 DDRC |= (1<<1) //A1
        #define SET_TP1 PORTC |= (1<<1)
        #define CLR_TP1 PORTC &= ~(1<<1)
        #define MODE_TP2 DDRC |= (1<<2) // A2
        #define SET_TP2 PORTC |= (1<<2)
        #define CLR_TP2 PORTC &= ~(1<<2)
        #define MODE_TP3 DDRC |= (1<<3) //A3
        #define SET_TP3 PORTC |= (1<<3) 
        #define CLR_TP3 PORTC &= ~(1<<3) 
        #define MODE_TP4 DDRC |= (1<<4) //A4 
        #define SET_TP4 PORTC |= (1<<4) 
        #define CLR_TP4 PORTC &= ~(1<<4) 
    #elif defined(__AVR_ATmega128__) ||defined(__AVR_ATmega1281__)||defined(__AVR_ATmega2561__)
    #elif defined (__SAM3X8E__)
        // Arduino Due
        #define MODE_TP1 pinMode( A1,OUTPUT )   // A1= PA24
        #define SET_TP1  REG_PIOA_SODR = (1<<24)
        #define CLR_TP1  REG_PIOA_CODR = (1<<24)
        #define MODE_TP2 pinMode( A2,OUTPUT )   // A2= PA23
        #define SET_TP2  REG_PIOA_SODR = (1<<23)
        #define CLR_TP2  REG_PIOA_CODR = (1<<23)
        #define MODE_TP3 pinMode( A3,OUTPUT )   // A3 = PA22
        #define SET_TP3  REG_PIOA_SODR = (1<<22)
        #define CLR_TP3  REG_PIOA_CODR = (1<<22)
        #define MODE_TP4 pinMode( A4,OUTPUT )   // A4 = PA6
        #define SET_TP4  REG_PIOA_SODR = (1<<6)
        #define CLR_TP4  REG_PIOA_CODR = (1<<6)
    #elif defined (__STM32F1__)
        // STM32F103... ( SPI2-Pins! pin 31-28 maple mini )
        #define MODE_TP1 pinMode( PB12,OUTPUT )   // TP1= PB12
        #define SET_TP1  gpio_write_bit( GPIOB,12, HIGH );
        #define CLR_TP1  gpio_write_bit( GPIOB,12, LOW );
        #define MODE_TP2 pinMode( PB13,OUTPUT )   // TP2= PB13
        #define SET_TP2  gpio_write_bit( GPIOB,13, HIGH );
        #define CLR_TP2  gpio_write_bit( GPIOB,13, LOW );
        #define MODE_TP3 pinMode( PB14,OUTPUT )   // TP3 = PB14
        #define SET_TP3  gpio_write_bit( GPIOB,14, HIGH );
        #define CLR_TP3  gpio_write_bit( GPIOB,14, LOW );
        #define MODE_TP4 pinMode( PB15,OUTPUT )   // TP4 = PB15
        #define SET_TP4  gpio_write_bit( GPIOB,15, HIGH );
        #define CLR_TP4  gpio_write_bit( GPIOB,15, LOW );
    #else
        #define MODE_TP1 DDRC |= (1<<3) //A3
        #define SET_TP1 PORTC |= (1<<3)
        #define CLR_TP1 PORTC &= ~(1<<3)
        #define MODE_TP2 DDRC |= (1<<2) // A2
        #define SET_TP2 PORTC |= (1<<2)
        #define CLR_TP2 PORTC &= ~(1<<2)
        #define MODE_TP3 
        #define SET_TP3 
        #define CLR_TP3 
        #define MODE_TP4 
        #define SET_TP4 
        #define CLR_TP4 
    #endif 
#else
    #define MODE_TP1 
    #define SET_TP1 
    #define CLR_TP1 
    #define MODE_TP2 
    #define SET_TP2 
    #define CLR_TP2 
    #define MODE_TP3 
    #define SET_TP3 
    #define CLR_TP3 
    #define MODE_TP4 
    #define SET_TP4 
    #define CLR_TP4 
    
#endif

// switch off TP2, TP3 temporary
    /*#undef  MODE_TP3
    #undef  SET_TP3 
    #undef  CLR_TP3 
    #define MODE_TP3 
    #define SET_TP3 
    #define CLR_TP3 */
    #undef  MODE_TP2 
    #undef  SET_TP2 
    #undef  CLR_TP2 
    #define MODE_TP2 
    #define SET_TP2 
    #define CLR_TP2 

#ifdef debugPrint
    #define DB_PRINT( x, ... ) { sprintf_P( dbgBuf, PSTR( x ), ##__VA_ARGS__ ) ; Serial.println( dbgBuf ); }
    static char dbgBuf[80];
#else
    #define DB_PRINT ;
#endif

// select timer to use
#ifdef __AVR_MEGA__
    // defines only for ATMega
    #ifdef TCNT3
        // Timer 3 is available, use it
        // #warning "Timer 3 used"
        #define TCNTx       TCNT3
        #define GET_COUNT   TCNT3
        #define TIMERx_COMPB_vect TIMER3_COMPB_vect
        #define TIMERx_COMPA_vect TIMER3_COMPA_vect
        #define OCRxB      OCR3B
        #define OCRxA      OCR3A
        #define TCCRxA     TCCR3A
        #define TCCRxB     TCCR3B
        #define WGMx3      WGM33
        #define WGMx2      WGM32
        #define ICRx       ICR3
        #define OCIExA     OCIE3A
        #define OCIExB     OCIE3B
        #define TIMSKx     TIMSK3
    #else
        // Timer 1 benutzen
        #define TCNTx       TCNT1
        #define GET_COUNT   TCNT1
        #define TIMERx_COMPB_vect TIMER1_COMPB_vect
        #define TIMERx_COMPA_vect TIMER1_COMPA_vect
        #define OCRxB      OCR1B
        #define OCRxA      OCR1A
        #define TCCRxA     TCCR1A
        #define TCCRxB     TCCR1B
        #define WGMx3      WGM13
        #define WGMx2      WGM12
        #define ICRx       ICR1
        #define OCIExA     OCIE1A
        #define OCIExB     OCIE1B
        #define TIMSKx     TIMSK1
    #endif    
        
#elif defined __STM32F1__
    //defines only for STM32
    #define MT_TIMER TIMER4     // Timer used by MobaTools
    #define STEP_CHN    2       // OCR channel for Stepper and Leds
    #define SERVO_CHN   1       // OCR channel for Servos
    #define GET_COUNT timer_get_count(MT_TIMER)
    
    //#define USE_SPI2          // Use SPI1 if not defined
#endif

// constants
static const int stepPattern[8] = {0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,0b0001 };

// Global Data for all instances and classes  --------------------------------
static uint8_t timerInitialized = false;
static uint8_t spiInitialized = false;

// Variables for servos
static servoData_t* lastServoDataP = NULL; //start of ServoData-chain
static byte servoCount = 0;
static servoData_t* pulseP = NULL;         // pulse Ptr in IRQ
static servoData_t* activePulseP = NULL;   // Ptr to pulse to stop
static servoData_t* stopPulseP = NULL;     // Ptr to Pulse whose stop time is already in OCR1
static servoData_t* nextPulseP = NULL;
static enum { PON, POFF } IrqType = PON; // Cycle starts with 'pulse on'
static word activePulseOff = 0;     // OCR-value of pulse end 
static word nextPulseLength = 0;
static bool speedV08 = true;    // Compatibility-Flag for speed method



// Variables for stepper motors
static stepperData_t *stepperRootP = NULL;    // start of stepper data chain ( NULL if no stepper object )
static uint8_t spiData[2]; // step pattern to be output on SPI
                            // low nibble of spiData[0] is SPI_1
                            // high nibble of spiData[1] is SPI_4
                            // spiData[1] is shifted out first
static uint8_t spiByteCount = 0;
static byte stepperCount = 0;
static uint8_t cyclesLastIRQ = 1;  // cycles since last IRQ

// variables for softLeds
static ledData_t* ledRootP = NULL; //start of ledData-chain
static byte ledCount = 0;
static uint8_t ledNextCyc = TIMERPERIODE  / CYCLETIME;     // next Cycle that is relevant for leds
static uint8_t ledCycleCnt = 0;    // count IRQ cycles within PWM cycle
//static uint8_t  ledStepIx = 0;      // Stepcounter for Leds ( Index in Array isteps , 0: start of pwm-Cycle )
//static uint8_t  ledNextStep = 0;    // next step needed for softleds
//static ledData_t*  ledDataP;              // pointer to active Led in ISR
//==========================================================================

// global functions / Interrupts

// ToDo: disabel/enable OCRB-Interrupt only
//#define _noStepIRQ noInterrupts
//#define _stepIRQ     interrupts
void  _noStepIRQ() {
        #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
            TIMSK &= ~( _BV(OCIExB) );    // enable compare interrupts
        #elif defined __AVR_MEGA__
            TIMSKx &= ~_BV(OCIExB) ; 
        #elif defined __STM32F1__
            //timer_cc_enable(MT_TIMER, STEP_CHN);
            noInterrupts;
        #endif
}
void  _stepIRQ() {
        #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
            TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
        #elif defined __AVR_MEGA__
            TIMSKx |= _BV(OCIExB) ; 
        #elif defined __STM32F1__
            //timer_cc_enable(MT_TIMER, STEP_CHN);
            interrupts();
        #endif
}

// ---------- OCRxB Compare Interrupt used for stepper motor and Softleds ----------------
#pragma GCC optimize "O3"
#ifdef __AVR_MEGA__
ISR ( TIMERx_COMPB_vect) {
#elif defined __STM32F1__
void ISR_Stepper(void) {
#endif
  // Timer1 Compare B, used for stepper motor, starts every CYCLETIME us
    // 26-09-15 An Interrupt is only created at timeslices, where data is to output
    stepperData_t *stepperDataP;         // actual stepper data in IRQ
    uint8_t i, spiChanged, changedPins, bitNr;
    uint16_t tmp;
    uint8_t nextCycle = TIMERPERIODE  / CYCLETIME ;// min ist one cycle per Timeroverflow
    SET_TP1;SET_TP4; // Oszimessung Dauer der ISR-Routine
    spiChanged = false;
    #ifdef __AVR_MEGA__
    interrupts(); // allow nested interrupts, because this IRQ may take long
    #endif
    stepperDataP = stepperRootP;
    // ---------------Stepper motors ---------------------------------------------
    while ( stepperDataP != NULL ) {
        // für maximal 4 Motore
        CLR_TP1;SET_TP1;    // spike for recognizing start of each stepper
        if ( stepperDataP->output == A4988_PINS ) {
            // reset step pulse - pulse is max one cycle lenght
            #ifdef FAST_PORTWRT
            *stepperDataP->portPins[0].Adr &= ~stepperDataP->portPins[0].Mask;
            #else
            digitalWrite( stepperDataP->pins[0], LOW );
            #endif
        }
        //if ( stepperDataP->activ && stepperDataP->stepCnt > 0 ) {
        if ( stepperDataP->rampState != STOPPED ) {
            // only active motors
            //SET_TP2;
            stepperDataP->cycCnt+=cyclesLastIRQ;
            if ( stepperDataP->cycCnt >= stepperDataP->aCycSteps ) {
                // Do one step
                SET_TP2;
                stepperDataP->cycCnt = 0 ;
                // update position for absolute positioning
                stepperDataP->stepsFromZero += stepperDataP->patternIxInc;
                
                // sign of patternIxInc defines direction
                /*stepperDataP->patternIx += stepperDataP->patternIxInc;
                if ( stepperDataP->patternIx > 7 ) stepperDataP->patternIx = 0;
                if ( stepperDataP->patternIx < 0 ) stepperDataP->patternIx += 8;*/
                //#define _patIx stepperDataP->patternIx
                int8_t _patIx;
                _patIx = stepperDataP->patternIx + stepperDataP->patternIxInc;
                if ( _patIx > 7 ) _patIx = 0;
                if ( _patIx < 0 ) _patIx += 8;SET_TP1;
                stepperDataP->patternIx = _patIx;
                
                // store pattern data
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
                    SET_TP2;
                    spiData[0] = (spiData[0] & 0xf0) | ( stepPattern[ _patIx ] );
                    spiChanged = true; 
                    CLR_TP2;
                    break;
                  case SPI_2:
                    spiData[0] = (spiData[0] & 0x0f) | ( stepPattern[ _patIx ] <<4 );
                    spiChanged = true;
                    CLR_TP2;
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
                    CLR_TP1;
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
                    // Set step pulse ( will be resettet in next IRQ )
                    nextCycle = 1;
                    #ifdef FAST_PORTWRT
                    *stepperDataP->portPins[0].Adr |= stepperDataP->portPins[0].Mask;
                    #else
                    digitalWrite( stepperDataP->pins[0], HIGH );
                    #endif
                    break;
                  default:
                    // should never be reached
                    break;
                }
                CLR_TP2;
                // ------------------ check if last step -----------------------------------
                if ( --stepperDataP->stepCnt == 0 ) {
                    // this was the last step.
                    if (stepperDataP->stepCnt2 > 0 ) { // check if we have to start a movement backwards
                        // yes, change Direction and go stpCnt2 Steps
                        stepperDataP->patternIxInc = -stepperDataP->patternIxInc;
                        stepperDataP->stepCnt = stepperDataP->stepCnt2;
                        stepperDataP->stepCnt2 = 0;
                        stepperDataP->rampState = RAMPACCEL;
                    } else {    
                        stepperDataP->aCycSteps = TIMERPERIODE;    // no more Interrupts for this stepper needed
                        stepperDataP->rampState = STOPPED;
                        CLR_TP2;
                    }
                }
                // --------------- compute nexte steplength ------------------------------------
                //CLR_TP2;
                // ramp state machine
                switch ( stepperDataP->rampState ) {
                  case  RAMPACCEL:
                    // we are accelerating the motor
                    CLR_TP1; //SET_TP2;
                    if (stepperDataP->stepsInRamp > stepperDataP->stepRampLen ) {
                        // we reached the end of the ramp
                        stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                        stepperDataP->aCycRemain = 0;
                        stepperDataP->rampState = CRUISING;
                    } else 
                        stepperDataP->aCycSteps = stepperDataP->cyctXramplen / (stepperDataP->stepsInRamp + RAMPOFFSET) +1;
                    // do we have to start deceleration ( remeining steps < steps in ramp so far )
                    // Ramp must be same length in accelerating and decelerating!
                    if ( stepperDataP->stepCnt <= (long)( stepperDataP->stepsInRamp  ) ) {
                        CLR_TP2;
                        stepperDataP->rampState = STARTDECEL;
                        //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                        //SET_TP2;
                    } else {
                        // still in ramp
                        stepperDataP->stepsInRamp ++;
                    }    
                    SET_TP1;
                    break;
                  case STARTDECEL:
                    stepperDataP->rampState = RAMPDECEL;
                    stepperDataP->stepsInRamp = stepperDataP->stepCnt;
                  case RAMPDECEL:
                    CLR_TP1; //SET_TP2;
                    // we are stopping the motor
                    if ( stepperDataP->stepCnt > (long)( stepperDataP->stepsInRamp ) ) {
                        //CLR_TP2;
                        //steps to move has changed, accelerate again with next step
                        stepperDataP->rampState = RAMPACCEL;
                        //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                        //SET_TP2;
                    }
                    stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( --stepperDataP->stepsInRamp + RAMPOFFSET ) +1 ;
                    SET_TP1;
                    break;

                case CRUISING:
                    // Not in ramp, targetspeed reached
                    CLR_TP1; CLR_TP2;
                    stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                    stepperDataP->aCycRemain += stepperDataP->tCycRemain;
                    if  ( stepperDataP->aCycRemain > CYCLETIME ) {
                        stepperDataP->aCycRemain -= CYCLETIME;
                        stepperDataP->aCycSteps++;
                    }
                    // do we have to start the deceleration
                    if ( stepperDataP->stepCnt <= stepperDataP->stepRampLen ) {
                        // in mode without ramp ( stepRampLen = 0 ) , this can never be true
                        stepperDataP->rampState = STARTDECEL;
                    }
                    
                    SET_TP1;
                    break;
                    
                case STOPPED:
                    //stopped
                    SET_TP1; CLR_TP2;
                    break;
                } // End of ramp-statemachine
                //SET_TP2;
                // end of compute next steplen
            } // End of do one step
            
             nextCycle = min ( nextCycle, stepperDataP->aCycSteps-stepperDataP->cycCnt );
            //SET_TP1;
        } // end of 'if stepper active AND moving'
        CLR_TP1;
        /*if ( stepperDataP->output == A4988_PINS ) {
            // reset step pulse
            #ifdef FAST_PORTWRT
            *stepperDataP->portPins[0].Adr &= ~stepperDataP->portPins[0].Mask;
            #else
            digitalWrite( stepperDataP->pins[0], LOW );
            #endif
        }*/
        stepperDataP = stepperDataP->nextStepperDataP;
        SET_TP1;
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
    //CLR_TP2;
    //============  End of steppermotor ======================================
    ledData_t*  ledDataP;              // pointer to active Led in ISR
    // ---------------------- softleds -----------------------------------------------
    //CLR_TP3;
    ledCycleCnt += cyclesLastIRQ;
    //SET_TP3;
    if ( ledCycleCnt >= ledNextCyc ) {
        // this IRQ is relevant for softleds
        //SET_TP3;
        ledNextCyc = LED_CYCLE_MAX; // there must be atleast one IRQ per PWM Cycle
        if ( ledCycleCnt >= LED_CYCLE_MAX ) {
            // start of a new PWM Cycle - switch all leds with rising/falling state to on
            ledCycleCnt = 0;
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP1;
                // loop over led-objects
                // yes it's ugly, but because of performance reasons this is done a little bit assembler like
                static const void * pwm0tab[]  { &&pwm0end,&&pwm0end,&&pwm0end,         // NOTATTACHED, STATE_OFF, STATE_ON
                            &&incfast0,&&decfast0,&&incslow0,&&decslow0,&&inclin0,&&declin0 };
                goto  *pwm0tab[ledDataP->state] ;
                  incfast0:
                  incslow0:
                  inclin0:
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
                    // check if led off is reached
                    if ( ledDataP->aCycle >=  LED_CYCLE_MAX-1 ) {
                        // led is full on, remove from active-chain
                        //SET_TP2;
                        ledDataP->state = STATE_ON;
                        *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                        if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                        ledDataP->aCycle = 0;
                        //CLR_TP2;
                    } else { // switch to next PWM step
                        //ledNextCyc = min( ledDataP->aCycle, ledNextCyc);
                        if ( ledNextCyc > ledDataP->aCycle ) ledNextCyc = ledDataP->aCycle;
                        ledDataP->actPulse = true;
                    }
                    goto pwm0end;
                  decfast0:
                  decslow0:
                  declin0:
                    // switch off led 
                   if ( ledDataP->aCycle <= 0  ) {
                        // led is full off, remove from active-chain
                        //SET_TP2;
                        ledDataP->state = STATE_OFF;
                        *ledDataP->backLedDataPP = ledDataP->nextLedDataP;
                        if ( ledDataP->nextLedDataP ) ledDataP->nextLedDataP->backLedDataPP = ledDataP->backLedDataPP;
                        //CLR_TP2;
                        ledDataP->aCycle = 0;
                    } else { // switch to next PWM step
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
                pwm0end: // end of 'switch'
                ;//CLR_TP1;
            } // end of led loop
        } else { // is switchofftime within PWM cycle
            for ( ledDataP=ledRootP; ledDataP!=NULL; ledDataP = ledDataP->nextLedDataP ) {
                //SET_TP4;
                if ( ledDataP->actPulse ) {
                    // led is within PWM cycle with output high
                    if ( ledDataP->aCycle <= ledCycleCnt ) {
                        //SET_TP4;
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
                        //CLR_TP4;
                        ledDataP->actPulse = false;
                        // determine length of next PWM Cyle
                        switch ( ledDataP->state ) {
                          case INCFAST:
                            ledDataP->aStep += ledDataP->speed;
                            if ( ledDataP->aStep > LED_STEP_MAX ) ledDataP->aStep = LED_STEP_MAX;
                            ledDataP->aCycle = iSteps[ledDataP->aStep];
                            break;
                          case DECFAST:
                            ledDataP->aStep += ledDataP->speed;
                            if ( ledDataP->aStep > LED_STEP_MAX ) ledDataP->aStep = LED_STEP_MAX;
                            ledDataP->aCycle = LED_CYCLE_MAX-iSteps[ledDataP->aStep];
                            break;
                          case INCSLOW:
                            if ( --ledDataP->stpCnt < ledDataP->speed ) {
                                ledDataP->aStep += 1;
                                ledDataP->stpCnt = 1;
                            }
                            if ( ledDataP->aStep > LED_STEP_MAX ) ledDataP->aStep = LED_STEP_MAX;
                            ledDataP->aCycle = iSteps[ledDataP->aStep];
                            break;
                          case DECSLOW:
                            if ( --ledDataP->stpCnt < ledDataP->speed ) {
                                ledDataP->aStep += 1;
                                ledDataP->stpCnt = 1;
                            }
                            if ( ledDataP->aStep > LED_STEP_MAX ) ledDataP->aStep = LED_STEP_MAX;
                            ledDataP->aCycle = LED_CYCLE_MAX-iSteps[ledDataP->aStep];
                            break;
                          case INCLIN:
                            ledDataP->aCycle += ledDataP->speed;
                            if ( ledDataP->aCycle > LED_CYCLE_MAX-1 ) ledDataP->aCycle = LED_CYCLE_MAX-1;
                            //ledNextCyc = min( ledDataP->aCycle, ledNextCyc);
                            break;
                          case DECLIN:
                            ledDataP->aCycle -= ledDataP->speed;
                            if ( ledDataP->aCycle <= 0 ) ledDataP->aCycle = 0;
                            //ledNextCyc = min( ledDataP->aCycle, ledNextCyc);
                            break;
                          default:
                            // no action if state is one of NOTATTACHED, STATE_ON, STATE_OFF
                            break;
                        }
                        
                    } else { // next necessary step
                       //SET_TP2;
                       ledNextCyc = min( ledDataP->aCycle, ledNextCyc);
                       //CLR_TP2;
                    }
                }
                //CLR_TP4;
            }
        }
        //CLR_TP3;
     } // end of softleds 
    //CLR_TP3;
    nextCycle = min( nextCycle, ( ledNextCyc-ledCycleCnt ) );
    //SET_TP3;
    // ======================= end of softleds =====================================
        
    
    
    cyclesLastIRQ = nextCycle;
    // set compareregister to next interrupt time;
     noInterrupts(); // when manipulating 16bit Timerregisters IRQ must be disabled
     CLR_TP1;
    // compute next IRQ-Time in us, not in tics, so we don't need long
    #ifdef __AVR_MEGA__
    if ( nextCycle == 1 )  {
        // this is timecritical, was IRQ time longer then CYCELTIME?
        // compute length of current IRQ ( which startet at OCRxB )
        // we assume a max. runtime of 1000 Tics ( = 500µs , what nevver should happen )
        tmp = GET_COUNT - OCRxB ;
        if ( tmp > 1000 ) tmp += TIMER_OVL_TICS; // there was a timer overflow
        if ( tmp > (CYCLETICS-10) ) {
            // runtime was too long, next IRQ mus be started immediatly
            SET_TP3;
            tmp = GET_COUNT+10; 
        } else {
            tmp = OCRxB + CYCLETICS;
        }
        OCRxB = ( tmp > TIMER_OVL_TICS ) ? tmp -= TIMER_OVL_TICS : tmp ;
        CLR_TP3;
    } else {
        // time till next IRQ is more then one cycletime
        // compute next IRQ-Time in us, not in tics, so we don't need long
        tmp = ( OCRxB / TICS_PER_MICROSECOND + nextCycle * CYCLETIME );
        if ( tmp > TIMERPERIODE ) tmp = tmp - TIMERPERIODE;
        OCRxB = tmp * TICS_PER_MICROSECOND;
    }
    #elif defined __STM32F1__
    tmp = ( timer_get_compare(MT_TIMER, STEP_CHN) / TICS_PER_MICROSECOND + nextCycle * CYCLETIME );
    if ( tmp > TIMERPERIODE ) tmp = tmp - TIMERPERIODE;
    timer_set_compare( MT_TIMER, STEP_CHN, tmp * TICS_PER_MICROSECOND) ;
    #endif
    SET_TP1;
    interrupts();
    CLR_TP1; CLR_TP4; // Oszimessung Dauer der ISR-Routine
}
// ---------- SPI interupt used for output stepper motor data -------------
extern "C" {
#ifdef __AVR_MEGA__
ISR ( SPI_STC_vect ) { 
    //SET_TP3;
    // output step-pattern on SPI, set SS when ready
    if ( spiByteCount++ == 0 ) {
        // end of shifting out high Byte, shift out low Byte
        SPDR = spiData[0];
    } else {
        // end of data shifting
        digitalWrite( SS, HIGH );
        spiByteCount = 0;
    }
    //CLR_TP3;
}
#elif defined __STM32F1__
    #ifdef USE_SPI2
void __irq_spi2(void) {// STM32
    static int rxData;
    rxData = spi_rx_reg(SPI2);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI2_NSS_PIN,HIGH);
    #else
void __irq_spi1(void) {// STM32
    static int rxData;
    rxData = spi_rx_reg(SPI1);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI1_NSS_PIN,HIGH);
    #endif
}
#endif
}
#ifdef FIXED_POSITION_SERVO_PULSES
// ---------- OCRxA Compare Interrupt used for servo motor ----------------
// Positions of servopulses within 20ms cycle are fixed -  8 servos
#define PULSESTEP ( 40000 / MAX_SERVOS )
#ifdef __AVR_MEGA__
ISR ( TIMERx_COMPA_vect) {
#elif defined __STM32F1__
void ISR_Servo( void) {
    uint16_t OCRxA;
#endif
    // Timer1 Compare A, used for servo motor
    if ( IrqType == POFF ) {
        SET_TP1; // Oszimessung Dauer der ISR-Routine OFF
        IrqType = PON ; // it's always alternating
        // switch off previous started pulse
        #ifdef FAST_PORTWRT
        *pulseP->portAdr &= ~pulseP->bitMask;
        #else
        digitalWrite( pulseP->pin, LOW );
        #endif
        // Set next startpoint of servopulse
        if ( (pulseP = pulseP->prevServoDataP) == NULL ) {
            // Start over
            OCRxA = FIRST_PULSE;
            pulseP = lastServoDataP;
        } else {
            // The pointerchain comes from the end of the servos, but servoIx is incremented starting
            // from the first servo. Pulses must be sorted in ascending order.
            OCRxA = FIRST_PULSE + (servoCount-1-pulseP->servoIx) * PULSESTEP;
        }
        CLR_TP1; // Oszimessung Dauer der ISR-Routine OFF
    } else {
        SET_TP2; // Oszimessung Dauer der ISR-Routine ON
        // look for next pulse to start
        if ( pulseP->soll < 0 ) {
            // no pulse to output, switch to next startpoint
            if ( (pulseP = pulseP->prevServoDataP) == NULL ) {
                // Start over
                OCRxA = FIRST_PULSE;
                pulseP = lastServoDataP;
            } else {
                OCRxA = FIRST_PULSE + (servoCount-1-pulseP->servoIx) * PULSESTEP;
            }
        } else { // found pulse to output
            if ( pulseP->ist == pulseP->soll ) {
                // no change of pulselength
                if ( pulseP->offcnt > 0 ) pulseP->offcnt--;
            } else if ( pulseP->ist < pulseP->soll ) {
                pulseP->offcnt = OFF_COUNT;
                if ( pulseP->ist < 0 ) pulseP->ist = pulseP->soll; // first position after attach
                else pulseP->ist += pulseP->inc;
                if ( pulseP->ist > pulseP->soll ) pulseP->ist = pulseP->soll;
            } else {
                pulseP->offcnt = OFF_COUNT;
                pulseP->ist -= pulseP->inc;
                if ( pulseP->ist < pulseP->soll ) pulseP->ist = pulseP->soll;
            } 
            OCRxA = (pulseP->ist/SPEED_RES) + GET_COUNT - 4; // compensate for computing time
            if ( pulseP->on && (pulseP->offcnt+pulseP->noAutoff) > 0 ) {
                CLR_TP1;
                #ifdef FAST_PORTWRT
                *pulseP->portAdr |= pulseP->bitMask;
                #else
                digitalWrite( pulseP->pin, HIGH );
                #endif
                SET_TP1;
            }
            IrqType = POFF;
        } 
        CLR_TP2; // Oszimessung Dauer der ISR-Routine ON
    } //end of 'pulse ON'
    #ifdef __STM32F1__
    timer_set_compare(MT_TIMER,  SERVO_CHN, OCRxA);
    #endif 
}

#else // create overlapping servo pulses
// Positions of servopulses within 20ms cycle are variable, max 2 pulses at the same time
// 27.9.15 with variable overlap, depending on length of next pulse: 16 Servos
// 2.1.16 Enable interrupts after timecritical path (e.g. starting/stopping servo pulses)
//        so other timecritical tasks can interrupt (nested interrupts)
static bool searchNextPulse() {
    //SET_TP2;
   while ( pulseP != NULL && pulseP->soll < 0 ) {
        //SET_TP4;
        pulseP = pulseP->prevServoDataP;
        //CLR_TP4;
    }
    //CLR_TP2;
    if ( pulseP == NULL ) {
        // there is no more pulse to start, we reached the end
        //SET_TP2; CLR_TP2;
        return false;
    } else { // found pulse to output
        //SET_TP2;
        if ( pulseP->ist == pulseP->soll ) {
            // no change of pulselength
            if ( pulseP->offcnt > 0 ) pulseP->offcnt--;
        } else if ( pulseP->ist < pulseP->soll ) {
            pulseP->offcnt = OFF_COUNT;
            if ( pulseP->ist < 0 ) pulseP->ist = pulseP->soll; // first position after attach
            else pulseP->ist += pulseP->inc;
            if ( pulseP->ist > pulseP->soll ) pulseP->ist = pulseP->soll;
        } else {
            pulseP->offcnt = OFF_COUNT;
            pulseP->ist -= pulseP->inc;
            if ( pulseP->ist < pulseP->soll ) pulseP->ist = pulseP->soll;
        } 
        //CLR_TP2;
        return true;
    } 
} //end of 'searchNextPulse'

// ---------- OCRxA Compare Interrupt used for servo motor ----------------
#ifdef __AVR_MEGA__
ISR ( TIMERx_COMPA_vect) {
#elif defined __STM32F1__
void ISR_Servo( void) {
    uint16_t OCRxA;
#endif
    // Timer1 Compare A, used for servo motor
    if ( IrqType == POFF ) { // Pulse OFF time
        SET_TP1; // Oszimessung Dauer der ISR-Routine OFF
        //SET_TP3; // Oszimessung Dauer der ISR-Routine
        IrqType = PON ; // it's (nearly) always alternating
        // switch off previous started pulse
        #ifdef FAST_PORTWRT
        *stopPulseP->portAdr &= ~stopPulseP->bitMask;
        #else
        digitalWrite( stopPulseP->pin, LOW );
        #endif
        if ( nextPulseLength > 0 ) {
            // there is a next pulse to start, compute starttime 
            // set OCR value to next starttime ( = endtime of running pulse -overlap )
            // next starttime must behind actual timervalue and endtime of next pulse must
            // lay after endtime of runningpuls + safetymargin (it may be necessary to start
            // another pulse between these 2 ends)
            word tmpTCNT1 = GET_COUNT + MARGINTICS/2;
            interrupts();
            //CLR_TP3 ;
            OCRxA = max ( ((long)activePulseOff + (long) MARGINTICS - (long) nextPulseLength), ( tmpTCNT1 ) );
        } else {
            // we are at the end, no need to start another pulse in this cycle
            if ( activePulseOff ) {
                // there is still a running pulse to stop
                //SET_TP1; // Oszimessung Dauer der ISR-Routine
                OCRxA = activePulseOff;
                IrqType = POFF;
                stopPulseP = activePulseP;
                activePulseOff = 0;
                //CLR_TP1; // Oszimessung Dauer der ISR-Routine
            } else { // was last pulse, start over
                pulseP = lastServoDataP;
                nextPulseLength = 0;
                OCRxA = FIRST_PULSE;
            }
        }
        CLR_TP1; // Oszimessung Dauer der ISR-Routine OFF
    } else { // Pulse ON - time
        SET_TP2; // Oszimessung Dauer der ISR-Routine ON
        //if ( pulseP == lastServoDataP ) SET_TP3;
        // look for next pulse to start
        // do we know the next pulse already?
        if ( nextPulseLength > 0 ) {
            // yes we know, start this pulse and then look for next one
            word tmpTCNT1= GET_COUNT-4; // compensate for computing time
            if ( nextPulseP->on && (nextPulseP->offcnt+nextPulseP->noAutoff) > 0 ) {
                // its a 'real' pulse, set output pin
                //CLR_TP1;
                #ifdef FAST_PORTWRT
                *nextPulseP->portAdr |= nextPulseP->bitMask;
                #else
                digitalWrite( nextPulseP->pin, HIGH );
                #endif
            }
            interrupts(); // the following isn't time critical, so allow nested interrupts
            //SET_TP3;
            // the 'nextPulse' we have started now, is from now on the 'activePulse', the running activPulse is now the
            // pulse to stop next.
            stopPulseP = activePulseP; // because there was a 'nextPulse' there is also an 'activPulse' which is the next to stop
            OCRxA = activePulseOff;
            activePulseP = nextPulseP;
            activePulseOff = activePulseP->ist/SPEED_RES + tmpTCNT1; // end of actually started pulse
            nextPulseLength = 0;
            //SET_TP1;
        }
        if ( searchNextPulse() ) {
            // found a pulse
            if ( activePulseOff == 0 ) {
                // it is the first pulse in the sequence, start it
                activePulseP = pulseP; 
                activePulseOff = pulseP->ist/SPEED_RES + GET_COUNT - 4; // compensate for computing time
                if ( pulseP->on && (pulseP->offcnt+pulseP->noAutoff) > 0 ) {
                    // its a 'real' pulse, set output pin
                    #ifdef FAST_PORTWRT
                    *pulseP->portAdr |= pulseP->bitMask;
                    #else
                    digitalWrite( pulseP->pin, HIGH );
                    #endif
                }
                word tmpTCNT1 = GET_COUNT;
                interrupts(); // the following isn't time critical, so allow nested interrupts
                //SET_TP3;
                // look for second pulse
                //SET_TP4;
                pulseP = pulseP->prevServoDataP;
                //CLR_TP4;
                if ( searchNextPulse() ) {
                    // there is a second pulse - this is the 'nextPulse'
                    nextPulseLength = pulseP->ist/SPEED_RES;
                    nextPulseP = pulseP;
                    //SET_TP4;
                    pulseP = pulseP->prevServoDataP;
                    //CLR_TP4;
                    // set Starttime for 2. pulse in sequence
                    OCRxA = max ( ((long)activePulseOff + (long) MARGINTICS - (long) nextPulseLength), ( tmpTCNT1 + MARGINTICS/2 ) );
                } else {
                    // no next pulse, there is only one pulse
                    OCRxA = activePulseOff;
                    activePulseOff = 0;
                    stopPulseP = activePulseP;
                    IrqType = POFF;
                }
            } else {
                // its a pulse in sequence, so this is the 'nextPulse'
                nextPulseLength = pulseP->ist/SPEED_RES;
                nextPulseP = pulseP;
                //SET_TP4;
                pulseP = pulseP->prevServoDataP;
                //CLR_TP4;
                IrqType = POFF;
            }
        } else {
            // found no pulse, so the last one is running or no pulse at all
            
            if ( activePulseOff == 0 ) {
                // there wasn't any pulse, restart
                pulseP = lastServoDataP;
                nextPulseLength = 0;
                OCRxA = FIRST_PULSE;
            } else {
                // is last pulse, don't start a new one
                IrqType = POFF;
            }
        }
        //CLR_TP2; CLR_TP3; // Oszimessung Dauer der ISR-Routine ON
    } //end of 'pulse ON'
    #ifdef __STM32F1__
    timer_set_compare(MT_TIMER,  SERVO_CHN, OCRxA);
    #endif 
    //CLR_TP1; CLR_TP3; // Oszimessung Dauer der ISR-Routine
}

#endif // VARIABLE_POSITION_SERVO_PULSES

// ------------ end of Interruptroutines ------------------------------
#pragma GCC optimize "Os"

static void seizeTimer1() {
# ifdef __AVR_MEGA__
    uint8_t oldSREG = SREG;
    cli();
    
    TCCRxA =0; /* CTC Mode, ICRx is TOP */
    TCCRxB = _BV(WGMx3) | _BV(WGMx2) /* CTC Mode, ICRx is TOP */
  | _BV(CS11) /* div 8 clock prescaler */
  ;
    ICRx = TIMERPERIODE * TICS_PER_MICROSECOND;  // timer periode is 20000us 
    OCRxA = FIRST_PULSE;
    OCRxB = 400;
    // Serial.print( " Timer initialized " ); Serial.println( TIMSKx, HEX );
    SREG = oldSREG;  // undo cli() 
#elif defined __STM32F1__
    timer_init( MT_TIMER );
    timer_pause(MT_TIMER);
    // IRQ-Priorität von timer 4 interrupt auf lowest (15) setzen
    nvic_irq_set_priority ( NVIC_TIMER4, 15); // Timer 4 - stmduino sets all priorities to lowest level
                                              // To be sure we set it here agai. These long lasting IRQ's 
                                              // MUST be lowest priority
    /*nvic_irq_set_priority ( NVIC_EXTI0, 8); // DCC-Input IRQ must be able to interrupt other long low priority IRQ's
    nvic_irq_set_priority(NVIC_SYSTICK, 0); // Systic must be able to interrupt DCC-IRQ to get correct micros() values
    */
    timer_oc_set_mode( MT_TIMER, SERVO_CHN, TIMER_OC_MODE_FROZEN, 0 );  // comparison between output compare register and counter 
                                                                //has no effect on the outputs
    timer_oc_set_mode( MT_TIMER, STEP_CHN, TIMER_OC_MODE_FROZEN, 0 );
    timer_set_prescaler(MT_TIMER, 36-1 );    // = 0.5µs Tics at 72MHz
    timer_set_reload(MT_TIMER, TIMERPERIODE * TICS_PER_MICROSECOND );
    timer_set_compare(MT_TIMER, STEP_CHN, 400 );
    timer_attach_interrupt(MT_TIMER, TIMER_CC2_INTERRUPT, (voidFuncPtr)ISR_Stepper );
    timer_set_compare(MT_TIMER, SERVO_CHN, FIRST_PULSE );
    timer_attach_interrupt(MT_TIMER, TIMER_CC1_INTERRUPT, ISR_Servo );
    timer_resume(MT_TIMER);
#endif
    timerInitialized = true;  
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
}

static void initSPI() {
    // initialize SPI hardware.
    // MSB first, default Clk Level is 0, shift on leading edge
#ifdef __AVR_MEGA__
    byte tmp;
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
    
#elif defined __STM32F1__
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

#endif
    spiInitialized = true;  
}
// ========================= Class Definitions ============================================
// --------- Class Stepper ---------------------------------
// Class-specific Variables
outUsed_t Stepper4::outputsUsed;

// constructor -------------------------
Stepper4::Stepper4(int steps ) {
    // constuctor for stepper Class, initialize data
    Stepper4::initialize ( steps, HALFSTEP, 1 );
}

Stepper4::Stepper4(int steps, uint8_t mode ) {
    // constuctor for stepper Class, initialize data
    Stepper4::initialize ( steps, mode, 1 );
}

Stepper4::Stepper4(int steps, uint8_t mode,uint8_t minStepTime ) {
    // constuctor for stepper Class, initialize data
    Stepper4::initialize ( steps, mode, minStepTime );
}

// private functions ---------------
void Stepper4::initialize ( int steps360, uint8_t mode, uint8_t minStepTime ) {
    // create new instance
    stepperIx = stepperCount ;
    stepsRev = steps360;       // number of steps for full rotation in fullstep mode
    if ( mode != FULLSTEP && mode != A4988 ) mode = HALFSTEP;
    stepMode = mode;
    // initialize data for interrupts
    _stepperData.stepCnt = 0;         // don't move
    _stepperData.patternIx = 0;
    _stepperData.patternIxInc = mode;         // positive direction
    //minCycSteps = minStepTime*1000/CYCLETIME;         // minStepTime in ms, cycletime in us
    _stepperData.aCycSteps = TIMERPERIODE; //MIN_STEPTIME/CYCLETIME; 
    _stepperData.tCycSteps = _stepperData.aCycSteps; 
    _stepperData.tCycRemain = 0;                // work with remainder when cruising
    _stepperData.stepsFromZero = 0;
    _stepperData.rampState = STOPPED;
    _stepperData.stepRampLen             = 0;               // initialize with no acceleration  
    _stepperData.activ = 0;
    _stepperData.output = NO_OUTPUT;          // unknown, not attached yet
    _stepperData.nextStepperDataP = NULL;
    // add at end of chain
    stepperData_t **tmpPP = &stepperRootP;
    while ( *tmpPP != NULL ) tmpPP = &((*tmpPP)->nextStepperDataP);
    *tmpPP = &_stepperData;
    if( stepperCount++ >= MAX_STEPPER )  {
        stepMode = NOSTEP;      // invalid instance ( too mach objects )
    }
    
}
long Stepper4::getSFZ() {
    // get step-distance from zero point
    // irq must be disabled, because stepsFromZero is updated in interrupt
    long tmp;
    noInterrupts();
    tmp = _stepperData.stepsFromZero;
    interrupts();
    return tmp / stepMode;
}

bool Stepper4::_chkRunning() {
    // is the stepper moving?
    /*bool tmp;
    _noStepIRQ();
    tmp = _stepperData.stepCnt != 0;
    _stepIRQ();
    return tmp;*/
    return ( _stepperData.rampState != STOPPED );
}

// public functions -------------------
uint8_t Stepper4::attach( byte stepP, byte dirP ) {
    // step motor driver A4988 is used
    byte pins[2];
    if ( stepMode != A4988 ) return 0;    // false mode
    DB_PRINT( "Attach4988, S=%d, D=%d", stepP, dirP );
    
    pins[0] = stepP;
    pins[1] = dirP;
    return Stepper4::attach( A4988_PINS, pins );
}
uint8_t Stepper4::attach( byte pin1, byte pin2, byte pin3, byte pin4 ) {
    byte pins[4];
    pins[0] = pin1;
    pins[1] = pin2;
    pins[2] = pin3;
    pins[3] = pin4;
    return Stepper4::attach( SINGLE_PINS, pins );
}
uint8_t Stepper4::attach(byte outArg) {
    return Stepper4::attach( outArg, (byte *)NULL );
}
    
uint8_t Stepper4::attach( byte outArg, byte pins[] ) {
    // outArg must be one of PIN8_11 ... SPI_4 or SINGLE_PINS, A4988_PINS
    if ( stepMode == NOSTEP ) { DB_PRINT("Attach: invalid Object ( Ix = %d)", stepperIx ); return 0; }// Invalid object
    uint8_t attachOK = true;
    switch ( outArg ) {
      #ifdef __AVR_MEGA__
      case PIN4_7:
        if ( Stepper4::outputsUsed.pin4_7 ) {
            // output already in use
            attachOK = false;
        } else {
            // Port D initiieren, Pin4-7 as Output
            Stepper4::outputsUsed.pin4_7 = true;
            DDRD |= 0xf0;
            PORTD &= 0x0f;
        }
        break;
      case PIN8_11:
        if ( spiInitialized || Stepper4::outputsUsed.pin8_11 ) {
            // PIN8_11 and SPI cannot be used simultaneously ( this is not true for Arduino mega )
            attachOK = false;
        } else {
            Stepper4::outputsUsed.pin8_11 = true;
            DDRB |= 0x0f;
            PORTB &= 0xf0;
        }
        break;
      #endif
      case SPI_1:
      case SPI_2:
      case SPI_3:
      case SPI_4:
        // check if already in use or if PIN8_11 is in use
        if ( (Stepper4::outputsUsed.outputs & (1<<(outArg-1))) || Stepper4::outputsUsed.pin8_11 ) {
            // incompatible!
            attachOK = false;
        } else {
            if ( !spiInitialized ) initSPI();
            Stepper4::outputsUsed.outputs |= (1<<(outArg-1));
        }
        break;
      case SINGLE_PINS:
        // 4 single output pins - as yet there is no check if they are allowed!
        for ( byte i = 0; i<4; i++ ) {
            #ifdef FAST_PORTWRT
            // compute portadress and bitnumber
            _stepperData.portPins[i].Adr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[ pins[i]])]);
            _stepperData.portPins[i].Mask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pins[i]]);
            #else // store pins directly
            _stepperData.pins[i] = pins[i];
            #endif
            pinMode( pins[i], OUTPUT );
            digitalWrite( pins[i], LOW );
        }
        break;
      case A4988_PINS:
        // 2 single output pins (step and direction) - as yet there is no check if they are allowed!
        for ( byte i = 0; i<2; i++ ) {
            #ifdef FAST_PORTWRT
            // compute portadress and bitnumber
            _stepperData.portPins[i].Adr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[ pins[i]])]);
            _stepperData.portPins[i].Mask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pins[i]]);
            #else // store pins directly
            _stepperData.pins[i] = pins[i];
            #endif
            stepMode = HALFSTEP;                      // There are no real stepmodes in A4988 - mode
            _stepperData.patternIxInc = 1;  // defines direction
            pinMode( pins[i], OUTPUT );
            digitalWrite( pins[i], LOW );
        }
        break;
     default:
        // invalid Arg
        attachOK = false;
    }
    if ( attachOK ) {
        if ( !timerInitialized) seizeTimer1();
        _stepperData.output = outArg;
        _stepperData.activ = 1;
        // enable compareB- interrupt
        #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
            TIMSK |= ( _BV(OCIExB) );    // enable compare interrupts
        #elif defined __AVR_MEGA__
            TIMSKx |= _BV(OCIExB) ; 
        #elif defined __STM32F1__
            timer_cc_enable(MT_TIMER, STEP_CHN);
        #endif
    }
    DB_PRINT( "attach: output=%d, attachOK=%d", _stepperData.output, attachOK );
    //Serial.print( "Attach Stepper, Ix= "); Serial.println( stepperIx );
    return attachOK;
}

void Stepper4::detach() {   // no more moving, detach from output
    if ( _stepperData.output == NO_OUTPUT ) return ; // not attached
    
    _stepperData.output = NO_OUTPUT;
    _stepperData.activ = 0;
    _stepperData.rampState = STOPPED;
}

int Stepper4::setSpeed( int rpm10 ) {
    // Set speed in rpm*10. Step time is computed internally based on CYCLETIME and
    // steps per full rotation (stepsRev)
    if ( _stepperData.output == NO_OUTPUT ) return 0 ; // not attached
    //_stepperData.aCycSteps = (((60L*10L*1000000L / CYCLETIME )*10 / stepsRev/ rpm10) +5)/10;
    _stepSpeed10 = (long)rpm10 * stepsRev / 60 ;
    _stepSpeed10 = min( 1000000L / MIN_STEPTIME * 10, _stepSpeed10 );
    _setRampValues();       // compute all values for actual ramp
    
    return _stepperData.aCycSteps;
}

uint16_t Stepper4::setSpeedSteps( uint16_t speed10 ) {
    // Speed in steps per sec * 10
    return setSpeedSteps( speed10, _stepperData.stepRampLen );
    // Rampenlänge in Steps ( entsprechend aktueller Beschleunigung )
    //_stepperData.stepsToStop = (long)_stepSpeed10 * _stepSpeed10 / 200 / _stepAccel ;
}

uint16_t Stepper4::setRampLen( uint16_t rampSteps ) {
    // set length of ramp ( from stop to actual target speed ) in steps
    return setSpeedSteps( _stepSpeed10, rampSteps );
}

uint16_t Stepper4::setSpeedSteps( uint16_t speed10, int16_t rampLen ) {
    // Set speed and length of ramp to reach speed ( from stop )
    if ( _stepperData.output == NO_OUTPUT ) return 0; // not attached
    _stepperData.stepRampLen = abs(rampLen);    // negative values are invalid
    _stepSpeed10 = min( 1000000L / MIN_STEPTIME * 10, speed10 );
    return _setRampValues();       // compute all values for actual ramp and speed
}

uint16_t Stepper4::_setRampValues() {
    // compute target steplength and check weather speed and ramp fit together: 
    int16_t stepRampLen;          // Length of ramp in steps
    rampStats_t rampState;        // State of acceleration/deceleration
    uint16_t tCycSteps;           // nbr of IRQ cycles per step ( target value of motorspeed  )
    uint16_t tCycRemain;          // Remainder of division when computing tCycSteps
    uint16_t aCycSteps;           // nbr of IRQ cycles per step ( actual motorspeed  )
    uint16_t aCycRemain;          // Remainder of division when computing aCycSteps
    uint16_t cyctXramplen;        // precompiled  tCycSteps*rampLen*RAMPOFFSET

    stepRampLen = _stepperData.stepRampLen;
    tCycSteps = ( 1000000L * 10  / _stepSpeed10 ) / CYCLETIME; 
    tCycRemain = ( 1000000L * 10  / _stepSpeed10 ) % CYCLETIME; 
    // tcyc * (rapmlen+3) must be less then 65000, otherwise ramplen is adjusted accordingly
    long tmp =  (long)tCycSteps * ( stepRampLen + RAMPOFFSET ) ;
    if ( tmp > 65000L ) {
        // adjust ramplen
        stepRampLen = 65000/tCycSteps - RAMPOFFSET;
        if( stepRampLen < 0 ) stepRampLen = 0;
        cyctXramplen = tCycSteps * ( stepRampLen + RAMPOFFSET ) ;
    } else {
        cyctXramplen = tmp;
    }
    // recompute all relevant rampvalues according to actual speed and ramplength
    // ToDo: recompute according to actual stepper state ( if it is not in STOPPED state )
    _noStepIRQ();
    _stepperData.tCycSteps = tCycSteps;
    _stepperData.tCycRemain = tCycRemain;
    _stepperData.cyctXramplen = cyctXramplen;
    _stepperData.stepRampLen = stepRampLen;
    _stepIRQ();
    
    if ( _stepperData.stepRampLen == 0 ) {
        // without ramp
        //_stepperData.aCycSteps = _stepperData.tCycSteps;
    } else{
        //with ramp
        // lock IRQ, because we use variables that are changed in IRQ
        if ( _stepperData.rampState == STOPPED ) {
            // stepper is stopped, nothing to do
            //_stepperData.rampState = RAMPACCEL;
            //_stepperData.stepsInRamp = RAMPOFFSET;
            
        } else {
        noInterrupts();
            // stepper is running
            // ToDo!!
        interrupts();
        }
    }
    DB_PRINT( "RampValues:, Spd=%d, rmpLen=%d, tcyc=%d, trest=%d, acyc=%d", _stepSpeed10, _stepperData.stepRampLen,
                    _stepperData.tCycSteps, _stepperData.tCycRemain, _stepperData.aCycSteps );
    return _stepperData.stepRampLen;
}

void Stepper4::doSteps( long stepValue ) {
    // rotate stepValue steps
    // if the motor is already moving, this is counted from the actual position.
    // This means in ramp mode the motor may go beyond the desired position and than turn backwards 
    // to reach the targetposition ( stepValue steps away from actual position ) .

    long stepCnt;                 // nmbr of steps to take
    int8_t patternIxInc;
    
    if ( _stepperData.output == NO_OUTPUT ) return; // not attached
    //Serial.print( "doSteps: " ); Serial.println( stepValue );
    
    stepsToMove = stepValue;
    stepCnt = abs(stepValue);
    //DB_PRINT( " stepsToMove: %d ",stepsToMove );
    
    if ( _stepperData.stepRampLen > 0 ) {
        // stepping with ramp
        
        if ( _chkRunning() ) {  // is the stepper moving?
            // yes, check if direction is to change
            
            if (  ( _stepperData.patternIxInc > 0 && stepValue > 0 ) || ( _stepperData.patternIxInc < 0 && stepValue < 0 ) ) {
                // no change in Direction
                switch ( _stepperData.rampState ) {
                  case RAMPACCEL:
                    // We are accelerating, so we have to break this?
                    // targetposition within stopramp?
                    _noStepIRQ();
                    if ( stepCnt <= _stepperData.stepsInRamp ) {
                        _stepperData.stepCnt = _stepperData.stepsInRamp;
                        _stepperData.stepCnt2 = _stepperData.stepsInRamp-stepCnt;
                    } else { 
                        _stepperData.stepCnt = stepCnt;
                    }
                    DB_PRINT( "ACCEL: sct=%u, sct2=%u", _stepperData.stepCnt, _stepperData.stepCnt2 );
                    _stepIRQ();
                    break;
                  case RAMPDECEL:
                    // targetposition within stopramp?
                    _noStepIRQ();
                    if ( stepCnt <= _stepperData.stepsInRamp ) {
                        _stepperData.stepCnt2 = _stepperData.stepsInRamp-stepCnt;
                    } else { 
                        _stepperData.stepCnt = stepCnt;
                    }
                    DB_PRINT( "DECEL: sct=%u, sct2=%u", _stepperData.stepCnt, _stepperData.stepCnt2 );
                    _stepIRQ();
                    break;
                  case CRUISING:
                    // targetposition within stopramp?
                    _noStepIRQ();
                    if ( stepCnt <= _stepperData.stepRampLen ) {
                        _stepperData.stepCnt = _stepperData.stepRampLen;
                        _stepperData.stepCnt2 = _stepperData.stepRampLen-stepCnt;
                    } else { 
                        _stepperData.stepCnt = stepCnt;
                    }
                    _stepIRQ();
                     DB_PRINT( "CRUIS: sct=%u, sct2=%u", _stepperData.stepCnt, _stepperData.stepCnt2 );
                   break;
                }
                
            } else {
                // direction changes, stop and go backwards
                _noStepIRQ();
                switch ( _stepperData.rampState ) {
                  case RAMPACCEL:
                    // We are accelerating, we have to break this?
                    _stepperData.stepCnt = _stepperData.stepsInRamp;
                    break;
                  case RAMPDECEL:
                    // we ar already decelerating
                    break;
                  case CRUISING:
                    _stepperData.stepCnt = _stepperData.stepRampLen;
                    break;
                }
                _stepperData.stepCnt2 = _stepperData.stepCnt+stepCnt;
                _stepIRQ();

               
            }
        } else {
            // stepper does not move -> start a new move
            if ( stepValue != 0 ) {
                // we must move
                if ( stepValue > 0 ) patternIxInc = abs( _stepperData.patternIxInc );
                else     patternIxInc = -abs( _stepperData.patternIxInc );
                
                _noStepIRQ();
                _stepperData.patternIxInc   = patternIxInc;
                _stepperData.cycCnt         = 0;            // start with the next IRQ
                _stepperData.aCycSteps      = 0;
                _stepperData.aCycRemain     = 0;   
                _stepperData.rampState      = RAMPACCEL;
                _stepperData.stepsInRamp    = 0;
                _stepperData.stepCnt        = abs(stepsToMove);
                _stepIRQ();
            }
        }
    } else {
        // no ramp
        if ( stepValue > 0 ) patternIxInc = abs( _stepperData.patternIxInc );
        else     patternIxInc = -abs( _stepperData.patternIxInc );
        _noStepIRQ();
        _stepperData.patternIxInc = patternIxInc;
        _stepperData.stepCnt = abs(stepsToMove);
        _stepperData.cycCnt         = 0;            // start with the next IRQ
        _stepperData.aCycSteps      = _stepperData.tCycSteps;
        _stepperData.aCycRemain     = _stepperData.tCycRemain;   
        _stepperData.rampState      = CRUISING;
        _stepIRQ();
    }
    
    DB_PRINT( "StepValues:, sCnt=%ld, sCnt2=%ld, sMove=%ld, aCyc=%d", _stepperData.stepCnt, _stepperData.stepCnt2, stepsToMove, _stepperData.aCycSteps );
    
}


void Stepper4::setZero() {
    // set reference point for absolute positioning
    if ( _stepperData.output == NO_OUTPUT ) return; // not attached
    noInterrupts();
    _stepperData.stepsFromZero = 0;
    interrupts();
}

void Stepper4::write(long angleArg ) {
    // set next position as angle, measured from last setZero() - point
    //DB_PRINT("write: %d", angleArg);
    Stepper4::write( angleArg, 1 );
}

void Stepper4::write( long angleArg, byte fact ) {
    // for better resolution. angelArg/fact = angle in degrees
    // typical: fact = 10, angleArg in .1 degrees
    if ( _stepperData.output == NO_OUTPUT ) return ; // not attached
    bool negative;
    int angel2steps;
    negative =  ( angleArg < 0 ) ;
    //DB_PRINT( "angleArg: %d",angleArg ); //DB_PRINT( " getSFZ: ", getSFZ() );
    //Serial.print( "Write: " ); Serial.println( angleArg );
    angel2steps =  ( (abs(angleArg) * (long)stepsRev*10) / ( 360L * fact) +5) /10 ;
    if ( negative ) angel2steps = -angel2steps;
    doSteps(angel2steps  - getSFZ() );
}

void Stepper4::writeSteps( long stepPos ) {
    // go to position stepPos steps away from zeropoint
    if ( _stepperData.output == NO_OUTPUT ) return; // not attached

    doSteps(stepPos  - getSFZ() );
}

long Stepper4::read()
{   // returns actual position as degree
    if ( _stepperData.output == NO_OUTPUT ) return 0; // not attached

    long tmp = getSFZ();
    bool negative;
    negative = ( tmp < 0 );
	tmp = (abs(tmp)/stepsRev*360) + (( (abs(tmp)%stepsRev) *3600L / stepsRev ) +5) / 10;
    if ( negative ) tmp = -tmp;
    return  tmp;
}

long Stepper4::readSteps()
{   // returns actual position as steps
    if ( _stepperData.output == NO_OUTPUT ) return 0; // not attached

    return  getSFZ();
}



uint8_t Stepper4::moving() {
    // return how much still to move (percentage)
    long tmp;
    if ( _stepperData.output == NO_OUTPUT ) return 0; // not attached
    //Serial.print( _stepperData.stepCnt ); Serial.print(" "); 
    //Serial.println( _stepperData.aCycSteps );
    if ( stepsToMove == 0 ) {
        tmp = 0;        // there was nothing to move
    } else {
        noInterrupts(); // disable interrupt, because integer stepcnt is changed in TCR interrupt
        tmp = _stepperData.stepCnt;
        interrupts();  // undo cli() 
        if ( tmp > 0 ) {
            // do NOT return 0, even if less than 1%, because 0 means real stop of the motor
            if ( tmp < 2147483647L / 100 )
                //tmp = max ( (tmp * 100 / abs( stepsToMove)) , 1 );
                tmp = (tmp * 100 / abs( stepsToMove)) + 1;
            else
                //tmp = max ( (tmp  / ( abs( stepsToMove) / 100 ) ) , 1 );
                tmp =  (tmp  / ( abs( stepsToMove) / 100 ) ) + 1;
        }
    }
    return tmp ;
}

void Stepper4::rotate(int8_t direction) {
	// rotate endless ( not really, do maximum stepcount ;-)
    if ( _stepperData.output == NO_OUTPUT ) return; // not attached
    
	if (direction == 0 ) {
        if ( _stepperData.stepRampLen == 0 ) {
            // no ramp, identical to 'stop'
            stop();
        } else {
            // start decelerating
            _noStepIRQ();
            switch ( _stepperData.rampState ) {
              case STOPPED:
              case RAMPDECEL:
              case STARTDECEL:
                // already in Stop or decelerating - do nothing
                break;
              case RAMPACCEL:
                _stepperData.stepCnt = _stepperData.stepsInRamp;
                break;
              case CRUISING:
                _stepperData.stepCnt = _stepperData.stepRampLen;
                DB_PRINT( "rot: sCnt=%u\n\r", _stepperData.stepCnt );
                break;
            }
            _stepIRQ();
        }
	} else if (direction > 0 ) { // ToDo: Grenzwerte sauber berechnen
        doSteps(  2147483646L - _stepperData.stepRampLen );
	} else {
        doSteps( -2147483646L + _stepperData.stepRampLen);
    }
}

void Stepper4::stop() {
	// immediate stop of the motor
    if ( _stepperData.output == NO_OUTPUT ) return; // not attached
    
    noInterrupts();
	stepsToMove = 0;
    _stepperData.stepCnt = 0;
    interrupts();
}
///////////////////////////////////////////////////////////////////////////////////
// --------- Class Servo8 ---------------------------------
// Class-specific Variables

#ifdef WITHSERVO
const byte NO_ANGLE = 0xff;
const byte NO_PIN = 0xff;

Servo8::Servo8() : pin(NO_PIN),angle(NO_ANGLE),min16(1000/16),max16(2000/16)
{   servoData.servoIx = servoCount++;
    servoData.soll = -1;    // = not initialized
    noInterrupts();
    servoData.prevServoDataP = lastServoDataP;
    lastServoDataP = &servoData;
    interrupts();
}

void Servo8::setMinimumPulse(uint16_t t)
{
    t = t/16;
    if ( t >= MINPULSEWIDTH/16 && t < max16 ) min16 = t;
}

void Servo8::setMaximumPulse(uint16_t t)
{
    t = t/16;
    if ( t > min16 && t <= MAXPULSEWIDTH/16 ) max16 = t;
}


uint8_t Servo8::attach(int pinArg) {
    return attach( pinArg, MINPULSEWIDTH, MAXPULSEWIDTH, false );
}
uint8_t Servo8::attach(int pinArg, bool autoOff ) {
    return attach( pinArg, MINPULSEWIDTH, MAXPULSEWIDTH, autoOff );
}
uint8_t Servo8::attach(int pinArg, int pmin, int pmax ) {
    return attach( pinArg, pmin, pmax, false );
}

uint8_t Servo8::attach( int pinArg, int pmin, int pmax, bool autoOff ) {
    // return false if already attached or too many servos
    if ( pin != NO_PIN ||  servoData.servoIx >= MAX_SERVOS ) return 0;
    // set pulselength for angle 0 and 180
    if ( pmin >= MINPULSEWIDTH && pmin <= MAXPULSEWIDTH) min16 = pmin/16;
    if ( pmax >= MINPULSEWIDTH && pmax <= MAXPULSEWIDTH ) max16 = pmax/16;
	//DB_PRINT( "pin: %d, pmin:%d pmax%d autoOff=%d, min16=%d, max16=%d", pinArg, pmin, pmax, autoOff, min16, max16);
    
    // intialize objectspecific data
    lastPos = 3000*SPEED_RES ;    // initalize to middle position
    servoData.soll = -1;  // invalid position -> no pulse output
    servoData.ist = -1;   
    servoData.inc = 2000*SPEED_RES;  // means immediate movement
    servoData.pin = pinArg;
    servoData.on = false;  // create no pulses until next write
    servoData.noAutoff = autoOff?0:1 ;  
    #ifdef FAST_PORTWRT
    // compute portaddress and bitmask related to pin number
    servoData.portAdr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[ pinArg])]);
    servoData.bitMask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pinArg]);
    //DB_PRINT( "Idx: %d Portadr: 0x%x, Bitmsk: 0x%x", servoData.servoIx, servoData.portAdr, servoData.bitMask );
	#endif
    pin = pinArg;
    angle = NO_ANGLE;
    pinMode(pin,OUTPUT);
    digitalWrite(pin,LOW);

    if ( !timerInitialized) seizeTimer1();
    // initialize servochain pointer if not done already
    noInterrupts();
    if ( pulseP == NULL ) pulseP = lastServoDataP;
    interrupts();
    
    // enable compare-A interrupt
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
    TIMSK |=  _BV(OCIExA);   
    #elif defined __AVR_MEGA__
    //DB_PRINT( "IniOCR: %d", OCRxA );
    TIMSKx |=  _BV(OCIExA) ; 
    //DB_PRINT( "AttOCR: %d", OCRxA );
    #elif defined __STM32F1__
        timer_cc_enable(MT_TIMER, SERVO_CHN);
    #endif
    return 1;
}

void Servo8::detach()
{
    servoData.on = false;  
    servoData.soll = -1;  
    servoData.ist = -1;  
    servoData.pin = NO_PIN;  
    pin = NO_PIN;
}

void Servo8::write(int angleArg)
{   // set position to move to
    // values between 0 and 180 are interpreted as degrees,
    // values between MINPULSEWIDTH and MAXPULSEWIDTH are interpreted as microseconds
    static int newpos;
    #ifdef __AVR_MEGA__
	//DB_PRINT( "Write: angleArg=%d, Soll=%d, OCR=%u", angleArg, servoData.soll, OCRxA );
    #endif
    if ( pin != NO_PIN ) { // only if servo is attached
        //Serial.print( "Pin:" );Serial.print(pin);Serial.print("Wert:");Serial.println(angleArg);
        #ifdef __AVR_MEGA__
		//DB_PRINT( "Stack=0x%04x, &sIx=0x%04x", ((SPH&0x7)<<8)|SPL, &servoData.servoIx );
        #endif
        if ( angleArg < 0) angleArg = 0;
        if ( angleArg <= 255) {
            // pulse width as degrees (byte values are always degrees) 09-02-2017
            angle = min( 180,angleArg);

            newpos = map( angle, 0,180, min16*16, max16*16 ) * TICS_PER_MICROSECOND * SPEED_RES;
        } else {
            // pulsewidth as microseconds
            if ( angleArg < MINPULSEWIDTH ) angleArg = MINPULSEWIDTH;
            if ( angleArg > MAXPULSEWIDTH ) angleArg = MAXPULSEWIDTH;
            newpos = angleArg * TICS_PER_MICROSECOND * SPEED_RES;
            angle = map( angleArg, min16*16, max16*16, 0, 180 );  // angle in degrees
        }
        if ( servoData.soll < 0 ) {
            // Serial.println( "first write");
            // this is the first pulse to be created after attach
            servoData.on = true;
            lastPos = newpos;
            noInterrupts();
            servoData.soll= newpos ; // .ist - value is still -1 (invalid) -> will jump to .soll immediately
            interrupts();
            
        }
        else if ( newpos != servoData.soll ) {
            // position has changed, store old position, set new position
            lastPos = servoData.soll;
            noInterrupts();
            servoData.soll= newpos ;
            interrupts();
        }
        servoData.offcnt = OFF_COUNT;   // auf jeden Fall wieder Pulse ausgeben
    }
}

void Servo8::setSpeed( int speed, bool compatibility ) {
    // set global compatibility-Flag
    speedV08 = compatibility;
    setSpeed( speed );
}

void Servo8::setSpeed( int speed ) {
    // Set increment value for movement to new angle
    if ( pin != NO_PIN ) { // only if servo is attached
        if ( speedV08 ) speed *= SPEED_RES;
        noInterrupts();
        if ( speed == 0 )
            servoData.inc = 2000*SPEED_RES;  // means immiediate movement
        else
            servoData.inc = speed;
        interrupts();
    }
}

uint8_t Servo8::read() {
    // get position in degrees
    int value;
    if ( pin == NO_PIN ) return -1; // Servo not attached
    noInterrupts();
    value = servoData.ist;
    interrupts();
    return map( value/TICS_PER_MICROSECOND/SPEED_RES, min16*16, max16*16, 0, 180 );
}

int Servo8::readMicroseconds() {
    // get position in microseconds
    int value;
    if ( pin == NO_PIN ) return -1; // Servo not attached
    noInterrupts();
    value = servoData.ist;
    interrupts();
    return value/TICS_PER_MICROSECOND/SPEED_RES;   

}

uint8_t Servo8::moving() {
    // return how much still to move (percentage)
    if ( pin == NO_PIN ) return 0; // Servo not attached
    long total , remaining;
    total = abs( lastPos - servoData.soll );
    noInterrupts(); // disable interrupt, because integer servoData.ist is changed in interrupt
    remaining = abs( servoData.soll - servoData.ist );
    interrupts();  // allow interrupts again
    if ( remaining == 0 ) return 0;
    return ( remaining * 100 ) /  total +1;
}
uint8_t Servo8::attached()
{
    return ( pin != NO_PIN );
}
#endif


/////////////////////////////////////////////////////////////////////////////
//Class SoftLed - for Led with soft on / soft off ---------------------------
// Version with Software PWM
#ifdef WITHSOFTLED
SoftLed::SoftLed() {
    ledValid = LEDVALID;            // Flag 'object created'
    ledIx = ledCount++;
    ledData.speed    = 0;           // defines rising/falling timer
    ledData.aStep    = 0 ;          // actual PWM step
    ledData.aCycle   = 0;           // actual cycle ( =length of PWM pule )
    ledData.stpCnt   = 0;           // counter for PWM cycles on same step (for low speed)
    ledData.actPulse = false;       // PWM pulse is active
    ledData.state    = NOTATTACHED; // initialize 
    setpoint = OFF ;                // initialize to off
    ledType = LINEAR;
    ledData.nextLedDataP = NULL;    // don't put in ISR chain
    ledData.invFlg = false;
}

void SoftLed::mount( uint8_t stateVal ) {
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
    if ( ledValid != LEDVALID ) return false; // this is not a valid instance
    
    ledData.invFlg  = invArg;
    pinMode( pinArg, OUTPUT );
    //DB_PRINT( "Led attached, ledIx = 0x%x, Count = %d", ledIx, ledCount );
    ledData.state   = STATE_OFF ;   // initialize 
    ledSpeed        = 1;            // defines rising/falling timer
    ledData.aStep   = 0 ;           // actual PWM step
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

    return true;
}

void SoftLed::on(){
    if ( ledValid != LEDVALID ) return;  // this is not a valid instance
    uint8_t stateT;
    // Don't do anything if its already ON 
    if ( setpoint != ON  ) {
        setpoint        = ON ;
        ledData.aStep   = 0;
        ledData.stpCnt  = 0; 
        if ( ledType == LINEAR ) {
            stateT          = INCLIN;
            ledData.speed   = ledSpeed;
            ledData.aCycle  = 1;
        } else { // is bulb simulation
            stateT          = INCFAST;
            ledData.speed   = ledSpeed==1? -1 : ledSpeed / 3;
            if ( ledData.speed <= 0 ) {
                stateT      = INCSLOW;
                ledData.stpCnt = 1;
            }
            ledData.aCycle  = iSteps[0];
        }
        mount(stateT);
    }
    //DB_PRINT( "Led %d On, state=%d", ledIx, ledData.state);
}

void SoftLed::off(){
    if ( ledValid != LEDVALID ) return; // this is not a valid instance
    uint8_t stateT;
    // Dont do anything if its already OFF 
    if ( setpoint != OFF ) {
        //SET_TP3;
        setpoint            = OFF;
        ledData.aStep       = 0;
        ledData.stpCnt      = 0; 
        if ( ledType == LINEAR ) {
            stateT          = DECLIN;
            ledData.speed   = ledSpeed;
            ledData.aCycle  = LED_CYCLE_MAX-1;
        } else { // is bulb simulation
            //CLR_TP3;
            stateT = DECFAST;
            ledData.speed = ledSpeed==1? -1 : ledSpeed / 3;
            //SET_TP3;
            if ( ledData.speed <= 0 ) {
                stateT          = DECSLOW;
                ledData.stpCnt  = 1;
            }
            ledData.aCycle = LED_CYCLE_MAX + 1 - iSteps[0];
        }
        //CLR_TP3;
        mount(stateT);
    }
    //DB_PRINT( "Led %d Off, state=%d", ledIx, ledData.state);
}

void SoftLed::toggle( void ) {
    if ( ledValid != LEDVALID ) return; // this is not a valid instance
    if ( setpoint == ON  ) off();
    else on();
}

void SoftLed::write( uint8_t setpntVal, uint8_t ledPar ){
    if ( ledValid != LEDVALID ) return; // this is not a valid instance
    ledType = ledPar;
    write( setpntVal ) ;
}

void SoftLed::write( uint8_t setpntVal ){
    //DB_PRINT( "LedWrite ix= %d, valid= 0x%x, sp=%d, lT=%d", ledIx, ledValid, setpntVal, ledType );
    if ( ledValid != LEDVALID ) return; // this is not a valid instance
    if ( setpntVal == ON ) on(); else off();
    #ifdef debug
    // im Debugmode hier die Led-Daten ausgeben
    //DB_PRINT( "LedData[%d]\n\speed=%d, Type=%d, aStep=%d, stpCnt=%d, state=%d, setpoint= %d",
            ledValid, ledSpeed, ledType, ledData.aStep, ledData.stpCnt, ledData.state
                    , setpoint);
    //DB_PRINT( "ON=%d, NextCyc=%d, CycleCnt=%d, StepIx=%d, NextStep=%d", 
    //         ON, ledNextCyc, ledCycleCnt, ledStepIx, ledNextStep);
    #endif
}

void SoftLed::riseTime( int riseTime ) {
    if ( ledValid != LEDVALID ) return;
    // length of startphase in ms (min 20ms, max 1200ms )
    // the real risetime is only a rough approximate to this time
    // risetime is computed to a 'speed' Value with 1 beeing the slowest 
    // with speed value = 1 means risetime is (LED_CYCLE_MAX * LED_PWMTIME)
    // risetime = (LED_CYCLE_MAX * LED_PWMTIME) / speed
    // 
    // toDo: a better approximation to 'riseTime'
    int riseMax = (int)( LED_CYCLE_MAX * (int)LED_PWMTIME);
    if ( riseTime <= 20 ) riseTime = 20;
    if ( riseTime >= riseMax ) riseTime = riseMax;
    int tmp = ( (riseMax  *10) / ( riseTime  ) +5 ) /10;
    ledSpeed = tmp;
    //DB_PRINT( "ledSpeed[%d] = %d ( risetime=%d, riseMax=%d )", ledIx, ledSpeed, riseTime, riseMax );
}
#endif
////////////////////////////////////////////////////////////////////////////
// Class EggTimer - Timerverwaltung für Zeitverzögerungen in der Loop-Schleife
// 
void EggTimer::setTime(  long wert ) {
    endtime =  (long) millis() + ( (long)wert>0?wert:1 );
    active = true;
}

bool EggTimer::running() {
    if ( active ) active =  ( endtime - (long)millis() > 0 );
    return active;
}

long EggTimer::getTime() {
    // return remaining time
    if ( running() ) return endtime - (long)millis();
    else return 0;
}
EggTimer::EggTimer()
{
    active = false;
}


