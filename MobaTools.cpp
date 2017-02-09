
/*
  MobaTools V0.9
   (C) 03-2017 fpm fpm@mnet-online.de
   
  History:
  V0.9  03-2017
        Better resolution for the 'speed' paramter (programm starts in compatibility mode
        preparations for porting to STM32F1 platform
        
  V0.8 02-2017
        Enable Softleds an all digital outputs
  V0.7 01-2017
		Allow nested Interrupts with the servos. This allows more precise other
        interrupts e.g. for NmraDCC Library.
		A4988 stepper driver IC is supported (needs only 2 ports: step and direction)

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
//#define debug
#ifdef debug 
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
        #define MODE_TP3 
        #define SET_TP3 
        #define CLR_TP3 
        #define MODE_TP4 
        #define SET_TP4 
        #define CLR_TP4 
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
        // STM32F103...
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
    #define DB_PRINT( x, ... ) { sprintf_P( dbgBuf, PSTR( x ), __VA_ARGS__ ) ; Serial.println( dbgBuf ); }
    static char dbgBuf[80];
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
    
    #define DB_PRINT ;
#endif


// constants
static const int stepPattern[8] = {0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,0b0001 };

// Global Data for all instances and classes  --------------------------------
static uint8_t timerInitialized = false;
static uint8_t spiInitialized = false;

// Variables for servos
static servoData_t servoData[MAX_SERVOS];
static byte servoCount = 0;
static byte pulseIx = 0;    // pulse Index in IRQ
static enum { PON, POFF } IrqType = PON; // Cycle starts with 'pulse on'
static word activePulseOff = 0;     // OCR-value of pulse end 
static byte activePulseIx = 0;       // Index of pulse to stop
static byte stopPulseIx = 0;        // index of Pulse whose stop time is already in OCR1
static word nextPulseLength = 0;
static byte nextPulseIx = 0;
static bool speedV08 = true;    // Compatibility-Flag for speed method



// Variables for stepper motors
static stepperData_t stepperData[MAX_STEPPER];
static uint8_t spiData[2]; // step pattern to be output on SPI
                            // low nibble of spiData[0] is SPI_1
                            // high nibble of spiData[1] is SPI_4
                            // spiData[1] is shifted out first
static uint8_t spiByteCount = 0;
static byte stepperCount = 0;
static uint8_t cyclesLastIRQ = 1;  // cycles since last IRQ

// variables for softLeds
static ledData_t ledData[MAX_LEDS];
static byte ledCount = 0;
static uint8_t ledNextCyc = 1;     // next Cycle that is relevant for leds
static uint8_t ledCycleCnt = 0;    // count IRQ cycles within PWM cycle
//static uint8_t  ledStepIx = 0;      // Stepcounter for Leds ( Index in Array isteps , 0: start of pwm-Cycle )
//static uint8_t  ledNextStep = 0;    // next step needed for softleds
static uint8_t  ledIx;              // Index of active Led in ISR
//==========================================================================

// global functions / Interrupts


// ---------- OCR1B Compare Interrupt used for stepper motor and Softleds ----------------
#pragma GCC optimize "Os"
#ifdef __AVR_MEGA__
ISR ( TIMER1_COMPB_vect)
#elif defined __STM32F1__
void ISR_Stepper(void)
#endif
{ // Timer1 Compare B, used for stepper motor, starts every CYCLETIME us
    // 26-09-15 An Interrupt is only created at timeslices, where data is to output
    uint8_t i, spiChanged, changedPins, bitNr;
    uint16_t tmp;
    uint8_t nextCycle = 20000  / CYCLETIME ;// min ist one cycle per Timeroverflow
    
    SET_TP2; // Oszimessung Dauer der ISR-Routine
    spiChanged = false;
    interrupts(); // allow nested interrupts, because this IRQ may take long
    
    // ---------------Stepper motors ---------------------------------------------
    for ( i=0; i<stepperCount; i++ ) {
        // für maximal 4 Motore
        if ( stepperData[i].output == A4988_PINS ) {
            // reset step pulse - pulse is max one cycle lenght
            #ifdef FAST_PORTWRT
            *stepperData[i].portPins[0].Adr &= ~stepperData[i].portPins[0].Mask;
            #else
            digitalWrite( stepperData[i].pins[0], LOW );
            #endif
        }
        if ( stepperData[i].activ && stepperData[i].stepCnt > 0 ) {
            // only active motors
            stepperData[i].cycCnt+=cyclesLastIRQ;
            if ( stepperData[i].cycCnt >= stepperData[i].cycSteps ) {
                // Do one step
                stepperData[i].cycCnt = 0 ;
                // update position for absolute positioning
                stepperData[i].stepsFromZero += stepperData[i].patternIxInc;
                
                if ( !stepperData[i].endless ) --stepperData[i].stepCnt;
                // sign of patternIxInc defines direction
                stepperData[i].patternIx += stepperData[i].patternIxInc;
                if ( stepperData[i].patternIx > 7 ) stepperData[i].patternIx = 0;
                if ( stepperData[i].patternIx < 0 ) stepperData[i].patternIx += 8;
                
                // store pattern data
                switch ( stepperData[i].output ) {
                  #ifdef __AVR_MEGA__
                  case PIN4_7:
                    PORTD = (PORTD & 0x0f) | ( stepPattern[ stepperData[i].patternIx ] <<4 );   
                    break;
                  case PIN8_11:
                    PORTB = (PORTB & 0xf0) | ( stepPattern[ stepperData[i].patternIx ] );   
                    break;
                  #endif
                  case SPI_1:
                    spiData[0] = (spiData[0] & 0xf0) | ( stepPattern[ stepperData[i].patternIx ] );
                    spiChanged = true;                    
                    break;
                  case SPI_2:
                    spiData[0] = (spiData[0] & 0x0f) | ( stepPattern[ stepperData[i].patternIx ] <<4 );
                    spiChanged = true;
                    break;
                  case SPI_3:
                    spiData[1] = (spiData[1] & 0xf0) | ( stepPattern[ stepperData[i].patternIx ] );   
                    spiChanged = true;
                    break;
                  case SPI_4:
                    spiData[1] = (spiData[1] & 0x0f) | ( stepPattern[ stepperData[i].patternIx ] <<4 );
                    spiChanged = true;
                    break;
                  case SINGLE_PINS : // Outpins are individually defined
                    changedPins = stepPattern[ stepperData[i].patternIx ] ^ stepperData[i].lastPattern;
                    for ( bitNr = 0; bitNr < 4; bitNr++ ) {
                        if ( changedPins & (1<<bitNr ) ) {
                            // bit Changed, write to pin
                            if ( stepPattern[ stepperData[i].patternIx ] & (1<<bitNr) ) {
                                #ifdef FAST_PORTWRT
                                *stepperData[i].portPins[bitNr].Adr |= stepperData[i].portPins[bitNr].Mask;
                                #else
                                digitalWrite( stepperData[i].pins[bitNr], HIGH );
                                #endif
                            } else {
                                #ifdef FAST_PORTWRT
                                *stepperData[i].portPins[bitNr].Adr &= ~stepperData[i].portPins[bitNr].Mask;
                                #else    
                                digitalWrite( stepperData[i].pins[bitNr], LOW );
                                #endif    
                            }
                        }
                    }
                    stepperData[i].lastPattern = stepPattern[ stepperData[i].patternIx ];
                    break;
                  case A4988_PINS : // output step-pulse and direction
                    // direction first
                    if ( stepperData[i].patternIxInc > 0 ) {
                        // turn forward 
                        #ifdef FAST_PORTWRT
                        *stepperData[i].portPins[1].Adr |= stepperData[i].portPins[1].Mask;
                        #else
                        digitalWrite( stepperData[i].pins[1], HIGH );
                        #endif
                    } else {
                        // turn backwards
                        #ifdef FAST_PORTWRT
                        *stepperData[i].portPins[1].Adr &= ~stepperData[i].portPins[1].Mask;
                        #else
                        digitalWrite( stepperData[i].pins[1], LOW );
                        #endif
                    }    
                    // Set step pulse ( will be resettet in next IRQ )
                    #ifdef FAST_PORTWRT
                    *stepperData[i].portPins[0].Adr |= stepperData[i].portPins[0].Mask;
                    #else
                    digitalWrite( stepperData[i].pins[0], HIGH );
                    #endif
                    
                  default:
                    // should never be reached
                    break;
                }
            }
            nextCycle = min ( nextCycle, stepperData[i].cycSteps-stepperData[i].cycCnt );
        } // end of 'if stepper active'
    } // end of stepper-loop
    
    // shift out spiData, if SPI is active
    if ( spiInitialized && spiChanged ) {
        digitalWrite( SS, LOW );
        #ifdef __AVR_MEGA__
        spiByteCount = 0;
        SPDR = spiData[1];
        #elif defined __STM32F1__
        digitalWrite(BOARD_SPI2_NSS_PIN,LOW);
        spi_tx_reg(SPI2, (spiData[1]<<8) + spiData[0] );
        #endif
    }
    //============  End of steppermotor ======================================
    
    // ---------------------- softleds -----------------------------------------------
    ledCycleCnt += cyclesLastIRQ;
    if ( ledCycleCnt >= ledNextCyc ) {
        // this IRQ is relevant for softleds
        ledNextCyc = LED_CYCLE_MAX; // there must be atleast one IRQ per PWM Cycle
        if ( ledCycleCnt >= LED_CYCLE_MAX ) {
            // start of a new PWM Cycle - switch all leds rising/falling state to on
            ledCycleCnt = 0;
            for ( ledIx=0; ledIx<ledCount; ledIx++ ) {
                SET_TP1;
                // loop over active Leds
                // yes it's ugly, but because of performance reasons this is done a little bit assembler like
                static void * const pwm0tab[]  { &&pwm0end,&&pwm0end,
                            &&incfast0,&&decfast0,&&incslow0,&&decslow0,&&inclin0,&&declin0 };
                goto  *pwm0tab[ledData[ledIx].state] ;
                  incfast0:
                  incslow0:
                  inclin0:
                    SET_TP4;
                    // switch on led with linear characteristic
                   #ifdef FAST_PORTWRT
                    *ledData[ledIx].portPin.Adr |= ledData[ledIx].portPin.Mask;
                    #else
                    digitalWrite( ledData[ledIx].pin, HIGH );
                    #endif
                    // check if led off is reached
                    if ( ledData[ledIx].aCycle >=  LED_CYCLE_MAX-1 ) {
                        SET_TP3;
                        // led is full on
                        ledData[ledIx].state = ON;
                        ledData[ledIx].aCycle = 0;
                        CLR_TP3;
                    } else { // switch to next PWM step
                        //ledNextCyc = min( ledData[ledIx].aCycle, ledNextCyc);
                        if ( ledNextCyc > ledData[ledIx].aCycle ) ledNextCyc = ledData[ledIx].aCycle;
                        ledData[ledIx].actPulse = true;
                    }
                    CLR_TP4;
                    goto pwm0end;
                  decfast0:
                  decslow0:
                  declin0:
                    CLR_TP2;
                    // switch off led 
                   if ( ledData[ledIx].aCycle <= 0  ) {
                        SET_TP3;
                        // led is full off
                        ledData[ledIx].state = OFF;
                        ledData[ledIx].aCycle = 0;
                        CLR_TP3;
                    } else { // switch to next PWM step
                        #ifdef FAST_PORTWRT
                        *ledData[ledIx].portPin.Adr |= ledData[ledIx].portPin.Mask;
                        #else
                        digitalWrite( ledData[ledIx].pin, HIGH );
                        #endif
                        //ledNextCyc = min( ledData[ledIx].aCycle, ledNextCyc);
                        if ( ledNextCyc > ledData[ledIx].aCycle ) ledNextCyc = ledData[ledIx].aCycle;
                        ledData[ledIx].actPulse = true;
                    }
                    SET_TP2;
                pwm0end: // end of 'switch'
                CLR_TP1;
            } // end of led loop
        } else { // is switchofftime within PWM cycle
            for ( ledIx=0; ledIx<ledCount; ledIx++ ) {
                SET_TP3;
                if ( ledData[ledIx].actPulse ) {
                    // led is within PWM cycle with output high
                    if ( ledData[ledIx].aCycle <= ledCycleCnt ) {
                        CLR_TP3;
                        #ifdef FAST_PORTWRT
                        *ledData[ledIx].portPin.Adr &= ~ledData[ledIx].portPin.Mask;
                        #else
                        digitalWrite( ledData[ledIx].pin, LOW );
                        #endif
                        ledData[ledIx].actPulse = false;
                        // determine length of next PWM Cyle
                        switch ( ledData[ledIx].state ) {
                          case INCFAST:
                            ledData[ledIx].aStep += ledData[ledIx].speed;
                            if ( ledData[ledIx].aStep > LED_STEP_MAX ) ledData[ledIx].aStep = LED_STEP_MAX;
                            ledData[ledIx].aCycle = iSteps[ledData[ledIx].aStep];
                            break;
                          case DECFAST:
                            ledData[ledIx].aStep += ledData[ledIx].speed;
                            if ( ledData[ledIx].aStep > LED_STEP_MAX ) ledData[ledIx].aStep = LED_STEP_MAX;
                            ledData[ledIx].aCycle = LED_CYCLE_MAX-iSteps[ledData[ledIx].aStep];
                            break;
                          case INCSLOW:
                            if ( --ledData[ledIx].stpCnt < ledData[ledIx].speed ) {
                                ledData[ledIx].aStep += 1;
                                ledData[ledIx].stpCnt = 1;
                            }
                            if ( ledData[ledIx].aStep > LED_STEP_MAX ) ledData[ledIx].aStep = LED_STEP_MAX;
                            ledData[ledIx].aCycle = iSteps[ledData[ledIx].aStep];
                            break;
                          case DECSLOW:
                            SET_TP4;
                            if ( --ledData[ledIx].stpCnt < ledData[ledIx].speed ) {
                            CLR_TP4;
                                ledData[ledIx].aStep += 1;
                                ledData[ledIx].stpCnt = 1;
                            SET_TP4;
                            }
                            if ( ledData[ledIx].aStep > LED_STEP_MAX ) ledData[ledIx].aStep = LED_STEP_MAX;
                            ledData[ledIx].aCycle = LED_CYCLE_MAX-iSteps[ledData[ledIx].aStep];
                            CLR_TP4;
                            break;
                          case INCLIN:
                            ledData[ledIx].aCycle += ledData[ledIx].speed;
                            if ( ledData[ledIx].aCycle > LED_CYCLE_MAX-1 ) ledData[ledIx].aCycle = LED_CYCLE_MAX-1;
                            //ledNextCyc = min( ledData[ledIx].aCycle, ledNextCyc);
                            break;
                          case DECLIN:
                            ledData[ledIx].aCycle -= ledData[ledIx].speed;
                            if ( ledData[ledIx].aCycle <= 0 ) ledData[ledIx].aCycle = 0;
                            //ledNextCyc = min( ledData[ledIx].aCycle, ledNextCyc);
                            break;
                          default:
                            break;
                        }
                        SET_TP3;
                        
                    } else { // next necessary step
                       SET_TP1;
                       ledNextCyc = min( ledData[ledIx].aCycle, ledNextCyc);
                       CLR_TP1;
                    }
                }
                CLR_TP3;
            }
        }
        CLR_TP1;
     } // end of softleds 
    nextCycle = min( nextCycle, ( ledNextCyc-ledCycleCnt ) );
    CLR_TP2;
    // ======================= end of softleds =====================================
    
    
    
    cyclesLastIRQ = nextCycle;
    SET_TP2;
    // set compareregister to next interrupt time;
     noInterrupts(); // when manipulating 16bit Timerregisters IRQ must be disabled
    // compute next IRQ-Time in us, not in tics, so we don't need long
    #ifdef __AVR_MEGA__
    tmp = ( OCR1B / TICS_PER_MICROSECOND + nextCycle * CYCLETIME );
    if ( tmp > 20000 ) tmp = tmp - 20000;
    OCR1B = tmp * TICS_PER_MICROSECOND;
    #elif defined __STM32F1__
    tmp = ( timer_get_compare(MT_TIMER, STEP_CHN) / TICS_PER_MICROSECOND + nextCycle * CYCLETIME );
    if ( tmp > 20000 ) tmp = tmp - 20000;
    timer_set_compare( MT_TIMER, STEP_CHN, tmp * TICS_PER_MICROSECOND) ;
    #endif
    interrupts();
    
    CLR_TP2; // Oszimessung Dauer der ISR-Routine
}
// ---------- SPI interupt used for output stepper motor data -------------
extern "C" {
#ifdef __AVR_MEGA__
ISR ( SPI_STC_vect )
{   // output step-pattern on SPI, set SS when ready
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
void __irq_spi2(void) {// STM32
    static int rxData;
    rxData = spi_rx_reg(SPI2);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI2_NSS_PIN,HIGH);
}
#endif
}
#ifdef FIXED_POSITION_SERVO_PULSES
// ---------- OCR1A Compare Interrupt used for servo motor ----------------
// Positions of servopulses within 20ms cycle are fixed -  8 servos
#define PULSESTEP ( 40000 / MAX_SERVOS )
#ifdef __AVR_MEGA__
ISR ( TIMER1_COMPA_vect) {
#elif defined __STM32F1__
void ISR_Servo( void) {
    uint16_t OCR1A;
#endif
    // Timer1 Compare A, used for servo motor
    SET_TP1; // Oszimessung Dauer der ISR-Routine
    if ( IrqType == POFF ) {
        IrqType = PON ; // it's always alternating
        // switch off previous started pulse
        #ifdef FAST_PORTWRT
        *servoData[pulseIx].portAdr &= ~servoData[pulseIx].bitMask;
        #else
        digitalWrite( servoData[pulseIx].pin, LOW );
        #endif
        // Set next startpoint of servopulse
        if ( ++pulseIx >= MAX_SERVOS ) {
            // Start over
            OCR1A = FIRST_PULSE;
            pulseIx = 0;
        } else {
            OCR1A = FIRST_PULSE + pulseIx * PULSESTEP;
        }
    } else {
        // look for next pulse to start
        if ( servoData[pulseIx].soll < 0 ) {
            // no pulse to output, switch to next startpoint
            if ( ++pulseIx >= MAX_SERVOS ) {
                // Start over
                OCR1A = FIRST_PULSE;
                pulseIx = 0;
            } else {
                OCR1A = FIRST_PULSE + pulseIx * PULSESTEP;
            }
        } else { // found pulse to output
            if ( servoData[pulseIx].ist == servoData[pulseIx].soll ) {
                // no change of pulselength
                if ( servoData[pulseIx].offcnt > 0 ) servoData[pulseIx].offcnt--;
            } else if ( servoData[pulseIx].ist < servoData[pulseIx].soll ) {
                servoData[pulseIx].offcnt = OFF_COUNT;
                if ( servoData[pulseIx].ist < 0 ) servoData[pulseIx].ist = servoData[pulseIx].soll; // first position after attach
                else servoData[pulseIx].ist += servoData[pulseIx].inc;
                if ( servoData[pulseIx].ist > servoData[pulseIx].soll ) servoData[pulseIx].ist = servoData[pulseIx].soll;
            } else {
                servoData[pulseIx].offcnt = OFF_COUNT;
                servoData[pulseIx].ist -= servoData[pulseIx].inc;
                if ( servoData[pulseIx].ist < servoData[pulseIx].soll ) servoData[pulseIx].ist = servoData[pulseIx].soll;
            } 
            OCR1A = (servoData[pulseIx].ist/SPEED_RES) + GET_COUNT - 4; // compensate for computing time
            if ( servoData[pulseIx].on && (servoData[pulseIx].offcnt+servoData[pulseIx].noAutoff) > 0 ) {
                CLR_TP1;
                #ifdef FAST_PORTWRT
                *servoData[pulseIx].portAdr |= servoData[pulseIx].bitMask;
                #else
                digitalWrite( servoData[pulseIx].pin, HIGH );
                #endif
                SET_TP1;
            }
            IrqType = POFF;
        } 
    } //end of 'pulse ON'
    #ifdef __STM32F1__
    timer_set_compare(MT_TIMER,  SERVO_CHN, OCR1A);
    #endif 
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}

#else // create overlapping servo pulses
// Positions of servopulses within 20ms cycle are variable, max 2 pulses at the same time
// 27.9.15 with variable overlap, depending on length of next pulse: 16 Servos
// 2.1.16 Enable interrupts after timecritical path (e.g. starting/stopping servo pulses)
//        so other timecritical tasks can interrupt (nested interrupts)
static bool searchNextPulse() {
    while ( pulseIx < servoCount && servoData[pulseIx].soll < 0 ) {
        //SET_TP2;
        pulseIx++;
        //CLR_TP2;
    }
    if ( pulseIx >= servoCount ) {
        // there is no more pulse to start, we reached the end
        return false;
    } else { // found pulse to output
        if ( servoData[pulseIx].ist == servoData[pulseIx].soll ) {
            // no change of pulselength
            if ( servoData[pulseIx].offcnt > 0 ) servoData[pulseIx].offcnt--;
        } else if ( servoData[pulseIx].ist < servoData[pulseIx].soll ) {
            servoData[pulseIx].offcnt = OFF_COUNT;
            if ( servoData[pulseIx].ist < 0 ) servoData[pulseIx].ist = servoData[pulseIx].soll; // first position after attach
            else servoData[pulseIx].ist += servoData[pulseIx].inc;
            if ( servoData[pulseIx].ist > servoData[pulseIx].soll ) servoData[pulseIx].ist = servoData[pulseIx].soll;
        } else {
            servoData[pulseIx].offcnt = OFF_COUNT;
            servoData[pulseIx].ist -= servoData[pulseIx].inc;
            if ( servoData[pulseIx].ist < servoData[pulseIx].soll ) servoData[pulseIx].ist = servoData[pulseIx].soll;
        } 
        return true;
    } 
} //end of 'searchNextPulse'

// ---------- OCR1A Compare Interrupt used for servo motor ----------------
#ifdef __AVR_MEGA__
ISR ( TIMER1_COMPA_vect) {
#elif defined __STM32F1__
void ISR_Servo( void) {
    uint16_t OCR1A;
#endif
    // Timer1 Compare A, used for servo motor
    if ( IrqType == POFF ) { // Pulse OFF time
        SET_TP3; // Oszimessung Dauer der ISR-Routine
        IrqType = PON ; // it's (nearly) always alternating
        // switch off previous started pulse
        #ifdef FAST_PORTWRT
        *servoData[stopPulseIx].portAdr &= ~servoData[stopPulseIx].bitMask;
        #else
        digitalWrite( servoData[stopPulseIx].pin, LOW );
        #endif
        if ( nextPulseLength > 0 ) {
            // there is a next pulse to start, compute starttime 
            // set OCR value to next starttime ( = endtime of running pulse -overlap )
            // next starttime must behind actual timervalue and endtime of next pulse must
            // lay after endtime of runningpuls + safetymargin (it may be necessary to start
            // another pulse between these 2 ends)
            word tmpTCNT1 = GET_COUNT + MARGINTICS/2;
            interrupts();
            CLR_TP3 ;
            OCR1A = max ( ((long)activePulseOff + (long) MARGINTICS - (long) nextPulseLength), ( tmpTCNT1 ) );
        } else {
            // we are at the end, no need to start another pulse in this cycle
            if ( activePulseOff ) {
                // there is still a running pulse to stop
                SET_TP1; // Oszimessung Dauer der ISR-Routine
                OCR1A = activePulseOff;
                IrqType = POFF;
                stopPulseIx = activePulseIx;
                activePulseOff = 0;
                CLR_TP1; // Oszimessung Dauer der ISR-Routine
            } else { // was last pulse, start over
                pulseIx = 0;
                nextPulseLength = 0;
                OCR1A = FIRST_PULSE;
            }
        }
        CLR_TP3 ;
    } else { // Pulse ON - time
        SET_TP1; // Oszimessung Dauer der ISR-Routine
        // look for next pulse to start
        // do we know the next pulse already?
        if ( nextPulseLength > 0 ) {
            // yes we know, start this pulse and then look for next one
            word tmpTCNT1= GET_COUNT-4; // compensate for computing time
            if ( servoData[nextPulseIx].on && (servoData[nextPulseIx].offcnt+servoData[nextPulseIx].noAutoff) > 0 ) {
                // its a 'real' pulse, set output pin
                CLR_TP1;
                #ifdef FAST_PORTWRT
                *servoData[nextPulseIx].portAdr |= servoData[nextPulseIx].bitMask;
                #else
                digitalWrite( servoData[nextPulseIx].pin, HIGH );
                #endif
            }
            interrupts(); // the following isn't time critical, so allow nested interrupts
            SET_TP3;
            // the 'nextPulse' we have started now, is from now on the 'activePulse', the running activPulse is now the
            // pulse to stop next.
            stopPulseIx = activePulseIx; // because there was a 'nextPulse' there is also an 'activPulse' which is the next to stop
            OCR1A = activePulseOff;
            activePulseIx = nextPulseIx;
            activePulseOff = servoData[activePulseIx].ist/SPEED_RES + tmpTCNT1; // end of actually started pulse
            nextPulseLength = 0;
            SET_TP1;
        }
        if ( searchNextPulse() ) {
            // found a pulse
            if ( activePulseOff == 0 ) {
                // it is the first pulse in the sequence, start it
                activePulseIx = pulseIx; 
                activePulseOff = servoData[pulseIx].ist/SPEED_RES + GET_COUNT - 4; // compensate for computing time
                if ( servoData[pulseIx].on && (servoData[pulseIx].offcnt+servoData[pulseIx].noAutoff) > 0 ) {
                    // its a 'real' pulse, set output pin
                    #ifdef FAST_PORTWRT
                    *servoData[pulseIx].portAdr |= servoData[pulseIx].bitMask;
                    #else
                    digitalWrite( servoData[pulseIx].pin, HIGH );
                    #endif
                }
                word tmpTCNT1 = GET_COUNT;
                interrupts(); // the following isn't time critical, so allow nested interrupts
                SET_TP3;
                // look for second pulse
                pulseIx++;
                if ( searchNextPulse() ) {
                    // there is a second pulse - this is the 'nextPulse'
                    nextPulseLength = servoData[pulseIx].ist/SPEED_RES;
                    nextPulseIx = pulseIx++;
                    // set Starttime for 2. pulse in sequence
                    OCR1A = max ( ((long)activePulseOff + (long) MARGINTICS - (long) nextPulseLength), ( tmpTCNT1 + MARGINTICS/2 ) );
                } else {
                    // no next pulse, there is only one pulse
                    OCR1A = activePulseOff;
                    activePulseOff = 0;
                    stopPulseIx = activePulseIx;
                    IrqType = POFF;
                }
            } else {
                // its a pulse in sequence, so this is the 'nextPulse'
                nextPulseLength = servoData[pulseIx].ist/SPEED_RES;
                nextPulseIx = pulseIx++;
                IrqType = POFF;
            }
        } else {
            // found no pulse, so the last one is running or no pulse at all
            
            if ( activePulseOff == 0 ) {
                // there wasn't any pulse, restart
                pulseIx = 0;
                nextPulseLength = 0;
                OCR1A = FIRST_PULSE;
            } else {
                // is last pulse, don't start a new one
                IrqType = POFF;
            }
        }
        CLR_TP1; CLR_TP3; // Oszimessung Dauer der ISR-Routine
       
    } //end of 'pulse ON'
    #ifdef __STM32F1__
    timer_set_compare(MT_TIMER,  SERVO_CHN, OCR1A);
    #endif 
}

#endif // VARIABLE_POSITION_SERVO_PULSES

// ------------ end of Interruptroutines ------------------------------

static void seizeTimer1()
{
# ifdef __AVR_MEGA__
    uint8_t oldSREG = SREG;
    cli();
    
    TCCR1A =0; /* CTC Mode, ICR1 is TOP */
    TCCR1B = _BV(WGM13) | _BV(WGM12) /* CTC Mode, ICR1 is TOP */
  | _BV(CS11) /* div 8 clock prescaler */
  ;
    ICR1 = 20000 * TICS_PER_MICROSECOND;  // timer periode is 20000us 
    OCR1A = FIRST_PULSE;
    OCR1B = 400;
    // Serial.print( " Timer initialized " ); Serial.println( TIMSK1, HEX );
    SREG = oldSREG;  // undo cli() 
#elif defined __STM32F1__
    timer_init( MT_TIMER );
    timer_pause(MT_TIMER);
    timer_oc_set_mode( MT_TIMER, SERVO_CHN, TIMER_OC_MODE_FROZEN, 0 );  // comparison between output compare register and counter 
                                                                //has no effect on the outputs
    timer_oc_set_mode( MT_TIMER, STEP_CHN, TIMER_OC_MODE_FROZEN, 0 );
    timer_set_prescaler(MT_TIMER, 36-1 );    // = 0.5µs Tics at 72MHz
    timer_set_reload(MT_TIMER, 20000 * TICS_PER_MICROSECOND );
    timer_set_compare(MT_TIMER, SERVO_CHN, FIRST_PULSE );
    timer_attach_interrupt(MT_TIMER, TIMER_CC1_INTERRUPT, ISR_Servo );
    timer_set_compare(MT_TIMER, STEP_CHN, 400 );
    timer_attach_interrupt(MT_TIMER, TIMER_CC2_INTERRUPT, ISR_Stepper );
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
    // use SPI 2 interface
    spi_init(SPI2);
    spi_config_gpios(SPI2, 1,  // initialize as master
                     PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_device, PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_bit,        
                     PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_device, PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MISO_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MOSI_PIN].gpio_bit);

    uint32 flags = (SPI_FRAME_MSB | SPI_CR1_DFF_16_BIT | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(SPI2, (spi_baud_rate)SPI_BAUD_PCLK_DIV_16, (spi_mode)SPI_MODE_0, flags);
    spi_irq_enable(SPI2, SPI_RXNE_INTERRUPT);
    pinMode( BOARD_SPI2_NSS_PIN, OUTPUT);
    digitalWrite( BOARD_SPI2_NSS_PIN, LOW );

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
    stepMode = NOSTEP;
    if( stepperCount < MAX_STEPPER )  {
        // create new instance
        stepperIx = stepperCount++ ;
        stepsRev = steps360;       // number of steps for full rotation in fullstep mode
        if ( mode != FULLSTEP && mode != A4988 ) mode = HALFSTEP;
        // initialize data for interrupts
        stepperData[stepperIx].stepCnt = 0;         // don't move
        stepMode = mode;
        stepperData[stepperIx].patternIx = 0;
        stepperData[stepperIx].patternIxInc = mode;    // positive direction
        minCycSteps = minStepTime*1000/CYCLETIME; // minStepTime in ms, cycletime in us
        stepperData[stepperIx].cycSteps = 2;        // set to maximum speed, 1 Step every 2 Irq's
        stepperData[stepperIx].stepsFromZero = 0;
        stepperData[stepperIx].activ = 0;
        stepperData[stepperIx].endless = 0;
        stepperData[stepperIx].output = NO_OUTPUT;          // unknown
    }
    
}
long Stepper4::getSFZ() {
    // get step-distance from zero point
    // irq must be disabled, because stepsFromZero is updated in interrupt
    long tmp;
    noInterrupts();
    tmp = stepperData[stepperIx].stepsFromZero;
    interrupts();
    return tmp / stepMode;
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
    if ( stepMode == NOSTEP ) return 0; // Invalid object
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
            stepperData[stepperIx].portPins[i].Adr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[ pins[i]])]);
            stepperData[stepperIx].portPins[i].Mask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pins[i]]);
            #else // store pins directly
            stepperData[stepperIx].pins[i] = pins[i];
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
            stepperData[stepperIx].portPins[i].Adr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[ pins[i]])]);
            stepperData[stepperIx].portPins[i].Mask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pins[i]]);
            #else // store pins directly
            stepperData[stepperIx].pins[i] = pins[i];
            #endif
            stepMode = HALFSTEP;                      // There are no real stepmodes in A4988 - mode
            stepperData[stepperIx].patternIxInc = 1;  // defines direction
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
        stepperData[stepperIx].output = outArg;
        stepperData[stepperIx].activ = 1;
        // enable compareB- interrupt
        #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
            TIMSK |= ( _BV(OCIE1B) );    // enable compare interrupts
        #elif defined __AVR_MEGA__
            TIMSK1 |= _BV(OCIE1B) ; 
        #elif defined __STM32F1__
            timer_cc_enable(MT_TIMER, STEP_CHN);
        #endif
    }
    DB_PRINT( "attach: output=%d, attachOK=%d", stepperData[stepperIx].output, attachOK );
    //Serial.print( "Attach Stepper, Ix= "); Serial.println( stepperIx );
    return attachOK;
}

void Stepper4::detach()
{   // no more moving, detach from output
    if ( stepMode == NOSTEP ) return ; // Invalid object
    
    stepperData[stepperIx].output = 0;
    stepperData[stepperIx].activ = 0;
    
}

int Stepper4::setSpeed( int rpm10 ) {
    // Set speed in rpm*10. Step time is computed internally based on CYCLETIME and
    // steps per full rotation (stepsRev)
    if ( stepMode == NOSTEP ) return 0; // Invalid object
    stepperData[stepperIx].cycSteps = (((60L*10L*1000000L / CYCLETIME )*10 / stepsRev/ rpm10) +5)/10;
    return stepperData[stepperIx].cycSteps;
    
}

void Stepper4::setZero() {
    // set reference point for absolute positioning
    if ( stepMode == NOSTEP ) return ; // Invalid object
    noInterrupts();
    stepperData[stepperIx].stepsFromZero = 0;
    interrupts();
}

void Stepper4::write(long angleArg ) {
    // set next position as angle, measured from last setZero() - point
    DB_PRINT("write: %d", angleArg);
    Stepper4::write( angleArg, 1 );
}

void Stepper4::write( long angleArg, byte fact ) {
    // for better resolution. angelArg/fact = angle in degrees
    // typical: fact = 10, angleArg in .1 degrees
    if ( stepMode == NOSTEP ) return ; // Invalid object
    bool negative;
    int angel2steps;
    negative =  ( angleArg < 0 ) ;
    DB_PRINT( "angleArg: %d",angleArg ); //DB_PRINT( " getSFZ: ", getSFZ() );
    //Serial.print( "Write: " ); Serial.println( angleArg );
    angel2steps =  ( (abs(angleArg) * (long)stepsRev*10) / ( 360L * fact) +5) /10 ;
    if ( negative ) angel2steps = -angel2steps;
    doSteps(angel2steps  - getSFZ() );
}

void Stepper4::writeSteps( long stepPos ) {
    // go to position stepPos steps away from zeropoint
    if ( stepMode == NOSTEP ) return ; // Invalid object

    doSteps(stepPos  - getSFZ() );
}

long Stepper4::read()
{   // returns actual position as degree
    if ( stepMode == NOSTEP ) return 0; // Invalid object

    long tmp = getSFZ();
    bool negative;
    negative = ( tmp < 0 );
	tmp = (abs(tmp)/stepsRev*360) + (( (abs(tmp)%stepsRev) *3600L / stepsRev ) +5) / 10;
    if ( negative ) tmp = -tmp;
    return  tmp;
}

long Stepper4::readSteps()
{   // returns actual position as steps
    if ( stepMode == NOSTEP ) return 0; // Invalid object

    return  getSFZ();
}

void Stepper4::doSteps( long stepValue ) {
    // rotate stepValue steps
    if ( stepMode == NOSTEP ) return ; // Invalid object
    //Serial.print( "doSteps: " ); Serial.println( stepValue );
    stepsToMove = stepValue;
     DB_PRINT( " stepsToMove: %d ",stepsToMove );
    if ( stepValue > 0 ) stepperData[stepperIx].patternIxInc = abs( stepperData[stepperIx].patternIxInc );
    else stepperData[stepperIx].patternIxInc = -abs( stepperData[stepperIx].patternIxInc );
    noInterrupts();
    stepperData[stepperIx].stepCnt = abs(stepsToMove);
    interrupts();
}


uint8_t Stepper4::moving() {
    // return how much still to move (percentage)
    int tmp;
    if ( stepMode == NOSTEP ) return 0; // Invalid object
    //Serial.print( stepperData[stepperIx].stepCnt ); Serial.print(" "); 
    //Serial.println( stepperData[stepperIx].cycSteps );
    if ( stepsToMove == 0 ) {
        tmp = 0;        // there was nothing to move
    } else {
        noInterrupts(); // disable interrupt, because integer stepcnt is changed in TCR interrupt
        tmp = stepperData[stepperIx].stepCnt;
        interrupts();  // undo cli() 
        if ( tmp > 0 ) {
            // do NOT return 0, even if less than 1%, because 0 means real stop of the motor
            tmp = max ( ((long)tmp * 100L / abs( stepsToMove)) , 1 );
        }
    }
    return tmp ;
}

void Stepper4::rotate(int8_t direction) {
	// rotate endless
    if ( stepMode == NOSTEP ) return; // Invalid object
    
	if (direction == 0 ) {
        // identical to 'stop'
		stop();
	} else {
		noInterrupts();
		stepperData[stepperIx].endless = true;
		stepperData[stepperIx].stepCnt = 1;
		if ( direction > 0 ) {
            stepperData[stepperIx].patternIxInc = abs( stepperData[stepperIx].patternIxInc );
            stepsToMove = 1;
         } else {
            stepperData[stepperIx].patternIxInc = -abs( stepperData[stepperIx].patternIxInc );
            stepsToMove = -1;
         }
		interrupts();
	}
}

void Stepper4::stop() {
	// immediate stop of the motor
    if ( stepMode == NOSTEP ) return ; // Invalid object
    
    noInterrupts();
    stepperData[stepperIx].endless = false;
	stepsToMove = 0;
    stepperData[stepperIx].stepCnt = 0;
    interrupts();
}
///////////////////////////////////////////////////////////////////////////////////
// --------- Class Servo8 ---------------------------------
// Class-specific Variables


#define NO_ANGLE (0xff)

Servo8::Servo8() : pin(0),angle(NO_ANGLE),min16(1000/16),max16(2000/16)
{   servoIndex = servoCount++;
    servoData[servoIndex].soll = -1;    // = not initialized
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
    if ( pin != 0 ||  servoIndex >= MAX_SERVOS ) return 0;
    // set pulselength for angle 0 and 180
    if ( pmin >= MINPULSEWIDTH && pmin <= MAXPULSEWIDTH) min16 = pmin/16;
    if ( pmax >= MINPULSEWIDTH && pmax <= MAXPULSEWIDTH ) max16 = pmax/16;
	DB_PRINT( "pin: %d, pmin:%d pmax%d autoOff=%d, min16=%d, max16=%d", pinArg, pmin, pmax, autoOff, min16, max16);
    
    // intialize objectspecific data
    lastPos = 3000*SPEED_RES ;    // initalize to middle position
    servoData[servoIndex].soll = -1;  // invalid position -> no pulse output
    servoData[servoIndex].ist = -1;   
    servoData[servoIndex].inc = 2000*SPEED_RES;  // means immediate movement
    servoData[servoIndex].pin = pinArg;
    servoData[servoIndex].on = false;  // create no pulses until next write
    servoData[servoIndex].noAutoff = autoOff?0:1 ;  
    #ifdef FAST_PORTWRT
    // compute portaddress and bitmask related to pin number
    servoData[servoIndex].portAdr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[ pinArg])]);
    servoData[servoIndex].bitMask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pinArg]);
    DB_PRINT( "Idx: %d Portadr: 0x%x, Bitmsk: 0x%x", servoIndex, servoData[servoIndex].portAdr, servoData[servoIndex].bitMask );
	DB_PRINT( "Stack=0x%04x, &sIx=0x%04x", ((SPH&0x7)<<8)|SPL, &servoIndex );
    #endif
    pin = pinArg;
    angle = NO_ANGLE;
    pinMode(pin,OUTPUT);
    digitalWrite(pin,LOW);

    if ( !timerInitialized) seizeTimer1();
    // enable compare-A interrupt
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
    TIMSK |=  _BV(OCIE1A);   
    #elif defined __AVR_MEGA__
    TIMSK1 |=  _BV(OCIE1A) ; 
    #elif defined __STM32F1__
        timer_cc_enable(MT_TIMER, SERVO_CHN);
    #endif

    return 1;
}

void Servo8::detach()
{
    servoData[servoIndex].on = false;  
    servoData[servoIndex].soll = -1;  
    servoData[servoIndex].ist = -1;  
    servoData[servoIndex].pin = 0;  
    pin = 0;
}

void Servo8::write(int angleArg)
{   // set position to move to
    // values between 0 and 180 are interpreted as degrees,
    // values between MINPULSEWIDTH and MAXPULSEWIDTH are interpreted as microseconds
    static int newpos;
	DB_PRINT( "Write: angleArg=%d, Soll=%d", angleArg, servoData[servoIndex].soll );
    if ( pin > 0 ) { // only if servo is attached
        Serial.print( "Pin:" );Serial.print(pin);Serial.print("Wert:");Serial.println(angleArg);
        #ifdef __AVR_MEGA__
		DB_PRINT( "Stack=0x%04x, &sIx=0x%04x", ((SPH&0x7)<<8)|SPL, &servoIndex );
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
        if ( servoData[servoIndex].soll < 0 ) {
            // Serial.println( "first write");
            // this is the first pulse to be created after attach
            servoData[servoIndex].on = true;
            lastPos = newpos;
            noInterrupts();
            servoData[servoIndex].soll= newpos ; // .ist - value is still -1 (invalid) -> will jump to .soll immediately
            interrupts();
			DB_PRINT( "FirstWrite: Ix=%d,%d Soll=%d", servoIndex, this->servoIndex, servoData[servoIndex].soll );
            
        }
        else if ( newpos != servoData[servoIndex].soll ) {
            // position has changed, store old position, set new position
            lastPos = servoData[servoIndex].soll;
            noInterrupts();
            servoData[servoIndex].soll= newpos ;
            interrupts();
			DB_PRINT( "NextWrite: Ix= %d,%d Soll=%d, On=%d", servoIndex, this->servoIndex, servoData[servoIndex].soll, servoData[servoIndex].on );
        }
    }
}

void Servo8::setSpeed( int speed, bool compatibility ) {
    // set global compatibility-Flag
    speedV08 = compatibility;
    setSpeed( speed );
}

void Servo8::setSpeed( int speed ) {
    // Set increment value for movement to new angle
    // ToDo: Set compatibility mode for Version 0.8 end earlier
    if ( pin > 0 ) { // only if servo is attached
        if ( speedV08 ) speed *= SPEED_RES;
        noInterrupts();
        if ( speed == 0 )
            servoData[servoIndex].inc = 2000*SPEED_RES;  // means immiediate movement
        else
            servoData[servoIndex].inc = speed;
        interrupts();
    }
}

uint8_t Servo8::read() {
    // get position in degrees
    int value;
    if ( pin == 0 ) return -1; // Servo not attached
    noInterrupts();
    value = servoData[servoIndex].ist;
    interrupts();
    return map( value/TICS_PER_MICROSECOND/SPEED_RES, min16*16, max16*16, 0, 180 );
}

int Servo8::readMicroseconds() {
    // get position in microseconds
    int value;
    if ( pin == 0 ) return -1; // Servo not attached
    noInterrupts();
    value = servoData[servoIndex].ist;
    interrupts();
    return value/TICS_PER_MICROSECOND/SPEED_RES;   

}

uint8_t Servo8::moving() {
    // return how much still to move (percentage)
    if ( pin == 0 ) return 0; // Servo not attached
    long total , remaining;
    total = abs( lastPos - servoData[servoIndex].soll );
    noInterrupts(); // disable interrupt, because integer servoData[servoIndex].ist is changed in interrupt
    remaining = abs( servoData[servoIndex].soll - servoData[servoIndex].ist );
    interrupts();  // allow interrupts again
    if ( remaining == 0 ) return 0;
    return ( remaining * 100 ) /  total +1;
}
uint8_t Servo8::attached()
{
    return ( pin > 0 );
}



/////////////////////////////////////////////////////////////////////////////
//Class SoftLed - for Led with soft on / soft off ---------------------------
// Version with Software PWM

SoftLed::SoftLed() {
    ledIx = ledCount++;
        if ( ledIx < MAX_LEDS ) {
        ledData[ledIx].speed = 0;       // defines rising/falling timer
        ledData[ledIx].aStep = 0 ;      // actual PWM step
        ledData[ledIx].state = OFF ;    // initialize to off
        ledData[ledIx].setpoint = OFF ; // initialize to off
        ledType = LINEAR;
    }
}

uint8_t SoftLed::attach(uint8_t pinArg){
    // Led-Ausgang mit Softstart. 
    if ( ledIx >= MAX_LEDS ) return false;
    pinMode( pinArg, OUTPUT );
    DB_PRINT( "Led attached, ledCount = %d", ledCount );
    ledSpeed = 1;                   // defines rising/falling timer
    ledData[ledIx].aStep = 0 ;      // actual PWM step
    ledData[ledIx].state = OFF ;    // initialize to off
    ledData[ledIx].setpoint = OFF ; // initialize to off

    #ifdef FAST_PORTWRT
    ledData[ledIx].portPin.Adr = (byte *) pgm_read_word_near(&port_to_output_PGM[pgm_read_byte_near(&digital_pin_to_port_PGM[pinArg])]);
    ledData[ledIx].portPin.Mask = pgm_read_byte_near(&digital_pin_to_bit_mask_PGM[pinArg]);
    #else
    ledData[ledIx].pin=pinArg ;      // Pin-Nbr 
    #endif
    if ( !timerInitialized ) seizeTimer1();
    // enable compareB- interrupt
    #if defined(__AVR_ATmega8__)|| defined(__AVR_ATmega128__)
        TIMSK |= ( _BV(OCIE1B) );    // enable compare interrupts
    #elif defined __AVR_MEGA__
        TIMSK1 |= _BV(OCIE1B) ; 
    #elif defined __STM32F1__
        timer_cc_enable(MT_TIMER, STEP_CHN);
    #endif
  

    return true;
}

void SoftLed::on(){
    if ( ledIx >= MAX_LEDS ) return;
    // Don't do anything if its already ON 
    if ( ledData[ledIx].setpoint != ON  ) {
        SET_TP4;
        ledData[ledIx].setpoint=ON ;
        ledData[ledIx].aStep = 0;
        ledData[ledIx].stpCnt = 0; 
        if ( ledType == LINEAR ) {
            ledData[ledIx].state = INCLIN;
            ledData[ledIx].speed = ledSpeed;
            ledData[ledIx].aCycle = 1;
        } else { // is bulb simulation
            SET_TP4;
            ledData[ledIx].state = INCFAST;
            ledData[ledIx].speed = ledSpeed==1? -1 : ledSpeed / 3;
            CLR_TP4;
            if ( ledData[ledIx].speed <= 0 ) {
                ledData[ledIx].state = INCSLOW;
                ledData[ledIx].stpCnt = 1;
            }
            ledData[ledIx].aCycle = iSteps[0];
        }
        CLR_TP4;
    }
}

void SoftLed::off(){
    if ( ledIx >= MAX_LEDS ) return;
    // Dont do anything if its already OFF 
    if ( ledData[ledIx].setpoint != OFF ) {
        SET_TP3;
        ledData[ledIx].setpoint=OFF ;
        ledData[ledIx].aStep = 0;
        ledData[ledIx].stpCnt = 0; 
        if ( ledType == LINEAR ) {
            ledData[ledIx].state = DECLIN;
            ledData[ledIx].speed = ledSpeed;
            ledData[ledIx].aCycle = LED_CYCLE_MAX-1;
        } else { // is bulb simulation
            CLR_TP3;
            ledData[ledIx].state = DECFAST;
            ledData[ledIx].speed = ledSpeed==1? -1 : ledSpeed / 3;
            SET_TP3;
            if ( ledData[ledIx].speed <= 0 ) {
                ledData[ledIx].state = DECSLOW;
                ledData[ledIx].stpCnt = 1;
            }
            ledData[ledIx].aCycle = LED_CYCLE_MAX + 1 - iSteps[0];
        }
        CLR_TP3;
    }
}

void SoftLed::toggle( void ) {
    if ( ledIx >= MAX_LEDS ) return;
    if ( ledData[ledIx].setpoint == ON  ) off();
    else on();
}

void SoftLed::write( uint8_t setpoint, uint8_t ledPar ){
    ledType = ledPar;
    write( setpoint ) ;
}

void SoftLed::write( uint8_t setpoint ){
    if ( ledIx >= MAX_LEDS ) return;
    if ( setpoint == ON ) on(); else off();
    #ifdef debug1
    // im Debugmode hier die Led-Daten ausgeben
    DB_PRINT( "LedData[%d]\n\speed=%d, Type=%d, aStep=%d, stpCnt=%d, state=%d, setpoint= %d",
            ledIx, ledSpeed, ledType, ledData[ledIx].aStep, ledData[ledIx].stpCnt, ledData[ledIx].state
                    , ledData[ledIx].setpoint);
    //DB_PRINT( "ON=%d, NextCyc=%d, CycleCnt=%d, StepIx=%d, NextStep=%d", 
    //         ON, ledNextCyc, ledCycleCnt, ledStepIx, ledNextStep);
    #endif
}

void SoftLed::riseTime( int riseTime ) {
    if ( ledIx >= MAX_LEDS ) return;
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
    DB_PRINT( "ledSpeed[%d] = %d ( risetime=%d, riseMax=%d )", ledIx, ledSpeed, riseTime, riseMax );
}

////////////////////////////////////////////////////////////////////////////
// Class EggTimer - Timerverwaltung für Zeitverzögerungen in der Loop-Schleife

void EggTimer::setTime(  long wert ) {
    timervalue = millis() + wert;
}

bool EggTimer::running() {
    return ( timervalue >= millis() );
}

EggTimer::EggTimer()
{
    timervalue = millis();
}


