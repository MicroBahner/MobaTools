/*
  MobaTools.h - a library for model railroaders
  Author: fpm, fpm@mnet-mail.de
  Copyright (c) 2019 All right reserved.

  Functions for the stepper part of MobaTools
*/
#include <MobaTools.h>
#include <avr/interrupt.h>
#include <Arduino.h>

// Global Data for all instances and classes  --------------------------------
extern uint8_t timerInitialized;
static uint8_t spiInitialized = false;

// constants
static const int stepPattern[8] = {0b0011, 0b0010, 0b0110, 0b0100, 0b1100, 0b1000, 0b1001,0b0001 };

// Variables for stepper motors
static stepperData_t *stepperRootP = NULL;    // start of stepper data chain ( NULL if no stepper object )
static uint8_t spiData[2]; // step pattern to be output on SPI
                            // low nibble of spiData[0] is SPI_1
                            // high nibble of spiData[1] is SPI_4
                            // spiData[1] is shifted out first
static uint8_t spiByteCount = 0;
static byte stepperCount = 0;

//==========================================================================

// global functions / Interrupts


#pragma GCC optimize "O3"   // optimize ISR for speed
void stepperISR(uint8_t cyclesLastIRQ) {
    SET_TP4;
    stepperData_t *stepperDataP;         // actual stepper data in IRQ
    uint8_t i, spiChanged, changedPins, bitNr;
    uint16_t tmp;
    SET_TP1;SET_TP4; // Oszimessung Dauer der ISR-Routine
    spiChanged = false;
    #ifdef __AVR_MEGA__
    interrupts(); // allow nested interrupts, because this IRQ may take long
    #endif
    stepperDataP = stepperRootP;
    // ---------------Stepper motors ---------------------------------------------
    while ( stepperDataP != NULL ) {
        CLR_TP1;    // spike for recognizing start of each stepper
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
                if ( _patIx < 0 ) _patIx += 8;
                stepperDataP->patternIx = _patIx;
                CLR_TP2;SET_TP2;
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
                    nextCycle = 1; // will be resettet in next cycle
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
                    // get new values for speed an ramp ( may be that these are the same as actual )
                    stepperDataP->tCycSteps = stepperDataP->tCycSteps2 ;
                    stepperDataP->tCycRemain = stepperDataP->tCycRemain2 ;
                    stepperDataP->stepRampLen = stepperDataP->stepRampLen2;
                    stepperDataP->cyctXramplen = stepperDataP->cyctXramplen2;
                    if (stepperDataP->stepCnt2 > 0 ) { // check if we have to start a movement backwards
                        // yes, change Direction and go stpCnt2 Steps
                        stepperDataP->patternIxInc = -stepperDataP->patternIxInc;
                        stepperDataP->stepCnt = stepperDataP->stepCnt2;
                        stepperDataP->stepCnt2 = 0;
                        stepperDataP->rampState = RAMPACCEL;
                    } else {    
                        stepperDataP->aCycSteps = TIMERPERIODE;    // no more Interrupts for this stepper needed
                        stepperDataP->rampState = STOPPED;
                        //CLR_TP2;
                    }
                }
                // --------------- compute nexte steplength ------------------------------------
                SET_TP2;
                // ramp state machine
                switch ( stepperDataP->rampState ) {
                  case  RAMPACCEL:
                    // we are accelerating the motor
                    if (stepperDataP->stepsInRamp > stepperDataP->stepRampLen ) {
                        // we reached the end of the ramp
                        stepperDataP->aCycSteps = stepperDataP->tCycSteps;
                        stepperDataP->aCycRemain = stepperDataP->tCycRemain;
                        stepperDataP->rampState = CRUISING;
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
                        stepperDataP->rampState = STARTDECEL;
                        //DB_PRINT( "scnt=%ld, sIR=%u\n\r", stepperDataP->stepCnt, stepperDataP->stepsInRamp );
                        //SET_TP2;
                    } else {
                        // still in ramp
                        stepperDataP->stepsInRamp ++;
                    }    
                    break;
                  case STARTDECEL:
                    //SET_TP3;
                    stepperDataP->rampState = RAMPDECEL;
                    stepperDataP->stepsInRamp = stepperDataP->stepCnt;
                    //CLR_TP3;
                  case RAMPDECEL:
                    // we are stopping the motor
                    if ( stepperDataP->stepCnt > (long)( stepperDataP->stepsInRamp ) ) {
                        //CLR_TP2;
                        //steps to move has changed, accelerate again with next step
                        stepperDataP->rampState = RAMPACCEL;
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
                
                  case SPEEDDECEL:
                    // lower speed to new value 
                    stepperDataP->aCycSteps = stepperDataP->cyctXramplen / ( --stepperDataP->stepsInRamp + RAMPOFFSET ) ;//+1 ;
                        stepperDataP->aCycRemain += stepperDataP->cyctXramplen % (stepperDataP->stepsInRamp + RAMPOFFSET);
                        if ( stepperDataP->aCycRemain > (stepperDataP->stepsInRamp + RAMPOFFSET) ) {
                            stepperDataP->aCycSteps++;
                            stepperDataP->aCycRemain -= (stepperDataP->stepsInRamp + RAMPOFFSET);
                        }
                    //if ( stepperDataP->aCycSteps >= stepperDataP->tCycSteps2 || stepperDataP->stepsInRamp == 0 ) {
                    if (  stepperDataP->stepsInRamp <=  stepperDataP->stepsInRampStop ) {
                        // new targestspeed reached
                        //SET_TP3;
                        stepperDataP->tCycSteps = stepperDataP->tCycSteps2;
                        stepperDataP->tCycRemain = stepperDataP->tCycRemain2;
                        stepperDataP->aCycRemain = stepperDataP->tCycRemain2;
                        stepperDataP->cyctXramplen = stepperDataP->cyctXramplen2;
                        stepperDataP->stepRampLen = stepperDataP->stepRampLen2;
                        stepperDataP->rampState = CRUISING;
                        //CLR_TP3;
                    }
                    //ToDo - do we have to stop the motor
                    break;
                    
                  case CRUISING:
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
                        stepperDataP->rampState = STARTDECEL;
                    }
                    
                    break;
                    
                  case STOPPED:
                    //stopped
                    //CLR_TP2;
                    break;
                } // End of ramp-statemachine
                CLR_TP2;
            } // End of do one step
            nextCycle = min ( nextCycle, stepperDataP->aCycSteps-stepperDataP->cycCnt );
            //SET_TP1;
        } // end of 'if stepper active AND moving'
        //CLR_TP1;
        stepperDataP = stepperDataP->nextStepperDataP;
        SET_TP1; //CLR_TP3;
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
    CLR_TP1;
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
    static int rxData;
    rxData = spi_rx_reg(SPI1);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI1_NSS_PIN,HIGH);
}
    #endif
#endif
} // end of extern "C"

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
    _stepperData.rampState = INACTIVE;
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
    return ( _stepperData.rampState != STOPPED && _stepperData.rampState != INACTIVE );
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
        _stepperData.rampState = STOPPED;
        setSpeedSteps( DEF_SPEEDSTEPS, DEF_RAMP );
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
    return setSpeedSteps( min( 1000000L / MIN_STEPTIME * 10, (long)rpm10 * stepsRev / 60 ) ) ;
}

uint16_t Stepper4::setSpeedSteps( uint16_t speed10 ) {
    // Speed in steps per sec * 10
    // without a new ramplen, the ramplen is adjusted according to the speedchange
    uint16_t stepRampLen =_stepperData.stepRampLen;
    //DB_PRINT("sSS1: sRL=%u, spd=%u", stepRampLen, speed10 );
    if ( stepRampLen == 0  ) stepRampLen = 1;
    stepRampLen = (long)speed10*stepRampLen/_stepSpeed10;
    //DB_PRINT("sSS2: sRL=%u, spd=%u", stepRampLen, _stepSpeed10 );
    return setSpeedSteps( speed10, stepRampLen );
    // Rampenlänge in Steps ( entsprechend aktueller Beschleunigung )
    //_stepperData.stepsToStop = (long)_stepSpeed10 * _stepSpeed10 / 200 / _stepAccel ;
}

uint16_t Stepper4::setRampLen( uint16_t rampSteps ) {
    // set length of ramp ( from stop to actual target speed ) in steps
    return setSpeedSteps( _stepSpeed10, rampSteps );
}

uint16_t Stepper4::setSpeedSteps( uint16_t speed10, int16_t rampLen ) {
    // Set speed and length of ramp to reach speed ( from stop )
    rampStats_t rampState;      // State of acceleration/deceleration
    uint16_t tCycSteps;         // nbr of IRQ cycles per step ( new target value of motorspeed  )
    uint16_t tCycRemain;        // Remainder of division when computing tCycSteps
    long     tMicroSteps;       // Microseconds per step
    //uint16_t aCycSteps;         // nbr of IRQ cycles per step ( actual motorspeed  )
    //uint16_t aCycRemain;        // Remainder of division when computing aCycSteps
    uint16_t cyctXramplen;      // precompiled  tCycSteps*rampLen*RAMPOFFSET
    int16_t stepRampLen;        // new ramplen (may be changed to old ramp length)
    int16_t newRampLen;         // new ramplen
    int16_t stepsInRamp;        // actual stepcounter in ramp - may be recomputed if speed or
                                // ramplen is changed on the fly
    uint16_t oldSpeed10;        // old target speed
    uint16_t actSpeed10;        // actual speed
    bool  storeFlg = true;      // store new values immediately
    uint16_t fullRampLen;

    if ( _stepperData.output == NO_OUTPUT ) return 0; // --------------->>>>>>>>>>>>>>>>not attached
    
    // compute new speed values, adjust length of ramp if necessary
    actSpeed10 = oldSpeed10 = _stepSpeed10;
    newRampLen = abs(rampLen);    // negative values are invalid
    if (newRampLen > MAXRAMPLEN ) newRampLen = MAXRAMPLEN;
    _stepSpeed10 = min( 1000000L / MIN_STEPTIME * 10, speed10 );
    DB_PRINT( "rampLen-new=%u, ramplenParam=%u", newRampLen, rampLen );
    // compute target steplength and check whether speed and ramp fit together: 
    tMicroSteps = ( 1000000L * 10  / _stepSpeed10 );
    tCycSteps = tMicroSteps / CYCLETIME; 
    tCycRemain = tMicroSteps % CYCLETIME; 
    // tcyc * (rapmlen+RAMPOFFSET) must be less then 65000, otherwise ramplen is adjusted accordingly
    long tmp =  tMicroSteps * ( newRampLen + RAMPOFFSET ) / CYCLETIME ;
    if ( tmp > 65000L ) {
        // adjust ramplen
        newRampLen = 65000L * CYCLETIME / tMicroSteps - RAMPOFFSET;
        if( newRampLen < 0 ) newRampLen = 0;
        cyctXramplen = tMicroSteps * ( newRampLen + RAMPOFFSET ) / CYCLETIME;
    } else {
        cyctXramplen = tmp;
    }
    
    // recompute all relevant rampvalues according to actual speed and ramplength
    // This needs to be done only, if a ramp is defined, the stepper is moving
    // and the speed an ramp values changed
    // In all other cases the new speed/ramp values will get active immediately
    DB_PRINT( "actRampLen=%u, cXr-new=%u, xCr-old=%u", newRampLen, cyctXramplen, _stepperData.cyctXramplen );
    _noStepIRQ(); SET_TP2;
    if ( (_stepperData.stepRampLen + newRampLen ) != 0
        && _stepperData.rampState != INACTIVE
        && _stepperData.rampState != STOPPED
        &&  cyctXramplen != _stepperData.cyctXramplen ) {
        // local variables to hold data that might change in IRQ:
        // If there was a step during recomputing the rampvalues, we must recompute again
        // recomputing the rampvalues lasts too long to stop the IRQ during the whole time
        long        __stepCnt;
        uint16_t    __tCycSteps;
        uint16_t    __tCycRemain;
        rampStats_t __rampState;
        uint16_t    __stepsInRamp;
        long        __newStepCnt;
        long        __newStepCnt2;
        
        DB_PRINT("Speed changed! New: tcyc=%u, ramp=%u, cXr=%u",tCycSteps,newRampLen,cyctXramplen );
         do {
            // read actual ISR values
            __tCycSteps     =  _stepperData.tCycSteps;
            __tCycRemain    =  _stepperData.tCycRemain;
            __rampState     =  _stepperData.rampState;
            __stepsInRamp   =  _stepperData.stepsInRamp;
            __stepCnt       =   _stepperData.stepCnt;
            __newStepCnt    = 0;    // if stepcnt is to be changed
            __newStepCnt2   = 0;
            _stepIRQ(); CLR_TP2;
            // if we are in a ramp, and speed changed several times during this ramp, than oldSpeed10 is not
            // the targetspeed still used in ISR. We must compute oldSpeed 10 from actual tCyc values.
            oldSpeed10 = 1000000L * 10 / ((long)__tCycSteps * CYCLETIME + __tCycRemain );
            DB_PRINT("OldSpeed = %u", oldSpeed10);
            //with ramp and ramp or speed changed 
            // compute actual speed. If we are cruising ( constant speed ) this is the old target speed
            // if we are in a ramp this is the speed according to actual stepsInRamp (Glchg 14)
            if ( __rampState != CRUISING ) {
                actSpeed10 = (long) oldSpeed10 * (__stepsInRamp + RAMPOFFSET ) / (_stepperData.stepRampLen + RAMPOFFSET );
            }
            DB_PRINT("   Act: State=%s, aSpd10=%u, tcyc=%u, ramp=%u", rsC[__rampState], actSpeed10, __tCycSteps,_stepperData.stepRampLen );

            if ( actSpeed10 > _stepSpeed10 ) {
                //  ==========  speed changed to slower ============================
                DB_PRINT ( "Slower: %u/%u -> %u/%u", actSpeed10,_stepperData.stepRampLen,  _stepSpeed10, newRampLen );
                switch ( __rampState ) {
                  case RAMPSTART:
                  case RAMPACCEL: //
                    // we are accelerating,  go to decelerating
                    storeFlg = false;
                    // compute when to stop decelerationg ramp ( when new speed is reched ) (Glchg 13 )
                    _stepperData.stepsInRampStop =  (long)_stepSpeed10 * ( _stepperData.stepRampLen+RAMPOFFSET ) / oldSpeed10;
                    if ( _stepperData.stepsInRampStop < RAMPOFFSET ) _stepperData.stepsInRampStop = 0; else _stepperData.stepsInRampStop -= RAMPOFFSET;
                    __rampState = SPEEDDECEL;
                    DB_PRINT("State->%s, Stopsteps=%u, actStep=%u",rsC[__rampState], _stepperData.stepsInRampStop, __stepsInRamp );
                    break;
                  case CRUISING:  //
                    //SET_TP3;
                    // speed changed to slower, compute values for speedchange-ramp
                    // ramp is based an actual speed/rampdata
                    // new values will be attached in ISR after new speed is reached
                    storeFlg = false;
                    __stepsInRamp = _stepperData.stepRampLen;
                     // (Glchg 13 )
                    _stepperData.stepsInRampStop =  (long)_stepSpeed10 * ( _stepperData.stepRampLen+RAMPOFFSET ) / oldSpeed10;
                    if ( _stepperData.stepsInRampStop < RAMPOFFSET ) _stepperData.stepsInRampStop = 0; else _stepperData.stepsInRampStop -= RAMPOFFSET;
                    __rampState = SPEEDDECEL;
                    // Check whether we can reach targetposition with new values
                    // compute complete ramplength until stop under new conditions ( decelerate to new speed and than to stop ):
                    fullRampLen = __stepsInRamp - _stepperData.stepsInRampStop + _stepperData.stepRampLen2 ;
                    if ( fullRampLen > __stepCnt ) {
                        // we cannot reach the tagetposition, so we go beyond the targetposition and than back.
                        __newStepCnt2 = fullRampLen -__stepCnt;
                        __newStepCnt = fullRampLen;
                    }
                        
                    //CLR_TP3;
                    break;
                  case SPEEDDECEL:
                     // we are already decelerating to new speed, new values will get valid after stop or reaching new targetspeed    
                    // we must set a new stoppoint (Glchg 10)
                    _stepperData.stepsInRampStop =  ( ( ( (long)__tCycSteps * CYCLETIME + __tCycRemain ) * ( _stepperData.stepRampLen+RAMPOFFSET ) ) / tMicroSteps ) - RAMPOFFSET;
                    //_stepperData.stepsInRampStop =  (long)_stepSpeed10 * ( _stepperData.stepRampLen+RAMPOFFSET ) / oldSpeed10;
                    if ( _stepperData.stepsInRampStop < RAMPOFFSET ) _stepperData.stepsInRampStop = 0; else _stepperData.stepsInRampStop -= RAMPOFFSET;
                    DB_PRINT("State->%s, Stopsteps=%u, actStep=%u",rsC[__rampState], _stepperData.stepsInRampStop, __stepsInRamp );
                    storeFlg = false;
                   break;
                  case STARTDECEL: //
                  case RAMPDECEL:
                    // we are already decelerating to stoppoint, new values will get valid after stop 
                    storeFlg = false;
                    break;
                  default:
                    ;
                } // end switch rampstate ( to lower speed )
            } else if ( actSpeed10 < _stepSpeed10 ) {
                //  ==========  speed changed to faster ============================
                DB_PRINT ( "Faster: %u/%u -> %u/%u", actSpeed10,_stepperData.stepRampLen,  _stepSpeed10, newRampLen );
                switch ( __rampState ) {
                  case RAMPSTART:
                  case RAMPACCEL: //
                    // we are accelerating, accelerate to new target speed (based on new ramp )
                    // compute actual steps in ramp, based on new ramp data (Glchg 13)
                    __stepsInRamp = ( (long)cyctXramplen * (__stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
                    if ( __stepsInRamp < RAMPOFFSET ) __stepsInRamp = 0; else __stepsInRamp -= RAMPOFFSET;
                    __rampState = RAMPACCEL;
                    break;
                  case CRUISING:  //
                    SET_TP3;
                     // speed changed to faster
                    // ramp is based an new speed/rampdata
                    __stepsInRamp =  (  tMicroSteps  * ( newRampLen+RAMPOFFSET ) ) / ((long)__tCycSteps * CYCLETIME + __tCycRemain );
                    if ( __stepsInRamp < RAMPOFFSET ) __stepsInRamp = 0; else __stepsInRamp -= RAMPOFFSET;
                    _stepperData.aCycRemain = 0;
                    if ( __stepsInRamp >= __stepCnt ) {
                        // targetposition is within ramplen, we cannot accelerate, but have to stop
                        __newStepCnt2 = __stepsInRamp - __stepCnt;
                        __newStepCnt = __stepsInRamp;
                        __rampState = RAMPDECEL;
                    } else { 
                        __rampState = RAMPACCEL;
                    }
                        
                    CLR_TP3;
                    break;
                  case SPEEDDECEL:
                    // accelerate again, ramp is based an new speed/rampdata (Glchg 12)
                    __stepsInRamp = ( (long)cyctXramplen * (__stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
                    if ( __stepsInRamp < RAMPOFFSET ) __stepsInRamp = 0; else __stepsInRamp -= RAMPOFFSET;
                    __rampState = RAMPACCEL;
                    break;
                  case STARTDECEL: //
                  case RAMPDECEL:
                    // we are already decelerating to stoppoint, new values will get valid after stop or reaching new targetspeed
                    // if ramp gets shorter we can accelerate again, and then decelerate faster to the stoppoint
                    if ( _stepperData.stepRampLen > newRampLen ) {
                        // new ramp is shorter than actual ramp (Glchg 12)
                        __stepsInRamp = ( (long)cyctXramplen * (__stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
                        if ( __stepsInRamp < RAMPOFFSET ) __stepsInRamp = 0; else __stepsInRamp -= RAMPOFFSET;
                        __rampState = RAMPACCEL;
                    } else {
                        storeFlg = false;
                    }
                    break;
                  default:
                    ;
                }
            } else {
                //  ==========  speed did not change ( only ramp length ) ============================
                DB_PRINT ( "NoChange: %u/%u -> %u/%u", actSpeed10,_stepperData.stepRampLen,  _stepSpeed10, newRampLen );
                switch ( __rampState ) {
                   case RAMPSTART:
                  case RAMPACCEL: //
                    // we are accelerating, use new rampvalues to accelerate
                    // compute actual steps in ramp, based on new ramp data (Glchg 13)
                    __stepsInRamp = ( (long)cyctXramplen * (__stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
                    if ( __stepsInRamp < RAMPOFFSET ) __stepsInRamp = 0; else __stepsInRamp -= RAMPOFFSET;
                    break;
                  case CRUISING:  //
                     // only ramplen changed. See if we can reach targetposition within new ramp
                    if ( newRampLen > __stepCnt ) {
                        // we cannot reach the tagetposition, start decelerating, and after ramp go backward to target.
                        __newStepCnt2 = newRampLen - __stepCnt;
                        __newStepCnt = newRampLen;
                    }
                    break;
                  case SPEEDDECEL:
                        // we are already decelerating to a new speed, new ramp values will get valid after stop or reaching new targetspeed    
                        // we must set a new stoppoint
                        //_stepperData.stepsInRampStop =  ( ( ( (long)__tCycSteps * CYCLETIME + __tCycRemain ) * ( _stepperData.stepRampLen+RAMPOFFSET ) ) / tMicroSteps ) - RAMPOFFSET;
                        storeFlg = false;
                    break;
                  case STARTDECEL: //
                  case RAMPDECEL:
                    // we are already decelerating to stoppoint, new values will get valid after stop or reaching new targetspeed
                    // if ramp gets shorter we can accelerate again, and then decelerate faster to the stoppoint
                    if ( _stepperData.stepRampLen > newRampLen ) {
                        // new ramp is shorter than actual ramp (Glchg 12)
                        __stepsInRamp = ( (long)cyctXramplen * (__stepsInRamp + RAMPOFFSET ) / _stepperData.cyctXramplen );
                        if ( __stepsInRamp < RAMPOFFSET ) __stepsInRamp = 0; else __stepsInRamp -= RAMPOFFSET;
                        __rampState = RAMPACCEL;
                    } else {
                        storeFlg = false;
                    }
                    break;
                  default:
                    ;
                }
            }
            _noStepIRQ(); SET_TP2;
        } while ( __stepCnt != _stepperData.stepCnt ); // if there was a step during computing, do it again
        _stepperData.rampState = __rampState;
        _stepperData.stepsInRamp = __stepsInRamp;
        if ( __newStepCnt != 0 ) {
            _stepperData.stepCnt = __newStepCnt;
            _stepperData.stepCnt2 = __newStepCnt2;
        }
    }
    // Store new values in backup-memory to be activated in ISR after stop or when new targetspeed is reached.
    // depending on the actual stepper-state, these values are also activated immediately ( if storeFlg is set 'true' ) )
    _stepperData.tCycSteps2 = tCycSteps;
    _stepperData.tCycRemain2 = tCycRemain;
    _stepperData.cyctXramplen2 = cyctXramplen;
    _stepperData.stepRampLen2 = newRampLen;
    
    if ( storeFlg ) {
        _stepperData.tCycSteps = tCycSteps;
        _stepperData.tCycRemain = tCycRemain;
        _stepperData.cyctXramplen = cyctXramplen;
        _stepperData.stepRampLen = newRampLen;
    }
    _stepIRQ(); CLR_TP2;
    
    DB_PRINT( "RampValues:, Spd=%u, rmpLen=%u, tcyc=%u, trest=%u, acyc=%u", _stepSpeed10, _stepperData.stepRampLen,
                    _stepperData.tCycSteps, _stepperData.tCycRemain, _stepperData.aCycSteps );
    DB_PRINT( "   - State=%s, Rampsteps=%u" , rsC[_stepperData.rampState], _stepperData.stepsInRamp );
    DB_PRINT( "   - tcyc2=%u, cXr2=%u, rampLen2=%u" , _stepperData.tCycSteps2, _stepperData.cyctXramplen2,  _stepperData.stepRampLen2 );
    return _stepperData.stepRampLen2;
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
                  case SPEEDDECEL:
                    // targetposition within stopramp?
                    _noStepIRQ();
                    if ( stepCnt <= _stepperData.stepsInRamp ) {
                        _stepperData.stepCnt2 = _stepperData.stepsInRamp-stepCnt;
                        _stepperData.rampState = RAMPDECEL;
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
                default:
                    DB_PRINT( "StateErr1:, sCnt=%ld, sCnt2=%ld, sMove=%ld, aCyc=%d", _stepperData.stepCnt, _stepperData.stepCnt2, stepsToMove, _stepperData.aCycSteps );

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
                  case SPEEDDECEL:
                    // we ar already decelerating to new speed, go till stop
                    _stepperData.stepCnt = _stepperData.stepsInRamp;
                    _stepperData.rampState = RAMPDECEL;
                    break;
                  case CRUISING:
                    _stepperData.stepCnt = _stepperData.stepRampLen;
                    break;
                default:
                    DB_PRINT( "StateErr2:, sCnt=%ld, sCnt2=%ld, sMove=%ld, aCyc=%d", _stepperData.stepCnt, _stepperData.stepCnt2, stepsToMove, _stepperData.aCycSteps );
                }
                _stepperData.stepCnt2 = _stepperData.stepCnt+stepCnt;
                _stepIRQ();
                DB_PRINT( "Dir-Change:, sCnt=%ld, sCnt2=%ld, sMove=%ld, aCyc=%d", _stepperData.stepCnt, _stepperData.stepCnt2, stepsToMove, _stepperData.aCycSteps );

               
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
                DB_PRINT("New Move: Steps:%ld", stepValue );
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
        DB_PRINT( "NoRamp:, sCnt=%ld, sCnt2=%ld, sMove=%ld, aCyc=%d", _stepperData.stepCnt, _stepperData.stepCnt2, stepsToMove, _stepperData.aCycSteps );

    }
    
    DB_PRINT( "StepValues:, sCnt=%ld, sCnt2=%ld, sMove=%ld, aCyc=%d", _stepperData.stepCnt, _stepperData.stepCnt2, stepsToMove, _stepperData.aCycSteps );
    DB_PRINT( "RampValues:, Spd=%u, rmpLen=%u, tcyc=%u, trest=%u, acyc=%u", _stepSpeed10, _stepperData.stepRampLen,
                    _stepperData.tCycSteps, _stepperData.tCycRemain, _stepperData.aCycSteps );
    DB_PRINT( "   - State=%s, Rampsteps=%u" , rsC[_stepperData.rampState], _stepperData.stepsInRamp );
    DB_PRINT( "   - tcyc2=%u, cXr2=%u, rampLen2=%u" , _stepperData.tCycSteps2, _stepperData.cyctXramplen2,  _stepperData.stepRampLen2 );
    
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
        _noStepIRQ(); // disable Stepper interrupt, because (long)stepcnt is changed in TCR interrupt
        tmp = _stepperData.stepCnt;
        _stepIRQ();  // enable stepper IRQ
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
    
    _noStepIRQ();
	stepsToMove = 0;
    _stepperData.rampState = STOPPED;
    _stepperData.stepCnt = 0;
    _stepIRQ();
}
