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

///////////////////////  Interrupt for Servos ////////////////////////////////////////////////////////////////////
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
        //CLR_TP1; // Oszimessung Dauer der ISR-Routine OFF
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
                //CLR_TP1;
                #ifdef FAST_PORTWRT
                *pulseP->portAdr |= pulseP->bitMask;
                #else
                digitalWrite( pulseP->pin, HIGH );
                #endif
                //SET_TP1;
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
// 6.6.19 Because stepper IRQ now can last very long, it is disabled during servo IRQ
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

// ---------- OCRxA Compare Interrupt used for servo motor (overlapping pulses) ----------------
#ifdef __AVR_MEGA__
ISR ( TIMERx_COMPA_vect) {
#elif defined __STM32F1__
void ISR_Servo( void) {
    uint16_t OCRxA;
#endif
    SET_SV3;
    // Timer1 Compare A, used for servo motor
    if ( IrqType == POFF ) { // Pulse OFF time
        //SET_TP1; // Oszimessung Dauer der ISR-Routine OFF
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
            #ifdef __AVR_MEGA__
            _noStepIRQ();   // Stepper IRQ may be too long and must not interrupt the servo IRQ
            interrupts();
            #endif
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
        //CLR_TP1; // Oszimessung Dauer der ISR-Routine OFF
    } else { // Pulse ON - time
        //SET_TP2; // Oszimessung Dauer der ISR-Routine ON
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
            #ifdef __AVR_MEGA__
            _noStepIRQ(); // Stepper ISR may be too long  and must not interrupt the servo IRQ
            interrupts(); // the following isn't time critical, so allow nested interrupts
            #endif
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
                TOG_TP2;
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
                #ifdef __AVR_MEGA__
                _noStepIRQ(); // Stepper ISR may be too long  and must not interrupt the servo IRQ
                interrupts(); // the following isn't time critical, so allow nested interrupts
                #endif
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
                    OCRxA = max ( ((long)activePulseOff + (long) MARGINTICS - (long) nextPulseLength), ( (long)tmpTCNT1 + MARGINTICS/2 ) );
                } else {
                    // no next pulse, there is only one pulse
                    OCRxA = activePulseOff;
                    activePulseOff = 0;
                    stopPulseP = activePulseP;
                    IrqType = POFF;
                }
                TOG_TP2;
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
    #ifdef __AVR_MEGA__
    _stepIRQ(); // allow Stepper IRQ again
    #endif
    CLR_SV3;
}

#endif // VARIABLE_POSITION_SERVO_PULSES

// ------------ end of Interruptroutines ------------------------------
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

