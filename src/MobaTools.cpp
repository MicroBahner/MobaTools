
/*
  MobaTools V1.0
   (C) 11-2017 fpm fpm@mnet-online.de
   
  History:
  V1.1 06-2019
        stepper now supports ramps (accelerating, decelerating )
        stepper speed has better resolution with high steprates
        optimized flash usage when not all functionality is used
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

#include <MoToBase.h>
#ifdef debugPrint
     const char *rsC[] = { "INACTIVE", "STOPPED", "RAMPSTART", "RAMPACCEL", "CRUISING", "STARTDECEL", "RAMPDECEL", "SPEEDDECEL" };    
#endif
void stepperISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));
void softledISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));

uint8_t timerInitialized = false;
uint8_t nextCycle;
static uint8_t cyclesLastIRQ = 1;  // cycles since last IRQ

// ---------- OCRxB Compare Interrupt used for stepper motor and Softleds ----------------
#ifdef __AVR_MEGA__
ISR ( TIMERx_COMPB_vect) {
#elif defined __STM32F1__
void ISR_Stepper(void) {
#endif
  // Timer1 Compare B, used for stepper motor, starts every CYCLETIME us
    // 26-09-15 An Interrupt is only created at timeslices, where data is to output
    uint16_t tmp;
    nextCycle = TIMERPERIODE  / CYCLETIME ;// min ist one cycle per Timeroverflow
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
    if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
   
    cyclesLastIRQ = nextCycle;
    // set compareregister to next interrupt time;
     CLR_TP1;
    // compute next IRQ-Time in us, not in tics, so we don't need long
    #ifdef __AVR_MEGA__
     noInterrupts(); // when manipulating 16bit Timerregisters IRQ must be disabled
    if ( nextCycle == 1 )  {
        // this is timecritical: Was the ISR running longer then CYCELTIME?
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
    long tmpL = ( timer_get_compare(MT_TIMER, STEP_CHN) + nextCycle * CYCLETICS );
    if ( tmpL > TIMER_OVL_TICS ) tmpL = tmpL - TIMER_OVL_TICS;
    timer_set_compare( MT_TIMER, STEP_CHN, tmpL ) ;
    #endif
    SET_TP1;
    CLR_TP1; CLR_TP4; // Oszimessung Dauer der ISR-Routine
    TOG_TP4;
}

void seizeTimer1() {
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
    timer_attach_interrupt(MT_TIMER, TIMER_STEPCH_IRQ, (voidFuncPtr)ISR_Stepper );
    timer_set_compare(MT_TIMER, SERVO_CHN, FIRST_PULSE );
    timer_attach_interrupt(MT_TIMER, TIMER_SERVOCH_IRQ, ISR_Servo );
    timer_resume(MT_TIMER);
#endif
    timerInitialized = true;  
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
}

// ========================= Class Definitions ============================================


