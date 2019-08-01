
/*
  MobaTools V1.x
   (C) 11-2017 fpm fpm@mnet-online.de
   
        
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

#include <MobaTools.h>

uint8_t timerInitialized = false;
void ISR_Stepper(void);     // defined in MoToISR.cpp


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
    timer_resume(MT_TIMER);
#endif
    timerInitialized = true;  
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
}

// ========================= Class Definitions ============================================
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



