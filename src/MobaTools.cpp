
/*
  MobaTools V2.x
   (C) 2020fpm fpm@mnet-online.de
   
        
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
#define debugTP
#define debugPrint
#include <utilities/MoToDbg.h>

#if 0 //def __AVR_MEGA__ ///////////////////////////////////////// AVR-Mega /////////////////////////////////////////////////////
bool timerInitialized = false;
void ISR_Stepper(void);     // defined in MoToISR.cpp
void seizeTimer1() {
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
    timerInitialized = true;  
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
    DB_PRINT("Testpins initialisiert");
}
#endif //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ AVR Mega ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#if 0//def __STM32F1__ //////////////////////////////////7777777/ STM32F1 //////////////////////////////////////////////////
bool timerInitialized = false;
void ISR_Stepper(void);     // defined in MoToISR.cpp
void seizeTimer1() {
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
    timerInitialized = true;  
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
    DB_PRINT("Testpins initialisiert");
}
#endif//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ STM32F1 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

#if 0 //def ESP8266 /////////////////////////////////////////////  ESP8266 /////////////////////////////////////////////////////
// definition of gpio ISR's ( there is one ISR entrypoint per gpio )
// at max 10 gpio's can be used at an ESP12: gpio 0,1,2,3,4,5,12,13,14,15
// gpio 6-11 is used for flash
// gpio16 has no interruptcapability ( but can be used as dir- or enable-pin for a stepper)
void ICACHE_RAM_ATTR gpioISR0() { gpioTab[0].MoToISR( gpioTab[0].IsrData ); }
void ICACHE_RAM_ATTR gpioISR1() { gpioTab[1].MoToISR( gpioTab[1].IsrData ); }
void ICACHE_RAM_ATTR gpioISR2() { gpioTab[2].MoToISR( gpioTab[2].IsrData ); }
void ICACHE_RAM_ATTR gpioISR3() { gpioTab[3].MoToISR( gpioTab[3].IsrData ); }
void ICACHE_RAM_ATTR gpioISR4() { gpioTab[4].MoToISR( gpioTab[4].IsrData ); }
void ICACHE_RAM_ATTR gpioISR5() { gpioTab[5].MoToISR( gpioTab[5].IsrData ); }
void ICACHE_RAM_ATTR gpioISR12() { gpioTab[6].MoToISR( gpioTab[6].IsrData ); }
void ICACHE_RAM_ATTR gpioISR13() { gpioTab[7].MoToISR( gpioTab[7].IsrData ); }
void ICACHE_RAM_ATTR gpioISR14() { gpioTab[8].MoToISR( gpioTab[8].IsrData ); }
void ICACHE_RAM_ATTR gpioISR15() { gpioTab[9].MoToISR( gpioTab[9].IsrData ); }

gpioISR_t gpioTab[MAX_GPIO] = {
        &gpioISR0,NULL,NULL
        ,&gpioISR1,NULL,NULL
        ,&gpioISR2,NULL,NULL
        ,&gpioISR3,NULL,NULL
        ,&gpioISR4,NULL,NULL
        ,&gpioISR5,NULL,NULL
        ,&gpioISR12,NULL,NULL
        ,&gpioISR13,NULL,NULL
        ,&gpioISR14,NULL,NULL
        ,&gpioISR15,NULL,NULL
       };

static const uint32_t mask6_11 = 0b00000111111000000;
static uint32_t gpioUsedMask = mask6_11; // gpio 6..11 are always 'in use'
bool gpioUsed( unsigned int gpio ) {
    return ( gpioUsedMask & ( 1<<gpio ) ) ;
}
void setGpio( unsigned int gpio ) {
    gpioUsedMask |= ( 1<<gpio );
}
void clrGpio( unsigned int gpio ) {
    gpioUsedMask &= ~( 1<<gpio );
    gpioUsedMask |= mask6_11;   // gpio 6...11 must never be cleared
}
#endif //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ End of ESP8266 specific section ^^^^^^^^^^^^^^^^^^^^^^^^^^

#if 0 //def ESP32 /////////////////////////////////////////////// ESP 32 /////////////////////////////////////////////////////
bool timerInitialized = false;
bool spiInitialized = false;
void ISR_Stepper(void);     // defined in MoToISR.cpp
timerConfig_t timerConfig;
portMUX_TYPE stepperMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE servoMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE softledMux = portMUX_INITIALIZER_UNLOCKED;
hw_timer_t * stepTimer = NULL;

void seizeTimer1() {
    // Initiieren des Stepper Timers ------------------------
    stepTimer = timerBegin(STEPPER_TIMER, DIVIDER, true); // true= countup
    timerAttachInterrupt(stepTimer, &ISR_Stepper, true);  // true= edge Interrupt
    timerAlarmWrite(stepTimer, ISR_IDLETIME*TICS_PER_MICROSECOND , false); // false = no autoreload );
    timerAlarmEnable(stepTimer);
    timerInitialized = true;  
    MODE_TP1;   // set debug-pins to Output
    MODE_TP2;
    MODE_TP3;
    MODE_TP4;
}

#endif

