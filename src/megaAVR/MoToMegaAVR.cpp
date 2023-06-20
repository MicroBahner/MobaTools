// AVR HW-spcific Functions
#ifdef ARDUINO_ARCH_MEGAAVR
#include <MobaTools.h>
#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>

//#warning "HW specfic - avr ---"

uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

//void ISR_Stepper(void);     // defined in MoToISR.cpp
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
// ---------- OCRxB Compare Interrupt used for stepper motor and Softleds ----------------
void stepperISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));
void softledISR(uint8_t cyclesLastIRQ) __attribute__ ((weak));
ISR ( TIMERx_COMPB_vect) {
    uint16_t tmp;
  // Timer1 Compare B, used for stepper motor, starts every CYCLETIME us
    // 26-09-15 An Interrupt is only created at timeslices, where data is to output
    SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
   if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
    // compute next IRQ-Time in us, not in tics, so we don't need long
    //noInterrupts(); // when manipulating 16bit Timerregisters IRQ must be disabled
    if ( nextCycle == 1 )  {
        CLR_TP1;
        noInterrupts();
        // this is timecritical: Was the ISR running longer then CYCELTIME?
        // compute length of current IRQ ( which startet at OCRxB )
        // we assume a max. runtime of 1000 Tics ( = 500µs , what nevver should happen )
        tmp = GET_COUNT - OCRxB ;
        if ( tmp > 1000 ) tmp += TIMER_OVL_TICS; // there was a timer overflow
        if ( tmp > (CYCLETICS-10) ) {
            // runtime was too long, next IRQ mus be started immediatly
            //SET_TP3;
            tmp = GET_COUNT+10; 
        } else {
            tmp = OCRxB + CYCLETICS;
        }
        OCRxB = ( tmp > TIMER_OVL_TICS ) ? tmp -= TIMER_OVL_TICS : tmp ;
        interrupts();
        SET_TP1;
    } else {
        // time till next IRQ is more then one cycletime
        // compute next IRQ-Time in us, not in tics, so we don't need long
        tmp = ( OCRxB / TICS_PER_MICROSECOND + nextCycle * CYCLETIME );
        if ( tmp > TIMERPERIODE ) tmp = tmp - TIMERPERIODE;
        OCRxB = tmp * TICS_PER_MICROSECOND;
    }
    cyclesLastIRQ = nextCycle;
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////

void seizeTimerAS() {
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
        // using timer TCA0 in normal mode.
        // CMP0 register used for servos
        // CMP1 register used for steppers and softleds
        noInterrupts();
        TCA0_SINGLE_CTRLA = TCA_SINGLE_CLKSEL_DIV8_gc;      // 0,5µs per tic, timer disabled
        TCA0_SINGLE_CTRLESET = TCA_SINGLE_CMD_RESET_gc;     // hard reset timer
        TCA0_SINGLE_CTRLB = TCA_SINGLE_WGMODE_NORMAL_gc;    // normal mode, no autolockUpdate, no pins active
        TCA0_SINGLE_CTRLC = 0;
        TCA0_SINGLE_CTRLD = 0;                              // no split mode ( user 16bit timer )
        //TCA0_SINGLE_CTRLECLR
        TCA0_SINGLE_CTRLESET = TCA_SINGLE_LUPD_bm;          // don't use buffered compare registes
        //TCA0_SINGLE_CTRLFCLR
        //TCA0_SINGLE_CTRLFSET
        //TCA0_SINGLE_INTCTRL = TCA_SINGLE_CMP0_bm | TCA_SINGLE_CMP1_bm; // enable cmp0 and cmp1 interrupt
        TCA0_SINGLE_INTFLAGS = 0;   // clear all interrupts
        TCA0_SINGLE_PER  = TIMERPERIODE * TICS_PER_MICROSECOND;  // timer periode is 20000us 
        TCA0_SINGLE_CMP0 = FIRST_PULSE;   
        TCA0_SINGLE_CMP1 = 400;  
        TCA0_SINGLE_CTRLA |= TCA_SINGLE_ENABLE_bm;          // Enable the timer
        interrupts();
        timerInitialized = true;  
        MODE_TP1;   // set debug-pins to Output
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
        DB_PRINT("Testpins initialisiert");
    }
}

extern uint8_t spiStepperData[2]; // step pattern to be output on SPI
extern uint8_t spiByteCount;


void enableSoftLedIsrAS() {
}

#endif