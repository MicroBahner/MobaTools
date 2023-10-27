// RA4M1 HW-spcific Functions
#ifdef ARDUINO_ARCH_RENESAS_UNO 
//#define bool int
#include <MobaTools.h>
//#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>

//#warning "HW specfic - RA4M1 ---"

uint8_t noStepISR_Cnt = 0;   // Counter for nested StepISr-disable

void stepperISR(int32_t cyclesLastIRQ)  __attribute__ ((weak));
void softledISR(uint32_t cyclesLastIRQ)  __attribute__ ((weak));
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
void ISR_Stepper() {
    // Timer4 Channel 1, used for stepper motor and softleds, starts every nextCycle us
    // nextCycle ist set in stepperISR and softledISR
    SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
    if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
	// next ISR must be at least MIN_STEP_CYCLE/4 beyond actual counter value ( time between to ISR's )
	int minOCR = timer_get_count(MT_TIMER);
	int nextOCR = timer_get_compare(MT_TIMER, STEP_CHN);
	if ( minOCR < nextOCR ) minOCR += TIMER_OVL_TICS; // timer had overflow already
    minOCR = minOCR + ( (MIN_STEP_CYCLE/4) * TICS_PER_MICROSECOND ); // minimumvalue for next OCR
	nextOCR = nextOCR + ( nextCycle * TICS_PER_MICROSECOND );
	if ( nextOCR < minOCR ) {
		// time till next ISR ist too short, set to mintime and adjust nextCycle
        SET_TP2;
		nextOCR = minOCR;
		nextCycle = ( nextOCR - timer_get_compare(MT_TIMER, STEP_CHN)  ) / TICS_PER_MICROSECOND;
        CLR_TP2;
	}
    if ( nextOCR > (uint16_t)TIMER_OVL_TICS ) nextOCR -= TIMER_OVL_TICS;
    timer_set_compare( MT_TIMER, STEP_CHN, nextOCR ) ;
    cyclesLastIRQ = nextCycle;
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////
void seizeTimerAS() {
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
		// Initialize GPT Timer

        MODE_TP1;
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
    }
}


void enableServoIsrAS() {
}

extern "C" {
// ------------------------  ISR for SPI-Stepper ------------------------
static int rxData;
#ifdef USE_SPI2
void __irq_spi2(void) {// STM32  spi2 irq vector
    rxData = spi_rx_reg(SPI2);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI2_NSS_PIN,HIGH);
}
#else
void __irq_spi1(void) {// STM32  spi1 irq vector
    //SET_TP4;
    rxData = spi_rx_reg(SPI1);            // Get dummy data (Clear RXNE-Flag)
    digitalWrite(BOARD_SPI1_NSS_PIN,HIGH);
    //CLR_TP4;
}
#endif
} // end of extern "C"



#endif
