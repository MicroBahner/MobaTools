// RA4M1 HW-spcific Functions
#ifdef ARDUINO_ARCH_RENESAS_UNO 
//#define bool int
#include <MobaTools.h>
//#define debugTP
//#define debugPrint
#include <utilities/MoToDbg.h>

#warning "HW specfic - RA4M1 ---"
// RA4M1 specific variables
FspTimer MoTo_timer;	// create Timer object

const uint32_t gptOfs = 0x100L;                      // offset betwen register blocks
//const R_GPT0_Type *gpt0RegP = (R_GPT0_Type *)R_GPT0_BASE;  // GPT0-Adresse
R_GPT0_Type *gptRegP = (R_GPT0_Type *)R_GPT0_BASE;                     // Pointer to active timer registers
R_ICU_Type *icuRegP = (R_ICU_Type *)R_ICU_BASE;      // Pointer to Interrupt registers
uint8_t IelsrIxOvf;                                  // Index for gpt overflow-Entry
// Event nbr for ICU table
uint16_t icuEventOvf;
uint16_t icuEventCmpA;
uint16_t icuEventCmpB;
// Index in ICU table for this event ( = NVIC-IRQ-Number )
IRQn_Type IRQnStepper ;		// NVIC-IRQ number for stepper IRQ ( cmpA )
IRQn_Type IRQnServo ;	 	// NVIC-IRQ number for servo IRQ    (cmpB )
uint8_t noStepISR_Cnt = 0;   // Counter for nested StepISr-disable

void stepperISR(int32_t cyclesLastIRQ)  __attribute__ ((weak));
void softledISR(uint32_t cyclesLastIRQ)  __attribute__ ((weak));
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
void ISR_Stepper() {
    // GPT Timer CCMPA, used for stepper motor and softleds, starts every nextCycle us
	// Quit irq-flag
	icuRegP->IELSR_b[IRQnStepper].IR = 0;

    // nextCycle ist set in stepperISR and softledISR
    SET_TP1;
    nextCycle = ISR_IDLETIME  / CYCLETIME ;// min ist one cycle per IDLETIME
    if ( stepperISR ) stepperISR(cyclesLastIRQ);
    //============  End of steppermotor ======================================
    if ( softledISR ) softledISR(cyclesLastIRQ);
    // ======================= end of softleds =====================================
    // set compareregister to next interrupt time;
	// next ISR must be at least MIN_STEP_CYCLE/4 beyond actual counter value ( time between to ISR's )
	uint32_t minOCR = gptRegP->GTCNT;
	uint32_t nextOCR = gptRegP->GTCCR[0];  // CCRA = Step cmp
	if ( minOCR < nextOCR ) minOCR += TIMER_OVL_TICS; // timer had overflow already
    minOCR = minOCR + ( (MIN_STEP_CYCLE/4) * TICS_PER_MICROSECOND ); // minimumvalue for next OCR
	nextOCR = nextOCR + ( nextCycle * TICS_PER_MICROSECOND );
	if ( nextOCR < minOCR ) {
		// time till next ISR ist too short, set to mintime and adjust nextCycle
        SET_TP2;
		nextOCR = minOCR;
		nextCycle = ( nextOCR - gptRegP->GTCCR[0]  ) / TICS_PER_MICROSECOND;
        CLR_TP2;
	}
    if ( nextOCR > (uint16_t)TIMER_OVL_TICS ) nextOCR -= TIMER_OVL_TICS;
    gptRegP->GTCCR[0] = nextOCR  ;
    cyclesLastIRQ = nextCycle;
    CLR_TP1; // Oszimessung Dauer der ISR-Routine
}
////////////////////////////////////////////////////////////////////////////////////////////
void seizeTimerAS() {
	int8_t tindex;  // used Timer;
    static bool timerInitialized = false;
    if ( !timerInitialized ) {
		// Initialize GPT Timer
		uint8_t timer_type = GPT_TIMER;
		tindex = FspTimer::get_available_timer(timer_type);
		// compute pointer to active timer registers
		gptRegP = (R_GPT0_Type *)((uint8_t *)R_GPT0_BASE + (0x100 * tindex));
		// compute ISR event numbers
		icuEventCmpA = evGPT0_CCMPA + (tindex * evGPT_OFSET); // Stepper
		icuEventCmpB = evGPT0_CCMPB + (tindex * evGPT_OFSET); // Servo

		MoTo_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, 60000, 20000, TIMER_SOURCE_DIV_16); 
		gptRegP->GTBER = 0x3;  // no Buffer operation
		MoTo_timer.open();
		MoTo_timer.start();

		gptRegP->GTCCR[0] = 0;
		gptRegP->GTCCR[1] = 0;
		gptRegP->GTPR = TIMER_OVL_TICS;

        MODE_TP1;
        MODE_TP2;
        MODE_TP3;
        MODE_TP4;
    }
}




#endif
