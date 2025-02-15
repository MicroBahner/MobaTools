// RP2040 HW-spcific Functions
#ifdef ARDUINO_ARCH_RP2040
#define debugTP
#define debugPrint
#include <MobaTools.h> // Hier nur Test der RP2040 Aufrufe
//#include "testGlobals.h" // global notwendige Definitionen ( sonst aus MobaTools.h )
#define debugTP
//#define debugPrint
//#include <utilities/MoToDbg.h>
#undef stepperISR // When calling stepperISR, the attribute '__not_in_flash_func' is not allowed
#warning "HW specfic - RP2040 ---"

// The RP-Timer always tics with 1µs, so TICS_PER_MICROSECND is always 1

uint8_t stepperAlarm;   // dynamically assigned when timer is initialized
timer_hw_t *stepperTimer; // Must be timer 0 on RP2040, maybe timer 1 on RP2350 (pi Pico 2 )
uint8_t stepperIRQNum;  // dynamically assigned when timer is initialized
uint8_t noStepISR_Cnt = 0;   // Counter for nested StepISr-disable

// bool spiInitialized = false;
void stepperISR(uint32_t cyclesLastIRQ)  __attribute__ ((weak));
nextCycle_t nextCycle;
static nextCycle_t cyclesLastIRQ = 1;  // cycles since last IRQ
static absolute_time_t lastAlarm, aktAlarm;
//void ISR_Stepper(uint alarmNum) {
void __not_in_flash_func(ISR_Stepper)() {
  // Clear the alarm irq
  hw_clear_bits(&stepperTimer->intr, 1u << stepperAlarm);
  // Timer running up, used for stepper motor. No reload of timer
  SET_TP1;
  nextCycle = ISR_IDLETIME ;// max time between IRQ's is IDLETIME
  cyclesLastIRQ = (aktAlarm - lastAlarm); // 
  //------------- call cumputing of next stepIRQ -----//
  if ( stepperISR ) stepperISR(cyclesLastIRQ);        //
  //--------------------------------------------------//
  lastAlarm = aktAlarm;
  aktAlarm = lastAlarm + nextCycle; // time until next Interrupt
  /*if ( aktAlarm - lastAlarm < MIN_STEP_CYCLE ) {
    // time till next ISR ist too short, set to mintime and adjust nextCycle
    CLR_TP1;
    aktAlarm =  lastAlarm + MIN_STEP_CYCLE;
  }*/
  while (timer_hardware_alarm_set_target (stepperTimer,stepperAlarm, aktAlarm )) { // mit Prüfung ob Alarmzeit bereits
    CLR_TP1;                                                    // überschritten 
    // TODO: Prüfung ab Timer die Alarmzeit bereits überschritten hat ( ohne die Prüfung macht die
    // Verwendung der 'langsamen' Variante eigentlich keinen Sinn.
    aktAlarm += MIN_STEP_CYCLE;
  }                                                          
  SET_TP1;
  CLR_TP1; // Oszimessung Dauer der ISR-Routine
}


bool seizeTimerAS() {
  static bool timerInitialized = false;
  // Initiieren des Stepper Timers ------------------------
  if ( !timerInitialized ) {
    stepperTimer = timer_get_instance (STP_TIMR_NBR);
    SET_TP2;
    //stepperTimer = PICO_DEFAULT_TIMER_INSTANCE();
    timer_hardware_alarm_claim_unused (stepperTimer,true); // Test: Dummy-Belegung des 1. Alarms
    stepperAlarm = timer_hardware_alarm_claim_unused (stepperTimer,true); // core will panic if none is available
    DB_PRINT("Alarm=%d\n", stepperAlarm); 
    stepperIRQNum = timer_hardware_alarm_get_irq_num (stepperTimer, stepperAlarm); // Needed if prority must be changed or Irq dis-/en-abled
    DB_PRINT("Alm=%d, Timer=%d, IRQNum=%d\n", stepperAlarm, timer_get_index(stepperTimer), stepperIRQNum ); Serial.flush();
    hw_set_bits(&stepperTimer->inte, 1u << stepperAlarm); // enable Alarm irq in timer-HW
    irq_set_exclusive_handler(stepperIRQNum, ISR_Stepper); // set IRQ handler
    irq_set_priority(stepperIRQNum, 10);    // default is 128 ( lower value = higher priority )
    irq_set_enabled (stepperIRQNum, true);  // enable Alarm IRQ in NVIC
    // NUR FÜR TEST: Alarm-Überlauf Timer kurz vor Überlauf stellen.
    stepperTimer->timelw = 4290000000L;
    stepperTimer->timehw = 0;
    
    lastAlarm = timer_time_us_64(stepperTimer);
    aktAlarm = timer_time_us_64(stepperTimer) + 500;//ISR_IDLETIME;
    timer_hardware_alarm_set_target (stepperTimer,stepperAlarm, aktAlarm); 
    Serial.println("set first target for alarm"); Serial.flush();
    timerInitialized = true;
    Serial.println("Timer initialisiert");
    CLR_TP2;
  }
  return timerInitialized;
}


void enableServoIsrAS() {
}


void enableSoftLedIsrAS() {

}

#ifndef USE_SPI1
// Set SPI pins ( RX/MISO pin is not defined/used )
  #ifdef ARDUINO_NANO_RP2040_CONNECT
      #define MOTO_SPI_SCK_PIN 6
      #define MOTO_SPI_TX_PIN 7
      #define MOTO_SPI_CSN_PIN 5
  #else
      #define MOTO_SPI_SCK_PIN 18
      #define MOTO_SPI_TX_PIN 19
      #define MOTO_SPI_CSN_PIN 17
  #endif
  spi_inst_t *stepperSPI = spi0;
#else
  // Set SPI pins ( RX/MISO pin is not defined/used )
  #ifdef ARDUINO_NANO_RP2040_CONNECT
      #define MOTO_SPI_SCK_PIN 6
      #define MOTO_SPI_TX_PIN 7
      #define MOTO_SPI_CSN_PIN 5
      spi_inst_t *stepperSPI = spi0; // no SPI1 available on Nano RP2040 connect
  #else
      #define MOTO_SPI_SCK_PIN 14
      #define MOTO_SPI_TX_PIN 15
      #define MOTO_SPI_CSN_PIN 13
      spi_inst_t *stepperSPI = spi1;
  #endif
#endif

uint32_t spiBaudrate = SPI_CLOCK;
void initSpiAS() {
    static uint8_t spiInitialized = false;
    if ( spiInitialized ) return;
    // initialize SPI hardware.
    // MSB first, default Clk Level is 0, shift on leading edge
    // Enable SPI 0 at 2 MHz and connect to GPIOs
    // spi_init(spi_default, 2000 * 1000); // 
    // Own init to set databits to 16
    //spi_reset(spi);
    reset_block_num(stepperSPI == spi0 ? RESET_SPI0 : RESET_SPI1);
    //spi_unreset(spi);
    unreset_block_num_wait_blocking(stepperSPI == spi0 ? RESET_SPI0 : RESET_SPI1);

    uint baud = spi_set_baudrate(stepperSPI, spiBaudrate);
    spi_set_format(stepperSPI, 16, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // Always enable DREQ signals -- harmless if DMA is not listening
    hw_set_bits(&spi_get_hw(stepperSPI)->dmacr, SPI_SSPDMACR_TXDMAE_BITS | SPI_SSPDMACR_RXDMAE_BITS);

    // Finally enable the SPI
    hw_set_bits(&spi_get_hw(stepperSPI)->cr1, SPI_SSPCR1_SSE_BITS);
    
    // gpio_set_function(MOTO_SPI_RX_PIN, GPIO_FUNC_SPI); // no reiceved data
    gpio_set_function(MOTO_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOTO_SPI_TX_PIN, GPIO_FUNC_SPI);
    gpio_set_function(MOTO_SPI_CSN_PIN, GPIO_FUNC_SPI);

    spiInitialized = true;  
}

#endif