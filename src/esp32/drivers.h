// ESP32 HW-specific parts for MobaTools
    //--------------------------------------------------------------------------------------------------------------
#ifndef ESP32_DRIVER_H
#define ESP32_DRIVER_H

#ifdef __cplusplus
extern "C" {
#endif
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv  ESP32  vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define IS_32BIT
#define IS_ESP  32

// ----------------   stepper related defines   ---------------------------------
// use of SPI interface
#ifdef USE_VSPI
    #define SPI_USED    VSPI
    #define MOSI        23
    #define SCK         18
    #define SS          5
#else
    // HSPI is used by default
    #define SPI_USED    HSPI
    #define MOSI        13
    #define SCK         14
    #define SS          15
#endif

struct timerConfig_t {
  union {
    struct {
      uint32_t reserved0:   10;
      uint32_t alarm_en:     1;             /*When set  alarm is enabled*/
      uint32_t level_int_en: 1;             /*When set  level type interrupt will be generated during alarm*/
      uint32_t edge_int_en:  1;             /*When set  edge type interrupt will be generated during alarm*/
      uint32_t divider:     16;             /*Timer clock (T0/1_clk) pre-scale value.*/
      uint32_t autoreload:   1;             /*When set  timer 0/1 auto-reload at alarming is enabled*/
      uint32_t increase:     1;             /*When set  timer 0/1 time-base counter increment. When cleared timer 0 time-base counter decrement.*/
      uint32_t enable:       1;             /*When set  timer 0/1 time-base counter is enabled*/
    };
    uint32_t val;
  };
} ;
extern bool timerInitialized;
// Prescaler for 64-Bit Timer ( input is 
#define DIVIDER     APB_CLK_FREQ/2/1000000  // 0,5µs Timertic ( 80MHz input freq )
#define TICS_PER_MICROSECOND 2              // bei 0,5 µs Timertic
// Mutexes für Zugriff auf Daten, die in ISR verändert werden
extern portMUX_TYPE stepperMux;

#define STEPPER_TIMER     3  // Timer 1, Group 1
extern hw_timer_t * stepTimer;
void seizeTimer1();
//void initSPI();             // initSPI is defined in MoToStepperAVR.inc ( it is only used with MoToStepper

// ----------------   defines for servo and softled ( ledc pwm hardware on ESP32 is used ) -----------------------
#define SERVO_TIMER         2
#define LED_TIMER           3

int8_t initPwmChannel( uint8_t pin, uint8_t timer );
void IRAM_ATTR setPwmDuty(int8_t pwmNbr, uint32_t duty );



#define SERVO_FREQ  50          // 20000 
#define SOFTLED_FREQ    100
#define LEDC_BITS  16          // bitresolution for duty cycle of servos and softleds
#define SERVO_CYCLE ( 1000000L / SERVO_FREQ ) // Servo cycle in uS
#define SOFTLED_CYCLE ( 1000000L / SOFTLED_FREQ ) // Servo cycle in uS
#define DUTY100     (( 1<<LEDC_BITS )-1)
// compute pulsewidth ( in usec ) to duty )
#define time2tic(pulse) ( ( (pulse) *  DUTY100) / SERVO_CYCLE )  
// compute duty to pulsewidth ( in uS )
#define tic2time(duty)  ( ( (duty) * SERVO_CYCLE) / DUTY100 )

extern portMUX_TYPE softledMux;
extern portMUX_TYPE servoMux;

    //^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ end of ESP32 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#ifdef __cplusplus
}
#endif
#define ARCHITECT_INCLUDE <esp32/MoToESP32.h>
#endif /* ESP32_DRIVER_H */

