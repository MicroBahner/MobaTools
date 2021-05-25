// ESP8266 HW-specific parts for MobaTools
//--------------------------------------------------------------------------------------------------------------
#ifndef ESP8266_DRIVER_H
#define ESP8266_DRIVER_H
//vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv ESP8266 vvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
#define IS_32BIT
#define IS_ESP  8266
#include <esp8266/ESP8266_waveform.h>
//on ESP8266 all values are in µsec
#define TICS_PER_MICROSECOND 1
bool gpioUsed( unsigned int gpio );
void setGpio( unsigned int gpio ) ;
void clrGpio( unsigned int gpio ) ;

// on ESP8266 tics are 1µs
#define time2tic(pulse) (pulse) * SPEED_RES
#define tic2time(duty)  (duty) / SPEED_RES


#define slPwm2tic(pulse) (pulse)
#define tic2slPwm(duty)  (duty)

// struct for gpio ISR Routines ( needs one struct elemet per GPIO
typedef struct {
    void (*gpioISR)();
    void (*MoToISR)(void *Data);
    void *IsrData;
}gpioISR_t;

extern gpioISR_t gpioTab[MAX_GPIO];
//convert gpio nbr to index in gpioTab:
#define gpio2ISRx(gpio) (gpio>5?gpio-6:gpio) // gpio 6..11 are not allowed
#define pin2Ix(gpio) (gpio>5?gpio-6:gpio) // gpio 6..11 are not allowed
#define chkGpio(gpio) ( (gpio<=5) || ( gpio>11 && gpio<=16 ) );

// ignore ESP32 semaphores in ISR
#define portENTER_CRITICAL_ISR(x)
#define portEXIT_CRITICAL_ISR(x)

// the following is included at the end of MoToBase.h
#define ARCHITECT_INCLUDE <esp8266/MoToESP8266.h>

//^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ end of ESP8266 ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
#endif