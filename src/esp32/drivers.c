// ESP32 HW-spcific Functions
#ifdef ARDUINO_ARCH_ESP32
#include <arduino.h>
#include "drivers.h"

#include "esp32-hal.h"
#include "freertos/semphr.h"
#include "esp32-hal-matrix.h"
#include "soc/dport_reg.h"
#include "soc/ledc_reg.h"
#include "soc/ledc_struct.h"

#warning "HW specfic drivers.c - ESP32 ---"

static uint16_t ledcUsed =0 ; // Bit field to mark used pwm channels
#define groupUsed(pwmNr)    ((pwmNr)%1)
#define channelUsed(pwmNr)  (((pwmNr)/2)%8)

// There are max 16 pwm ouputs. These are used by servos and softleds
struct pwmUse_t {
    union {
        struct {
            uint32_t pin     :8;     // used pwm HW ( 0... 15
            uint32_t inUse   :1;     // 0 
            uint32_t group   :1;     // leds group ( 0/1 )
            uint32_t timer   :2;     // Timer used ( 2 for servo, 3 for softled, 0/1 unused by MobaTools
            uint32_t channel :3;     // ledc  channel ( 0..7 )
            uint32_t reserved:22;
        };
        uint32_t value;
    };
}pwmUse[16];

int8_t _findPwmNbr( ) {
    // find a free pwm Nbr and return it
    // value is negative if no free channel available
    int8_t pwmNbr;
    for( pwmNbr=15; pwmNbr>=0; pwmNbr-- ) {
        if ( !( pwmUse[pwmNbr].inUse ) ) {
            pwmUse[pwmNbr].inUse = 1;
            pwmUse[pwmNbr].group = groupUsed(pwmNbr);
            pwmUse[pwmNbr].channel = channelUsed(pwmNbr);
            break;
        }
    }
    return pwmNbr;
}

int8_t freePwmNbr( uint8_t pwmNbr ) {
    if ( pwmNbr > 15 ) return false;
    ledcUsed &= ~(1<<pwmNbr);
    return true;
}

int8_t initPwmChannel( uint8_t pin, uint8_t timer ) {
    uint8_t pwmNbr = _findPwmNbr();
    if ( pwmNbr >= 0 ) {
        // found a free channel
        pwmUse[pwmNbr].timer = timer;
        pwmUse[pwmNbr].pin = pin;
        ledcSetup(pwmNbr, timer==SERVO_TIMER?SERVO_FREQ:SOFTLED_FREQ , LEDC_BITS );
    }
    return pwmNbr;
}

void IRAM_ATTR setPwmDuty(int8_t pwmNbr, uint32_t duty ){
    ledcWrite(pwmNbr, duty);
}


#endif // End of file