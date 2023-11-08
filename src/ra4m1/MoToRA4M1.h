#ifndef MOTORA4M1_H
#define MOTORA4M1_H
// RA4M1 specific defines for Cpp files

#include "FspTimer.h"
#include <bsp_api.h>

//#warning RA4M1 specific cpp includes
extern IRQn_Type IRQnStepper ;		// NVIC-IRQ number for stepper IRQ ( GPT cmpA )
extern IRQn_Type IRQnServo ;	 	// NVIC-IRQ number for servo IRQ   ( GPT cmpB )

extern uint8_t noStepISR_Cnt;   // Counter for nested StepISr-disable

void seizeTimerAS();

static inline __attribute__((__always_inline__)) void _noStepIRQ() {
    // disable stepper IRQ ( GPT cmpA match IRQ )
    NVIC_DisableIRQ(IRQnStepper);
    noStepISR_Cnt++;
    #if defined COMPILING_MOTOSTEPPER_CPP
        //Serial.println(noStepISR_Cnt);
        SET_TP3;
    #endif
}
static inline __attribute__((__always_inline__)) void  _stepIRQ(bool force = false) {
	// enable stepper IRQ ( GPT cmpA match IRQ )
    if ( force ) noStepISR_Cnt = 1;              //enable IRQ immediately
    if ( noStepISR_Cnt > 0 ) noStepISR_Cnt -= 1; // don't decrease if already 0 ( if enabling IRQ is called too often )
    if ( noStepISR_Cnt == 0 ) {
        #if defined COMPILING_MOTOSTEPPER_CPP
            CLR_TP3;
        #endif
        NVIC_EnableIRQ(IRQnStepper);

    }
    //Serial.println(noStepISR_Cnt);
}

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSERVO_CPP
void ISR_Servo( void );


static inline __attribute__((__always_inline__)) void enableServoIsrAS() {
    NVIC_EnableIRQ(IRQnServo);
}

#endif // COMPILING_MOTOSERVO_CPP

/////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSOFTLED32_CPP
static inline __attribute__((__always_inline__)) void enableSoftLedIsrAS() {
    timer_cc_enable(MT_TIMER, STEP_CHN);
}

#endif // COMPILING_MOTOSOFTLED_CPP

//////////////////////////////////////////////////////////////////////////////////////////////////
#if defined COMPILING_MOTOSTEPPER_CPP

static inline __attribute__((__always_inline__)) void enableStepperIsrAS() {
    timer_cc_enable(MT_TIMER, STEP_CHN);
}

static uint8_t spiInitialized = false;
static inline __attribute__((__always_inline__)) void initSpiAS() {
    if ( spiInitialized ) return;
    // initialize SPI hardware.
    // MSB first, default Clk Level is 0, shift on leading edge
    #ifdef USE_SPI2// use SPI 2 interface
    spi_init(SPI2);
    spi_config_gpios(SPI2, 1,  // initialize as master
                     PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_device, PIN_MAP[BOARD_SPI2_NSS_PIN].gpio_bit,        
                     PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_device, PIN_MAP[BOARD_SPI2_SCK_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MISO_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI2_MOSI_PIN].gpio_bit);

    uint32 flags = (SPI_FRAME_MSB | SPI_CR1_DFF_16_BIT | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(SPI2, (spi_baud_rate)SPI_BAUD_PCLK_DIV_64, (spi_mode)SPI_MODE_0, flags);
    spi_irq_enable(SPI2, SPI_RXNE_INTERRUPT);
    pinMode( BOARD_SPI2_NSS_PIN, OUTPUT);
    digitalWrite( BOARD_SPI2_NSS_PIN, LOW );

    #else// use SPI 1 interface
    spi_init(SPI1);
    spi_config_gpios(SPI1, 1,  // initialize as master
                     PIN_MAP[BOARD_SPI1_NSS_PIN].gpio_device, PIN_MAP[BOARD_SPI1_NSS_PIN].gpio_bit,        
                     PIN_MAP[BOARD_SPI1_SCK_PIN].gpio_device, PIN_MAP[BOARD_SPI1_SCK_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI1_MISO_PIN].gpio_bit,
                     PIN_MAP[BOARD_SPI1_MOSI_PIN].gpio_bit);

    uint32 flags = (SPI_FRAME_MSB | SPI_CR1_DFF_16_BIT | SPI_SW_SLAVE | SPI_SOFT_SS);
    spi_master_enable(SPI1, (spi_baud_rate)SPI_BAUD_PCLK_DIV_64, (spi_mode)SPI_MODE_0, flags);
    spi_irq_enable(SPI1, SPI_RXNE_INTERRUPT);
    pinMode( BOARD_SPI1_NSS_PIN, OUTPUT);
    digitalWrite( BOARD_SPI1_NSS_PIN, LOW );
    #endif
    spiInitialized = true;  
}

    static inline __attribute__((__always_inline__)) void startSpiWriteAS( uint8_t spiData[] ) {
        #ifdef USE_SPI2
        digitalWrite(BOARD_SPI2_NSS_PIN,LOW);
        spi_tx_reg(SPI2, (spiData[1]<<8) + spiData[0] );
        #else
        digitalWrite(BOARD_SPI1_NSS_PIN,LOW);
        spi_tx_reg(SPI1, (spiData[1]<<8) + spiData[0] );
        #endif
    }    
    

#endif // COMPILING_MOTOSTEPPER_CPP


#endif